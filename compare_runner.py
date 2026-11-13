"""
고정식 vs 적응형 신호 '나란히' 비교 (SUMO 2개 동시 실행)
------------------------------------------------------------
같은 차량 수요(동일 route)를 두 신호체계에 똑같이 투입해서,
적응형 신호가 교통정체(정지 차량)를 얼마나 줄이는지 한눈에 비교한다.

  ┌───────────────┐   ┌───────────────┐
  │ 왼쪽 창: 고정식 │   │ 오른쪽 창: 적응형 │
  │ (기준 녹색 고정) │   │ (3사이클마다 재배분) │
  └───────────────┘   └───────────────┘

- 두 창은 완전히 동일한 차량 입력을 받는다(같은 sumo/intersection.rou.xml). → 공정 비교.
- 차이는 '신호 시간 배분 방식'뿐. 단, 고정식은 보행자 현시가 DEFAULT_GREEN의 60초,
  적응형은 첫 재배분부터 PED_FIXED_GREEN의 30초다. (README '한계' 참고)
- 실행 중: 콘솔에 '라이브 점수판'(현재 대기줄·누적지연·개선율)을 막대로 표시.
- 종료 후: 비교 그래프(comparison.png) 저장 + 평균 대기시간 요약.

실행:  python compare_runner.py
       (GUI 없이 결과만 빠르게 보려면 아래 HEADLESS를 True로 바꾼다)
"""

import os
import sys
import xml.etree.ElementTree as ET

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

import traci  # noqa: E402

# sumo_runner의 함수/설정을 그대로 재사용 (신호 프로그램 생성·우회전·route·점유율 측정 등)
import sumo_runner as SR  # noqa: E402
from signal_allocation import (  # noqa: E402
    DEFAULT_GREEN,
    VEHICLE_PHASES,
    NUM_VEHICLE_PHASES,
    CYCLES_PER_BLOCK,
    TOTAL_GREEN_BUDGET,
    allocate_from_demand,
)

TLS_ID = SR.TLS_ID
APPROACH_EDGES = SR.APPROACH_EDGES
DIR_NAME = {1: "북", 2: "동", 3: "남", 4: "서"}

# HEADLESS=True 면 GUI/딜레이 없이 콘솔로 빠르게 돌려 그래프만 뽑는다.
#   (긴 시뮬레이션의 '더 좋은 그래프'를 오래 안 보고 바로 얻고 싶을 때)
# False 면 좌우 두 sumo-gui 창을 띄워 눈으로 비교.
HEADLESS = False

# 화면 배치(픽셀). 필요하면 모니터에 맞게 조정.
LEFT_POS = "0,0"
RIGHT_POS = "930,0"
WIN_SIZE = "900,1000"

# 무한 교착(정체로 안 빠지는 경우) 방지용 최대 스텝
MAX_STEPS = SR.SIM_DURATION * 2

CHART_FILE = "comparison.png"

# 평가에서 제외할 워밍업 시간(초). 첫 재배분(1블록) 전에는 두 방식 모두 기준값
# (DEFAULT_GREEN)으로 동일하게 동작하므로, 그 구간을 빼 개선 효과가 희석되지 않게 한다.
# 기본 타이밍의 1블록 길이 = 3사이클 × (차량+보행 녹색 합 + 방향별 황색)로 자동 계산.
WARMUP_SEC = CYCLES_PER_BLOCK * (TOTAL_GREEN_BUDGET + NUM_VEHICLE_PHASES * SR.YELLOW)
# SUMO 표준 지연 지표(timeLoss)·처리량 수집용 tripinfo 출력 파일 (두 연결 각각).
FIXED_TRIPINFO = "tripinfo_fixed.xml"
ADAP_TRIPINFO = "tripinfo_adaptive.xml"


def read_tripinfo(path, warmup):
    """tripinfo XML을 읽어 (통과 대수, timeLoss 합, waitingTime 합)을 반환.

    - depart(출발시각) < warmup 인 차량은 제외(워밍업).
    - timeLoss: 자유주행 대비 손실시간(초). 완전정지뿐 아니라 서행 지연까지 포함(표준 지연).
    - 파일이 없거나(gridlock으로 미완주 등) 파싱 실패 시 0으로 처리.
    """
    try:
        root = ET.parse(path).getroot()
    except (FileNotFoundError, ET.ParseError):
        return 0, 0.0, 0.0
    n, tl, wt = 0, 0.0, 0.0
    for ti in root.findall("tripinfo"):
        if float(ti.get("depart", 0)) < warmup:
            continue
        n += 1
        tl += float(ti.get("timeLoss", 0))
        wt += float(ti.get("waitingTime", 0))
    return n, tl, wt


def halting_on_approaches():
    """현재 연결(switch된 시뮬)의 4개 진입도로 위 '정지(halting) 차량 수' 합."""
    total = 0
    for edges in APPROACH_EDGES.values():
        for e in edges:
            total += traci.edge.getLastStepHaltingNumber(e)
    return total


def setup(label):
    """해당 연결에 우리 9-phase 프로그램을 기준값(DEFAULT_GREEN)으로 설치."""
    traci.switch(label)
    n, groups = SR.build_link_groups(TLS_ID)
    rt = SR.build_right_turn_indices(TLS_ID) if SR.RIGHT_TURN_ON_RED else frozenset()
    SR.install_program(TLS_ID, dict(DEFAULT_GREEN), n, groups, rt)
    return n, groups, rt


def _bar(value, vmax, width=22):
    """0~vmax 값을 길이 width 의 '#' 막대로."""
    if vmax <= 0:
        return " " * width
    fill = int(round(width * min(value, vmax) / vmax))
    return "#" * fill + "-" * (width - fill)


def print_scoreboard(t, fq, aq, fd, ad):
    """라이브 점수판: 현재 대기줄(fq/aq)과 누적지연(fd/ad)을 막대로."""
    qmax = max(fq, aq, 1)
    imp = (fd - ad) / fd * 100 if fd else 0
    print(f"[{t:>4}s] 현재 대기줄  고정식 |{_bar(fq, qmax)}| {fq:>3}   "
          f"적응형 |{_bar(aq, qmax)}| {aq:>3}")
    print(f"        누적 정지    고정식 {fd:>10,}   적응형 {ad:>10,}   "
          f"→ 적응형 {imp:+5.1f}% {'개선' if imp >= 0 else '악화'}")


def save_chart(times, fixed_cum, adap_cum, fixed_q, adap_q, seg_windows, seg_summary,
               ntrips, warmup_sec=0, metrics=None):
    """고정식 vs 적응형 비교 그래프 저장.

    metrics: 콘솔 요약과 동일한 워밍업 제외 지표 dict
             {imp_tl, imp_stop, f_stop, a_stop, f_tl, a_tl, f_n, a_n}.
             주어지면 제목·주석·요약박스를 이 값으로 그려 콘솔 출력과 일치시킨다.
    warmup_sec: 평가에서 제외한 워밍업 구간(초). >0이면 그래프에 음영으로 표시.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"[그래프 생략] matplotlib 없음: {e}")
        return

    # 한글 폰트 (Windows)
    for fam in ("Malgun Gothic", "AppleGothic", "NanumGothic"):
        try:
            plt.rcParams["font.family"] = fam
            break
        except Exception:
            pass
    plt.rcParams["axes.unicode_minus"] = False

    C_FIX, C_ADP = "#d1495b", "#2e6f95"   # 고정식=붉은색, 적응형=파란색
    m = metrics or {}
    # 정지 개선율: 지표 dict 우선, 없으면 곡선 끝값(구버전 호환)
    imp_stop = m.get("imp_stop",
                     (fixed_cum[-1] - adap_cum[-1]) / fixed_cum[-1] * 100 if fixed_cum[-1] else 0)
    imp_tl = m.get("imp_tl")   # timeLoss 개선율(있을 때만 제목에 표기)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    if imp_tl is not None:
        title = (f"고정식 vs 적응형 신호 비교  —  지연 {imp_tl:.1f}% · 정지 {imp_stop:.1f}% 감소"
                 f"  (워밍업 제외)")
    else:
        title = f"고정식 vs 적응형 신호 비교  —  적응형이 누적 정체 {imp_stop:.1f}% 감소"
    fig.suptitle(title, fontsize=15, fontweight="bold")

    tmin = [t / 60 for t in times]   # 분 단위 x축

    # (위) 누적 정지시간 곡선 — 벌어질수록 적응형 우위
    ax1.plot(tmin, fixed_cum, color=C_FIX, lw=2.2, label="고정식(기준)")
    ax1.plot(tmin, adap_cum, color=C_ADP, lw=2.2, label="적응형(우리)")
    ax1.fill_between(tmin, adap_cum, fixed_cum, color=C_ADP, alpha=0.10)
    ax1.set_ylabel("누적 정지시간 (대·초)")
    ax1.legend(loc="upper left")
    ax1.grid(alpha=0.3)
    gap = m.get("f_stop", fixed_cum[-1]) - m.get("a_stop", adap_cum[-1])  # 워밍업 제외 격차
    ax1.annotate(f"격차 = 적응형이 줄인 정체\n{gap:,.0f} 대·초 (워밍업 제외)",
                 xy=(tmin[-1], (fixed_cum[-1] + adap_cum[-1]) / 2),
                 xytext=(-160, 0), textcoords="offset points",
                 va="center", fontsize=10,
                 bbox=dict(boxstyle="round", fc="white", ec=C_ADP, alpha=0.9))

    # (아래) 순간 대기줄 — 고정식이 구간마다 치솟음
    ax2.plot(tmin, fixed_q, color=C_FIX, lw=1.0, alpha=0.9, label="고정식")
    ax2.plot(tmin, adap_q, color=C_ADP, lw=1.0, alpha=0.9, label="적응형")
    ax2.set_ylabel("순간 대기 차량 수")
    ax2.set_xlabel("시간 (분)")
    ax2.legend(loc="upper left")
    ax2.grid(alpha=0.3)

    # 구간 경계 + 붐비는 방향 표시
    for si, (t0, t1) in enumerate(seg_windows):
        busy = max(VEHICLE_PHASES, key=lambda d: seg_summary[si][d])
        for ax in (ax1, ax2):
            if si > 0:
                ax.axvline(t0 / 60, color="gray", ls="--", lw=0.8, alpha=0.6)
        ax1.text((t0 + t1) / 2 / 60, ax1.get_ylim()[1] * 0.04,
                 f"{DIR_NAME[busy]}쪽 러시", ha="center", fontsize=9, color="gray")

    # 워밍업 구간 음영 표시(평가 제외 구간) — 두 방식이 동일하게 동작하는 초반
    wmin = warmup_sec / 60
    if wmin > 0:
        for ax in (ax1, ax2):
            ax.axvspan(0, wmin, color="gray", alpha=0.12, zorder=0)
        ax1.text(wmin / 2, ax1.get_ylim()[1] * 0.9, "워밍업\n(평가 제외)",
                 ha="center", va="top", fontsize=8, color="gray")

    # 콘솔 요약과 동일한 처리량·총지연 요약 박스 (지표가 주어질 때만)
    if imp_tl is not None:
        txt = (f"처리량  적응형 {m['a_n']:,} / 고정 {m['f_n']:,} 대\n"
               f"총 지연  적응형 {m['a_tl']:,.0f} / 고정 {m['f_tl']:,.0f} 초")
        ax1.text(0.99, 0.03, txt, transform=ax1.transAxes, ha="right", va="bottom",
                 fontsize=9, bbox=dict(boxstyle="round", fc="white", ec="gray", alpha=0.9))

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(CHART_FILE, dpi=120)
    print(f"[그래프] 저장됨 → {CHART_FILE}")


def run():
    # 1) 동일 수요 생성 (두 창이 똑같은 차량 입력을 받도록 한 번만 생성)
    seg_summary, seg_windows, ntrips = [], [], 0
    if SR.REGENERATE_ROUTES:
        ntrips, seg_summary, seg_windows = SR.generate_vehicle_routes()
        print("[route] 두 신호체계에 동일 투입할 '시간대별' 수요(veh/h):")
        for si, (t0, t1) in enumerate(seg_windows):
            s = seg_summary[si]
            busy = max(VEHICLE_PHASES, key=lambda d: s[d])
            print(f"    구간{si+1} {int(t0//60)}~{int(t1//60)}분: "
                  f"북{s[1]:>5} 동{s[2]:>5} 남{s[3]:>5} 서{s[4]:>5}"
                  f"   → {DIR_NAME[busy]}쪽 러시")
        print(f"    → 총 {ntrips:,} 대\n")

    if HEADLESS:
        # 콘솔 sumo 2개 (그래프만 빠르게 생성)
        base = ["sumo", "-c", SR.SUMOCFG]
        traci.start(base + ["--tripinfo-output", FIXED_TRIPINFO], label="fixed")
        traci.start(base + ["--tripinfo-output", ADAP_TRIPINFO], label="adaptive")
    else:
        # 좌우 sumo-gui 2개 (눈으로 비교)
        common = ["sumo-gui", "-c", SR.SUMOCFG, "--start", "--quit-on-end",
                  "--delay", str(SR.GUI_DELAY_MS), "--window-size", WIN_SIZE,
                  "--gui-settings-file", SR.GUI_SETTINGS_FILE]
        traci.start(common + ["--window-pos", LEFT_POS,
                              "--tripinfo-output", FIXED_TRIPINFO], label="fixed")
        traci.start(common + ["--window-pos", RIGHT_POS,
                              "--tripinfo-output", ADAP_TRIPINFO], label="adaptive")

    # 두 연결 모두 동일한 기준 프로그램으로 시작
    n, groups, rt = setup("fixed")     # 고정식: 이후 재배분 안 함(계속 DEFAULT_GREEN)
    setup("adaptive")                  # 적응형: 초기엔 기준값, 이후 재배분

    # 적응형 블록/사이클 상태 (빨간 구간 10초 간격 점유율 평균 기반)
    traci.switch("adaptive")
    step_len = traci.simulation.getDeltaT()
    green_phase_dir = {2 * (d - 1): d for d in VEHICLE_PHASES}
    red_elapsed = {p: 0.0 for p in VEHICLE_PHASES}
    next_sample = {p: SR.SAMPLE_INTERVAL_SEC for p in VEHICLE_PHASES}
    red_samples = {p: [] for p in VEHICLE_PHASES}
    snap = {p: [] for p in VEHICLE_PHASES}
    cycles = 0
    last_phase = traci.trafficlight.getPhase(TLS_ID)

    fixed_delay = 0      # 고정식 누적 정지(대·초)
    adap_delay = 0       # 적응형 누적 정지(대·초)
    fixed_delay0 = adap_delay0 = 0   # 워밍업 종료 시점의 누적값(최종 지표에서 뺀다)
    step = 0

    # 그래프용 시계열 기록
    times, fixed_cum, adap_cum, fixed_qs, adap_qs = [], [], [], [], []

    while step < MAX_STEPS:
        # --- 고정식 한 스텝 ---
        traci.switch("fixed")
        fixed_alive = traci.simulation.getMinExpectedNumber() > 0
        fq = 0
        if fixed_alive:
            traci.simulationStep()
            fq = halting_on_approaches()
            fixed_delay += fq

        # --- 적응형 한 스텝 (+ 우리 알고리즘 재배분) ---
        traci.switch("adaptive")
        adap_alive = traci.simulation.getMinExpectedNumber() > 0
        aq = 0
        if adap_alive:
            traci.simulationStep()
            aq = halting_on_approaches()
            adap_delay += aq
            cur = traci.trafficlight.getPhase(TLS_ID)
            # 각 방향: 자기 녹색이 아니면(빨간/황색) 경과 누적 + 10초마다 점유율 샘플
            for d in VEHICLE_PHASES:
                if cur == 2 * (d - 1):
                    continue
                red_elapsed[d] += step_len
                if red_elapsed[d] >= next_sample[d]:
                    red_samples[d].append(SR.occupancy_150m(APPROACH_EDGES[d]))
                    next_sample[d] += SR.SAMPLE_INTERVAL_SEC
            # 현시 전환: 녹색 켜지는 방향의 빨간 구간 마감(평균) → 사이클/블록 처리
            if cur != last_phase:
                if cur in green_phase_dir:
                    d = green_phase_dir[cur]
                    if red_samples[d]:
                        snap[d].append(sum(red_samples[d]) / len(red_samples[d]))
                    red_elapsed[d] = 0.0
                    next_sample[d] = SR.SAMPLE_INTERVAL_SEC
                    red_samples[d] = []
                if cur == SR.FIRST_GREEN_PHASE:
                    cycles += 1
                    if cycles % CYCLES_PER_BLOCK == 0:
                        demand = {p: (sum(snap[p]) / len(snap[p]) if snap[p] else 0.0)
                                  for p in VEHICLE_PHASES}
                        green_times = allocate_from_demand(demand)
                        SR.install_program(TLS_ID, green_times, n, groups, rt)
                        snap = {p: [] for p in VEHICLE_PHASES}
            last_phase = cur

        step += 1

        # 워밍업 종료 시점의 누적 정지값을 기록 (이후 최종 지표에서 뺀다)
        if step == WARMUP_SEC:
            fixed_delay0, adap_delay0 = fixed_delay, adap_delay

        # 시계열 기록(매초)
        times.append(step)
        fixed_cum.append(fixed_delay)
        adap_cum.append(adap_delay)
        fixed_qs.append(fq)
        adap_qs.append(aq)

        # 라이브 점수판 (30초마다)
        if step % 30 == 0:
            print_scoreboard(step, fq, aq, fixed_delay, adap_delay)

        if not fixed_alive and not adap_alive:
            break

    # ---- 최종 요약 ----
    # (1) 정지 지표: 워밍업 구간을 제외한 순수 비교 구간의 누적 정지(대·초)
    f_stop = fixed_delay - fixed_delay0
    a_stop = adap_delay - adap_delay0
    imp_stop = (f_stop - a_stop) / f_stop * 100 if f_stop else 0

    # tripinfo는 연결이 닫혀야 완결되므로, 먼저 두 연결을 닫은 뒤 파싱한다.
    for lab in ("fixed", "adaptive"):
        try:
            traci.switch(lab)
            traci.close()
        except Exception:
            pass

    # (2) 지연 지표: SUMO 표준 timeLoss + 처리량(통과 대수), 워밍업 출발 차량 제외
    f_n, f_tl, _ = read_tripinfo(FIXED_TRIPINFO, WARMUP_SEC)
    a_n, a_tl, _ = read_tripinfo(ADAP_TRIPINFO, WARMUP_SEC)
    imp_tl = (f_tl - a_tl) / f_tl * 100 if f_tl else 0
    per = lambda v, n: v / n if n else 0.0

    # 그래프 저장 (콘솔과 동일 지표로 제목·주석·요약박스 구성, 워밍업 음영 표시)
    metrics = {"imp_tl": imp_tl, "imp_stop": imp_stop,
               "f_stop": f_stop, "a_stop": a_stop,
               "f_tl": f_tl, "a_tl": a_tl, "f_n": f_n, "a_n": a_n}
    save_chart(times, fixed_cum, adap_cum, fixed_qs, adap_qs,
               seg_windows, seg_summary, ntrips, WARMUP_SEC, metrics)

    print("=" * 64)
    print(f"[최종 결과]  동일 수요·신호만 다름  (워밍업 {WARMUP_SEC}s 제외)")
    print("-" * 64)
    print(f"  처리량(통과 대수)       고정식 {f_n:>10,} | 적응형 {a_n:>10,}")
    print(f"  총 지연 timeLoss(초)    고정식 {f_tl:>10,.0f} | 적응형 {a_tl:>10,.0f}")
    print(f"  1대당 평균 지연(초)     고정식 {per(f_tl, f_n):>10.1f} | 적응형 {per(a_tl, a_n):>10.1f}")
    print(f"  누적 정지(대·초)        고정식 {f_stop:>10,} | 적응형 {a_stop:>10,}")
    print("-" * 64)
    w_tl = "감소" if imp_tl >= 0 else "증가"
    w_st = "감소" if imp_stop >= 0 else "증가"
    print(f"  → 지연(timeLoss) {abs(imp_tl):.1f}% {w_tl},  정지 {abs(imp_stop):.1f}% {w_st}")
    if a_n < f_n * 0.98:
        print("  ⚠ 적응형 처리량이 더 적음 → 지연 감소가 '덜 처리'한 결과일 수 있으니 주의")
    print("=" * 64)


if __name__ == "__main__":
    run()
