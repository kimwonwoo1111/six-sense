"""
SUMO 연동 러너 — 실제 교차로에서 적응형 신호 제어 실행
------------------------------------------------------------
우리가 만든 배분 코어(allocate_from_demand)를 TraCI로 실제 SUMO 신호등에 연결한다.
핵심 알고리즘은 그대로 재사용하고, '입력'과 '출력(신호 시간)'만 SUMO와 주고받는다.
입력(수요)은 '차량 수'가 아니라 각 진입도로 150m 구간의 '점유율(%)'을 쓴다.
  - 각 방향이 빨간불이 된 뒤 10초마다 점유율을 재고, 다시 초록이 될 때까지의
    샘플 평균을 그 사이클의 값으로 잡는다. (대기 시작~출발 직전의 평균 정체)
  - 3사이클(1블록) 동안 방향별로 모은 값을 평균 내 다음 블록 배분에 쓴다.

[중요] SUMO(netconvert)가 교차로에 자동 생성하는 기본 신호 프로그램은
우리 5현시 모델(4방향 개별 + 보행자)과 다르다.
그래서 여기서는 setProgramLogic 으로 '우리만의 5현시 프로그램'을 새로 만들어 덮어쓴다.
  - 한 번에 한 방향(진입도로)만 녹색 → 상충(충돌) 없음.
  - 각 녹색 phase 뒤에 황색(YELLOW)을 자동으로 끼워 넣는다.
  - 마지막(보행자 현시)은 차량 전부 적색(보행자 전용 시간).
state 문자열(길이 = 제어 링크 수)은 손으로 쓰지 않고, 실제 제어 링크 구조에서 자동 생성한다.

[실행 준비]
  1) SUMO 설치 + SUMO_HOME 환경변수 등록 (traci 모듈을 SUMO_HOME/tools 에서 불러온다)
  2) 교차로 네트워크(intersection.net.xml)는 cross.nod.xml / cross.edg.xml 로 미리 생성해 둔 것을 쓴다.
     차량 수요(intersection.rou.xml)는 실행할 때 generate_vehicle_routes()가 자동 생성한다.
  3) python sumo_runner.py
"""

import os
import sys

# SUMO의 파이썬 도구(traci) 경로 등록
if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

import random  # noqa: E402
import traci  # noqa: E402  (SUMO_HOME 등록 후 import 해야 함)

# 우리가 만든 알고리즘 재사용 (시나리오/등급도 같은 파일에서 가져와 단일 소스 유지)
from signal_allocation import (  # noqa: E402
    allocate_from_demand,
    VEHICLE_PHASES,
    PED_PHASE,
    CYCLES_PER_BLOCK,
    DEFAULT_GREEN,
    SCENARIO_SCHEDULE,
    resolve_arrival_rates,
)

# ============================================================
# CONFIG
# ============================================================

SUMOCFG = "intersection.sumocfg"
USE_GUI = True     # True면 sumo-gui(화면), False면 sumo(콘솔)
GUI_DELAY_MS = 0   # sumo-gui 스텝 간 딜레이(ms). 0이면 딜레이 없이 최대 속도로 재생.
GUI_SETTINGS_FILE = "gui-settings.xml"  # 차량=박스, 보행자=원(크기3) 시각화 설정

# 실행할 때마다 signal_allocation.py 의 SCENARIO_SCHEDULE(시간대별 등급)를 읽어
# SUMO 차량 수요(intersection.rou.xml)를 자동 재생성한다.
# -> 알고리즘 파일의 SCENARIO_SCHEDULE만 바꾸면 SUMO에도 바로 반영됨.
REGENERATE_ROUTES = True
ROUTE_FILE = "intersection.rou.xml"
SIM_DURATION = 7200   # 수요 생성 시간(초). sumocfg의 end 와 맞춤. (2시간)
ROUTE_SEED = 42       # 등급 범위 안에서 실제 도착률을 뽑는 시드(재현용)

# 제어할 신호등 ID (깔끔한 4방향 교차로의 중앙 노드)
TLS_ID = "C"

# 우리 방향(1~4) -> 그 방향으로 '진입하는' 도로(edge) ID 목록
#   1=북(N) 2=동(E) 3=남(S) 4=서(W) 진입.  1·3 맞은편, 2·4 맞은편.
APPROACH_EDGES = {
    1: ["N_C"],    # 북쪽에서 진입
    2: ["E_C"],    # 동쪽에서 진입
    3: ["S_C"],    # 남쪽에서 진입
    4: ["W_C"],    # 서쪽에서 진입
}

YELLOW = 3        # 각 녹색 현시 뒤 황색 시간(초) [TODO: 실제 값]
PROGRAM_ID = "adaptive"   # 우리가 심는 신호 프로그램 이름

# ---- 밀도(점유율) 기반 수요 측정 설정 ----
# AI 비전 카메라가 감지하는 진입도로 구간 길이(m). 정지선에서 뒤로 이만큼을 본다.
DETECT_LENGTH = 150.0
# 각 방향이 '빨간불이 된 뒤' 몇 초마다 점유율을 샘플링할지.
# 그 방향이 다시 초록이 될 때까지 모은 샘플의 평균을 그 사이클의 값으로 쓴다.
SAMPLE_INTERVAL_SEC = 10.0

# 우회전 상시 통행(한국식): True면 우회전 차로는 신호와 무관하게 항상 통행 가능.
# SUMO에서 'g'(양보 녹색)로 두면 보행자·직진차량에는 양보하면서 우회전은 계속 흐른다.
RIGHT_TURN_ON_RED = True

# 우리 프로그램의 phase 배치(녹색+황색 교대):
#   0:방향1 녹색  1:방향1 황색  2:방향2 녹색  3:방향2 황색
#   4:방향3 녹색  5:방향3 황색  6:방향4 녹색  7:방향4 황색  8:보행자(전적색)
FIRST_GREEN_PHASE = 0     # 사이클 시작(방향1 녹색)의 phase 인덱스


# ============================================================
# 등급(SCENARIO) 기반 SUMO 차량 수요 자동 생성
# ============================================================

def generate_vehicle_routes(path=ROUTE_FILE, duration=SIM_DURATION, seed=ROUTE_SEED):
    """SCENARIO_SCHEDULE(시간대별 방향 등급) -> SUMO 차량 trip 파일 생성.

    - 전체 시간을 스케줄 구간 수만큼 균등 분할.
    - 각 구간에서 방향별 등급을 TRAFFIC_LEVELS로 환산(resolve_arrival_rates)해 도착률(대/초)을 정하고,
      그 구간의 시간창 동안 포아송(지수 간격)으로 진입도로마다 차량을 투입한다.
    - 목적지는 그 방향의 U턴을 제외한 나머지 3개 출구에서 랜덤 선택(직/좌/우 섞임).
    반환: (총 차량수, 구간별요약[{방향: veh/h}...], 구간시간창[(t0,t1)...])
    """
    dirs = [APPROACH_EDGES[d][0].split("_")[0] for d in VEHICLE_PHASES]
    nseg = len(SCENARIO_SCHEDULE)
    seg_len = duration / nseg

    trips = []
    seg_summary = []   # 구간별 방향 veh/h
    seg_windows = []   # 구간별 (시작초, 끝초)
    for si, seg in enumerate(SCENARIO_SCHEDULE):
        t0, t1 = si * seg_len, (si + 1) * seg_len
        seg_windows.append((t0, t1))
        rates = resolve_arrival_rates(seg, seed + si)   # {현시: 대/초}
        seg_summary.append({d: round(rates[d] * 3600) for d in VEHICLE_PHASES})
        for d in VEHICLE_PHASES:
            origin_edge = APPROACH_EDGES[d][0]
            o = origin_edge.split("_")[0]
            exits = ["C_" + x for x in dirs if x != o]   # U턴 제외한 3개 출구
            rate = rates[d]
            if rate <= 0:
                continue
            rng = random.Random(seed * 1000 + si * 10 + d)
            t = t0
            while True:
                t += rng.expovariate(rate)   # 다음 차량까지 간격(포아송)
                if t >= t1:
                    break
                trips.append((t, origin_edge, rng.choice(exits)))

    trips.sort(key=lambda x: x[0])          # SUMO는 depart 오름차순 요구
    with open(path, "w", encoding="utf-8") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write("<!-- signal_allocation.py 의 SCENARIO_SCHEDULE에서 자동 생성됨. "
                "(sumo_runner.generate_vehicle_routes) 직접 수정하지 말 것. -->\n")
        f.write("<routes>\n")
        f.write('    <vType id="car" vClass="passenger"/>\n')
        for i, (t, frm, to) in enumerate(trips):
            f.write(f'    <trip id="v{i}" type="car" depart="{t:.2f}" '
                    f'from="{frm}" to="{to}"/>\n')
        f.write("</routes>\n")

    return len(trips), seg_summary, seg_windows


# ============================================================
# 신호 프로그램 생성 (우리 5현시 -> SUMO state 문자열 자동 생성)
# ============================================================

def _edge_of(lane_id):
    """차선 ID('edge_0') -> edge ID('edge')."""
    return lane_id.rsplit("_", 1)[0]


def build_link_groups(tls_id):
    """신호등이 제어하는 링크를 진입 edge별 인덱스 목록으로 묶는다.

    반환: (전체 링크 수 n, {edge ID: [링크 인덱스...]})
    """
    links = traci.trafficlight.getControlledLinks(tls_id)
    n = len(links)
    groups = {}
    for i, l in enumerate(links):
        if l:  # l 예: [(fromLane, toLane, viaLane)]
            e = _edge_of(l[0][0])
            groups.setdefault(e, []).append(i)
    return n, groups


def _make_state(n, active_indices, ch):
    """길이 n 의 신호 state 문자열 생성. active_indices 위치만 ch, 나머지는 'r'."""
    s = ["r"] * n
    for i in active_indices:
        s[i] = ch
    return "".join(s)


def _direction_indices(groups, direction):
    """방향(1~4)에 해당하는 진입 edge들의 제어 링크 인덱스 모음."""
    idxs = []
    for e in APPROACH_EDGES[direction]:
        idxs.extend(groups.get(e, []))
    return idxs


def _ped_indices(n, groups):
    """보행자(횡단보도) 제어 링크 인덱스.
    = 차량 진입도로(APPROACH_EDGES)에 속하지 않는 나머지 제어 링크 전부.
    """
    veh = set()
    for d in VEHICLE_PHASES:
        veh.update(_direction_indices(groups, d))
    return [i for i in range(n) if i not in veh]


def build_right_turn_indices(tls_id):
    """우회전(dir='r') 연결에 해당하는 제어 링크 인덱스 집합을 찾는다.
    각 제어 링크의 (fromLane -> toLane)을 lane 연결 정보와 대조해 방향을 판정.
    """
    rt = set()
    links = traci.trafficlight.getControlledLinks(tls_id)
    for i, l in enumerate(links):
        if not l:
            continue
        frm, to = l[0][0], l[0][1]
        for ln in traci.lane.getLinks(frm, extended=True):
            # ln: (approachedLane, hasPrio, isOpen, hasFoe, approachedInternal,
            #      state, direction, length)
            if ln[0] == to and ln[6] == "r":
                rt.add(i)
                break
    return rt


def build_program(green_times, n, groups, rt_idxs=frozenset()):
    """우리 현시별 녹색 시간(green_times)으로 SUMO 신호 프로그램(Logic)을 만든다.

    각 차량 방향: (녹색 = 배분시간) + (황색 = YELLOW). 이때 횡단보도는 적색.
    보행자 현시: 모든 차량 적색 + 4개 횡단보도 전부 녹색(보행자 동시 횡단).
    우회전(rt_idxs): RIGHT_TURN_ON_RED=True면 모든 phase에서 최소 'g'(양보 녹색)로
                     둬서 신호와 무관하게 통행(보행자·직진차량엔 양보).
    """
    ped_idxs = _ped_indices(n, groups)

    def state(active_indices, ch):
        s = list(_make_state(n, active_indices, ch))
        if RIGHT_TURN_ON_RED:
            for i in rt_idxs:
                # 자기 현시 녹색('G')이면 우선권 유지, 그 외(적/황색)엔 'g'로 통행 허용
                if s[i] != "G":
                    s[i] = "g"
        return "".join(s)

    phases = []
    for d in VEHICLE_PHASES:            # 1,2,3,4
        idxs = _direction_indices(groups, d)
        # 차량 방향 녹색: 그 방향만 G, 횡단보도 포함 나머지는 r (+우회전 g)
        phases.append(traci.trafficlight.Phase(green_times[d], state(idxs, "G")))
        phases.append(traci.trafficlight.Phase(YELLOW, state(idxs, "y")))
    # 보행자 현시: 차량 전적색 + 횡단보도(ped_idxs) 녹색 (+우회전 g는 보행자에 양보)
    phases.append(traci.trafficlight.Phase(green_times[PED_PHASE], state(ped_idxs, "G")))
    return traci.trafficlight.Logic(PROGRAM_ID, 0, 0, phases)


def install_program(tls_id, green_times, n, groups, rt_idxs=frozenset()):
    """새 신호 프로그램을 만들어 신호등에 적용한다."""
    logic = build_program(green_times, n, groups, rt_idxs)
    traci.trafficlight.setProgramLogic(tls_id, logic)
    traci.trafficlight.setProgram(tls_id, PROGRAM_ID)


# ============================================================
# 150m 구간 점유율(밀도) 측정 (AI 비전 카메라의 대체 = SUMO에서 직접 읽음)
# ============================================================
#
# '차량 대수'가 아니라 '진입도로가 얼마나 꽉 차 있는지(정체 정도)'를 수요로 쓴다.
# 정지선(교차로쪽 끝)에서 뒤로 DETECT_LENGTH(150m) 안에 들어온 차량들의 길이 합을
# 그 구간 길이로 나눈 값(%) = 점유율. 값이 클수록 그 방향이 더 막혀 있다는 뜻.

_lane_len_cache = {}   # {lane_id: 길이(m)} 캐시


def _lane_length(lane_id):
    if lane_id not in _lane_len_cache:
        _lane_len_cache[lane_id] = traci.lane.getLength(lane_id)
    return _lane_len_cache[lane_id]


def occupancy_150m(edges):
    """진입 edge들의 '정지선 기준 150m 구간' 점유율(%)을 차로 평균으로 반환한다.

    - 각 차로에서 정지선(차로 끝)까지 거리 <= min(150m, 차로길이)인 차량만 포함.
    - 점유율 = (구간 안 차량 길이 합) / (구간 길이) * 100.  (차로별로 계산 후 평균)
    - 진입 edge가 여러 개면 소속 차로 전부를 한 묶음으로 평균.
    """
    occ_sum = 0.0
    lane_cnt = 0
    for e in edges:
        for i in range(traci.edge.getLaneNumber(e)):
            lane = f"{e}_{i}"
            L = _lane_length(lane)
            window = min(DETECT_LENGTH, L)   # 차로가 150m보다 짧으면 그 길이만큼
            occupied = 0.0
            for v in traci.lane.getLastStepVehicleIDs(lane):
                # getLanePosition = 차로 시작점 기준 위치 → 정지선까지 거리 = L - pos
                if L - traci.vehicle.getLanePosition(v) <= window:
                    occupied += traci.vehicle.getLength(v)
            occ_sum += occupied / window * 100.0
            lane_cnt += 1
    return occ_sum / lane_cnt if lane_cnt else 0.0


# ============================================================
# 메인 루프
# ============================================================

def run():
    # 0) SCENARIO_SCHEDULE(시간대별 등급) -> SUMO 차량 수요 자동 생성 (SUMO 실행 전에)
    if REGENERATE_ROUTES:
        ntrips, seg_summary, seg_windows = generate_vehicle_routes()
        print("[route 생성] 시간대별 방향 차량 수요(veh/h):")
        for si, (t0, t1) in enumerate(seg_windows):
            s = seg_summary[si]
            print(f"    구간{si+1} {int(t0//60)}~{int(t1//60)}분: "
                  f"북{s[1]:>5} 동{s[2]:>5} 남{s[3]:>5} 서{s[4]:>5}")
        print(f"    → 총 {ntrips:,} 대 생성 ({ROUTE_FILE})")

    sumo_bin = "sumo-gui" if USE_GUI else "sumo"
    cmd = [sumo_bin, "-c", SUMOCFG]
    if USE_GUI:
        # GUI를 열자마자 자동 시작 + 끝나면 자동 종료 + 스텝 딜레이 + 시각화 설정
        cmd += ["--start", "--quit-on-end", "--delay", str(GUI_DELAY_MS),
                "--gui-settings-file", GUI_SETTINGS_FILE]
    traci.start(cmd)

    tls_list = traci.trafficlight.getIDList()
    if TLS_ID not in tls_list:
        print("[신호등 ID 목록]", tls_list)
        print(f"\n[안내] CONFIG의 TLS_ID('{TLS_ID}')가 현재 네트워크에 없어요.")
        print("       위 목록에서 제어할 신호등 ID를 골라 CONFIG에 넣어주세요.")
        traci.close()
        return

    # 제어 링크 구조 파악 + 진입도로 확인
    n, groups = build_link_groups(TLS_ID)
    missing = [d for d in VEHICLE_PHASES
               if not any(e in groups for e in APPROACH_EDGES[d])]
    if missing:
        print(f"[안내] 다음 방향의 진입 edge를 신호 링크에서 못 찾음: {missing}")
        print("       이 신호등이 제어하는 진입 edge 목록:", list(groups.keys()))
        print("       CONFIG의 APPROACH_EDGES를 위 목록에 맞게 수정하세요.")
        traci.close()
        return

    # 우회전(상시 통행) 링크 파악
    rt_idxs = build_right_turn_indices(TLS_ID) if RIGHT_TURN_ON_RED else frozenset()

    # 최초 블록은 기준값으로 시작
    green_times = dict(DEFAULT_GREEN)
    install_program(TLS_ID, green_times, n, groups, rt_idxs)
    print(f"[시작] 신호 링크 {n}개, 진입도로 {list(groups.keys())}")
    print(f"[우회전 상시통행] {'ON' if RIGHT_TURN_ON_RED else 'OFF'} (링크 {sorted(rt_idxs)})")
    print(f"[블록 0] 기준 녹색초 적용: {green_times}")

    # ---- 점유율(밀도) 기반 수요 수집 준비 ----
    # 각 방향이 '빨간불'인 동안 SAMPLE_INTERVAL_SEC(10초)마다 점유율을 샘플링하고,
    # 그 방향이 다시 초록이 되는 순간 샘플 평균 = 그 사이클의 값(snap)으로 확정한다.
    step_len = traci.simulation.getDeltaT()          # 스텝당 시간(초)
    green_phase_dir = {2 * (d - 1): d for d in VEHICLE_PHASES}  # 녹색 phase -> 방향

    red_elapsed = {p: 0.0 for p in VEHICLE_PHASES}   # 방향별 현재 빨간 구간 경과(초)
    next_sample = {p: SAMPLE_INTERVAL_SEC for p in VEHICLE_PHASES}  # 다음 샘플 시각
    red_samples = {p: [] for p in VEHICLE_PHASES}    # 현재 빨간 구간의 점유율 샘플들
    snap = {p: [] for p in VEHICLE_PHASES}           # 이번 블록의 방향별 사이클 값(최대 5)
    cycles_done = 0
    last_phase = traci.trafficlight.getPhase(TLS_ID)

    # 시뮬레이션 종료까지 진행
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        cur_phase = traci.trafficlight.getPhase(TLS_ID)

        # 1) 각 방향: 자기 녹색이 아니면(=빨간/황색) 경과시간 누적 + 10초마다 점유율 샘플
        for d in VEHICLE_PHASES:
            if cur_phase == 2 * (d - 1):        # d가 지금 초록 → 샘플링 안 함
                continue
            red_elapsed[d] += step_len
            if red_elapsed[d] >= next_sample[d]:
                red_samples[d].append(occupancy_150m(APPROACH_EDGES[d]))
                next_sample[d] += SAMPLE_INTERVAL_SEC

        if cur_phase != last_phase:
            # 2) 어떤 방향의 녹색이 막 켜졌으면, 그 방향의 빨간 구간을 마감한다.
            #    빨간 동안 모은 샘플 평균 = 이번 사이클 값(snap). 이후 카운터 리셋.
            if cur_phase in green_phase_dir:
                d = green_phase_dir[cur_phase]
                if red_samples[d]:
                    snap[d].append(sum(red_samples[d]) / len(red_samples[d]))
                red_elapsed[d] = 0.0
                next_sample[d] = SAMPLE_INTERVAL_SEC
                red_samples[d] = []

            # 3) 첫 현시(방향1 녹색) 복귀 = 1사이클 완료. 3사이클마다 재배분.
            #    (위에서 방향1 값이 이미 snap에 반영된 뒤 블록을 마감 → 정확한 3개)
            if cur_phase == FIRST_GREEN_PHASE:
                cycles_done += 1
                if cycles_done % CYCLES_PER_BLOCK == 0:
                    demand = {p: (sum(snap[p]) / len(snap[p]) if snap[p] else 0.0)
                              for p in VEHICLE_PHASES}
                    green_times = allocate_from_demand(demand)   # ← 배분 코어 재사용
                    install_program(TLS_ID, green_times, n, groups, rt_idxs)
                    blk = cycles_done // CYCLES_PER_BLOCK
                    occ_str = {p: round(demand[p], 1) for p in VEHICLE_PHASES}
                    print(f"[블록 {blk}] 평균 점유율%(방1~4) {occ_str} "
                          f"-> 재배분 {green_times}")
                    snap = {p: [] for p in VEHICLE_PHASES}   # 다음 블록 초기화

        last_phase = cur_phase

    traci.close()
    print("시뮬레이션 종료.")


if __name__ == "__main__":
    run()
