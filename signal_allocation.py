"""
AI 비전 기반 교차로 적응형 신호 제어 알고리즘
------------------------------------------------
- 입력: 방향별 '수요값' (파이썬 시뮬레이션=차량 수 / SUMO 실차 연동=150m 구간 점유율%)
- 출력: 각 현시(phase)에 배분된 녹색 신호 시간(초)
- 동작: 3사이클(1블록) 동안 수집한 데이터로 다음 블록의 시간을 배분 (롤링 윈도우)
- 배분 코어 allocate_from_demand()는 수요의 '비율'만 쓰므로 입력 단위(대수/점유율)는 무관.

주석의 [TODO] 표시는 아직 확정되지 않은 임시값이므로 추후 조정한다.
"""

import random
import statistics
from dataclasses import dataclass, field

# ============================================================
# 1. 설정값 (TODO: 실제 값으로 조정)
# ============================================================

NUM_VEHICLE_PHASES = 4      # 차량 현시 수 (4방향)
CYCLES_PER_BLOCK = 3        # 한 블록 = 3 사이클

# 현시 ID: 1~4 = 차량(각 방향 직진+좌회전), 5 = 보행자
VEHICLE_PHASES = [1, 2, 3, 4]
PED_PHASE = 5
ALL_PHASES = VEHICLE_PHASES + [PED_PHASE]

# 현시별 기준(디폴트) 녹색 시간(초) [TODO: 실제 기준값으로]
# - 적응형: 첫 블록(재배분 전)과 '수요가 전혀 없을 때'의 차량 현시 시간으로 쓴다.
# - 고정식(비교 기준선): 이 값을 끝까지 그대로 쓴다. 이때 보행자 현시도 60초다.
#   (적응형은 첫 재배분부터 보행자 현시를 PED_FIXED_GREEN=30초로 운영 → README '한계' 참고)
DEFAULT_GREEN = {
    1: 60,   # 방향 1 직진+좌회전
    2: 60,   # 방향 2 직진+좌회전
    3: 60,   # 방향 3 직진+좌회전
    4: 60,   # 방향 4 직진+좌회전
    5: 60,   # 보행자 (모든 방향 동시 횡단)
}

MIN_GREEN = 40   # 현시별 최소 녹색 시간(초) [TODO]
MAX_GREEN = 80   # 현시별 최대 녹색 시간(초) [TODO]

# 차량 수요 가중치 (방향별로 다르게 두고 싶을 때 사용) [TODO]
VEHICLE_WEIGHT = 1.0   # 차량 1대의 가중치

# 보행자 현시는 '고정 시간'으로 둔다. (수요와 무관)
# 이유: 실제 보행 신호는 사람 수가 아니라 '횡단거리 ÷ 보행속도'로 결정된다.
#       예) 횡단거리 15m ÷ 보행속도 1.0m/s = 15초 + 여유시간 → 약 30초
#       사람이 5명이든 50명이든 한 번에 나란히 건너므로 시간은 거의 같다.
PED_FIXED_GREEN = 30   # 보행자 고정 녹색 시간(초) [TODO: 실제 횡단거리 기준]

# 한 사이클 총 녹색 시간(고정). 기본값들의 합으로 고정한다. [TODO: 고정 여부 결정]
# 이 중 보행자 몫(PED_FIXED_GREEN)은 고정이고, 나머지를 차량 4개 현시가 나눠 갖는다.
TOTAL_GREEN_BUDGET = sum(DEFAULT_GREEN.values())


# ============================================================
# 2. 입력 데이터 구조 (AI 비전 -> 알고리즘)
# ============================================================

@dataclass
class BlockCounts:
    """한 블록(3사이클) 동안 수집한 카운트 데이터.

    vehicle_counts: {현시ID(1~4): 차량 수 합계}
    ped_count:      보행자 수 합계
    """
    vehicle_counts: dict = field(default_factory=lambda: {p: 0 for p in VEHICLE_PHASES})
    ped_count: int = 0


# ============================================================
# 3. 핵심 배분 로직
# ============================================================

def compute_demand(counts: BlockCounts) -> dict:
    """각 '차량 현시'의 수요(demand)를 가중치를 반영해 계산한다.

    보행자 현시는 고정 시간이므로 수요 계산에서 제외한다.
    반환: {차량 현시ID(1~4): 가중 수요값}
    """
    demand = {}
    for p in VEHICLE_PHASES:
        demand[p] = counts.vehicle_counts.get(p, 0) * VEHICLE_WEIGHT
    return demand


def allocate_from_demand(demand: dict) -> dict:
    """방향별 '수요값(demand)'을 받아 현시별 녹색 시간을 배분한다. (배분 코어)

    demand 는 방향(1~4)마다 '클수록 녹색을 더 줘야 하는' 양수 값이면 무엇이든 된다.
    - 파이썬 큐 시뮬레이션: 차량 수(count) 기반  (compute_demand)
    - SUMO 실차 연동:      150m 구간 점유율(%) 기반  (sumo_runner)
    스케일은 무관하다(비율만 사용). 값이 0이면 그 방향은 최소 시간만 받는다.

    방식(초안):
      0) 보행자 현시는 고정 시간(PED_FIXED_GREEN)을 먼저 떼어 놓는다.
      1) 나머지 시간(vehicle_budget)을 차량 4개 현시가 나눠 갖는다.
         1-1) 각 차량 현시에 최소 시간(MIN_GREEN)을 먼저 보장한다.
         1-2) 남은 시간을 수요(demand)에 비례하여 배분한다.
         1-3) 각 현시가 MAX_GREEN을 넘지 않도록 제한한다.
    반환: {현시ID: 배분된 녹색 시간(초)}
    """
    allocation = {}

    # 0) 보행자 현시: 항상 고정 시간
    allocation[PED_PHASE] = PED_FIXED_GREEN

    # 차량 4개 현시가 나눠 가질 총 시간 (전체 - 보행자 고정분)
    vehicle_budget = TOTAL_GREEN_BUDGET - PED_FIXED_GREEN

    total_demand = sum(demand.get(p, 0) for p in VEHICLE_PHASES)

    # 수요가 전혀 없으면 차량 현시는 기준(디폴트) 시간으로 동작
    if total_demand <= 0:
        for p in VEHICLE_PHASES:
            allocation[p] = DEFAULT_GREEN[p]
        return allocation

    # 1-1) 차량 현시 최소 시간 우선 보장
    for p in VEHICLE_PHASES:
        allocation[p] = MIN_GREEN
    remaining = vehicle_budget - MIN_GREEN * len(VEHICLE_PHASES)

    # 1-2) 남은 시간을 수요 비례로 배분
    for p in VEHICLE_PHASES:
        extra = remaining * (demand.get(p, 0) / total_demand)
        allocation[p] += extra

    # 1-3) 최대 시간 제한 + 정수(초) 반올림 [TODO: 초과분 재분배 규칙]
    for p in VEHICLE_PHASES:
        allocation[p] = min(round(allocation[p]), MAX_GREEN)

    return allocation


def allocate_green_times(counts: BlockCounts) -> dict:
    """차량 수(count) 기반 배분 — 파이썬 큐 시뮬레이션용.

    수집된 BlockCounts -> 차량 수 수요(compute_demand) -> 배분 코어(allocate_from_demand).
    SUMO 러너는 점유율 수요를 직접 만들어 allocate_from_demand를 호출한다.
    """
    return allocate_from_demand(compute_demand(counts))


# ============================================================
# 4. 성능 평가 (기존 고정식 vs 적응형 비교 시뮬레이션)
# ============================================================
#
# 실제 신호기가 없으므로 간단한 대기열(queue) 시뮬레이션으로 비교한다.
#  - 동일한 도착 데이터를 두 방식(고정식 / 적응형)에 똑같이 적용한다. (공정 비교)
#  - 각 현시가 녹색일 때 해당 대기열이 통과율(SATURATION_FLOW)만큼 빠져나간다.
#  - 매 초 남아 있는 대기열의 총합을 누적한 값 = 총 대기시간(지연) 지표.

# 차량 통과율(대/초): 녹색일 때 초당 빠져나가는 차량 수.
# 1차로 포화교통류율 ≈ 1,800대/h ≈ 0.5대/초. 여기서는 3차로 기준 → 0.5 × 3 = 1.5대/초.
SATURATION_FLOW = 1.5   # 3차로 기준 차량 통과율 (대/초) [TODO]
PED_FLOW = 0.5          # 보행자 통과율 (명/초) [TODO]

# 통과율 테이블 (현시별)
FLOW_RATE = {p: SATURATION_FLOW for p in VEHICLE_PHASES}
FLOW_RATE[PED_PHASE] = PED_FLOW


# 교통 강도 변동 설정
# 실제 교통은 같은 '매우혼잡'이라도 시간대(사이클)마다 몰렸다 뜸했다 한다.
# 그래서 일정 시간 창(FLUCTUATION_WINDOW)마다 도착률에 랜덤 배수를 곱해 출렁이게 한다.
FLUCTUATION_WINDOW = 150      # 이 초 단위마다 교통 강도가 새로 정해짐 (대략 1사이클)
FLUCTUATION_RANGE = (0.3, 1.7)  # 강도 배수 범위 (평균 ≈ 1.0 -> 전체 평균은 등급 유지)


def generate_arrivals(arrival_rates: dict, duration: int, seed: int = 42) -> dict:
    """현시별 도착 데이터를 생성한다. (초 단위, 포아송 + 시간대별 강도 변동)

    arrival_rates: {현시ID: 초당 평균 도착 수} (등급이 정한 '기준' 도착률)
    duration:      전체 시뮬레이션 시간(초)

    동작:
      - 시간을 FLUCTUATION_WINDOW(약 1사이클)로 나누고, 창마다 강도 배수를 랜덤으로 뽑는다.
      - 그 창 동안은 (기준 도착률 × 강도 배수)를 평균으로 포아송 도착을 생성한다.
      - 배수 평균이 1.0이라 '전체 평균 통행량'은 등급대로 유지되지만,
        개별 사이클은 한산(배수 0.3)~폭주(배수 1.7)까지 다양해진다.
    반환: {현시ID: [초별 도착 수 리스트]}
    """
    rng = random.Random(seed)
    arrivals = {p: [] for p in ALL_PHASES}

    # 방향별로, 시간 창마다 강도 배수를 미리 뽑아둔다 (방향끼리 독립적으로 출렁임)
    num_windows = duration // FLUCTUATION_WINDOW + 1
    multipliers = {
        p: [rng.uniform(*FLUCTUATION_RANGE) for _ in range(num_windows)]
        for p in ALL_PHASES
    }

    for t in range(duration):
        w = t // FLUCTUATION_WINDOW          # 현재 시각이 속한 시간 창
        for p in ALL_PHASES:
            rate = arrival_rates.get(p, 0) * multipliers[p][w]
            # 포아송 근사: 평균 rate 인 도착 수 (0/1/2...) 생성
            arrivals[p].append(_poisson(rng, rate))
    return arrivals


def _poisson(rng: random.Random, lam: float) -> int:
    """평균 lam 인 포아송 난수 (Knuth 알고리즘)."""
    if lam <= 0:
        return 0
    import math
    L = math.exp(-lam)
    k = 0
    p = 1.0
    while True:
        k += 1
        p *= rng.random()
        if p <= L:
            return k - 1


def simulate(arrivals: dict, duration: int, adaptive: bool):
    """신호 시뮬레이션을 돌리고 (총 대기시간, 블록별 이력)을 반환한다.

    adaptive=False -> 기존 고정식 (DEFAULT_GREEN 그대로)
    adaptive=True  -> 적응형 (3사이클마다 재배분)

    반환:
      total_wait: 총 대기시간(지연) 지표
      history:    블록별 [{"block", "arrivals", "green"}] 리스트
                  - arrivals: 그 블록 동안 현시별 도착 수
                  - green:    그 블록에 '적용 중이던' 녹색 시간
    """
    queues = {p: 0.0 for p in ALL_PHASES}
    total_wait = 0.0

    green_times = dict(DEFAULT_GREEN)     # 현재 적용 중인 배분 시간
    phase_idx = 0
    cur_phase = ALL_PHASES[phase_idx]
    green_remaining = green_times[cur_phase]
    cycles_done = 0

    # 한 블록 동안 도착 수를 누적 (AI 비전이 세는 값에 해당)
    block = BlockCounts()
    history = []
    block_num = 0

    # 사이클 단위 도착 수 누적용
    cycle_arr = {p: 0 for p in ALL_PHASES}   # 현재 사이클 도착 수
    cycle_log = []                            # 현재 블록에 속한 사이클들의 기록

    for t in range(duration):
        # 1) 도착 반영 (대기열 + 블록/사이클 카운트 누적)
        for p in ALL_PHASES:
            arrived = arrivals[p][t]
            queues[p] += arrived
            cycle_arr[p] += arrived
            if p == PED_PHASE:
                block.ped_count += arrived
            else:
                block.vehicle_counts[p] += arrived

        # 2) 현재 녹색 현시 통과 처리
        discharge = min(queues[cur_phase], FLOW_RATE[cur_phase])
        queues[cur_phase] -= discharge

        # 3) 남은 대기열 누적 = 대기시간
        total_wait += sum(queues.values())

        # 4) 녹색 시간 경과 및 현시 전환
        green_remaining -= 1
        if green_remaining <= 0:
            phase_idx += 1
            if phase_idx >= len(ALL_PHASES):
                phase_idx = 0
                cycles_done += 1
                # (a) 사이클 완료: 이번 사이클 도착 수 기록 후 초기화
                cycle_log.append(dict(cycle_arr))
                cycle_arr = {p: 0 for p in ALL_PHASES}
                # (b) 3사이클(1블록) 완료 시점
                if cycles_done % CYCLES_PER_BLOCK == 0:
                    block_num += 1
                    # 이번 블록 이력 기록: 적용 중이던 green + 도착 수 + 사이클 내역
                    arr = dict(block.vehicle_counts)
                    arr[PED_PHASE] = block.ped_count
                    history.append({
                        "block": block_num,
                        "arrivals": arr,
                        "green": dict(green_times),
                        "cycles": cycle_log,
                    })
                    # 적응형이면 이 블록 데이터로 다음 블록 시간 재배분
                    if adaptive:
                        green_times = allocate_green_times(block)
                    block = BlockCounts()   # 다음 블록 수집 초기화
                    cycle_log = []          # 다음 블록 사이클 기록 초기화
            cur_phase = ALL_PHASES[phase_idx]
            green_remaining = green_times[cur_phase]

    return total_wait, history


# 교통 수준(등급)별 초당 평균 도착 범위 (대/초) — 3차로 기준
# 정확한 숫자 대신 '등급'으로 시나리오를 정하면, 이 범위 안에서 랜덤으로 실제 도착률이 정해진다.
TRAFFIC_LEVELS = {
    "매우한산": (0.025, 0.05),   # 약 90~180 대/h
    "한산":     (0.05, 0.10),   # 약 180~360 대/h
    "보통":     (0.12, 0.20),   # 약 430~720 대/h
    "혼잡":     (0.25, 0.35),   # 약 900~1,260 대/h
    "매우혼잡": (0.38, 0.48),   # 약 1,370~1,730 대/h
}

# ------------------------------------------------------------
# 시나리오: 각 방향(1~4)과 보행자의 혼잡 '등급'.
#   여기만 바꾸면 파이썬 시뮬레이션(evaluate)과 SUMO(sumo_runner) 양쪽에 함께 반영된다.
#   (sumo_runner가 이 SCENARIO를 읽어 방향별 SUMO 차량 수요를 생성)
# ------------------------------------------------------------
SCENARIO = {
    1: "혼잡",   # 방향1 (SUMO: 북 N_C)
    2: "보통",       # 방향2 (SUMO: 동 E_C)
    3: "혼잡",       # 방향3 (SUMO: 남 S_C)
    4: "매우혼잡",       # 방향4 (SUMO: 서 W_C)
    PED_PHASE: "보통",  # 보행자 (신호 시간엔 영향 없지만 대기열/보행량에 반영)
}

# ------------------------------------------------------------
# 시간대별 수요 시나리오(SUMO 비교용): '붐비는 방향'이 시간에 따라 바뀐다.
#   전체 시뮬레이션 시간을 아래 구간 수만큼 균등 분할해, 구간마다 등급을 바꿔 적용.
#   -> 고정식 신호는 한 세팅으로 굳어 있어 매 구간 어딘가 막히지만,
#      적응형은 3사이클마다 재배분해 바뀌는 수요를 따라간다. (적응형 강점 부각)
#   sumo_runner가 이 SCENARIO_SCHEDULE로 SUMO 차량 수요를 생성한다.
# ------------------------------------------------------------
SCENARIO_SCHEDULE = [
    # 구간1: 북(1)이 폭주
    {1: "혼잡", 2: "한산",   3: "매우한산", 4: "보통",   PED_PHASE: "보통"},
    # 구간2: 동(2)이 폭주
    {1: "보통",   2: "매우혼잡", 3: "한산",   4: "매우한산", PED_PHASE: "보통"},
    # 구간3: 남(3)이 폭주
    {1: "매우한산", 2: "보통",   3: "혼잡", 4: "한산",   PED_PHASE: "보통"},
    # 구간4: 서(4)가 폭주
    {1: "한산",   2: "매우한산", 3: "보통",   4: "혼잡", PED_PHASE: "보통"},
]


def resolve_arrival_rates(scenario: dict, seed: int) -> dict:
    """등급 시나리오 -> 실제 도착률(대/초)로 변환한다.

    각 현시의 등급에 해당하는 범위 안에서 랜덤으로 실제 도착률을 뽑는다.
    (같은 seed면 같은 결과 -> 재현 가능)
    """
    rng = random.Random(seed)
    rates = {}
    for p, level in scenario.items():
        lo, hi = TRAFFIC_LEVELS[level]
        rates[p] = rng.uniform(lo, hi)
    return rates


# 한 블록(3사이클)의 대략적인 길이(초). 등급 도착률 -> 블록당 차량 수 환산에 사용.
BLOCK_SECONDS = CYCLES_PER_BLOCK * TOTAL_GREEN_BUDGET


def generate_block_counts(scenario: dict, seed=None) -> BlockCounts:
    """등급 시나리오 -> 한 블록 동안의 랜덤 차량/보행자 수(BlockCounts)를 생성한다.

    각 방향의 '등급'에 해당하는 도착률 범위를 블록 길이(초)만큼 곱해
    '한 블록 동안 몇 대가 도착했는지'를 랜덤으로 만든다.
      예) 매우혼잡(0.38~0.48대/초) × 750초 ≈ 285~360대
    """
    rng = random.Random(seed)
    counts = BlockCounts()
    for p in VEHICLE_PHASES:
        lo, hi = TRAFFIC_LEVELS[scenario[p]]
        counts.vehicle_counts[p] = rng.randint(round(lo * BLOCK_SECONDS),
                                               round(hi * BLOCK_SECONDS))
    if PED_PHASE in scenario:
        lo, hi = TRAFFIC_LEVELS[scenario[PED_PHASE]]
        counts.ped_count = rng.randint(round(lo * BLOCK_SECONDS),
                                       round(hi * BLOCK_SECONDS))
    return counts


NUM_RUNS = 50        # 통계용 반복 실행 횟수
SIM_DURATION = 3600  # 1회 시뮬레이션 시간(초) = 1시간


def run_once(scenario: dict, seed: int, duration: int = SIM_DURATION):
    """시나리오+시드 하나로 1회 시뮬레이션을 돌리고 결과를 반환한다.

    반환: (개선율%, 고정식 대기, 적응형 대기, 실제 도착률, 적응형 블록 이력)
    """
    # 등급 -> 실제 랜덤 도착률 -> 초별 랜덤 도착 데이터
    rates = resolve_arrival_rates(scenario, seed)
    arrivals = generate_arrivals(rates, duration, seed=seed)

    # 동일한 도착 데이터로 두 방식 시뮬레이션 (공정 비교)
    adaptive_wait, history = simulate(arrivals, duration, adaptive=True)
    fixed_wait, _ = simulate(arrivals, duration, adaptive=False)

    improvement = (fixed_wait - adaptive_wait) / fixed_wait * 100 if fixed_wait else 0
    return improvement, fixed_wait, adaptive_wait, rates, history


def evaluate():
    """등급 시나리오로 50회 반복 실행 -> 개선율 분포를 출력한다."""

    # 방향별 혼잡 '등급' 시나리오 (모듈 상수 SCENARIO 사용 -> SUMO와 동일 소스)
    scenario = SCENARIO

    print("=== 초기 시나리오 (방향별 혼잡 등급) ===")
    for p in VEHICLE_PHASES:
        print(f"  방향{p}: {scenario[p]}")
    print(f"  보행자: {scenario[PED_PHASE]}")
    print()

    # 여러 시드로 반복 실행하여 개선율을 모은다
    improvements = []
    rep = None   # 대표 1회(상세 로그용)
    for seed in range(NUM_RUNS):
        imp, fixed_w, adap_w, rates, history = run_once(scenario, seed)
        improvements.append(imp)
        if rep is None:
            rep = (imp, fixed_w, adap_w, rates, history)

    # ---- 대표 1회 상세 로그 ----
    imp0, fixed0, adap0, rates0, history0 = rep
    print(f"=== 대표 실행 1회 상세 (seed=0) ===")
    for p in VEHICLE_PHASES:
        print(f"  방향{p}: 실제 도착률 {rates0[p]:.2f}대/초 ≈ {rates0[p]*3600:,.0f}대/h")
    print()
    print("  블록·사이클별 도착 차량수(방1/방2/방3/방4) — 같은 등급이어도 사이클마다 출렁임")
    for h in history0:
        g = h["green"]
        a = h["arrivals"]
        grn_str = f"{g[1]:>2}/{g[2]:>2}/{g[3]:>2}/{g[4]:>2}/{g[PED_PHASE]:>2}"
        print(f"  [블록 {h['block']}] 적용 녹색초(방1~4/보행): {grn_str}")
        for i, c in enumerate(h["cycles"], 1):
            print(f"       사이클 {i}: {c[1]:>3}/{c[2]:>3}/{c[3]:>3}/{c[4]:>3}")
        print(f"       └ 블록합계: {a[1]:>3}/{a[2]:>3}/{a[3]:>3}/{a[4]:>3}")
    print(f"  → 이 실행의 개선율: {imp0:.1f}% (고정식 {fixed0:,.0f} → 적응형 {adap0:,.0f})")
    print()

    # ---- 50회 반복 통계 ----
    mean_imp = statistics.mean(improvements)
    std_imp = statistics.stdev(improvements)
    print(f"=== {NUM_RUNS}회 반복 개선율 분포 (총 대기시간 기준) ===")
    print(f"  평균 개선율 : {mean_imp:.1f}%")
    print(f"  범위       : {min(improvements):.1f}% ~ {max(improvements):.1f}%")
    print(f"  표준편차    : ±{std_imp:.1f}%")


# ============================================================
# 5. 실행
# ============================================================

if __name__ == "__main__":
    # 기존 고정식 vs 적응형 성능 비교 시뮬레이션
    evaluate()
