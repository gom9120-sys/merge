# -*- coding: utf-8 -*-
"""merge.ino 현재 코드 기준 초음파 센싱 알고리즘 구조도 SVG 생성."""

W, H = 1290, 1700
FS = "NanumGothic, 'Malgun Gothic', 'Apple SD Gothic Neo', sans-serif"
FM = "NanumGothicCoding, 'D2Coding', Consolas, monospace"

C = {
    "ink": "#1c232e", "ink2": "#4d5766", "ink3": "#7b8697",
    "line": "#8a94a3", "frame": "#2f6f8f",
    "proc": "#eef4f8", "proc_s": "#2f6f8f",
    "dec": "#fdf4e0", "dec_s": "#b8862b",
    "idle": "#eaf3ed", "idle_s": "#3a7357",
    "hole": "#fbeae7", "hole_s": "#b1443a",
    "step": "#f2eef8", "step_s": "#6b52a3",
    "out": "#e9eff5", "out_s": "#48606f",
    "panel": "#fbfcfd", "panel_s": "#c8d0d9",
    "note": "#f5f7f4", "note_s": "#9aab9f",
}
o = []
def add(s): o.append(s)
def esc(t): return t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")

def txt(x, y, s, size=12, anchor="middle", fill=C["ink"], weight="400", mono=False):
    add(f'<text x="{x}" y="{y}" font-family="{FM if mono else FS}" font-size="{size}" '
        f'font-weight="{weight}" fill="{fill}" text-anchor="{anchor}">{esc(s)}</text>')

def lines(cx, y0, ls, size=12, lh=15, anchor="middle", fill=C["ink"], weight="400", mono=False):
    for i, s in enumerate(ls):
        txt(cx, y0 + i * lh, s, size, anchor, fill, weight, mono)

def box(x, y, w, h, ls, fill, stroke, size=12, lh=15, weight="400", head=None, rx=3):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" '
        f'stroke="{stroke}" stroke-width="1.4"/>')
    n = len(ls) + (1 if head else 0)
    total = n * lh
    y0 = y + h / 2 - total / 2 + lh - 4
    if head:
        txt(x + w / 2, y0, head, size + 0.5, "middle", C["ink"], "700")
        y0 += lh
    lines(x + w / 2, y0, ls, size, lh, "middle", C["ink"], weight)

def diamond(cx, cy, w, h, ls, size=11.5, lh=14):
    pts = f"{cx},{cy-h/2} {cx+w/2},{cy} {cx},{cy+h/2} {cx-w/2},{cy}"
    add(f'<polygon points="{pts}" fill="{C["dec"]}" stroke="{C["dec_s"]}" stroke-width="1.4"/>')
    total = len(ls) * lh
    lines(cx, cy - total / 2 + lh - 3, ls, size, lh)

def arr(x1, y1, x2, y2, label=None, lx=None, ly=None, dash=False, color=None):
    col = color or C["line"]
    d = ' stroke-dasharray="5 4"' if dash else ""
    add(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{col}" stroke-width="1.5"'
        f'{d} marker-end="url(#a)"/>')
    if label:
        txt(lx if lx is not None else (x1 + x2) / 2,
            ly if ly is not None else (y1 + y2) / 2 - 5, label, 11, "middle", C["ink2"])

def poly(pts, color=None, dash=False):
    col = color or C["line"]
    d = ' stroke-dasharray="5 4"' if dash else ""
    p = " ".join(f"{a},{b}" for a, b in pts)
    add(f'<polyline points="{p}" fill="none" stroke="{col}" stroke-width="1.5"{d} '
        f'marker-end="url(#a)"/>')

def frame(x, y, w, h, title, color):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="5" fill="none" '
        f'stroke="{color}" stroke-width="1.6"/>')
    add(f'<rect x="{x+14}" y="{y-11}" width="{len(title)*12+22}" height="22" rx="3" fill="#ffffff"/>')
    txt(x + 25, y + 5, title, 13.5, "start", color, "700")

def panel(x, y, w, title, items, lh=16, size=11.5):
    hgt = 34 + len(items) * lh + 10
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{hgt}" rx="4" fill="{C["panel"]}" '
        f'stroke="{C["panel_s"]}" stroke-width="1.3"/>')
    txt(x + 14, y + 23, title, 12.5, "start", C["ink"], "700")
    add(f'<line x1="{x+14}" y1="{y+30}" x2="{x+w-14}" y2="{y+30}" stroke="{C["panel_s"]}" stroke-width="1"/>')
    for i, (s, mono) in enumerate(items):
        txt(x + 14, y + 48 + i * lh, s, size, "start", C["ink2"], "400", mono)
    return y + hgt

# ---------- 문서 ----------
add(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}">')
add(f'<defs><marker id="a" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
    f'markerHeight="7" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="{C["line"]}"/></marker></defs>')
add(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')
txt(28, 42, "초음파 센싱 알고리즘 구조도", 22, "start", C["ink"], "700")
txt(28, 63, "merge.ino  ·  절대 기준거리 방식  ·  센서 3개 (좌 65° / 중앙 45° / 우 65°)",
    12.5, "start", C["ink3"])

MX, MW = 24, 950   # 본문 영역
PX, PW = 1000, 266 # 우측 패널

# ===== 1. 측정 루프 =====
frame(MX, 88, MW, 178, "1. 초음파 측정 루프 (라운드로빈)", C["frame"])
bw, bh, by = 138, 66, 122
xs = [MX + 20 + i * (bw + 15) for i in range(6)]
loop_boxes = [
    ("센서 i 선택", ["좌 / 중 / 우", "순환"]),
    ("초음파 트리거", ["10 µs HIGH"]),
    ("에코 수신", ["그 자리에서", "끝까지 측정"]),
    ("거리 계산", ["raw = 시간 ×", "0.343 / 2 (mm)"]),
    ("3점 중앙값", ["한 프레임짜리", "헛에코 제거"]),
    ("편차 계산", ["dev =", "(d − d0) × k"]),
]
for i, (hd, ls) in enumerate(loop_boxes):
    box(xs[i], by, bw, bh, ls, C["proc"], C["proc_s"], size=11, lh=14, head=hd)
    if i:
        arr(xs[i] - 15, by + bh / 2, xs[i] - 2, by + bh / 2)
poly([(xs[5] + bw / 2, by + bh), (xs[5] + bw / 2, by + bh + 22),
      (xs[0] + bw / 2, by + bh + 22), (xs[0] + bw / 2, by + bh + 2)])
txt(MX + MW / 2, by + bh + 40, "US_PING_INTERVAL_MS(15 ms) 간격으로 순환 — 센서당 45 ms",
    11.5, "middle", C["ink2"], mono=False)

# 판정으로 내려가는 화살표
poly([(MX + MW / 2, 266), (MX + MW / 2, 292)])

# ===== 2. 판정 흐름 =====
F2Y, F2H = 300, 1090
frame(MX, F2Y, MW, F2H, "2. 한 센서의 판정 흐름 (새 측정마다 한 프레임)", C["frame"])

cx1 = MX + 176           # 상단 체인 중심
box(cx1 - 105, F2Y + 32, 210, 44, ["(편차 dev, 시각 now)"], C["proc"], C["proc_s"],
    size=11, lh=14, head="새 측정 도착")
arr(cx1, F2Y + 76, cx1, F2Y + 96)

diamond(cx1, F2Y + 126, 210, 62, ["측정 실패인가?", "(에코 없음 · 범위 밖)"])
box(cx1 + 160, F2Y + 100, 236, 54,
    ["상태와 중앙값 버퍼는 그대로 두고", "놓친 시간만 다음 프레임에 넘긴다"],
    C["note"], C["note_s"], size=10.5, lh=14, head="판정 없음")
arr(cx1 + 105, F2Y + 126, cx1 + 158, F2Y + 126, "예", cx1 + 130, F2Y + 120)
arr(cx1, F2Y + 157, cx1, F2Y + 180, "아니오", cx1 - 26, F2Y + 174)

diamond(cx1, F2Y + 212, 210, 62, ["보정이 끝났는가?", "(유효 20프레임)"])
box(cx1 + 160, F2Y + 180, 236, 70,
    ["기준거리 d0와 잡음 σ 누적", "창(0.6~1.6×) 밖이면 버리고 재시작", "E,US_ODD — 그 센서는 판정 제외"],
    C["note"], C["note_s"], size=10.5, lh=14, head="보정 중")
arr(cx1 + 105, F2Y + 212, cx1 + 158, F2Y + 212, "아니오", cx1 + 132, F2Y + 206)
arr(cx1, F2Y + 243, cx1, F2Y + 266, "예", cx1 - 18, F2Y + 260)

diamond(cx1, F2Y + 298, 210, 62, ["측정 간격 > 0.25 s?", "(오래 끊겼는가)"])
box(cx1 + 160, F2Y + 274, 236, 48, ["상태 · 표 · 버퍼 초기화 후 진행"],
    C["note"], C["note_s"], size=10.5, lh=14, head="연속성 끊김")
arr(cx1 + 105, F2Y + 298, cx1 + 158, F2Y + 298, "예", cx1 + 130, F2Y + 292)
arr(cx1, F2Y + 329, cx1, F2Y + 352, "아니오", cx1 - 26, F2Y + 346)

box(cx1 - 130, F2Y + 354, 260, 40, ["현재 상태에 따라 분기 (센서마다 독립)"],
    C["out"], C["out_s"], size=11.5, lh=14)

# 세 상태 열
CY = F2Y + 424
colw = 296
cols = [MX + 20, MX + 20 + colw + 11, MX + 20 + 2 * (colw + 11)]
for i, (x, name, sub, fill, stroke) in enumerate([
        (cols[0], "IDLE", "평지 — 방향을 세는 중", C["idle"], C["idle_s"]),
        (cols[1], "HOLE", "홈을 지나는 중", C["hole"], C["hole_s"]),
        (cols[2], "STEP", "노면이 올라옴 — 턱인지 경사로인지", C["step"], C["step_s"])]):
    add(f'<rect x="{x}" y="{CY}" width="{colw}" height="{646}" rx="4" fill="{fill}" '
        f'stroke="{stroke}" stroke-width="1.3" opacity="0.55"/>')
    txt(x + colw / 2, CY + 26, name, 15, "middle", C["ink"], "700")
    txt(x + colw / 2, CY + 44, sub, 10.5, "middle", C["ink2"])
    poly([(x + colw / 2, F2Y + 394), (x + colw / 2, CY - 2)]) if i != 0 else None

poly([(cols[0] + colw / 2, F2Y + 394), (cols[0] + colw / 2, CY - 2)])

# 열 안쪽에 배선 레인을 두어 분기선과 라벨이 열 밖으로 나가지 않게 한다
BW = colw - 56          # 박스 너비 240
DW = BW - 60            # 다이아몬드 너비 180

def col_x(i):
    x = cols[i] + 28
    return x, x + BW / 2, x - 14, x + BW + 14   # 박스x, 중심, 좌레인, 우레인

# --- IDLE 열 ---
x, cxA, laneL, laneR = col_x(0)
box(x, CY + 58, BW, 76,
    ["dev > 홈시작 → 홈 표 +1", "−dev > 턱시작 → 턱 표 +1", "그 외 → 두 표 모두 0"],
    "#ffffff", C["idle_s"], size=11, lh=15, head="편차 방향으로 표를 센다")
arr(cxA, CY + 134, cxA, CY + 152)
diamond(cxA, CY + 182, DW, 56, ["홈 표 ≥ 2 프레임?"])
arr(cxA, CY + 210, cxA, CY + 228, "예", cxA - 16, CY + 224)
box(x, CY + 230, BW, 78,
    ["state = HOLE", "depth = dev,  width = 빔 띠", "→ 즉시 DANGER · HOLE"],
    C["hole"], C["hole_s"], size=11, lh=15, head="홈 진입")
txt(cxA + DW / 2 + 6, CY + 176, "아니오", 10.5, "start", C["ink2"])
poly([(cxA + DW / 2, CY + 182), (laneR, CY + 182), (laneR, CY + 340),
      (cxA + 4, CY + 340), (cxA + 4, CY + 356)])
diamond(cxA, CY + 386, DW, 56, ["턱 표 ≥ 2 프레임?"])
arr(cxA, CY + 414, cxA, CY + 432, "예", cxA - 16, CY + 428)
box(x, CY + 434, BW, 62, ["state = STEP", "peak = −dev"],
    C["step"], C["step_s"], size=11, lh=15, head="턱 후보 진입")
txt(cxA - DW / 2 - 6, CY + 380, "아니오", 10.5, "end", C["ink2"])
poly([(cxA - DW / 2, CY + 386), (laneL, CY + 386), (laneL, CY + 530),
      (cxA - 4, CY + 530), (cxA - 4, CY + 546)])
box(x, CY + 548, BW, 44, ["표만 갱신하고 끝 → SAFE"], "#ffffff", C["idle_s"], size=11, lh=14)

# --- HOLE 열 ---
x, cxB, laneL, laneR = col_x(1)
box(x, CY + 58, BW, 62, ["width += 속도 × Δt", "(앱에 보낼 참고값 · 판정엔 미사용)"],
    "#ffffff", C["hole_s"], size=10.5, lh=14, head="너비 적분")
arr(cxB, CY + 120, cxB, CY + 138)
box(x, CY + 140, BW, 44, ["depth = max(depth, dev)"], "#ffffff", C["hole_s"], size=11, lh=14)
arr(cxB, CY + 184, cxB, CY + 202)
diamond(cxB, CY + 232, DW, 60, ["노면이 돌아왔는가?", "dev < 복귀 임계"])
arr(cxB, CY + 262, cxB, CY + 282, "예", cxB - 16, CY + 278)
box(x, CY + 284, BW, 62, ["state = IDLE", "width · depth 지움"],
    C["idle"], C["idle_s"], size=11, lh=15, head="홈 종료")
txt(cxB + DW / 2 + 6, CY + 226, "아니오", 10.5, "start", C["ink2"])
poly([(cxB + DW / 2, CY + 232), (laneR, CY + 232), (laneR, CY + 380),
      (cxB + 4, CY + 380), (cxB + 4, CY + 396)])
box(x, CY + 398, BW, 48, ["계속 홈 안 → 이번 프레임 SAFE"], "#ffffff", C["hole_s"], size=11, lh=14)
box(x, CY + 470, BW, 84,
    ["빔 띠(51~86 mm)가 이미 safe_gap", "(42 mm)보다 길다. 그래서 보이기", "시작한 홈은 정의상 바퀴가 빠지는",
     "크기 — 진입 시점에 DANGER 확정."],
    C["note"], C["note_s"], size=10.5, lh=15, head="왜 진입 즉시 DANGER인가")

# --- STEP 열 ---
x, cxC, laneL, laneR = col_x(2)
box(x, CY + 58, BW, 62, ["peak = max(peak, −dev)", "(최대 상승량을 계속 추적)"],
    "#ffffff", C["step_s"], size=10.5, lh=14, head="최대 편차 추적")
arr(cxC, CY + 120, cxC, CY + 138)
diamond(cxC, CY + 172, DW, 64, ["peak ≥ 턱확정 임계?", "(중앙 34 mm)"])
arr(cxC, CY + 204, cxC, CY + 224, "예", cxC - 16, CY + 220)
box(x, CY + 226, BW, 78, ["턱 높이 = peak", "state = IDLE", "→ DANGER · STEP"],
    C["hole"], C["hole_s"], size=11, lh=15, head="턱 확정")
txt(cxC + DW / 2 + 6, CY + 166, "아니오", 10.5, "start", C["ink2"])
poly([(cxC + DW / 2, CY + 172), (laneR, CY + 172), (laneR, CY + 336),
      (cxC + 4, CY + 336), (cxC + 4, CY + 352)])
diamond(cxC, CY + 386, DW, 60, ["노면이 돌아왔는가?", "−dev < 복귀 임계"])
arr(cxC, CY + 416, cxC, CY + 436, "예", cxC - 16, CY + 432)
box(x, CY + 438, BW, 66, ["턱이라 하기엔 너무 작았다", "state = IDLE → 경보 없음"],
    C["idle"], C["idle_s"], size=11, lh=15, head="경사로 · 잔요철")
txt(cxC - DW / 2 - 6, CY + 380, "아니오", 10.5, "end", C["ink2"])
poly([(cxC - DW / 2, CY + 386), (laneL, CY + 386), (laneL, CY + 528),
      (cxC - 4, CY + 528), (cxC - 4, CY + 544)])
box(x, CY + 546, BW, 46, ["계속 추적 → 이번 프레임 SAFE"], "#ffffff", C["step_s"], size=11, lh=14)

# ===== 3. 결과 =====
F3Y = F2Y + F2H + 26
frame(MX, F3Y, MW, 150, "3. 결과 처리와 출력", C["frame"])
bw3, bh3 = 214, 78
x3 = [MX + 22, MX + 22 + 232, MX + 22 + 464, MX + 22 + 696]
box(x3[0], F3Y + 36, bw3, bh3, ["1.5초 유지", "같은 등급이면 원인을 뒤집지 않음"],
    C["out"], C["out_s"], size=11, lh=15, head="위험 래치")
arr(x3[0] + bw3, F3Y + 75, x3[1] - 2, F3Y + 75)
box(x3[1], F3Y + 36, bw3, bh3, ["세 센서 중 가장 높은 등급과", "그 원인을 앱 값으로 삼는다"],
    C["out"], C["out_s"], size=11, lh=15, head="종합")
arr(x3[1] + bw3, F3Y + 75, x3[2] - 2, F3Y + 75)
box(x3[2], F3Y + 36, bw3, bh3, ["H,risk,hazard,sensor,", "dist,width,depth", "— 바뀌는 순간 즉시"],
    C["proc"], C["proc_s"], size=10.5, lh=14, head="경보 줄")
arr(x3[2] + bw3, F3Y + 75, x3[3] - 2, F3Y + 75)
box(x3[3], F3Y + 36, bw3, bh3, ["T,3,… ,risk,hazard", "— 500 ms 주기 상태"],
    C["proc"], C["proc_s"], size=10.5, lh=14, head="텔레메트리")

# ===== 우측 패널 =====
py = 88
py = panel(PX, py, PW, "설치 파라미터 (센서별)", [
    ("US_MOUNT_HEIGHT_M[i]  = 0.16 m", True),
    ("US_TILT_DEG[i] = 65 / 45 / 65", True),
    ("US_BEAM_HALF_ANGLE_DEG = 7.5", True),
    ("", False),
    ("좌·우 65°  빔 띠 51 mm", False),
    ("           전방 주시 75 mm", False),
    ("           기준거리 168 mm", False),
    ("중앙 45°   빔 띠 86 mm", False),
    ("           전방 주시 160 mm", False),
    ("           기준거리 202 mm", False),
]) + 16
py = panel(PX, py, PW, "임계값은 어디서 오는가", [
    ("부팅 보정의 잡음 σ 배수", False),
    ("  턱시작 4σ · 턱확정 8σ · 홈시작 4σ", False),
    ("", False),
    ("여기에 기하 하한 두 개를 씌운다", False),
    ("① 홈시작 ≥ 점프 상한 × 1.15", False),
    ("   h(sin(θ+α)/sin(θ−α) − 1)", True),
    ("   → 뜨면 빔 띠보다 넓은 홈", False),
    ("② 턱확정 ≥ 전방주시 × tan10° × 1.2", False),
    ("   → 경사로를 턱으로 보지 않음", False),
    ("", False),
    ("실제 값 (중앙 45°)", False),
    ("  턱시작 10 · 턱확정 34 · 홈시작 56 mm", False),
]) + 16
py = panel(PX, py, PW, "상태 · 원인 코드", [
    ("risk    0 SAFE · 1 CAUTION · 2 DANGER", True),
    ("hazard  0 NONE · 1 STEP · 2 HOLE", True),
    ("sensor  0 좌 · 1 중 · 2 우 · 255 없음", True),
    ("", False),
    ("원본 detect.py의 RAMP / WIDE_HOLE /", False),
    ("STAIR / EXIT_STEP 은 없어졌다.", False),
    ("턱과 홈 둘로 합쳤다.", False),
]) + 16
py = panel(PX, py, PW, "판정 원리 요약", [
    ("· 프레임 간 차이가 아니라 부팅 때", False),
    ("  실측한 평지 기준거리와 비교한다", False),
    ("· 편차의 부호가 종류를 정한다", False),
    ("  길면 홈 · 짧으면 턱", False),
    ("· 홈은 보이면 곧 위험 (빔 띠 > safe_gap)", False),
    ("· 턱만 크기를 따져 경사로와 가른다", False),
    ("· 속도 추정은 판정에 쓰이지 않는다", False),
]) + 16
panel(PX, py, PW, "색상", [
    ("초록  IDLE · 정상 종료", False),
    ("빨강  DANGER 로 나가는 경로", False),
    ("보라  STEP (턱 판별 중)", False),
    ("노랑  분기 조건", False),
    ("회색  판정하지 않고 빠지는 길", False),
])

# 각주
fy = H - 62
for i, s in enumerate([
    "※ 모든 임계값은 '노면 높이 변화(m)' 기준이다. 측정 거리 편차에 sin(유효 각도)를 곱해 환산한다.",
    "※ 각 센서는 독립 상태(state, peak, width, depth, 표, 임계값, 기준거리)를 가진다.",
    "※ 한 센서의 새 측정이 끝날 때마다 위 흐름을 한 프레임 진행한다 — 센서당 약 45 ms.",
]):
    txt(W / 2, fy + i * 17, s, 11, "middle", C["ink3"])

add("</svg>")
open("docs/algorithm_flow.svg", "w", encoding="utf-8").write("\n".join(o))
print("SVG 생성 완료")
