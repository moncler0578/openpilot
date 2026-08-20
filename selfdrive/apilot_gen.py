#!/usr/bin/env python3
"""
APilotMan(APM) 파라미터 정의 생성기  -  g_abcd 전용

selfdrive/ui/qt/offroad/settings.cc 를 파싱해서
selfdrive/apilot.json (APM 이 읽는 파라미터 목록) 을 다시 만든다.

사용법 (EON 또는 PC):
    python3 selfdrive/apilot_gen.py

settings.cc 에 ParamValueControlF / ParamControl 을 추가하면
이 스크립트만 다시 돌리면 APM 목록에 자동으로 반영된다.

주의
 - apilot.json 은 반드시 ASCII(\\uXXXX 이스케이프)로 쓴다.
   APM 의 SshSession.exec() 가 응답을 EUC-KR 로 디코딩하기 때문에
   UTF-8 한글을 그대로 쓰면 앱에서 깨져 보인다.
 - APM 은 값을 Integer.parseInt() 로 읽는다. 실수(float)로 저장되는
   파라미터는 BLOCKLIST 에 넣어 제외한다.
"""
import io
import json
import os
import re
import sys
from collections import OrderedDict

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SETTINGS = os.path.join(ROOT, "selfdrive/ui/qt/offroad/settings.cc")
MANAGER = os.path.join(ROOT, "selfdrive/manager/manager.py")
OUT = os.path.join(ROOT, "selfdrive/apilot.json")

VERSION = "g_abcd"

# 패널 → APM 그룹 버튼 이름 (표시 순서대로)
GROUPS = OrderedDict([
    ("TogglesPanel",      "기본"),
    ("CruisePanel",       "크루즈"),
    ("LongitudinalPanel", "종방향"),
    ("VIPPanel",          "횡방향"),
    ("CommunityPanel",    "커브내비"),
    ("__S9__",            "S9HUD"),
    ("UISettingsPanel",   "화면"),
])

# 실수로 저장되어 APM 이 파싱하지 못하거나, 정수로 덮어쓰면 위험한 파라미터
BLOCKLIST = {
    "OffsetTotal",        # "0.050000" 형태의 미터 단위 실수. 정수로 쓰면 5m 가 된다
}

# settings.cc 의 전용 위젯으로만 편집되는 파라미터 (수동 정의)
EXTRA = [
    dict(group="VIPPanel", name="LateralTorqueAccelFactor",
         title="LAT ACCEL FACTOR (x1000)",
         descr="횡가속도 대비 토크 계수 x1000. 2700 = 2.70.\n"
               "값 증가(+): 조향이 약해짐 / 값 감소(-): 조향이 강해짐. 범위 500~4500.",
         min=500, max=4500, step=50, default=2500),
    dict(group="VIPPanel", name="LateralTorqueFriction",
         title="FRICTION (x1000)",
         descr="정지마찰 보상값 x1000. 80 = 0.080.\n"
               "값 증가(+): 중앙 부근 반응이 빨라짐 / 너무 크면 직진에서 흔들림. 범위 0~200.",
         min=0, max=200, step=5, default=80, force=True),   # 0.080 지정
    dict(group="VIPPanel", name="AdjustLaneOffset",
         title="차선 여유공간 자동보정 (cm)",
         descr="좌우 여유공간이 비대칭일 때 여유 있는 쪽으로 경로 이동. 0=끔, 0~40cm.",
         min=0, max=40, step=5, default=0),
    dict(group="VIPPanel", name="DynamicLaneProfile",
         title="동적 차선모드",
         descr="0: 차선 사용 / 1: 차선 미사용(E2E) / 2: 자동 전환.",
         min=0, max=2, step=1, default=0),
    dict(group="VIPPanel", name="AutoLaneChangeTimer",
         title="자동 차선변경 대기시간",
         descr="0: 즉시 / 1: 0.1s / 2: 0.5s / 3: 1.0s / 4: 1.5s / 5: 2.0s.",
         min=0, max=5, step=1, default=0),
    dict(group="LongitudinalPanel", name="MyDrivingMode",
         title="현재 주행모드",
         descr="1: SAFE / 2: ECO / 3: NORMAL / 4: FAST. 주행 중 즉시 반영됩니다.",
         min=1, max=4, step=1, default=3),
]

# ── C++ 파서 ─────────────────────────────────────────────────────────────
ESC = {"n": "\n", "t": "\t", '"': '"', "\\": "\\", "'": "'"}


def split_args(s):
    args, depth, cur, i, instr = [], 0, "", 0, False
    while i < len(s):
        c = s[i]
        if instr:
            cur += c
            if c == "\\":
                cur += s[i + 1]
                i += 2
                continue
            if c == '"':
                instr = False
            i += 1
            continue
        if c == '"':
            instr = True
            cur += c
            i += 1
            continue
        if c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
        if c == "," and depth == 0:
            args.append(cur.strip())
            cur = ""
            i += 1
            continue
        cur += c
        i += 1
    if cur.strip():
        args.append(cur.strip())
    return args


def as_str(tok):
    out, i = [], 0
    while i < len(tok):
        if tok[i] == '"':
            i += 1
            while i < len(tok) and tok[i] != '"':
                if tok[i] == "\\":
                    out.append(ESC.get(tok[i + 1], tok[i + 1]))
                    i += 2
                else:
                    out.append(tok[i])
                    i += 1
            i += 1
        else:
            i += 1
    return "".join(out)


def as_int(tok):
    tok = tok.strip()
    return int(tok) if re.fullmatch(r"-?\d+", tok) else None


def find_calls(src, name):
    out = []
    for m in re.finditer(r"\bnew\s+" + name + r"\s*\(", src):
        i, depth, instr, start = m.end(), 1, False, m.end()
        while i < len(src) and depth:
            c = src[i]
            if instr:
                if c == "\\":
                    i += 2
                    continue
                if c == '"':
                    instr = False
            elif c == '"':
                instr = True
            elif c == "(":
                depth += 1
            elif c == ")":
                depth -= 1
            i += 1
        out.append((m.start(), split_args(src[start:i - 1])))
    return out


def build():
    src = io.open(SETTINGS, encoding="utf-8").read()

    sec = [(m.start(), m.group(1))
           for m in re.finditer(r"^([A-Za-z0-9_]+)::([A-Za-z0-9_]+)\(", src, re.M)]

    def section_of(pos):
        name = "?"
        for start, n in sec:
            if start <= pos:
                name = n
            else:
                break
        return name

    # manager.py 기본값
    defaults = {}
    mgr = io.open(MANAGER, encoding="utf-8").read()
    blk = re.search(r"default_params[^=]*=\s*\[(.*?)\n  \]", mgr, re.S)
    if blk:
        for k, v in re.findall(r'\("([A-Za-z0-9_]+)",\s*b?"([^"]*)"\)', blk.group(1)):
            defaults[k] = v

    params = OrderedDict()

    def add(group, name, title, descr, vmin, vmax, step, default, force=False):
        if not name or name in params or name in BLOCKLIST:
            return
        # 기본값 우선순위: force 지정 > manager.py default_params > settings.cc vdefault
        # (manager.py 가 신규 기기에 실제로 써넣는 값이므로 그쪽을 정답으로 본다)
        if not force:
            md = defaults.get(name)
            if md is not None and re.fullmatch(r"-?\d+", md or ""):
                default = int(md)
        if None in (vmin, vmax, default):
            print("!! 범위 파싱 실패, 건너뜀:", name, file=sys.stderr)
            return
        if name.startswith("EonClusterHud"):
            group = "__S9__"
        params[name] = dict(
            group=GROUPS.get(group, group), name=name,
            title=title, descr=descr.strip(),
            min=vmin, max=vmax, default=default, unit=max(1, step or 1))

    # 1) ParamValueControlF(param, title, desc, icon, min, max, step, decimals, default)
    for pos, a in find_calls(src, "ParamValueControlF"):
        if len(a) < 9:
            continue
        add(section_of(pos), as_str(a[0]), as_str(a[1]), as_str(a[2]),
            as_int(a[4]), as_int(a[5]), as_int(a[6]), as_int(a[8]))

    # 2) std::array 테이블 + 루프 (CruiseSpeed / CruiseMaxVals / TFollowGap)
    for tbl in re.finditer(r"std::array<std::tuple<const char\*, const char\*, int>,\s*\d+>\s*"
                           r"(\w+)\s*=\s*\{\{(.*?)\}\};(.*?)\}\n", src, re.S):
        rows = re.findall(r'\{\s*"([A-Za-z0-9_]+)"\s*,\s*"((?:[^"\\]|\\.)*)"\s*,\s*(-?\d+)\s*\}',
                          tbl.group(2))
        body = tbl.group(3)
        call = find_calls(body, "ParamValueControlF")
        if not rows or not call:
            continue
        a = call[0][1]
        if len(a) < 9:
            continue
        grp = section_of(tbl.start())
        for key, title, dv in rows:
            add(grp, key, as_str('"%s"' % title), as_str(a[2]),
                as_int(a[4]), as_int(a[5]), as_int(a[6]), int(dv))

    # 3) ParamControl(param, title, desc, icon) → 0/1 토글
    for pos, a in find_calls(src, "ParamControl"):
        if len(a) < 3:
            continue
        name = as_str(a[0])
        d = defaults.get(name, "0")
        add(section_of(pos), name, as_str(a[1]), as_str(a[2]),
            0, 1, 1, 1 if d in ("1", "True") else 0)

    # 4) TogglesPanel toggle_defs
    tb = re.search(r"toggle_defs\{(.*?)\n  \};", src, re.S)
    if tb:
        for t in re.finditer(r'\{\s*("(?:[^"\\]|\\.)*")\s*,\s*("(?:[^"\\]|\\.)*")\s*,'
                             r'\s*((?:"(?:[^"\\]|\\.)*"\s*)+),', tb.group(1), re.S):
            name = as_str(t.group(1))
            d = defaults.get(name, "0")
            add("TogglesPanel", name, as_str(t.group(2)), as_str(t.group(3)),
                0, 1, 1, 1 if d in ("1", "True") else 0)

    # 5) 전용 위젯 파라미터
    for e in EXTRA:
        add(e["group"], e["name"], e["title"], e["descr"],
            e["min"], e["max"], e["step"], int(e["default"]),
            force=bool(e.get("force")))

    # 그룹 버튼 순서 = 최초 등장 순서 → GROUPS 순서로 정렬
    order = list(GROUPS.values())
    items = sorted(params.values(), key=lambda p: (order.index(p["group"])
                                                   if p["group"] in order else 99))

    doc = {"apilot": VERSION, "params": items}
    with io.open(OUT, "w", encoding="ascii") as f:
        json.dump(doc, f, ensure_ascii=True, indent=1)
        f.write("\n")
    return items


if __name__ == "__main__":
    items = build()
    from collections import Counter
    print("apilot.json 생성:", OUT, "-", len(items), "개")
    for g, c in Counter(p["group"] for p in items).items():
        print("  %-10s %d" % (g, c))
