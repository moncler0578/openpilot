#!/usr/bin/env python3
"""
APilotMan(APM) 값 덤프 스크립트  -  g_abcd 전용

APM 의 설정화면(ParamsActivity)은 SSH 로 이 명령을 실행한다.

    /data/openpilot/selfdrive/apilot.py ; cat /data/backup_params.json

이 스크립트는 selfdrive/apilot.json 에 정의된 파라미터들의 현재 값을 읽어
/data/backup_params.json 에 아래 형식으로 쓴다.

    [{"filename": "NavigationOnOpenpilot", "content": "1"}, ...]

지켜야 하는 제약
 1. stdout 으로 아무것도 출력하지 않는다.
    APM 은 이 명령의 표준출력 전체를 JSONArray 로 파싱한다.
    print 한 줄이라도 섞이면 설정화면이 통째로 비어버린다.
 2. content 는 반드시 정수 문자열이어야 한다.
    APM 은 Integer.parseInt() 를 쓰고, 하나라도 실패하면 예외가 나서
    그 뒤 파라미터가 전부 목록에서 빠진다.
    → 값이 비었거나 실수/문자열이면 apilot.json 의 default 로 대체한다.
 3. 예외를 밖으로 던지지 않는다. 실패해도 빈 배열을 쓰고 조용히 끝낸다.
"""
import io
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFS = os.path.join(HERE, "apilot.json")
PARAMS_DIR = os.environ.get("PARAMS_DIR", "/data/params/d")
OUT = os.environ.get("APM_BACKUP", "/data/backup_params.json")


def read_raw(name):
    try:
        with open(os.path.join(PARAMS_DIR, name), "rb") as f:
            return f.read().decode("utf-8", "ignore").strip()
    except Exception:
        return ""


def to_int(raw, default):
    """파라미터 값을 APM 이 읽을 수 있는 정수로 정규화한다."""
    if raw in ("", None):
        return default
    if raw in ("True", "true"):
        return 1
    if raw in ("False", "false"):
        return 0
    try:
        return int(raw)
    except ValueError:
        pass
    try:                       # "1.0" 같은 실수 표기 방어
        return int(round(float(raw)))
    except ValueError:
        return default


def main():
    out = []
    try:
        with io.open(DEFS, encoding="utf-8") as f:
            defs = json.load(f)["params"]
    except Exception:
        defs = []

    for p in defs:
        try:
            name = p["name"]
            val = to_int(read_raw(name), int(p.get("default", 0)))
            out.append({"filename": name, "content": str(val)})
        except Exception:
            continue

    try:
        tmp = OUT + ".tmp"
        with io.open(tmp, "w", encoding="ascii") as f:
            json.dump(out, f, ensure_ascii=True)
        os.rename(tmp, OUT)
        os.chmod(OUT, 0o666)
    except Exception:
        pass


if __name__ == "__main__":
    try:
        main()
    except Exception:
        pass
    sys.exit(0)
