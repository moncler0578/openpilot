# APM(APilotMan 0.983) 연동 파일 — g_abcd

APK 는 수정하지 않는다. APM 은 EON 의 아래 두 파일만 읽는다.

| APM 동작 | 실행하는 명령 |
|---|---|
| 파라미터 목록 | `cat /data/openpilot/selfdrive/apilot.json` |
| 현재 값 | `/data/openpilot/selfdrive/apilot.py ; cat /data/backup_params.json` |
| 값 변경 | `echo -n <값> > /data/params/d/<이름>` |

## 포함 파일
- `selfdrive/apilot.json` — 파라미터 정의 128개 (g_abcd settings.cc 기준, 순수 ASCII)
- `selfdrive/apilot.py` — 값 덤프 (실행권한 필요)
- `selfdrive/apilot_gen.py` — settings.cc → apilot.json 재생성기

## 적용 (PC, Git Bash)
    cd ~/OneDrive/Desktop/<작업폴더>/openpilot
    unzip -o ~/Downloads/apm_g_abcd.zip
    git add selfdrive/apilot.json selfdrive/apilot.py selfdrive/apilot_gen.py
    git update-index --chmod=+x selfdrive/apilot.py
    git commit -m "apm: apilot.json/apilot.py for g_abcd"
    git push

`git update-index --chmod=+x` 를 빼먹으면 EON 에서 apilot.py 가 실행되지 않아
APM 설정화면이 빈 채로 뜬다.

## 적용 (EON)
    cd /data/openpilot && git pull
    ls -l selfdrive/apilot.py        # -rwxr-xr-x 확인, 아니면 chmod +x
    ./selfdrive/apilot.py && head -c 200 /data/backup_params.json

## 설정 항목을 추가했을 때
settings.cc 에 ParamValueControlF / ParamControl 을 추가한 뒤 EON 에서

    python3 selfdrive/apilot_gen.py

를 돌리면 apilot.json 이 갱신된다. (PC 에는 python 이 없으므로 EON 에서 실행하고
결과를 커밋하거나, 다음번에 다시 만들어 달라고 요청할 것)

## 주의사항 (APK 디컴파일로 확인한 제약)
1. `SshSession.exec()` 가 응답을 **EUC-KR** 로 디코딩한다.
   → apilot.json 은 반드시 ASCII(`\uXXXX`)로 쓸 것. UTF-8 한글은 앱에서 깨진다.
2. 값은 `Integer.parseInt()` 로 읽고, 하나라도 실패하면 그 뒤 파라미터가 전부
   목록에서 사라진다. → apilot.py 가 빈값/실수/문자열을 default 로 대체한다.
3. `apilot.py` 는 stdout 에 아무것도 출력하면 안 된다(`;cat` 결과와 섞여 파싱 실패).
4. `OffsetTotal` 은 제외했다. "0.050000" 실수로 저장되므로 APM 이 파싱하지 못하고,
   정수로 덮어쓰면 0.05m 가 5m 로 들어간다. EON UI 에서만 조정할 것.
5. 앱의 백업 메뉴는 **현재 선택된 그룹만** 저장한다(전체 백업 아님).

## 그룹 구성
기본 8 / 크루즈 25 / 종방향 35 / 횡방향 24 / 커브내비 13 / S9HUD 17 / 화면 6

## 원본(ajouatom/apilot c2-master)과의 차이
- 원본 `selfdrive/apilot.json` 은 **EUC-KR 인코딩** (109개, "apilot":"20220111").
  → 앱이 EUC-KR 로 디코딩한다는 분석이 실물로 확인됨. 이쪽은 ASCII 이스케이프라 동일하게 안전.
- 원본에는 `egroup/etitle/edescr` 영문 필드가 있으나 **앱은 읽지 않는다**(group/title/descr 만 사용).
- 원본 `apilot.py` 는 /data/params/d 의 100바이트 미만 파일을 전부 raw 로 덤프한다.
  이쪽은 apilot.json 에 있는 것만 정수로 정규화해서 덤프 → parseInt 예외로 목록이
  통째로 사라지는 사고를 막는다.
- 파라미터 공통 50 / 원본에만 59 / g_abcd 에만 78. 공통 중 범위·기본값이 다른 것 42건은
  전부 g_abcd settings.cc 기준으로 맞춰져 있다(포크가 갈라진 결과, 정상).

## LateralTorqueFriction
`0.080` 지정에 따라 default 를 **80** 으로 넣었다(범위 0~200, x1000).
`LateralTorqueAccelFactor` 는 manager.py 값 2500(=2.50) 유지.
실제 적용은 `LateralTorqueCustom` 이 1 일 때만 되고(latcontrol_torque.py:140),
default 는 APM 표시에만 쓰이므로 EON 실동작값과는 별개다.

## 기본값 우선순위 (2026-08-19 결정: 매니저 값 기준)
apilot_gen.py 는 `manager.py` 의 `default_params` 값을 정답으로 쓴다.
`settings.cc` 의 vdefault 는 manager 에 키가 없을 때만 쓴다.
신규 기기가 실제로 받는 값이 manager.py 쪽이기 때문.

그 결과 아래 6건이 settings.cc 표시값이 아니라 manager 값으로 들어갔다.

| 파라미터 | manager.py (채택) | settings.cc 표시값 |
|---|---|---|
| LongitudinalActuatorDelayLowerBound | 50 | 0 |
| LongitudinalActuatorDelayUpperBound | 50 | 0 |
| SteerActuatorDelay | 50 | 10 |
| LateralTorqueKpV | 70 | 10 |
| LateralTorqueKiV | 20 | 10 |
| LateralTorqueKf | 85 | 100 |

예외 1건: `LateralTorqueFriction` 은 지정에 따라 **80**(0.080) — manager 값 10 을 덮어썼다
(apilot_gen.py EXTRA 의 `force=True`).

EON UI 쪽도 맞추려면 settings.cc 의 vdefault(마지막에서 두 번째 인자)를
위 표의 manager 값으로 바꾸면 된다. 이 zip 에는 settings.cc 를 넣지 않았다.
