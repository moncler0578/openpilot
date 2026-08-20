# Android remote HUD (experimental)

This optional companion moves the 1920x462 HUD render, JPEG compression and
TURZX `1cbe:0092` USB upload from the EON to an Android phone.  The EON sends a
small UDP JSON telemetry packet at 10 Hz. The already-compressed TMAP JPEG
received from the existing phone sender is forwarded unchanged over TCP; EON
does not decode, resize, composite, or re-encode it.

## EON

The manager starts `remote_hud` persistently, but it sleeps unless enabled:

```sh
python - <<'PY'
from common.params import Params
p = Params()
p.put_bool("EonClusterHud", True)
p.put("EonClusterHudOutputMode", "1")  # 0=EON direct, 1=S9 remote
PY
```

Keep both devices on the same hotspot/Wi-Fi network. UDP port 7210 and TCP port
7211 must be reachable. To return instantly to the direct EON HUD, keep
`EonClusterHud` true and set `EonClusterHudOutputMode` to `0`. The same choices
are exposed as **EON 직접** and **S9 원격** in the EON settings UI.

## Rooted Galaxy S9 TMAP sender

Build and install the `app` with Android Studio (minSdk 26; rooted Galaxy S9
running LineageOS 20 / Android 13 is the target). Root access is not
required, so it does not interfere with the existing Carrot/TMAP sender setup.
Connect the TURZX panel through a powered USB-C OTG adapter. Select **EON Remote
HUD** in E-Mirror's auto-launch list. The app starts its foreground service and
closes its activity immediately; it never requests screen-capture permission.
It receives the original TMAP JPEG back from EON, draws the driving and system
panels, encodes JPEG quality 55 at 8 FPS, and uploads it to the panel. Version
0.2 renders the perspective road, model path surface, lane lines, road edges,
two radar leads, a shaded 3D-style ego/lead vehicle, turn signals and BSD on the
Galaxy S9. The EON only publishes compact scene coordinates and does no HUD
rendering. Version 0.3 keeps the existing road-surface colors while bending the
road to the model edges, draws perspective-scaled dashed lane markings, rotates
PNG vehicle sprites with the path, and replaces the old BSD dot/triangle with a
rear-quarter vehicle sprite. JPEG writes are split into 16 KiB USB chunks for
older Galaxy S9 Android builds.

Version 0.4 resamples path, lane, and road-edge geometry at fixed forward
distances and applies five-frame-class EMA smoothing. Low-confidence geometry
is rejected while the last valid shape is held for 500 ms. Road edges are kept
ordered, total road width is clamped to 4.5-18.0 m, lane crossings are removed,
and lanes/path are constrained inside the stabilized road surface. These guards
prevent short model dropouts and sharp curves from lifting lane marks off the
road or collapsing the road polygon.

USB access may require one approval after initial installation. Selecting the
app as the default handler for `1cbe:0092` lets Android grant access and launch
it automatically on later USB attachments/reboots.

Vehicle control and CAN messages are never accepted from the phone.

## v0.19 (full 3D + USB freeze fix)

### 3D 주행씬 (`World3D.java`)

v0.18 까지의 `project()` 는 세로 `d/(13+d)`, 가로 `66/(1+d/17)` 이라는 서로
다른 근사식을 썼기 때문에 거리에 따른 도로 폭 / 차선 간격 / 차량 크기의 비율이
물리적으로 맞지 않았다. v0.19 는 단일 핀홀 카메라 하나만 쓴다.

```
depth = X + CAM_BACK                 (차 뒤 9.5 m)
u     = CX      - FOCAL * Y / depth  (f = 520 px)
v     = HORIZON + FOCAL * (CAM_H - Z) / depth   (노면 위 3.4 m, 지평선 y=249)
```

노면 · 차선 · 도로경계 · 건물 · 차량이 전부 같은 변환을 통과하므로 원근이
서로 어긋나지 않는다. 튜닝하려면 `World3D` 상단의 `FOCAL / CAM_H / CAM_BACK /
HORIZON` 네 개만 만지면 된다.

* 건물은 24 m 격자에 절대좌표로 고정돼 있고 누적 주행거리(`worldOdoM`)만큼
  뒤로 흘러간다. v0.18 은 거리 배열이 고정이라 달려도 제자리였다.
  앞면 + 안쪽 측면을 8꼭짓점 투영으로 그린다 (카메라가 지붕보다 낮으므로
  지붕면은 없는 게 맞다).
* 차선 두께는 도색폭 0.16 m 기준으로 거리에 따라 실제 비율대로 가늘어진다.
  주행차선(모델 index 1·2)은 노란색, 나머지는 흰색, 도로경계는 회색.
* 차량은 접지점 빌보드. 폭 1.9 m 가 `FOCAL/depth` 로 환산된다. 접지 그림자
  타원의 납작한 정도도 `CAM_H/depth` 로 계산한다.
* 패널이 237 px 로 납작해 높은 건물은 위가 잘리므로, 상단 46 px 헤이즈
  그라디언트로 배경에 녹아들게 처리했다.

### USB (`TurzxDisplay.java`)

* **매 프레임 `clearHalt()` 제거.** 주행 2~5 분 뒤 화면이 굳던 증상의 원인으로
  보인다. libusb 의 `clear_halt()` 는 커널 `usb_clear_halt()` 를 거쳐 호스트
  쪽 data toggle 까지 초기화하지만, 안드로이드 `UsbDeviceConnection` 에는 그
  ioctl 이 없어 우리가 보내는 `controlTransfer(0x02,0x01,...)` 는 장치 쪽
  toggle 만 DATA0 으로 만든다. 정상 상태에서 반복하면 양쪽 toggle 이 어긋나고,
  그 뒤로는 `bulkTransfer` 는 성공한 것처럼 보이는데 패널은 마지막 프레임을
  붙들고 있게 된다. 이제 `recoverAfterError()` — 즉 전송 실패 뒤에만 부른다.
* 패널 무응답 감시: `drainInput()` 이 읽은 패킷 수를 세어, 응답을 본 적 있는
  패널이 15 초 이상 조용하면 재초기화한다. 원래 조용한 패널에서는 감시가
  켜지지 않으므로 헛된 리셋이 없다.
* 프레임 중간에 쓰기가 실패하면 남은 길이를 0 으로 채워 패널의 수신 카운터를
  끝낸다. 안 그러면 패널이 프레임 대기 상태로 고착돼 재삽입 전까지 안 풀린다.

### 렌더 성능 (`HudService.java`)

* v0.18 은 프레임마다 1920x462 비트맵을 만들고 회전 복사본을 하나 더 만들었다
  (약 3.5 MB/프레임, 8 fps 면 28 MB/s). 이제 462x1920 세로 버퍼 하나를 잡아
  두고 캔버스 행렬로 회전/미러한다.
* `Paint` / `Path` / `RectF` / `ByteArrayOutputStream` 을 필드로 돌려 쓴다.
* 모델 경로를 프레임당 한 번만 배열로 풀고 선형보간한다. v0.18 의
  `pathCenterAt()` 은 호출마다 JSON 배열 전체를 훑는 최근접 탐색이었고
  프레임당 100 회 가까이 불렸다.
* EON UDP 가 3 초 이상 끊기면 마지막 상태로 얼지 않고 "EON 연결 끊김" 을
  표시한다. v0.18 은 속도 0 인 채 굳어 패널이 멈춘 것처럼 보였다.

### 건물 끄기

건물은 실제 주변 지형이 아니라 24 m 격자에 해시로 찍어내는 장식이다. 실제
데이터(도로 커브 · 차선 · 도로경계 · 앞차)와 헷갈리면 EON 에서 끌 수 있다.

```sh
python - <<'PY'
from common.params import Params
Params().put("EonClusterHudBuildings", "0")   # 0=끔, 1=켬(기본)
PY
```

패킷 키는 `hudBuildings` 이다. `remote_hud.py` 의 패킷 조립부에 아래 한 줄을
추가하면 위 Params 가 반영된다.

```python
"hudBuildings": int(self.params.get("EonClusterHudBuildings", encoding="utf8") or 1),
```

끄면 하늘 · 지면 · 노면 · 차선 · 차량만 남아 실제 인식 데이터만 보인다.

## v0.19.2 (5:4:1 레이아웃)

패널 폭 비율을 4:2:4 에서 **5:4:1** 로, 순서를 주행 → TMAP → SYSTEM 으로 바꿨다.

| 패널 | v0.19.1 | v0.19.2 |
|---|---|---|
| 주행 | 0~765 | **0~952** |
| SYSTEM | 776~1144 | 1728~1920 |
| TMAP | 1152~1920 | 960~1720 |

* 주행 패널의 요소 **크기는 그대로 두고 위치만** 넓어진 폭에 맞춰 벌렸다.
  속도 68pt, 핸들·SET·카메라 반지름 36, 카드 148x78 전부 v0.19.1 과 동일하다.
  3D 씬만 폭 952 로 넓어져 도로 양옆이 더 보인다.
* SYSTEM 은 폭이 192px 뿐이라 라벨을 위, 값을 아래로 쌓았다.
  코어별 사용률 줄은 자리가 없어 뺐고 ACCEL 은 두 줄로 남겼다.
* ATC 안내는 주행 패널에서 **TMAP 좌하단(962~1106 / 330~456)** 으로 옮겼다.
  크기는 이전과 같고, `remainDist > 0` 조건을 추가해 **경로가 살아 있을 때만**
  나온다. 목적지 없이 떠 있지 않는다.
* TBT 1행은 티맵 PNG 를 그대로 쓰되 폭을 480 -> 400 으로 줄였다.
  고가차도·복잡분기 아이콘이 살아 있어야 하므로 직접 그리지 않는다.
* TBT 2행(다음 회전)은 1행과 같은 녹색으로 앱이 직접 그린다 (폭 232).
  패킷의 `navi.next` 가 있어야 표시되며, EON 쪽 `apply_eon_v0192.py` 로 추가한다.
* 프레임 0 = 일시정지가 아니라 **패널 끄기**. 검은 프레임 한 장을 보내고
  밝기를 1 로 내린다. v0.19.1 까지는 전송만 멈춰 마지막 화면이 남았다.
* 건물 on/off 는 패킷 `hudBuildings` 로 제어된다 (기본 1). EON 쪽 키 등록과
  전달은 `apply_eon_v0192.py` 가 처리한다.

## v0.19.3 (TBT 확대 · 출력 모드)

* TBT 1행을 좌상단 기준으로 그리고 가로를 2배(400 -> 700)로 키웠다.
  `drawTbtImage()` 가 실제로 그린 높이를 돌려주므로 2행이 1행 **바로 아래에
  붙는다**. 이전에는 박스 안 세로 가운데 정렬이라 두 줄이 어긋나 보였다.
* 2행도 좌측을 1행과 같은 968 로 맞췄다.
* 새 파라미터 `EonClusterHudOutputMode` (패킷 `hudOutputMode`)
  * **1** — 주행 / 지도 / 시스템 (기본)
  * **2** — 시스템 자리에 실시간 디버그 (CPU·TEMP·SPEED·SET·GAP·LEAD·REL·FPS·JPEG)
* `EonClusterHudBuildings` 를 설정 UI 에 노출했다 (0 끔 / 1 켬).

## v0.19.4 (TBT 배너를 폰 화면과 같은 형태로)

티맵이 보내주는 `tbt_current_full.png` 는 "157m 교차로" 가 **한 줄로 붙은
가로형** 이라, 폰 화면처럼 거리 아래에 도로명을 넣을 수 없다. 이미지를 아무리
키우거나 줄여도 안쪽 배치는 못 바꾼다.

그래서 1행을 직접 그린다 (342x126). 패킷에 `turnType` / `turnDist` / `title`
이 이미 들어 있다.

```
 ┌──────────────────────────┐
 │  ↱   83 m                │   거리 46pt
 │      교차로              │   도로명 26pt
 └──────────────────────────┘
```

되돌리기용으로 PNG 를 그리는 `drawTbtImage()` 는 남겨 뒀다.
호출부 한 줄만 바꾸면 이전 방식으로 돌아간다.

**트레이드오프** — 직접 그리면 티맵의 정교한 회전 아이콘(고가차도, 복잡분기)
대신 앱의 단순 화살표가 나온다. 좌/우/유턴/직진은 문제없다.

## v0.19.5 (S9 리모트 패널)

`EonClusterHudOutputMode` 에 **3 = S9 리모트** 를 추가했다. SYSTEM 자리(192px)에
폰 자신의 상태와 USB 경로 진단 6줄이 들어간다.

| 줄 | 뜻 | 출처 |
|---|---|---|
| SoC | 폰 SoC 온도 | `/sys/class/thermal/thermal_zone*` 중 cpu/big/soc 존 |
| CPU | 폰 전체 CPU 사용률 | `/proc/stat` 첫 줄 차분 |
| MEM | 앱이 쓰는 힙 | `Runtime.totalMemory - freeMemory` |
| USB ERR | USB 오류 누적 | `usbErrorStreak` |
| PANEL | 패널이 마지막 응답 후 경과 | `TurzxDisplay.silenceMs()`, 응답 본 적 없으면 `--` |
| LINK | 마지막 USB 재연결 후 경과 | 열림 상태 전이 시각 |

화면이 굳는 순간 이 셋만 보면 원인이 갈린다.

* USB ERR 이 오르면 → 전송 자체가 실패 (허브 전원 / 호스트)
* PANEL 만 커지면 → 앱은 보내는데 패널이 응답을 끊음
* LINK 가 자꾸 0 으로 돌아가면 → 재연결을 반복하는 중

읽기 실패는 전부 `--` 로 떨어지므로 루팅 여부나 SELinux 정책에 상관없이 안전하다.

### 엔진·냉각수 온도

차량 CAN 이 안 붙으면 `carState` 가 0.0 을 올려 실제 0°C 와 구분되지 않았다.
`sm.alive["carState"]` 로 걸러 `--` 가 나오게 했다.

## v0.19.7 (S9 SoC 온도 · CPU 를 root 로 읽기)

v0.19.5 에서 SoC 와 CPU 가 `--` 로만 나왔다. 안드로이드 12+ 는 일반 앱의
`/proc/stat` 과 `/sys/class/thermal` 접근을 막는다(hidepid + SELinux).
MEM 만 나온 건 그것이 Java API(`Runtime`) 라서다.

이제 이렇게 동작한다.

1. 직접 읽기를 먼저 시도한다 (권한이 있으면 root 없이 끝)
2. 막히면 `su -c` 로 한 번에 덤프해서 읽는다
3. root 도 없으면 `suUnavailable` 을 세우고 다시 시도하지 않는다 (계속 `--`)

프로세스를 띄우는 일이라 렌더 스레드가 아니라 **전용 스레드(`hud-stats`)에서
3초마다** 돌린다. 프레임 타이밍에 영향이 없다.

온도는 `cpu` / `big` / `soc` / `apollo` / `atlas` 가 이름에 들어간 존을 우선하고,
없으면 가장 높은 존을 쓴다. Exynos 9810 은 존 이름이 기기마다 달라서다.

**첫 실행 때 Magisk 권한 요청이 한 번 뜬다.** 허용해야 값이 나온다.

## v0.20.0 (주행씬 v2)

* **정차 중 좌우 흔들림 수정.** 정차하면 modelV2 position 이 0 근처에만
  몰리는데(10초 예측인데 속도가 0이라 거리가 안 나옴), 노면·건물은 180m
  앞까지 그려야 하니 전부 외삽 구간이었다. 기울기 상한이 0.12 라 먼 쪽이
  ±8m 씩 흔들렸다. 상한을 0.06 으로 낮추고 20m 에 걸쳐 0 으로 감쇠시킨다.
* 경로 보간을 **Catmull-Rom** 으로. 각진 곡선이 사라진다.
* 노면 폭을 고정 ±4.45m 가 아니라 **인식된 roadEdges** 로 만든다.
  경계가 없으면 기존 고정폭으로 폴백.
* 도로 경계를 **연석 높이(0.13m)** 로 세워 평면감을 없앴다.
* 카메라를 뒤/위로 물림 — 9.5→13.0m, 3.4→4.6m. 자차가 화면을 덜 먹는다.
* **다크 / 라이트 팔레트.** 기존 `EonClusterHudTheme` (0 자동 / 1 다크 /
  2 라이트) 를 주행씬과 게이지 색에도 적용한다. 0 이면 19시~7시 다크.
* **BSD 를 차량 그림에서 경고 띠로.** openpilot 은 옆차 유무만 알고 앞뒤
  위치는 모른다. 특정 지점에 차를 그리면 없는 정보를 주장하는 셈이라,
  차선 전체를 띠로 표시한다. `EonClusterHudBsdStyle`
  (1 막대만 / 2 옅은 띠 / 3 진한 띠, 기본 2).

### EON 주의

`REMOTE_LAYOUT` 에서 `driveBg`/`roadTop`/`roadBottom`/`pathColor` 를 뺐다.
이 값들이 있으면 앱의 테마 팔레트를 매 패킷마다 덮어써서 hudTheme 가
주행씬에 반영되지 않는다.

### v0.20.1 — 차량 모양 선택

`EonClusterHudCarStyle` (패킷 `hudCarStyle`) 추가.

* **1 = 사진 스프라이트** (기본). 기존 `hud_ego_car.png` / `hud_other_car.png`.
* **2 = 3D 박스.** 뒷면 / 윗면 / 보이는 옆면 세 면에 명암을 준 단색 박스.
  폭 1.86m x 높이 1.46m x 길이 4.6m, 뒷면에 후미등. 카메라가 4.6m 높이라
  윗면이 보이는 게 맞다. 자차는 후미등을 생략하고 파란 계열.

BSD 는 이 설정과 무관하게 항상 경고 띠다 (옆차 앞뒤 위치를 모르므로).

### v0.20.2 — 경로 정합 + 33점 전송

**1. 모델 점 33개 전부 전송** (`remote_hud.py` `_line_points`)

기존 `step = max(1, count // 12)` 는 33점을 17점으로 솎았다. 급커브에서
Catmull-Rom 보간이 실제 곡률을 못 따라간다. 이제 전부 보낸다.
기하 데이터 1.6KB -> 3.2KB, EON 부하(1~3%)는 그대로. **앱 변경 없음.**

**2. `OffsetTotal` 반영** (패킷 `pathOffset`)

`lateral_planner.py:157` 이 `path_xyz[:, 1] += self.offset_total` 로 최종
경로에만 오프셋을 더한다. 모델의 laneLines / roadEdges 에는 안 들어간다.
그래서 앱도 **경로 리본에만** 더한다. 차선·경계는 원본 그대로.

**3. `liveCalibration` pitch 반영** (패킷 `calibPitch`)

`rpyCalib[1]` (rad, ±0.15 클램프) 을 받아 수평선을 `FOCAL * tan(pitch)`
만큼(±46px) 옮긴다. HORIZON 은 상수지만 실제 EON 카메라는 캘리브에 따라
다르므로, 이 보정으로 세로 구도가 이온 화면에 가까워진다.

`liveCalibration` 을 SubMaster 구독 목록에 추가했다.

**한계** — 이온 화면과 완전히 같아지지는 않는다. EON 은 실제 카메라 영상 위에
카메라 intrinsics 로 투영하고, 앱은 가상 노면 위에 합성 카메라로 그린다.
오프셋과 pitch 를 맞추면 "치우쳐 보이는" 문제는 사라지지만 곡선 형상은
여전히 조금 다르다.

## v0.27 (최종 MPC·ATC 경로 표시)

* EON이 `modelV2.position` 대신 `lateralPlan.dPathPoints`를 전송한다.
  X축은 lateral planner와 같은 `v_plan[0] * T_IDXS`, Z축은 같은 시간의
  `modelV2.position.z`를 사용한다.
* 패킷의 `pathFinal`이 참이면 최종 MPC 경로 유효구간 안에서는 TMAP 원거리
  곡선을 다시 혼합하지 않는다. ATC 지도 곡률이 HUD에서 이중 적용되지 않는다.
* 최종 경로에는 `OffsetTotal`이 이미 포함되어 있으므로 이 경우 패킷
  `pathOffset`은 0으로 보낸다. MPC가 무효이면 기존 모델 경로와 별도 오프셋
  방식으로 자동 폴백한다.
