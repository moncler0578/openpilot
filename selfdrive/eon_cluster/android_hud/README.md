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
p.put("EonClusterHudOutputTarget", "2")  # 1=외부 HUD, 2=S9 화면, 3=동시 출력
PY
```

Keep both devices on the same hotspot/Wi-Fi network. UDP port 7210 and TCP port
7211 must be reachable. `EonClusterHudOutputTarget` selects the physical output:
external TURZX USB HUD (`1`), the S9 full-screen Activity (`2`), or both (`3`, default).
`EonClusterHudOutputMode` separately selects the information shown in the system
panel and does not select the output device.

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
* NOO 안내는 주행 패널에서 **TMAP 좌하단(962~1106 / 330~456)** 으로 옮겼다.
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

## v0.30 (출력 대상 선택)

EON UI에 `EonClusterHudOutputTarget`을 추가했다.

* **1 — 외부 HUD**: TURZX USB 패널만 갱신하고 S9 전체화면에는 대기 안내를 표시한다.
* **2 — S9 화면**: S9 전체화면만 갱신하고 USB 검색·전송을 중단한다.
* **3 — 동시 출력(기본)**: 외부 HUD와 S9 전체화면을 함께 갱신한다.

출력 대상이 바뀌면 앱 재시작 없이 반영된다. USB 출력이 꺼질 때는 패널에
검은 프레임을 한 번 전송한 뒤 연결을 닫아 마지막 화면이 남지 않게 한다.

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

## v0.27 (최종 MPC·NOO 경로 표시)

* EON이 `modelV2.position`이나 MPC 입력 기준값인 `dPathPoints` 대신,
  `lateralPlan.mpcPathX/mpcPathY`에 발행된 MPC 최적화 상태 경로를 전송한다.
  Z축은 같은 shooting-node 인덱스의 `modelV2.position.z`를 사용한다.
* 패킷의 `pathFinal`이 참이면 최종 MPC 경로 유효구간 안에서는 TMAP 원거리
  곡선을 다시 혼합하지 않는다. NOO 지도 곡률이 HUD에서 이중 적용되지 않는다.
* 최종 경로에는 `OffsetTotal`이 이미 포함되어 있으므로 이 경우 패킷
  `pathOffset`은 0으로 보낸다. MPC가 무효이면 기존 모델 경로와 별도 오프셋
  방식으로 자동 폴백한다.

## v0.28 (OSM 도로환경 + 장거리 안정화)

* 기존 건물·옆길에 **방음벽, 실제 가드레일, 개별 나무/tree_row, 근거리
  가로등**을 추가했다. 가로등은 전방 80m, 나무는 150m, 방음벽·가드레일은
  190m 안에서만 그리며 각각 객체 상한을 둬 S9 렌더 부하를 제한한다.
* 건물은 명시 `height`를 층수보다 우선하고, 높이 상한을 40m에서 300m로
  올렸다. `height`가 없을 때만 `building:levels × 3.2m`를 사용한다.
* OSM 응답은 레이어별 개수 제한과 타일당 2MiB 제한을 유지한다. 같은 구간은
  앱 캐시를 재사용하며 캐시는 최대 500파일/96MiB다.
* 메모리에는 현재 위치 주변 3×3 타일만 남기고, 지나간 타일과 대기 중인
  다운로드를 제거한다. 인접 타일에 중복 포함된 OSM ID도 프레임에서 한 번만
  렌더한다.
* 리모트 진단의 OSM 표시는 `타일/건물+환경객체` 형식이다. 예: `3/48+35`.

## v0.29 (S9 화면 HUD + TMAP 전환)

외부 TURZX 패널이 없어도 S9 화면에 HUD를 렌더하고 nMirror가 Android Auto로
전송할 수 있다. 최초 설치 후 앱 상태 화면에서 **HUD 화면 오버레이 권한**을
한 번 허용해야 한다.

* 부팅 경로의 30초 지연은 유지한다. TMAP이 먼저 실행된 뒤 포커스를 받지 않는
  `TYPE_APPLICATION_OVERLAY` HUD가 올라오므로 TMAP task를 다시 실행하거나
  백그라운드로 밀어내지 않는다.
* 휴대폰 화면 렌더와 TURZX USB 출력을 분리했다. USB `1cbe:0092`가 없어도
  UDP 7210 / TCP 7211을 받아 HUD와 원본 TMAP `map_main`을 계속 그린다.
* 우상단 **TMAP** 버튼은 HUD 프레임과 별도 overlay window다. 누르면 HUD만
  숨겨 이미 실행 중인 TMAP의 검색·키보드 화면을 드러내고, 남은 **HUD** 버튼을
  누르면 같은 HUD로 복귀한다.
* HUD 프레임은 `FLAG_NOT_FOCUSABLE | FLAG_NOT_TOUCHABLE`이므로 TMAP의 수명주기와
  입력 포커스를 건드리지 않는다. 전환 버튼만 터치를 받는다.
* 외부 USB 패널을 함께 연결하면 휴대폰 HUD와 USB JPEG 출력이 동시에 동작한다.
  USB 오류/재검색 중에도 휴대폰 HUD 렌더는 멈추지 않는다.

## v0.31 (nMirror 앱 선택형 전체화면 HUD)

v0.29의 `TYPE_APPLICATION_OVERLAY` 방식과 우상단 TMAP/HUD 버튼을 제거했다.

* 런처 Activity 이름은 **EON HUD**다. nMirror 왼쪽 앱 목록에 EON HUD와
  TMAP을 각각 추가해 앱 선택으로 전환한다.
* EON HUD를 선택하면 가로 고정 immersive Activity가 상태바와 내비게이션 바를
  숨기고 화면 전체를 채운다. TMAP은 HUD 뒤에 겹쳐 보이지 않는다.
* TMAP을 선택하면 HUD Activity가 백그라운드로 이동하고 TMAP이 전체화면을
  차지한다. HUD 서비스는 계속 실행돼 UDP/TCP 수신과 USB 출력을 유지한다.
* `SYSTEM_ALERT_WINDOW` 권한과 최초 실행 오버레이 권한 안내는 더 이상 필요 없다.
* 기존 1920x462 논리 프레임은 USB 출력용으로 유지하며, S9 Activity에서는
  nMirror 출력 화면 전체 destination에 맞춰 그린다.

## v0.32 (전체화면 HUD 시작 충돌 수정)

Android 13에서 전체화면 Activity가 콘텐츠 뷰를 만들기 전에 system bar
controller를 요청해 종료되던 문제를 수정했다. 콘텐츠 뷰를 먼저 연결하고 생성된
`DecorView`에서 controller를 가져오므로 nMirror에서 EON HUD를 선택해도 시작
단계에서 종료되지 않는다.

## v0.33 (TMAP 우선 HUD/TMAP 전환)

S9 부팅 후 nMirror가 기존처럼 TMAP을 먼저 실행한다. HUD 앱을 nMirror 앱 목록에서
직접 여는 대신 TMAP 위에 작은 **HUD** 전환 버튼만 표시한다.

* **HUD**를 누르면 독립 Activity가 HUD를 전체화면으로 표시한다.
* HUD 위의 **TMAP** 버튼을 누르면 HUD task를 닫아 이미 실행 중이던 TMAP으로
  돌아간다. TMAP 패키지를 다시 실행하지 않으므로 안내와 경로가 유지된다.
* `TYPE_APPLICATION_OVERLAY`는 작은 전환 버튼에만 사용한다. HUD 영상 자체는
  overlay가 아니며, 버튼 이외 영역은 TMAP 터치를 가로채지 않는다.
* 최초 한 번 앱 설정 화면에서 **HUD 전환 버튼 오버레이 권한**을 허용해야 한다.

## v0.34 (8인치 순정 화면 비율 최적화)

1920x462 HUD를 16:9 순정 내비 화면 전체에 강제로 늘리던 방식을 제거했다.

* 실제 nMirror View의 가로·세로 크기를 매 프레임 사용해 최대 `fit-center` 영역을
  계산한다. 해상도를 1280x720이나 1920x1080으로 가정하지 않는다.
* 원본 종횡비를 그대로 유지하므로 차량, 원형 계기와 글자가 세로로 늘어나지
  않는다. 16:9 화면에서 남는 위아래 영역은 검은 안전 여백으로 표시한다.
* destination 좌표와 크기를 정수 픽셀로 맞추고 재사용 Paint/RectF로 그려 잦은
  객체 생성과 경계 번짐을 줄인다.

## v0.35 (제네시스 8인치/9.2인치 화면 프로필)

S9 앱의 상태 화면에 **순정 내비 화면** 선택 항목을 추가했다.

* **자동 감지(권장)** — 현재 nMirror View의 실제 가로·세로 크기를 그대로 쓴다.
* **제네시스 순정 8인치** — 800x480(5:3) 안전영역에 맞춘다.
* **제네시스 순정 9.2인치** — 1280x720(16:9) 안전영역에 맞춘다.
* 선택값은 S9에 저장되고 다음 HUD 프레임부터 즉시 반영된다. 앱이나 서비스를
  재시작할 필요가 없다.
* 세 모드 모두 1920x462 HUD 원본 비율과 정수 destination 좌표를 유지한다.
  설정은 S9/nMirror 출력에만 적용되며 외부 TURZX HUD 출력은 바뀌지 않는다.

## v0.36 (nMirror 즐겨찾기 HUD 전환)

TMAP 화면 우상단의 시스템 오버레이 버튼을 제거하고, 기존 APK에 nMirror가 별도
앱처럼 표시하는 **HUD 전환** 런처 아이콘을 추가했다.

* nMirror 즐겨찾기에 **HUD 전환**을 한 번 등록한다.
* TMAP에서 아이콘을 누르면 HUD 전체화면을 열고, HUD에서 같은 아이콘을 다시
  누르면 HUD task만 닫아 실행 중이던 TMAP으로 돌아간다.
* `EON Remote HUD` 아이콘은 설정·상태 확인용으로 그대로 유지한다. 두 아이콘은
  같은 APK에 들어 있으므로 설치와 업데이트는 한 번만 하면 된다.
* `SYSTEM_ALERT_WINDOW` 권한과 `HudSwitchOverlay`를 제거해 TMAP 지도 위를 가리는
  버튼 및 오버레이 권한 안내가 더 이상 나타나지 않는다.

## v0.37 (8인치/9.2인치 네이티브 전체화면 UI)

수동 화면 프로필을 선택하면 1920x462 전체 프레임을 축소해 위아래에 남기던
검은 여백 대신 순정 화면용 네이티브 프레임을 별도로 만든다.

* **8인치**는 800x480, **9.2인치**는 1280x720 캔버스를 사용한다.
* 왼쪽 52px/80px는 nMirror 즐겨찾기 바 안전영역으로 비워 둔다.
* 남은 영역에는 주행 화면을 원본 비율로 확대하고, 지도/안내 패널을 우측 하단
  카드로 재배치한다. 하단 남는 공간은 기어·프로필·EON 연결 상태 표시줄로 쓴다.
* 각 원본 패널은 종횡비를 유지한 채 정수 픽셀 destination으로 복사하므로 차량,
  원형 계기, 지도 글자가 늘어나지 않는다.
* 외부 TURZX용 1920x462 `phoneFrame`은 별도로 유지해 기존 외부 HUD 구성과 회전,
  좌우 반전, USB JPEG 출력에는 변화가 없다.
* 자동 감지는 기존 원본 비율 `fit-center`를 유지한다. 네이티브 전체화면을 쓰려면
  앱 설정에서 8인치 또는 9.2인치를 직접 선택한다.

## v0.38 (선택 상태가 분명한 화면 설정)

순정 내비 화면 선택을 펼쳐야 상태를 알 수 있던 드롭다운에서, 세 항목이 항상
보이는 라디오 선택 방식으로 바꿨다.

* 카드 위쪽에 **✓ 현재 적용: ...**을 녹색 굵은 글씨로 항상 표시한다.
* 자동 감지·8인치·9.2인치 항목 중 현재 값의 원형 선택 표시가 켜진다.
* 항목을 누르면 저장과 현재 적용 문구가 동시에 즉시 바뀐다.

## v0.39 (원본 3열 배치 전체높이 순정 화면)

실차 비교 사진을 기준으로 8인치/9.2인치 전용 카드 재배치를 제거했다.

* 원본 HUD의 **주행 화면 → 지도 → 우측 시스템 정보** 순서를 그대로 유지한다.
* 1920x462 원본 한 프레임을 800x480 또는 1280x720 전체 높이로 확대해 위아래
  검은 여백을 없애고, 글자·차량·지도 이미지도 같은 화면 배율로 함께 확대한다.
* nMirror가 즐겨찾기 바 영역을 이미 제외하므로 앱이 추가로 비우던 52px/80px
  안전공간을 0으로 바꿔 HUD가 즐겨찾기 영역 오른쪽에 바로 붙는다.
* 별도 지도 카드와 하단 `HUD · 8인치 / EON 연결` 상태바는 제거한다.
* 외부 TURZX 출력은 계속 원본 1920x462 프레임을 사용하므로 영향을 받지 않는다.

## v0.40 (글자·차량·지도 비율 보존)

v0.39의 한 장 비트맵 비균일 확대는 8인치에서 X 0.42배/Y 1.04배가 되어
원과 차량이 세로로 약 2.5배 길어질 수 있으므로 사용하지 않는다.

* 원본 3열 좌표만 순정 화면에 맞추고, 한 장의 완성 프레임을 늘리는 방식은 제거했다.
* 속도·RPM·PRND·아이콘 등 각 위젯은 X/Y 동일 배율로 다시 그려 원과 글자 비율을
  유지한다.
* 3D 주행 장면과 차량은 화면 높이에 맞춰 동일 배율로 확대하고 좌우만 중앙
  크롭한다.
* TMAP 지도 역시 동일 배율 중앙 크롭을 사용해 지도 글자와 도로 모양이 눌리지
  않는다.
* 우측 시스템 패널은 원본 폭에 맞는 동일 배율로 가운데 표시한다.
* nMirror 추가 여백 0과 외부 TURZX 원본 출력 유지 조건은 v0.39와 같다.

## v0.41 (8/9.2인치 우측 정보 패널 맞춤)

* 순정 8인치와 9.2인치 화면 모두 우측 시스템/디버그/S9 정보 패널을 실제 화면 폭의 15%로 확보한다.
* 정보 패널 배경은 화면 전체 높이를 채우고, 내부 글자와 카드는 X/Y 동일 배율로 렌더링해 잘림과 찌그러짐을 막는다.
* 주행 화면 폭과 종횡비는 유지하며, 늘어난 정보 패널만큼 지도 오른쪽 영역을 정리한다.
* 자동 프로필과 외부 TURZX 1920x462 출력은 기존 배치를 그대로 유지한다.

## v0.42 (순정 화면 RPM 겹침 보정)

* 8인치와 9.2인치 순정 프로필에서 RPM 아크, `RPM` 라벨, 회전수 숫자를 실제 화면 기준 18px 함께 위로 이동한다.
* 속도 숫자와 RPM 표시 사이 간격을 확보하고 세 요소의 상대 위치는 그대로 유지한다.
* 자동 프로필과 외부 TURZX 1920x462 출력 위치는 변경하지 않는다.

## v0.43 (RPM 글자 행 겹침 수정)

* v0.42의 RPM 게이지 전체 이동은 취소했다.
* RPM 아크 위치는 유지하고 `RPM` 라벨 기준선은 y=82, 회전수 숫자 기준선은 y=86으로 올렸다.
* 기어(P/R/N/D), 도착시간, 주행모드가 놓인 y=116 행과 RPM 글자 영역을 분리한다.
* 8인치·9.2인치·자동 프로필 모두 같은 논리 좌표를 사용하므로 화면별 상대 위치가 일치한다.

## v0.44 (속도·RPM 전체 위치 보정)

* v0.43의 RPM 글자만 이동하는 방식은 취소했다.
* 순정 8인치와 9.2인치에서 속도 숫자와 RPM 아크, `RPM` 라벨, 회전수 숫자를 실제 화면 기준 18px 모두 함께 올린다.
* 기어(P/R/N/D), 도착시간, 주행모드 행은 기존 위치를 유지한다.
* RPM 아크와 글자·숫자의 내부 상대 위치는 원래 값으로 복원한다.
* 자동 프로필과 외부 TURZX 1920x462 출력 위치는 변경하지 않는다.

## v0.45 (실차 표시 화살표 기준 배치)

* 속도 숫자와 RPM 전체는 기존 v0.44처럼 실제 화면 기준 18px 위로 유지한다.
* 설정속도 원은 18px 위, 앞차·TPMS 카드는 18px 아래로 이동한다.
* 지도 다음 안내 작은 초록 박스는 18px 위, NOO 교차로 카드는 24px 아래로 이동한다.
* S9 리모트 정보는 네이티브 픽셀 좌표로 다시 그려 8인치와 9.2인치 모두 위·아래 빈 공간 없이 7개 항목을 균등 배치한다.
* 자동 프로필과 외부 TURZX 1920x462 출력 위치는 변경하지 않는다.

## v0.46 (OSM 디스크 캐시 자동 갱신)

* 환경 객체가 없는 이전 캐시와 분리하도록 OSM 캐시 버전을 `v3_`로 올려 최초 1회 새로 다운로드한다.
* 캐시 TTL은 7일이며, 만료된 타일은 한 개의 기존 작업 스레드에서 순차 갱신한다.
* stale-while-revalidate 방식으로 오래된 캐시를 즉시 표시한 뒤 백그라운드에서 새 응답으로 교체한다.
* 갱신 실패 시 기존 타일을 유지하고 90분 뒤 재시도한다. 표시할 캐시가 없는 최초 실패는 기존처럼 90초 뒤 재시도한다.
* 캐시 파일 `lastModified`는 마지막 다운로드 시각으로만 사용하며 읽을 때 더 이상 갱신하지 않는다.
* 기존 500파일/96MB 제한과 응답 2MB 제한은 유지한다.
