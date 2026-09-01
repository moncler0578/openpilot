package ai.comma.remotehud;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.ColorMatrix;
import android.graphics.ColorMatrixColorFilter;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.PorterDuff;
import android.graphics.PorterDuffColorFilter;
import android.graphics.Rect;
import android.graphics.RectF;
import android.graphics.Typeface;
import android.hardware.usb.UsbDevice;
import android.os.Build;
import android.os.IBinder;
import android.os.PowerManager;
import android.os.SystemClock;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/**
 * v0.19 변경점
 *
 *  1. 주행씬을 ModelWorldGL 의 실제 핀홀 원근투영으로 교체 (완전 3D).
 *  2. 프레임 버퍼 재사용 — v0.18 은 매 프레임 1920x462 비트맵을 만들고
 *     거기에 회전 복사본을 하나 더 만들었다(약 3.5MB/프레임, 8fps 면 28MB/s).
 *     이제 462x1920 세로 버퍼 하나만 잡아 두고 캔버스 행렬로 회전한다.
 *  3. Paint / Path / RectF 를 필드로 돌려 써 프레임당 할당을 없앴다.
 *  4. EON UDP 가 끊기면 마지막 상태를 그대로 얼려 두지 않고 화면에 알린다.
 *     (v0.18 은 속도 0 인 채로 굳어 패널이 멈춘 것처럼 보였다)
 *  5. USB halt 처리 정리 + 패널 무응답 감시는 TurzxDisplay v0.19 참고.
 *
 */
public final class HudService extends Service {

    private static volatile HudService activeInstance;

    private static final int UDP_PACKET_MAX_BYTES = 65507;

    static final String ACTION_RESCAN_USB = "ai.comma.remotehud.RESCAN_USB";
    static final String EXTRA_FROM_BOOT = "ai.comma.remotehud.FROM_BOOT";

    private static final String CHANNEL = "remote_hud";

    private static final int WIDTH = 1920;
    private static final int HEIGHT = 462;
    // 패널 폭 비율 5 : 4 : 1  (주행 : TMAP : SYSTEM)
    private static final int DRIVE_RIGHT = 952;
    private static final float DRIVE_CX = 476f;
    private static final int MAP_LEFT = 960;
    private static final int MAP_RIGHT = 1720;
    private static final float MAP_CX = 1340f;
    private static final int SYSTEM_LEFT = 1728;
    private static final int SYSTEM_RIGHT = 1920;

    /**
     * NOO 안내. 화살표·거리·상태줄 모두 깜박이지 않고 고정이며, 상태줄은 안내가
     * 없을 때도 항상 떠 있는다.
     * 과속카메라 아이콘(882, 171)·그 거리표시(y=231) 아래, TPMS 카드(위끝 376)
     * 위의 빈 공간에 세운다. 가로 기준은 TPMS 카드 가운데(865).
     */
    // 바로 아래 TPMS 카드(791~939) 의 가운데 865 에 맞춘다. 과속카메라 아이콘의
    // 882 를 쓰면 안내가 없어 상태줄만 남았을 때 카드보다 오른쪽으로 치우쳐 보인다.
    private static final float NOO_CX = 865f;
    // 위에서부터 화살표(278) → 남은거리(340) → 차선변경 진단(372) 순으로 쌓고,
    // 맨 아래 진단 글자가 TPMS 카드(위끝 376) 바로 위에 닿게 잡는다.
    private static final float NOO_CY = 278f;
    private static final float NOO_ARROW_SCALE = 1.4f;
    private static final float NOO_TEXT_DY = 62f;
    /** 차선변경 진단 문자열. TPMS 카드 바로 위, 깜박이지 않고 계속 떠 있는다. */
    private static final float NOO_LANE_DY = 94f;
    private static final float NOO_LANE_SIZE = 17f;
    private static final float NOO_ICON_H = 62f;
    /** 적용속도 표시는 SET 원(반지름 36) 오른쪽으로 이만큼 띄운다. */
    private static final float APPLY_DX = 52f;
    private static final int APPLY_OCHRE = Color.rgb(214, 168, 60);

    /** 속도 숫자 기준선. 예전 KM 라벨이 있던 자리로, 위쪽 RPM 아크 공간용. */
    private static final float SPEED_BASELINE = 118f;
    /** "RPM" / 회전수 숫자를 아크 끝에서 좌우 바깥으로 띄우는 간격. */
    private static final float RPM_LABEL_GAP = 16f;
    /** RPM 인셋 바 게이지: 레일 두 줄 사이를 짧은 바로 채운다.
     *  바깥 레일 반경 = 106+5 = 111 (예전 굵은 아크 바깥면과 동일),
     *  안쪽 레일 반경 = 106-6-3 = 97 → 속도 숫자(2자리 88px)와 16px 여유. */
    private static final int RPM_BARS = 44;
    private static final float RPM_BAR_W = 12f;
    private static final float RPM_RAIL_W = 2f;
    private static final float RPM_RAIL_OFF = 5f;

    private static final int USB_RESET_AFTER_ERRORS = 3;
    private static final int USB_SLOWDOWN_AFTER_ERRORS = 5;
    /** openDevice/claimInterface 가 조용히 실패한 횟수가 이만큼 쌓이면 포트를 재바인딩한다. */
    private static final int USB_OPEN_STALL_ATTEMPTS = 5;
    /** 포트 재바인딩 재시도 간격. 너무 자주 하면 재열거만 반복된다. */
    private static final long USB_OPEN_STALL_COOLDOWN_MS = 15000L;
    /**
     * 부팅 직후 Android USB 서비스가 완전히 준비되기 전에 권한 요청을 보내면
     * 확인창이 뜨지 않은 채 "권한 승인 대기"로 남는 S9가 있다. 네트워크와
     * 렌더러는 즉시 시작하되 USB 첫 검색만 잠깐 늦춘다.
     */
    private static final long BOOT_USB_SCAN_DELAY_MS = 5000L;

    /** EON 텔레메트리가 이보다 오래 끊기면 화면에 표시한다 */
    private static final long EON_STALE_MS = 3000L;
    /** 와이퍼 조작 직후 상태표시 강조 시간. */
    private static final long WIPER_MODE_HIGHLIGHT_MS = 2500L;

    private static volatile boolean serviceRunning;
    private static volatile boolean mapConnected;
    private static volatile boolean usbConnected;
    private static volatile boolean usbError;
    private static volatile float measuredFps;
    private static volatile int lastJpegBytes;
    private static volatile long lastJpegSentElapsed;
    private static volatile long lastRenderElapsed;
    private static volatile long lastEonRxElapsed;
    private static volatile String lastEonAddress = "--";
    private static volatile boolean udpReceiverBound;
    private static volatile long udpRawPacketCount;
    private static volatile int udpLastRawBytes;
    private static volatile long udpLastRawRxElapsed;
    private static volatile String udpReceiverError = "";
    private static volatile String usbStatus = "미연결 · 1CBE:0092";

    private TurzxDisplay display;
    private Bitmap egoCar;
    private final float[] leadSpriteInfo = new float[3];
    // 채도 0 + 밝기 0.82. 자차 그림을 앞차로 재사용할 때만 적용한다.
    private static final ColorMatrixColorFilter leadTint = buildLeadTint();

    private static ColorMatrixColorFilter buildLeadTint() {
        ColorMatrix m = new ColorMatrix();
        m.setSaturation(0f);
        ColorMatrix dim = new ColorMatrix(new float[] {
                0.82f, 0f, 0f, 0f, 0f,
                0f, 0.82f, 0f, 0f, 0f,
                0f, 0f, 0.82f, 0f, 0f,
                0f, 0f, 0f, 1f, 0f
        });
        m.postConcat(dim);
        return new ColorMatrixColorFilter(m);
    }
    private Bitmap speedBumpImage;
    /** 회전종류(TURN_*) 별 실제 화살표 그림. 없는 칸은 벡터로 폴백한다. */
    private final Bitmap[] turnImages = new Bitmap[10];
    private Bitmap wheelImage;   // res/drawable-nodpi/hud_wheel.png (없으면 기존 벡터 핸들)
    private Bitmap statusIcons;  // 순정 계기판 스타일: 미등/전조등/안전벨트/문 열림 PNG 스프라이트
    private Thread receiverThread;
    private Thread mapThread;
    private Thread renderThread;
    private int usbErrorStreak;
    private boolean usbReceiverRegistered;
    private PowerManager.WakeLock wakeLock;
    private volatile boolean workersStarted;
    private long frameIntervalMs = 125L;
    private volatile long mapFrameIntervalMs = 200L;
    private long lastMapAcceptedElapsed = 0L;
    private int configuredFps = 8;
    private int jpegQuality = 55;
    private int appliedBrightness = -1;
    private int configuredTheme = 0;
    private int configuredLanguage = 0;
    private int configuredRadarInfo = 4;
    private int configuredScreenMode = 1;
    /** 도로변 건물 표시 여부 (장식이므로 끌 수 있다) */
    /** BSD 표시 방식 1: 막대만 / 2: 옅은 면 / 3: 진한 면 */
    /** 차량 표현 1: 사진 스프라이트 / 2: 3D 박스 */
    /** 이번 프레임의 테마 (매 프레임 render() 에서 갱신) */
    private boolean frameDark = false;
    private WeatherService weather;
    /** 상단 밴드에 하늘을 깐 동안만 true. 이때 글자색을 하늘 밝기에 맞춘다. */
    private boolean skyBand = false;
    private boolean skyLightInk = false;
    /** 1: 주행·지도·시스템 / 2: 실시간 디버그 / 3: S9 리모트 */
    private int configuredOutputMode = 1;
    /** 패킷 hudTmapIcon. 티맵 회전 아이콘을 쓸지(1) 앱 내장 화살표를 쓸지(0). */
    private volatile boolean tmapIconEnabled = false;
    /** 패킷 hudJunction. 0: 끔 / 1: 분기 실사 / 2: 실사 + 도착정보 바. */
    private volatile int junctionMode = 2;
    /** 화면 구성 1: 주행·티맵·시스템, 2: 주행·티맵만 */
    private int configuredLayoutMode = 1;
    /** 출력 대상 1: 외부 USB HUD, 2: S9 화면, 3: 동시 출력 */

    // 회전 카운트다운 (carrot-wip leftSec 방식: 단조감소)
    /** 직전 프레임 남은초. 거리 튐으로 카운트가 역행하지 않게 상한으로 쓴다. */
    private int countdownLastSec = 100;
    /** 0 초를 처음 만난 시각. 잠깐 보여준 뒤 닫는다. */
    private long countdownZeroAtMs = 0L;

    // 와이퍼 상태표시. MIST 는 짧은 펄스라 레버를 놓은 뒤에도 잠시 유지한다.
    private int lastRawWiperMode = -1;
    private int heldWiperMode = 0;
    private long wiperModeChangedElapsed = 0L;

    // S9 자체 상태 (출력모드 3)
    private long lastReconnectElapsed = 0L;
    private long cpuLastTotal = 0L;
    private long cpuLastIdle = 0L;
    private volatile float s9CpuPercent = -1f;
    private volatile float s9TempC = -1f;
    private volatile boolean suUnavailable = false;
    private Thread statsThread;
    private int configuredOrientation = 0;
    private boolean configuredMirror = false;

    private long tripStartElapsed = 0L;
    private long tripLastElapsed = 0L;
    private double tripDistanceKm = 0.0d;
    /** 건물이 뒤로 흘러가도록 하기 위한 누적 주행거리(m) */

    // 렌더 재사용 자원 (렌더 스레드 전용)
    private final Matrix wheelMatrix = new Matrix();
    private ColorMatrixColorFilter wheelGray;
    private Bitmap outFrame;
    private Canvas outCanvas;
    private Bitmap phoneFrame;
    private Canvas phoneCanvas;
    private final Object phoneFrameLock = new Object();
    private final Paint phonePreviewPaint = new Paint(
            Paint.ANTI_ALIAS_FLAG | Paint.FILTER_BITMAP_FLAG | Paint.DITHER_FLAG);
    private final RectF phoneViewport = new RectF();
    /** 부팅 재연결 스레드에서도 갱신하므로 volatile 이어야 한다. */
    private volatile long nextUsbAttemptElapsed;
    private long nextOpenStallRecoverElapsed;
    private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Matrix outMatrix = new Matrix();
    private final RectF scratchRect = new RectF();
    private final Rect scratchIRect = new Rect();
    private final Path scratchPath = new Path();
    /** 문 열림 PNG: 흰 차체만 주간 배경용 진회색으로 바꾸고 빨간 문은 보존한다. */
    private final ColorMatrixColorFilter dayDoorFilter = new ColorMatrixColorFilter(
            new ColorMatrix(new float[] {
                    1f, -0.398f, -0.398f, 0f, 0f,
                    0f, 0.231f, 0f, 0f, 0f,
                    0f, 0f, 0.259f, 0f, 0f,
                    0f, 0f, 0f, 1f, 0f
            }));
    /** Lazy-created on the render thread. */
    private ModelWorldGL modelWorldGl;
    private final ByteArrayOutputStream jpegOut = new ByteArrayOutputStream(180000);

    private final AtomicBoolean running = new AtomicBoolean(false);
    private final AtomicReference<JSONObject> state = new AtomicReference<>(new JSONObject());
    private final AtomicReference<Bitmap> mapFrame = new AtomicReference<>();
    private final AtomicReference<Bitmap> tbtCurrentFrame = new AtomicReference<>();
    private final AtomicReference<Bitmap> tbtNextFrame = new AtomicReference<>();
    /** 티맵이 그린 현재 회전 아이콘(tbt_current_compact). 없으면 내장 그림/벡터. */
    private final AtomicReference<Bitmap> tbtCompactFrame = new AtomicReference<>();
    /** 티맵 분기 실사 이미지(crossroad_expanded). 안내가 끝나면 EON 이 파일을 지운다. */
    private final AtomicReference<Bitmap> crossroadFrame = new AtomicReference<>();
    private final AtomicReference<Bitmap> laneFrame = new AtomicReference<>();
    private final AtomicReference<InetAddress> eonAddress = new AtomicReference<>();
    private final Object assetLock = new Object();

    private final BroadcastReceiver usbReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            UsbDevice device = (UsbDevice) intent.getParcelableExtra("device");
            if (!TurzxDisplay.isTarget(device)) {
                return;
            }
            String action = intent.getAction();
            if ("android.hardware.usb.action.USB_DEVICE_DETACHED".equals(action)) {
                if (display != null) {
                    display.reset();
                }
                usbStatus = "분리됨 · 재연결 대기";
                usbConnected = false;
                usbError = false;
            } else if ("ai.comma.remotehud.USB_PERMISSION".equals(action)) {
                boolean granted = intent.getBooleanExtra("permission", false);
                if (display != null) {
                    display.reset();
                }
                usbStatus = granted ? "USB 권한 허용 · 연결 중" : "USB 권한 거부됨 · 재검색 필요";
                usbConnected = false;
                usbError = !granted;
            } else if ("android.hardware.usb.action.USB_DEVICE_ATTACHED".equals(action)) {
                requestUsbRescan();
            }
        }
    };

    // ── 상태 스냅샷 (MainActivity 표시용) ──────────────────────────────────

    public static final class StatusSnapshot {
        final boolean running;
        final boolean eonConnected;
        final String eonAddress;
        final boolean udpReceiverBound;
        final boolean udpRawPacketRecent;
        final long udpRawPacketCount;
        final int udpLastRawBytes;
        final String udpReceiverError;
        final boolean mapConnected;
        final String usbStatus;
        final boolean usbConnected;
        final boolean usbError;
        final float fps;
        final int lastJpegBytes;

        StatusSnapshot(boolean running, boolean eonConnected, String eonAddress,
                       boolean udpReceiverBound, boolean udpRawPacketRecent,
                       long udpRawPacketCount, int udpLastRawBytes, String udpReceiverError,
                       boolean mapConnected, String usbStatus, boolean usbConnected,
                       boolean usbError, float fps, int lastJpegBytes) {
            this.running = running;
            this.eonConnected = eonConnected;
            this.eonAddress = eonAddress;
            this.udpReceiverBound = udpReceiverBound;
            this.udpRawPacketRecent = udpRawPacketRecent;
            this.udpRawPacketCount = udpRawPacketCount;
            this.udpLastRawBytes = udpLastRawBytes;
            this.udpReceiverError = udpReceiverError;
            this.mapConnected = mapConnected;
            this.usbStatus = usbStatus;
            this.usbConnected = usbConnected;
            this.usbError = usbError;
            this.fps = fps;
            this.lastJpegBytes = lastJpegBytes;
        }
    }

    public static StatusSnapshot getStatusSnapshot() {
        long now = SystemClock.elapsedRealtime();
        boolean eonOk = serviceRunning && lastEonRxElapsed > 0 && now - lastEonRxElapsed < 2000;
        boolean rawPacketRecent = serviceRunning && udpLastRawRxElapsed > 0
                && now - udpLastRawRxElapsed < 2000;
        boolean fpsOk = serviceRunning && lastRenderElapsed > 0 && now - lastRenderElapsed < 2000;
        boolean jpegOk = serviceRunning && lastJpegSentElapsed > 0 && now - lastJpegSentElapsed < 2000;
        return new StatusSnapshot(serviceRunning, eonOk, lastEonAddress,
                serviceRunning && udpReceiverBound, rawPacketRecent,
                udpRawPacketCount, udpLastRawBytes, udpReceiverError,
                serviceRunning && mapConnected,
                usbStatus, serviceRunning && usbConnected, usbError,
                fpsOk ? measuredFps : 0.0f, jpegOk ? lastJpegBytes : 0);
    }

    // ── 라이프사이클 ──────────────────────────────────────────────────────

    @Override
    public void onCreate() {
        super.onCreate();
        activeInstance = this;
        weather = new WeatherService(this);
        // EON 이 붙기 전 첫 프레임부터 올바른 방향으로 그리기 위해 마지막 값을 복원한다.
        configuredOrientation = AppPrefs.getOrientation(this);
        configuredMirror = AppPrefs.isMirror(this);
        egoCar = BitmapFactory.decodeResource(getResources(), R.drawable.hud_ego_car);
        // 핸들 이미지는 선택 사항이라 R.drawable 을 직접 참조하지 않는다.
        // 파일이 없어도 빌드가 깨지지 않고, 있으면 자동으로 벡터 대신 쓰인다.
        int wheelId = getResources().getIdentifier("hud_wheel", "drawable", getPackageName());
        wheelImage = wheelId == 0 ? null : BitmapFactory.decodeResource(getResources(), wheelId);
        statusIcons = decodeUnscaled("hud_status_icons");
        // 과속방지턱 표지판은 EON assets/images/speed_bump.png 와 같은 그림을
        // drawable-nodpi 에 넣어 쓴다. 없으면 아래 벡터 폴백으로 그린다.
        int bumpId = getResources().getIdentifier("hud_speed_bump", "drawable", getPackageName());
        if (bumpId != 0) {
            BitmapFactory.Options bumpOpts = new BitmapFactory.Options();
            bumpOpts.inScaled = false;
            speedBumpImage = BitmapFactory.decodeResource(getResources(), bumpId, bumpOpts);
        }
        // NOO 방향표시용 화살표 그림 10종. 좌·우회전은 EON assets 원본이고 나머지는
        // 같은 두께(획/상자 = 24%)로 맞춰 그린 것이다. 파일이 없으면 벡터로 폴백한다.
        turnImages[TURN_STRAIGHT] = decodeUnscaled("hud_turn_s");
        turnImages[TURN_LEFT] = decodeUnscaled("hud_turn_l");
        turnImages[TURN_RIGHT] = decodeUnscaled("hud_turn_r");
        turnImages[TURN_STRAIGHT_LEFT] = decodeUnscaled("hud_turn_sl");
        turnImages[TURN_STRAIGHT_RIGHT] = decodeUnscaled("hud_turn_sr");
        turnImages[TURN_LEFT_RIGHT] = decodeUnscaled("hud_turn_lr");
        turnImages[TURN_UTURN] = decodeUnscaled("hud_turn_u");
        turnImages[TURN_SLIGHT_LEFT] = decodeUnscaled("hud_turn_gl");
        turnImages[TURN_SLIGHT_RIGHT] = decodeUnscaled("hud_turn_gr");
        turnImages[TURN_ARRIVE] = decodeUnscaled("hud_turn_arrive");
        NotificationManager nm = (NotificationManager) getSystemService(NOTIFICATION_SERVICE);
        nm.createNotificationChannel(new NotificationChannel(CHANNEL, "EON Remote HUD",
                NotificationManager.IMPORTANCE_LOW));
        Notification.Builder builder = new Notification.Builder(this, CHANNEL);
        startForeground(72, builder
                .setContentTitle("EON Remote HUD")
                .setContentText("상태 화면을 열려면 누르세요")
                .setSmallIcon(android.R.drawable.ic_menu_directions)
                .setContentIntent(PendingIntent.getActivity(this, 0,
                        new Intent(this, MainActivity.class),
                        PendingIntent.FLAG_UPDATE_CURRENT | PendingIntent.FLAG_IMMUTABLE))
                .setOngoing(true)
                .build());

        IntentFilter filter = new IntentFilter();
        filter.addAction("android.hardware.usb.action.USB_DEVICE_ATTACHED");
        filter.addAction("android.hardware.usb.action.USB_DEVICE_DETACHED");
        filter.addAction("ai.comma.remotehud.USB_PERMISSION");
        if (Build.VERSION.SDK_INT >= 33) {
            registerReceiver(usbReceiver, filter, Context.RECEIVER_NOT_EXPORTED);
        } else {
            registerReceiver(usbReceiver, filter);
        }
        usbReceiverRegistered = true;
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        boolean fromBoot = intent != null && intent.getBooleanExtra(EXTRA_FROM_BOOT, false);
        if (intent != null && ACTION_RESCAN_USB.equals(intent.getAction()) && running.get()) {
            requestUsbRescan();
            return START_STICKY;
        }
        if (running.get()) {
            return START_STICKY;
        }
        running.set(true);
        serviceRunning = true;
        mapConnected = false;
        usbConnected = false;
        usbError = false;
        measuredFps = 0.0f;
        lastJpegBytes = 0;
        lastRenderElapsed = 0L;
        udpReceiverBound = false;
        udpRawPacketCount = 0L;
        udpLastRawBytes = 0;
        udpLastRawRxElapsed = 0L;
        udpReceiverError = "";
        acquireWakeLock();

        // 예전처럼 서비스 전체를 30초 기다리게 하지 않는다. EON/TMAP 수신과
        // 화면 렌더는 바로 시작하고, 부팅 경로의 USB 권한 요청만 5초 늦춘다.
        if (fromBoot) {
            nextUsbAttemptElapsed = SystemClock.elapsedRealtime() + BOOT_USB_SCAN_DELAY_MS;
            usbStatus = "부팅 완료 · 외부 HUD 자동 연결 대기";
        }
        startWorkers();
        return START_STICKY;
    }

    private void startWorkers() {
        if (!running.get() || workersStarted) {
            return;
        }
        workersStarted = true;
        if (nextUsbAttemptElapsed <= SystemClock.elapsedRealtime()) {
            usbStatus = "휴대폰 HUD 실행 · 외부 USB 검색 중";
        }
        display = new TurzxDisplay(this);

        receiverThread = new Thread(new Runnable() {
            @Override
            public void run() {
                receiveLoop();
            }
        }, "hud-telemetry");
        mapThread = new Thread(new Runnable() {
            @Override
            public void run() {
                mapLoop();
            }
        }, "hud-tmap");
        renderThread = new Thread(new Runnable() {
            @Override
            public void run() {
                renderLoop();
            }
        }, "hud-render");
        statsThread = new Thread(new Runnable() {
            @Override
            public void run() {
                statsLoop();
            }
        }, "hud-stats");
        receiverThread.start();
        mapThread.start();
        renderThread.start();
        statsThread.start();

    }

    private void acquireWakeLock() {
        try {
            if (wakeLock == null) {
                PowerManager pm = (PowerManager) getSystemService(POWER_SERVICE);
                wakeLock = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "RemoteHUD::render");
                wakeLock.setReferenceCounted(false);
            }
            if (!wakeLock.isHeld()) {
                wakeLock.acquire();
            }
        } catch (Exception ignored) {
        }
    }

    private void releaseWakeLock() {
        try {
            if (wakeLock != null && wakeLock.isHeld()) {
                wakeLock.release();
            }
        } catch (Exception ignored) {
        }
        wakeLock = null;
    }

    @Override
    public void onTaskRemoved(Intent rootIntent) {
        if (AppPrefs.isAutoStart(this)) {
            try {
                startForegroundService(new Intent(getApplicationContext(), HudService.class));
            } catch (Exception ignored) {
            }
        }
        super.onTaskRemoved(rootIntent);
    }

    public void requestUsbRescan() {
        if (display != null) {
            display.reset();
        }
        usbStatus = "외부 HUD 재검색 중";
        usbConnected = false;
        usbError = false;
        measuredFps = 0.0f;
        lastJpegBytes = 0;
        usbErrorStreak = 0;
    }

    // ── 네트워크 ──────────────────────────────────────────────────────────

    private void receiveLoop() {
        while (running.get()) {
            DatagramSocket socket = null;
            try {
                // Keep the direct bind path proven on the installed S9. Only the
                // datagram buffer is enlarged for detailed modelV2 telemetry.
                socket = new DatagramSocket(7210);
                socket.setBroadcast(true);
                socket.setSoTimeout(1000);
                udpReceiverBound = true;
                udpReceiverError = "";
                byte[] buffer = new byte[UDP_PACKET_MAX_BYTES];
                while (running.get()) {
                    try {
                        DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                        socket.receive(packet);
                        udpRawPacketCount++;
                        udpLastRawBytes = packet.getLength();
                        udpLastRawRxElapsed = SystemClock.elapsedRealtime();
                        JSONObject decoded;
                        try {
                            decoded = new JSONObject(new String(packet.getData(), packet.getOffset(),
                                    packet.getLength(), "UTF-8"));
                        } catch (JSONException malformed) {
                            udpReceiverError = "JSON 오류";
                            continue;
                        }
                        state.set(decoded);
                        udpReceiverError = "";
                        eonAddress.set(packet.getAddress());
                        lastEonRxElapsed = SystemClock.elapsedRealtime();
                        lastEonAddress = packet.getAddress().getHostAddress();
                        byte[] ack = "HUD1".getBytes("US-ASCII");
                        socket.send(new DatagramPacket(ack, ack.length, packet.getAddress(), packet.getPort()));
                    } catch (SocketTimeoutException ignored) {
                    }
                }
            } catch (Exception e) {
                String message = e.getMessage();
                udpReceiverError = e.getClass().getSimpleName()
                        + (message == null || message.length() == 0 ? "" : ": " + message);
                SystemClock.sleep(1000L);
            } finally {
                udpReceiverBound = false;
                if (socket != null) {
                    try {
                        socket.close();
                    } catch (Exception ignored) {
                    }
                }
            }
        }
    }

    private static boolean tagEquals(byte[] h, String tag) {
        return h.length == 4 && h[0] == tag.charAt(0) && h[1] == tag.charAt(1)
                && h[2] == tag.charAt(2) && h[3] == tag.charAt(3);
    }

    private void replaceAsset(AtomicReference<Bitmap> target, byte[] data) {
        Bitmap decoded = data.length == 0 ? null : BitmapFactory.decodeByteArray(data, 0, data.length);
        if (data.length > 0 && decoded == null) {
            return;
        }
        Bitmap old = target.getAndSet(decoded);
        if (old != null && old != decoded) {
            old.recycle();
        }
    }

    private void mapLoop() {
        while (running.get()) {
            InetAddress address = eonAddress.get();
            if (address == null) {
                SystemClock.sleep(500L);
                continue;
            }
            Socket socket = null;
            try {
                socket = new Socket();
                socket.connect(new InetSocketAddress(address, 7211), 2000);
                mapConnected = true;
                socket.setSoTimeout(4000);
                DataInputStream in = new DataInputStream(socket.getInputStream());
                byte[] header = new byte[4];
                while (running.get() && address.equals(eonAddress.get())) {
                    in.readFully(header);
                    int length = in.readInt();
                    if (length < 0 || length > 2097152) {
                        throw new Exception("bad asset size");
                    }
                    byte[] data = new byte[length];
                    if (length > 0) {
                        in.readFully(data);
                    }
                    synchronized (assetLock) {
                        if (tagEquals(header, "MAP1")) {
                            long mapNow = SystemClock.elapsedRealtime();
                            if (lastMapAcceptedElapsed == 0 || mapNow - lastMapAcceptedElapsed >= mapFrameIntervalMs) {
                                replaceAsset(mapFrame, data);
                                lastMapAcceptedElapsed = mapNow;
                            }
                        } else if (tagEquals(header, "TBT1")) {
                            replaceAsset(tbtCurrentFrame, data);
                        } else if (tagEquals(header, "TBT2")) {
                            replaceAsset(tbtNextFrame, data);
                        } else if (tagEquals(header, "TBT3")) {
                            replaceAsset(tbtCompactFrame, data);
                        } else if (tagEquals(header, "XRD1")) {
                            replaceAsset(crossroadFrame, data);
                        } else if (tagEquals(header, "LANE")) {
                            replaceAsset(laneFrame, data);
                        } else {
                            throw new Exception("bad asset tag");
                        }
                    }
                }
            } catch (Exception e) {
                mapConnected = false;
                SystemClock.sleep(500L);
            } finally {
                if (socket != null) {
                    try {
                        socket.close();
                    } catch (Exception ignored) {
                    }
                }
            }
        }
        mapConnected = false;
    }

    // ── 렌더 루프 ─────────────────────────────────────────────────────────

    private void renderLoop() {
        long fpsStart = SystemClock.elapsedRealtime();
        long nextFrame = 0L;
        int frames = 0;

        while (running.get()) {
            long now = SystemClock.elapsedRealtime();
            if (now < nextFrame) {
                SystemClock.sleep(Math.min(20L, nextFrame - now));
                continue;
            }
            long due = now + frameIntervalMs;

            JSONObject currentState = state.get();
            // 출력 대상은 FPS 0(패널 끄기) 상태에서도 즉시 바뀌어야 한다.
            applyFrameConfiguration(currentState);
            int requestedFps = Math.max(0, Math.min(15, currentState.optInt("hudFps", 8)));
            if (requestedFps == 0) {
                if (configuredFps != 0) {
                    configuredFps = 0;
                    clearPhoneFrame();
                    if (display.isOpen()) {
                        try {
                            sendBlankFrame();
                            display.setBrightness(1);
                            appliedBrightness = 1;
                            lastJpegSentElapsed = SystemClock.elapsedRealtime();
                        } catch (Exception e) {
                            handleUsbError(e);
                        }
                    }
                }
                SystemClock.sleep(250L);
                nextFrame = SystemClock.elapsedRealtime();
                continue;
            }

            if (configuredFps == 0) {
                appliedBrightness = -1;
            }
            if (requestedFps != configuredFps) {
                configuredFps = requestedFps;
                frameIntervalMs = Math.max(67L, 1000L / configuredFps);
            }

            updateTrip(currentState, now);

            boolean usbReady = ensureUsbReady(now);
            Bitmap usbFrame = null;
            synchronized (assetLock) {
                Bitmap map = mapFrame.get();
                Bitmap tbtCurrent = tbtCurrentFrame.get();
                Bitmap tbtNext = tbtNextFrame.get();
                Bitmap lane = laneFrame.get();
                synchronized (phoneFrameLock) {
                    // phoneFrame 은 화면 출력용이 아니라 USB 회전 전의 논리
                    // 프레임이다. 외부 HUD 전용이 된 뒤에도 이 단계는 남는다.
                    renderPhone(currentState, map, tbtCurrent, tbtNext, lane);
                    if (usbReady) {
                        usbFrame = renderUsbFromPhone();
                    }
                }
            }
            lastRenderElapsed = SystemClock.elapsedRealtime();
            frames++;
            long span = lastRenderElapsed - fpsStart;
            if (span >= 1000L) {
                measuredFps = frames * 1000.0f / span;
                fpsStart = lastRenderElapsed;
                frames = 0;
            }

            if (usbFrame != null) {
                sendUsbFrame(usbFrame, currentState);
            }
            nextFrame = due;
        }
        // The EGL context was created and used on this render thread, so it
        // must also be released here when the service loop stops.
        if (modelWorldGl != null) {
            modelWorldGl.release();
            modelWorldGl = null;
        }
    }

    private void applyFrameConfiguration(JSONObject currentState) {
        int requestedMapFps = Math.max(2, Math.min(5, currentState.optInt("hudMapFps", 5)));
        mapFrameIntervalMs = Math.max(200L, 1000L / requestedMapFps);
        jpegQuality = Math.max(20, Math.min(95, currentState.optInt("hudJpegQuality", 55)));
        configuredTheme = Math.max(0, Math.min(2, currentState.optInt("hudTheme", 0)));
        configuredLanguage = Math.max(0, Math.min(1, currentState.optInt("hudLanguage", 0)));
        configuredRadarInfo = Math.max(0, Math.min(4, currentState.optInt("hudRadarInfo", 4)));
        configuredScreenMode = Math.max(1, Math.min(3, currentState.optInt("hudScreenMode", 1)));
        int requestedOrientation = currentState.optInt("hudOrientation", configuredOrientation) == 2 ? 2 : 0;
        boolean requestedMirror = currentState.optInt("hudMirror", configuredMirror ? 1 : 0) != 0;
        if (requestedOrientation != configuredOrientation) {
            configuredOrientation = requestedOrientation;
            AppPrefs.setOrientation(this, requestedOrientation);
        }
        if (requestedMirror != configuredMirror) {
            configuredMirror = requestedMirror;
            AppPrefs.setMirror(this, requestedMirror);
        }
        configuredOutputMode = Math.max(1, Math.min(3, currentState.optInt("hudOutputMode", 1)));
        tmapIconEnabled = currentState.optInt("hudTmapIcon", 0) != 0;
        junctionMode = Math.max(0, Math.min(2, currentState.optInt("hudJunction", 2)));
        configuredLayoutMode = Math.max(1, Math.min(2, currentState.optInt("hudLayoutMode", 1)));
    }

    private boolean ensureUsbReady(long now) {
        if (display.isOpen()) {
            return true;
        }
        if (now < nextUsbAttemptElapsed) {
            return false;
        }
        nextUsbAttemptElapsed = now + 1000L;
        try {
            if (!display.openOrRequestPermission()) {
                usbStatus = "휴대폰 HUD 실행 · " + display.describeStatus();
                usbConnected = false;
                usbError = false;
                recoverStalledOpen(now);
                return false;
            }
            lastReconnectElapsed = SystemClock.elapsedRealtime();
            usbStatus = "휴대폰 HUD + 외부 USB 연결됨";
            usbConnected = true;
            usbError = false;
            usbErrorStreak = 0;
            return true;
        } catch (Exception e) {
            handleUsbError(e);
            return false;
        }
    }

    /**
     * openDevice/claimInterface 실패는 예외가 아니라 false 로 돌아오므로
     * handleUsbError() 를 타지 않는다. 그래서 clearHalt·포트 재바인딩 같은
     * 복구가 하나도 돌지 않은 채 1초마다 같은 실패만 반복하게 된다.
     * 여기서 그 구멍을 메운다. usbErrorStreak 는 건드리지 않으므로 프레임이
     * 250ms 로 떨어지는 부작용은 생기지 않는다.
     */
    private void recoverStalledOpen(long now) {
        if (display.openFailureStreak() < USB_OPEN_STALL_ATTEMPTS) {
            return;
        }
        if (now < nextOpenStallRecoverElapsed) {
            return;
        }
        nextOpenStallRecoverElapsed = now + USB_OPEN_STALL_COOLDOWN_MS;
        usbStatus = "휴대폰 HUD 실행 · USB 포트 재바인딩 시도";
        boolean reset = UsbPortReset.resetPort(display.deviceNameOrNull());
        display.reset();
        nextUsbAttemptElapsed = now + (reset ? 2500L : 1500L);
    }

    /**
     * 앞차를 자차와 같은 그림으로 그린다. info = {중심 x, 접지 y, 폭}.
     * 원근 축소는 GL 이 계산한 폭을 그대로 쓰고, 높이는 그림 비율로 맞춘다.
     */
    private void drawLeadSprite(Canvas c, Paint p, float[] info,
                                float alpha, boolean braking) {
        float width = info[2];
        if (width < 6f || egoCar == null || egoCar.isRecycled()) {
            return;
        }
        float height = egoCar.getHeight() * width / egoCar.getWidth();
        float left = info[0] - width * 0.5f;
        float bottom = info[1];
        scratchRect.set(left, bottom - height, left + width, bottom);
        p.setShader(null);
        // 앞차는 자차와 같은 그림을 쓰므로 채도를 빼고 살짝 어둡게 해서 구분한다.
        p.setColorFilter(leadTint);
        p.setFilterBitmap(true);
        p.setAlpha(Math.max(0, Math.min(255, Math.round(alpha * 255f))));
        c.drawBitmap(egoCar, null, scratchRect, p);
        p.setColorFilter(null);
        if (braking) {
            // 제동 중이면 뒷면 좌우에 붉은 표시. 그림 자체에는 등이 없다.
            p.setStyle(Paint.Style.FILL);
            p.setColor(Color.rgb(255, 55, 62));
            p.setAlpha(Math.max(0, Math.min(255, Math.round(alpha * 245f))));
            float lampW = width * 0.20f;
            float lampH = Math.max(1.5f, height * 0.16f);
            float lampY = bottom - height * 0.34f;
            c.drawRect(left + width * 0.10f, lampY,
                    left + width * 0.10f + lampW, lampY + lampH, p);
            c.drawRect(left + width * 0.90f - lampW, lampY,
                    left + width * 0.90f, lampY + lampH, p);
        }
        p.setAlpha(255);
    }

    private void sendUsbFrame(Bitmap frame, JSONObject currentState) {
        try {
            int requestedBrightness = Math.max(0,
                    Math.min(100, currentState.optInt("hudBrightness", 65)));
            if (requestedBrightness == 0) {
                requestedBrightness = darkTheme() ? 35 : 65;
            }
            if (requestedBrightness != appliedBrightness) {
                display.setBrightness(requestedBrightness);
                appliedBrightness = requestedBrightness;
            }

            jpegOut.reset();
            frame.compress(Bitmap.CompressFormat.JPEG, jpegQuality, jpegOut);
            byte[] jpeg = jpegOut.toByteArray();
            display.sendJpeg(jpeg);
            lastJpegBytes = jpeg.length;
            lastJpegSentElapsed = SystemClock.elapsedRealtime();
            usbConnected = true;
            usbError = false;
            usbErrorStreak = 0;

            if (display.isUnresponsive(15000L)) {
                usbStatus = "패널 무응답 · 재초기화 (휴대폰 HUD 정상)";
                display.close();
                appliedBrightness = -1;
                usbErrorStreak++;
                nextUsbAttemptElapsed = SystemClock.elapsedRealtime() + 600L;
            }
        } catch (Exception e) {
            handleUsbError(e);
        }
    }

    private void handleUsbError(Exception e) {
        usbConnected = false;
        usbError = true;
        usbErrorStreak++;
        display.recoverAfterError();
        display.close();
        appliedBrightness = -1;
        nextUsbAttemptElapsed = SystemClock.elapsedRealtime() + 500L;
        if (usbErrorStreak >= USB_RESET_AFTER_ERRORS) {
            usbStatus = "USB 복구 중 · 휴대폰 HUD는 계속 실행";
            boolean reset = UsbPortReset.resetPort(display.deviceNameOrNull());
            display.reset();
            if (!reset) {
                usbStatus = "USB 오류 · " + e.getMessage() + " (휴대폰 HUD 정상)";
            }
            nextUsbAttemptElapsed = SystemClock.elapsedRealtime() + 2500L;
        } else {
            usbStatus = "USB 오류 · " + e.getMessage() + " (휴대폰 HUD 정상)";
        }
        if (usbErrorStreak >= USB_SLOWDOWN_AFTER_ERRORS && frameIntervalMs < 250L) {
            frameIntervalMs = 250L;
        }
    }

    // ── 렌더 ──────────────────────────────────────────────────────────────

    /**
     * 세로 방향 출력 버퍼를 한 번만 잡고 계속 재사용한다. 회전/미러는
     * 캔버스 행렬로 처리하므로 v0.18 처럼 회전 복사본을 새로 만들 필요가 없다.
     */
    private Canvas beginUsbFrame() {
        if (outFrame == null || outFrame.isRecycled()) {
            outFrame = Bitmap.createBitmap(HEIGHT, WIDTH, Bitmap.Config.RGB_565);
            outCanvas = new Canvas(outFrame);
        }
        outMatrix.reset();
        outMatrix.setScale(configuredMirror ? -1f : 1f, 1f);
        outMatrix.postRotate(configuredOrientation == 2 ? 90f : -90f);
        scratchRect.set(0f, 0f, WIDTH, HEIGHT);
        outMatrix.mapRect(scratchRect);
        outMatrix.postTranslate(-scratchRect.left, -scratchRect.top);
        outCanvas.setMatrix(outMatrix);
        return outCanvas;
    }

    /** USB 회전 전의 논리 가로 프레임. 화면 출력용이 아니다. */
    private Canvas beginPhoneFrame() {
        if (phoneFrame == null || phoneFrame.isRecycled()) {
            phoneFrame = Bitmap.createBitmap(WIDTH, HEIGHT, Bitmap.Config.RGB_565);
            phoneCanvas = new Canvas(phoneFrame);
        }
        phoneCanvas.setMatrix(null);
        return phoneCanvas;
    }

    private void clearPhoneFrame() {
        synchronized (phoneFrameLock) {
            Canvas c = beginPhoneFrame();
            c.drawColor(Color.BLACK);
        }
    }

    /** c2_lastSupper 의 S9 전체화면 Activity 에 최신 g_hud 프레임을 전달한다. */
    static boolean drawFullscreenFrame(Canvas canvas, int width, int height) {
        HudService service = activeInstance;
        if (service == null || width <= 0 || height <= 0) {
            return false;
        }
        synchronized (service.phoneFrameLock) {
            Bitmap frame = service.phoneFrame;
            if (frame == null || frame.isRecycled()) {
                return false;
            }
            canvas.drawColor(Color.BLACK);
            float scale = Math.min((float) width / frame.getWidth(),
                    (float) height / frame.getHeight());
            float drawWidth = frame.getWidth() * scale;
            float drawHeight = frame.getHeight() * scale;
            float left = (width - drawWidth) * 0.5f;
            float top = (height - drawHeight) * 0.5f;
            service.phoneViewport.set(left, top, left + drawWidth, top + drawHeight);
            canvas.drawBitmap(frame, null, service.phoneViewport, service.phonePreviewPaint);
            return true;
        }
    }

    /** 패널을 끌 때 보내는 검은 프레임 한 장. */
    private void sendBlankFrame() throws Exception {
        Canvas c = beginUsbFrame();
        c.drawColor(Color.BLACK);
        jpegOut.reset();
        outFrame.compress(Bitmap.CompressFormat.JPEG, 40, jpegOut);
        byte[] jpeg = jpegOut.toByteArray();
        display.sendJpeg(jpeg);
        lastJpegBytes = jpeg.length;
    }

    private Bitmap renderUsbFromPhone() {
        Canvas c = beginUsbFrame();
        c.drawBitmap(phoneFrame, 0f, 0f, phonePreviewPaint);
        return outFrame;
    }

    private void renderPhone(JSONObject s, Bitmap map, Bitmap tbtCurrent, Bitmap tbtNext, Bitmap lane) {
        Canvas c = beginPhoneFrame();
        drawFrame(c, s, map, tbtCurrent, tbtNext, lane);
    }

    private void drawFrame(Canvas c, JSONObject s, Bitmap map, Bitmap tbtCurrent,
                           Bitmap tbtNext, Bitmap lane) {
        Paint p = paint;
        p.reset();
        p.setAntiAlias(true);
        frameDark = darkTheme();
        c.drawColor(Color.rgb(5, 8, 12));

        drawDriving(c, p, s);

        if (configuredLayoutMode == 1) {
            JSONObject l = layout(s);
            int save = beginElement(c, l, "system", 1824f, 231f);
            if (configuredOutputMode == 3) {
                drawS9Remote(c, p);
            } else if (configuredOutputMode == 2) {
                drawSystemDebug(c, p, s);
            } else {
                drawSystem(c, p, s);
            }
            c.restoreToCount(save);
        }

        if (configuredScreenMode == 2) {
            drawDebugRight(c, p, s);
        } else if (configuredScreenMode == 3) {
            drawTripRight(c, p, s);
        } else {
            drawMap(c, p, s, map, tbtCurrent, tbtNext, lane);
        }
        applyThemeOverlay(c, p);
        // 순정 계기판 경고는 우측 TMAP 위에 독립된 흰색 팝업으로 표시한다.
        // 야간 지도 마스크 뒤에 그려 항상 선명한 흰색을 유지한다.
        drawOemWarningPopup(c, p, s);
    }

    private boolean eonStale() {
        long last = lastEonRxElapsed;
        return last == 0L || SystemClock.elapsedRealtime() - last > EON_STALE_MS;
    }

    private void drawDriving(Canvas c, Paint p, JSONObject s) {
        JSONObject l = layout(s);
        boolean stale = eonStale();
        boolean enabled = !stale && s.optBoolean("enabled", false);

        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        int driveBg = lc(l, "driveBg", frameDark ? Color.rgb(22, 28, 36) : Color.rgb(226, 229, 231));
        p.setColor(driveBg);
        c.drawRect(0f, 0f, DRIVE_RIGHT, 462f, p);

        int roadTop = lc(l, "roadTop",
                frameDark ? Color.rgb(50, 58, 68) : Color.rgb(210, 215, 219));
        int roadBottom = lc(l, "roadBottom",
                frameDark ? Color.rgb(62, 72, 84) : Color.rgb(198, 204, 209));
        int pathColor = lc(l, "pathColor",
                frameDark ? Color.rgb(40, 150, 255) : Color.rgb(24, 126, 224));

        int worldSave = c.save();
        // Canvas 렌더러가 없어졌으므로 GL 을 끄는 스위치도 없앴다. 끌 수 있게
        // 두면 주행 패널이 배경색만 남는다.
        boolean glDrawn = false;
        if (!stale) {
            if (modelWorldGl == null) {
                modelWorldGl = new ModelWorldGL(this);
            }
            glDrawn = modelWorldGl.draw(c, p, s, enabled, driveBg, roadTop,
                    roadBottom, pathColor, frameDark,
                    (float) s.optDouble("hudRoadZ", 100d),
                    (float) s.optDouble("pitch", 0d),
                    (float) s.optDouble("hudPitchDyn", 60d),
                    (float) s.optDouble("calibPitch", 0d),
                    egoCar != null && !egoCar.isRecycled(),
                    s.optInt("hudGuardrail", 1) != 0,
                    Math.max(0, Math.min(100, s.optInt("hudHaze", 55))));
            if (glDrawn && egoCar != null && !egoCar.isRecycled()) {
                // 앞차도 자차와 같은 그림으로. 먼 차부터 그려 근경이 덮게 한다.
                for (int leadIndex = 1; leadIndex >= 0; leadIndex--) {
                    if (!modelWorldGl.leadSprite(leadIndex, leadSpriteInfo)) {
                        continue;
                    }
                    drawLeadSprite(c, p, leadSpriteInfo,
                            modelWorldGl.leadSpriteAlpha(leadIndex),
                            modelWorldGl.leadSpriteBraking(leadIndex));
                }
            }
            if (glDrawn && egoCar != null && !egoCar.isRecycled()) {
                // Preserve the approved ego-car artwork in the GL preview.
                float carWidth = 78f;
                float carHeight = egoCar.getHeight() * carWidth / egoCar.getWidth();
                scratchRect.set(DRIVE_CX - carWidth * 0.5f, 433f - carHeight,
                        DRIVE_CX + carWidth * 0.5f, 433f);
                p.setAlpha(255);
                p.setFilterBitmap(true);
                c.drawBitmap(egoCar, null, scratchRect, p);
            }
        }

        if (!glDrawn) {
            // Canvas 폴백 렌더러는 제거됐다. GL 이 못 그린 프레임(EON 정지, EGL 실패,
            // 경로 점 부족)에서는 주행 패널을 배경색으로 지운다. 지우지 않으면
            // 재사용 비트맵에 직전 장면이 남아 화면이 굳은 것처럼 보인다.
            p.setShader(null);
            p.setStyle(Paint.Style.FILL);
            p.setAlpha(255);
            p.setColor(ModelWorldGL.blend(driveBg, Color.BLACK,
                    frameDark ? 0.15f : 0.10f));
            c.drawRect(0f, ModelWorldGL.TOP, DRIVE_RIGHT, ModelWorldGL.BOTTOM, p);
        }
        c.restoreToCount(worldSave);

        int blinkerSave = beginElement(c, l, "blinkers", DRIVE_CX, 386f);
        drawBlinkers(c, p, s, stale);
        c.restoreToCount(blinkerSave);

        drawSkyBand(c, p);

        // Existing lamp art stays unchanged; move the complete row inside
        // the panel, centred on the outside-temperature/range row.
        int save = beginElement(c, l, "lights", 70f, 36f);
        drawLights(c, p, s);
        c.restoreToCount(save);

        int save2 = beginElement(c, l, "prnd", 90f, 116f);
        drawGearAndCoolant(c, p, s);
        c.restoreToCount(save2);

        int save3 = beginElement(c, l, "speed", DRIVE_CX, SPEED_BASELINE);
        drawSpeed(c, p, stale ? -1 : s.optInt("speed", 0));
        c.restoreToCount(save3);

        int saveRpm = beginElement(c, l, "rpm", DRIVE_CX, 118f);
        drawRpm(c, p, stale ? -1 : s.optInt("rpm", -1), lv(l, "rpmRedline", 6500f));
        c.restoreToCount(saveRpm);

        int modeSave = beginElement(c, l, "mode", 938f, 116f);
        drawModeAndEta(c, p, s);
        c.restoreToCount(modeSave);
        int rangeSave = beginElement(c, l, "range", 932f, 44f);
        drawRange(c, p, s);
        drawOutsideTemp(c, p, s);
        c.restoreToCount(rangeSave);

        p.setStyle(Paint.Style.STROKE);
        p.setColor(hairline());
        p.setStrokeWidth(1f);
        c.drawLine(18f, 129f, 934f, 129f, p);

        int save4 = beginElement(c, l, "wheel", 70f, 171f);
        int steerWarning = s.optBoolean("steerFaultPermanent", false) ? 2
                : (s.optBoolean("steerFaultTemporary", false) ? 1 : 0);
        drawSteeringWheel(c, p, 70f, 171f, (float) s.optDouble("steer", 0d),
                enabled, steerWarning);
        c.restoreToCount(save4);

        int save5 = beginElement(c, l, "set", DRIVE_CX, 171f);
        drawSetSpeed(c, p, DRIVE_CX, 171f, s.optInt("set", 0), enabled, s);
        drawApplySpeed(c, p, s);
        c.restoreToCount(save5);

        int save6 = beginElement(c, l, "camera", 882f, 171f);
        int bumpDist = stale ? 0 : (int) Math.round(s.optDouble("bumpDist", 0d));
        if (bumpDist > 0) {
            // EON onroad.cc drawSpeedLimit 과 같은 규칙: 방지턱이 있으면
            // 과속카메라 대신 이 자리를 쓴다.
            drawBumpIcon(c, p, 882f, 171f, bumpDist);
        } else if (!stale) {
            // packet 의 camera(=camLimitSpeed 또는 sectionLimitSpeed) 는
            // remote_hud._packet 에서 이미 EON drawSpeedLimit 과 같은 우선순위로
            // 골라 보낸다. 도로 제한속도(limit) 는 여기에 그리지 않는다.
            drawCamera(c, p, 882f, 171f, s.optInt("camera", 0), s.optInt("cameraDist", 0),
                    s.optBoolean("cameraSection", false));
        }
        c.restoreToCount(save6);
        skyBand = false;

        // 세로 기준점을 TPMS 카드와 같은 415 로 맞춘다. 서로 다른 기준점을 쓰면
        // 순정 화면의 위젯 배율 역보정에서 그 차이만큼 사이가 벌어진다.
        int nooSave = beginElement(c, l, "noo", NOO_CX, 415f);
        if (!stale) {
            drawNooTurn(c, p, s);
        }
        c.restoreToCount(nooSave);

        int save7 = beginElement(c, l, "lead", 82f, 415f);
        drawLeadCard(c, p, s.optJSONObject("lead"));
        c.restoreToCount(save7);

        int save8 = beginElement(c, l, "tpms", 865f, 415f);
        drawTpms(c, p, s);
        c.restoreToCount(save8);

        int alertSave = beginElement(c, l, "alert", DRIVE_CX, 336f);
        JSONObject alertBox = stale ? null : s.optJSONObject("alert");
        if (stale) {
            resetCountdown();
        } else if (alertBox == null) {
            // openpilot 이벤트 알림이 우선. 없을 때만 회전 카운트다운을 같은 자리에 띄운다.
            alertBox = turnCountdownAlert(s);
        }
        drawAlert(c, p, alertBox);

        if (stale) {
            p.setShader(null);
            p.setStyle(Paint.Style.FILL);
            p.setColor(Color.argb(210, 26, 30, 34));
            scratchRect.set(307f, 296f, 645f, 356f);
            c.drawRoundRect(scratchRect, 10f, 10f, p);
            text(c, p, lang("EON 연결 끊김", "EON LINK LOST"), DRIVE_CX, 322f, 24f,
                    Color.rgb(255, 148, 118), Paint.Align.CENTER);
            text(c, p, lastEonAddress, DRIVE_CX, 345f, 15f, Color.rgb(186, 194, 200), Paint.Align.CENTER);
        }
        c.restoreToCount(alertSave);

        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(2f);
        p.setColor(cardEdge());
        scratchRect.set(2f, 2f, DRIVE_RIGHT - 2f, 458f);
        c.drawRoundRect(scratchRect, 18f, 18f, p);
    }

    /**
     * openpilot 이벤트 알림. EON 직접모드 renderer.py `_draw_alert` 와 같은
     * 소스(controlsState alertText1/2 · alertStatus · alertSize)를 쓴다.
     * 다만 이쪽 주행패널은 배경이 밝아서 흰 글씨가 안 읽히므로,
     * 어두운 반투명 박스를 깔고 그 위에 얹는다.
     */
    private void drawAlert(Canvas c, Paint p, JSONObject alert) {
        if (alert == null) {
            return;
        }
        String text1 = collapse(alert.optString("text1", ""));
        String text2 = collapse(alert.optString("text2", ""));
        if (text1.length() == 0) {
            text1 = text2;
            text2 = "";
        }
        if (text1.length() == 0) {
            return;
        }

        String status = alert.optString("status", "").toLowerCase(Locale.US);
        boolean critical = status.contains("critical");
        boolean prompt = status.contains("prompt") || status.contains("warning");
        int titleColor = critical ? Color.rgb(255, 82, 96)
                : prompt ? Color.rgb(255, 174, 82) : Color.WHITE;

        boolean full = alert.optString("size", "").toLowerCase(Locale.US).contains("full");
        float maxWidth = 860f;
        float titleSize = full ? 38f : 30f;
        p.setTypeface(Typeface.create("sans", Typeface.BOLD));
        p.setTextSize(titleSize);
        while (titleSize > 20f && p.measureText(text1) > maxWidth) {
            titleSize -= 2f;
            p.setTextSize(titleSize);
        }
        float titleWidth = p.measureText(text1);
        float detailSize = 20f;
        float detailWidth = 0f;
        if (text2.length() > 0) {
            p.setTextSize(detailSize);
            while (detailSize > 14f && p.measureText(text2) > maxWidth) {
                detailSize -= 2f;
                p.setTextSize(detailSize);
            }
            detailWidth = p.measureText(text2);
        }

        float boxWidth = Math.min(920f, Math.max(titleWidth, detailWidth) + 56f);
        float boxHeight = text2.length() > 0 ? titleSize + detailSize + 44f : titleSize + 34f;
        float cx = DRIVE_CX;
        float cy = 336f;
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.argb(226, 20, 24, 28));
        scratchRect.set(cx - boxWidth / 2f, cy - boxHeight / 2f,
                cx + boxWidth / 2f, cy + boxHeight / 2f);
        c.drawRoundRect(scratchRect, 12f, 12f, p);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(2f);
        p.setColor(titleColor);
        c.drawRoundRect(scratchRect, 12f, 12f, p);

        if (text2.length() > 0) {
            text(c, p, text1, cx, cy - 4f, titleSize, titleColor, Paint.Align.CENTER);
            text(c, p, text2, cx, cy + detailSize + 12f, detailSize,
                    Color.rgb(228, 233, 237), Paint.Align.CENTER);
        } else {
            text(c, p, text1, cx, cy + titleSize * 0.36f, titleSize, titleColor, Paint.Align.CENTER);
        }
    }

    /** 제네시스 순정 경고를 우측 TMAP 중앙의 흰색 팝업으로 표시한다. */
    private void drawOemWarningPopup(Canvas c, Paint p, JSONObject s) {
        if (eonStale()) {
            return;
        }
        if (s.optBoolean("aebSystemFault", false)) {
            drawAebSystemPopup(c, p);
            return;
        }
        if (s.optBoolean("blindSpotSystemFault", false)) {
            drawBlindSpotSystemPopup(c, p);
            return;
        }
        JSONObject doors = s.optJSONObject("doors");
        JSONObject windows = s.optJSONObject("windows");
        // The factory cluster only reminds the driver about open glass while
        // parked.  Never let a deliberately lowered window cover TMAP while
        // driving; doors, hood and trunk remain unconditional safety alerts.
        JSONObject visibleWindows = "P".equals(s.optString("gear", "")) ? windows : null;
        if (hasVehicleOpening(doors, visibleWindows)) {
            drawVehicleOpenPopup(c, p, doors, visibleWindows);
            return;
        }
        if (s.optBoolean("lowFuelWarning", false)) {
            drawLowFuelPopup(c, p, s.optDouble("distanceToEmpty", -1d));
            return;
        }
        drawParkingSensorPopup(c, p, s);
    }

    private void drawWhiteWarningPanel(Canvas c, Paint p, float cx) {
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.argb(92, 0, 0, 0));
        scratchRect.set(cx - 176f, 124f, cx + 184f, 402f);
        c.drawRoundRect(scratchRect, 13f, 13f, p);
        p.setColor(Color.rgb(250, 251, 252));
        scratchRect.set(cx - 180f, 118f, cx + 180f, 396f);
        c.drawRoundRect(scratchRect, 13f, 13f, p);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(4f);
        p.setColor(Color.rgb(154, 163, 171));
        c.drawRoundRect(scratchRect, 13f, 13f, p);
    }

    /** Factory-cluster AEB/FCA failure message (not an active braking event). */
    private void drawAebSystemPopup(Canvas c, Paint p) {
        final float cx = mapCenterX();
        drawWhiteWarningPanel(c, p, cx);
        drawWarningTriangle(c, p, cx, 165f, 27f);
        text(c, p, lang("긴급제동 시스템을", "CHECK EMERGENCY"),
                cx, 235f, 29f, Color.rgb(35, 39, 43), Paint.Align.CENTER);
        text(c, p, lang("점검하십시오", "BRAKING SYSTEM"),
                cx, 272f, 29f, Color.rgb(35, 39, 43), Paint.Align.CENTER);

        // 순정 경고의 충돌 차량 표식을 작은 벡터로 재현한다.
        p.setShader(null);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(7f);
        p.setStrokeCap(Paint.Cap.ROUND);
        p.setColor(Color.rgb(226, 159, 22));
        scratchRect.set(cx - 72f, 318f, cx - 14f, 353f);
        c.drawRoundRect(scratchRect, 9f, 9f, p);
        scratchRect.set(cx + 14f, 318f, cx + 72f, 353f);
        c.drawRoundRect(scratchRect, 9f, 9f, p);
        c.drawLine(cx - 18f, 335f, cx + 18f, 335f, p);
        p.setStrokeCap(Paint.Cap.BUTT);
        drawCollisionStar(c, p, cx, 311f, 14f);
    }

    /** Factory blind-spot/LCA failure message and rear-radar pictogram. */
    private void drawBlindSpotSystemPopup(Canvas c, Paint p) {
        final float cx = mapCenterX();
        drawWhiteWarningPanel(c, p, cx);
        drawWarningTriangle(c, p, cx, 158f, 25f);
        text(c, p, lang("후측방 경보 시스템을", "CHECK BLIND-SPOT"),
                cx, 225f, 28f, Color.rgb(35, 39, 43), Paint.Align.CENTER);
        text(c, p, lang("점검하십시오", "WARNING SYSTEM"),
                cx, 261f, 28f, Color.rgb(35, 39, 43), Paint.Align.CENTER);

        final int amber = Color.rgb(226, 146, 17);
        final float cy = 330f;
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(amber);
        scratchRect.set(cx - 19f, cy - 31f, cx + 19f, cy + 33f);
        c.drawRoundRect(scratchRect, 9f, 9f, p);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(5f);
        p.setStrokeCap(Paint.Cap.ROUND);
        p.setColor(amber);
        scratchRect.set(cx - 69f, cy - 29f, cx - 13f, cy + 29f);
        c.drawArc(scratchRect, 102f, 156f, false, p);
        scratchRect.set(cx + 13f, cy - 29f, cx + 69f, cy + 29f);
        c.drawArc(scratchRect, 282f, 156f, false, p);
        p.setStrokeWidth(3f);
        scratchRect.set(cx - 88f, cy - 43f, cx - 4f, cy + 43f);
        c.drawArc(scratchRect, 112f, 136f, false, p);
        scratchRect.set(cx + 4f, cy - 43f, cx + 88f, cy + 43f);
        c.drawArc(scratchRect, 292f, 136f, false, p);
        p.setStrokeCap(Paint.Cap.BUTT);
    }

    /** Show every open door, window, hood and trunk in the large TMAP warning area. */
    private void drawVehicleOpenPopup(Canvas c, Paint p, JSONObject doors,
                                      JSONObject windows) {
        final float cx = mapCenterX();
        final float cy = 288f;
        drawWhiteWarningPanel(c, p, cx);
        boolean sideDoorOpen = hasOpenDoor(doors);
        boolean windowOpen = hasOpenWindow(windows);
        boolean hoodOpen = doors != null && doors.optBoolean("hood", false);
        boolean trunkOpen = doors != null && doors.optBoolean("trunk", false);
        int typeCount = (sideDoorOpen ? 1 : 0) + (windowOpen ? 1 : 0)
                + (hoodOpen ? 1 : 0) + (trunkOpen ? 1 : 0);
        String titleKo;
        String titleEn;
        if (typeCount > 1) {
            titleKo = "열림 상태를 확인하십시오";
            titleEn = "CHECK OPENINGS";
        } else if (sideDoorOpen) {
            titleKo = "문이 열려 있습니다";
            titleEn = "DOOR OPEN";
        } else if (windowOpen) {
            titleKo = "창문이 열려 있습니다";
            titleEn = "WINDOW OPEN";
        } else if (hoodOpen) {
            titleKo = "보닛이 열려 있습니다";
            titleEn = "HOOD OPEN";
        } else {
            titleKo = "트렁크가 열려 있습니다";
            titleEn = "TRUNK OPEN";
        }
        text(c, p, lang(titleKo, titleEn),
                cx, 163f, 29f, Color.rgb(35, 39, 43), Paint.Align.CENTER);

        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.rgb(204, 211, 217));
        scratchRect.set(cx - 34f, cy - 67f, cx + 34f, cy + 70f);
        c.drawRoundRect(scratchRect, 15f, 15f, p);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(3f);
        p.setColor(Color.rgb(71, 79, 86));
        c.drawRoundRect(scratchRect, 15f, 15f, p);
        scratchRect.set(cx - 25f, cy - 43f, cx + 25f, cy + 35f);
        c.drawRoundRect(scratchRect, 10f, 10f, p);
        c.drawLine(cx - 29f, cy - 20f, cx + 29f, cy - 20f, p);
        c.drawLine(cx - 29f, cy + 26f, cx + 29f, cy + 26f, p);

        // The front of the vehicle points upward.  Hood and trunk panels
        // hinge away from the body so their position stays unmistakable.
        p.setColor(Color.rgb(232, 48, 58));
        p.setStyle(Paint.Style.FILL);
        if (hoodOpen) {
            scratchPath.rewind();
            scratchPath.moveTo(cx - 31f, cy - 56f);
            scratchPath.lineTo(cx - 42f, cy - 94f);
            scratchPath.lineTo(cx + 42f, cy - 94f);
            scratchPath.lineTo(cx + 31f, cy - 56f);
            scratchPath.close();
            c.drawPath(scratchPath, p);
        }
        if (trunkOpen) {
            scratchPath.rewind();
            scratchPath.moveTo(cx - 31f, cy + 56f);
            scratchPath.lineTo(cx - 42f, cy + 94f);
            scratchPath.lineTo(cx + 42f, cy + 94f);
            scratchPath.lineTo(cx + 31f, cy + 56f);
            scratchPath.close();
            c.drawPath(scratchPath, p);
        }
        if (doors != null && doors.optBoolean("fl", false)) {
            drawOpenDoorLeaf(c, p, cx - 31f, cy - 43f, cy - 13f,
                    cx - 78f, cy - 57f, cy - 23f);
        }
        if (doors != null && doors.optBoolean("fr", false)) {
            drawOpenDoorLeaf(c, p, cx + 31f, cy - 43f, cy - 13f,
                    cx + 78f, cy - 57f, cy - 23f);
        }
        if (doors != null && doors.optBoolean("rl", false)) {
            drawOpenDoorLeaf(c, p, cx - 31f, cy + 8f, cy + 38f,
                    cx - 78f, cy + 22f, cy + 56f);
        }
        if (doors != null && doors.optBoolean("rr", false)) {
            drawOpenDoorLeaf(c, p, cx + 31f, cy + 8f, cy + 38f,
                    cx + 78f, cy + 22f, cy + 56f);
        }

        // Open glass is blue and remains visible even when its door is also
        // open.  Each bar follows the matching FL/FR/RL/RR side window.
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeCap(Paint.Cap.ROUND);
        p.setStrokeWidth(8f);
        p.setColor(Color.rgb(36, 160, 224));
        if (windows != null && windows.optBoolean("fl", false)) {
            c.drawLine(cx - 25f, cy - 40f, cx - 25f, cy - 16f, p);
        }
        if (windows != null && windows.optBoolean("fr", false)) {
            c.drawLine(cx + 25f, cy - 40f, cx + 25f, cy - 16f, p);
        }
        if (windows != null && windows.optBoolean("rl", false)) {
            c.drawLine(cx - 25f, cy + 7f, cx - 25f, cy + 32f, p);
        }
        if (windows != null && windows.optBoolean("rr", false)) {
            c.drawLine(cx + 25f, cy + 7f, cx + 25f, cy + 32f, p);
        }
        p.setStrokeCap(Paint.Cap.BUTT);
    }

    /** Factory low-fuel warning, retaining the cluster's remaining range. */
    private void drawLowFuelPopup(Canvas c, Paint p, double distanceToEmpty) {
        final float cx = mapCenterX();
        final int amber = Color.rgb(232, 158, 18);
        drawWhiteWarningPanel(c, p, cx);
        drawWarningTriangle(c, p, cx, 158f, 25f);
        text(c, p, lang("연료가 부족합니다", "LOW FUEL"),
                cx, 232f, 30f, Color.rgb(35, 39, 43), Paint.Align.CENTER);
        if (distanceToEmpty >= 0d && distanceToEmpty < 1000d) {
            text(c, p, lang("주행가능거리 ", "RANGE ")
                            + Math.round(distanceToEmpty) + " km",
                    cx, 270f, 20f, Color.rgb(91, 98, 104), Paint.Align.CENTER);
        }

        p.setShader(null);
        p.setColor(amber);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(7f);
        scratchRect.set(cx - 34f, 300f, cx + 14f, 365f);
        c.drawRoundRect(scratchRect, 7f, 7f, p);
        scratchRect.set(cx - 23f, 310f, cx + 3f, 329f);
        c.drawRoundRect(scratchRect, 3f, 3f, p);
        scratchPath.rewind();
        scratchPath.moveTo(cx + 14f, 315f);
        scratchPath.lineTo(cx + 35f, 326f);
        scratchPath.lineTo(cx + 35f, 354f);
        scratchPath.quadTo(cx + 35f, 364f, cx + 25f, 364f);
        scratchPath.lineTo(cx + 14f, 364f);
        c.drawPath(scratchPath, p);
        c.drawLine(cx - 42f, 368f, cx + 20f, 368f, p);
    }

    private void drawWarningTriangle(Canvas c, Paint p, float cx, float cy, float radius) {
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.rgb(255, 190, 38));
        scratchPath.rewind();
        scratchPath.moveTo(cx, cy - radius);
        scratchPath.lineTo(cx - radius * 0.92f, cy + radius * 0.75f);
        scratchPath.lineTo(cx + radius * 0.92f, cy + radius * 0.75f);
        scratchPath.close();
        c.drawPath(scratchPath, p);
        p.setColor(Color.rgb(54, 57, 59));
        scratchRect.set(cx - 3f, cy - 12f, cx + 3f, cy + 7f);
        c.drawRoundRect(scratchRect, 3f, 3f, p);
        c.drawCircle(cx, cy + 15f, 3.5f, p);
    }

    private void drawCollisionStar(Canvas c, Paint p, float cx, float cy, float radius) {
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.rgb(226, 159, 22));
        scratchPath.rewind();
        for (int i = 0; i < 16; i++) {
            double a = -Math.PI / 2d + i * Math.PI / 8d;
            float r = (i & 1) == 0 ? radius : radius * 0.42f;
            float x = cx + (float) Math.cos(a) * r;
            float y = cy + (float) Math.sin(a) * r;
            if (i == 0) scratchPath.moveTo(x, y); else scratchPath.lineTo(x, y);
        }
        scratchPath.close();
        c.drawPath(scratchPath, p);
    }

    /**
     * Mirror the Genesis PAS11 six-zone parking display.  PAS11 levels are
     * preserved as 0..7; higher levels are rendered with increasing urgency.
     * Returns true only when a valid PAS11 frame contains an active sector.
     */
    private boolean drawParkingSensorPopup(Canvas c, Paint p, JSONObject s) {
        JSONObject sensors = s.optJSONObject("parkingSensors");
        if (sensors == null || !sensors.optBoolean("valid", false)) {
            return false;
        }
        int fl = parkingLevel(sensors, "fl");
        int fc = parkingLevel(sensors, "fc");
        int fr = parkingLevel(sensors, "fr");
        int rl = parkingLevel(sensors, "rl");
        int rc = parkingLevel(sensors, "rc");
        int rr = parkingLevel(sensors, "rr");
        if ((fl | fc | fr | rl | rc | rr) == 0) {
            return false;
        }

        final float cx = mapCenterX();
        final float cy = 270f;
        drawWhiteWarningPanel(c, p, cx);
        text(c, p, lang("주차 거리 경고", "PARKING DISTANCE WARNING"),
                cx, 153f, 22f, Color.rgb(48, 53, 57), Paint.Align.CENTER);

        // Simplified top view keeps the six PAS sectors immediately legible.
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.rgb(210, 216, 221));
        scratchRect.set(cx - 25f, cy - 39f, cx + 25f, cy + 48f);
        c.drawRoundRect(scratchRect, 13f, 13f, p);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(2f);
        p.setColor(Color.rgb(82, 91, 98));
        c.drawRoundRect(scratchRect, 13f, 13f, p);
        scratchRect.set(cx - 17f, cy - 23f, cx + 17f, cy + 22f);
        c.drawRoundRect(scratchRect, 8f, 8f, p);
        p.setStrokeWidth(4f);
        c.drawLine(cx - 22f, cy - 19f, cx - 29f, cy - 12f, p);
        c.drawLine(cx + 22f, cy - 19f, cx + 29f, cy - 12f, p);

        // Android arc angles: 270 degrees is forward/up, 90 is rear/down.
        drawParkingSector(c, p, cx, cy, 207f, 37f, fl);
        drawParkingSector(c, p, cx, cy, 251.5f, 37f, fc);
        drawParkingSector(c, p, cx, cy, 296f, 37f, fr);
        drawParkingSector(c, p, cx, cy, 27f, 37f, rr);
        drawParkingSector(c, p, cx, cy, 71.5f, 37f, rc);
        drawParkingSector(c, p, cx, cy, 116f, 37f, rl);
        return true;
    }

    private int parkingLevel(JSONObject sensors, String key) {
        return Math.max(0, Math.min(7, sensors.optInt(key, 0)));
    }

    private void drawParkingSector(Canvas c, Paint p, float cx, float cy,
                                   float start, float sweep, int level) {
        if (level <= 0) {
            return;
        }
        int color = level >= 5 ? Color.rgb(238, 70, 62)
                : (level >= 3 ? Color.rgb(255, 183, 48) : Color.rgb(98, 207, 112));
        // Bring high urgency closer to the vehicle while retaining a larger,
        // easy-to-see stroke on the small external panel.
        float radius = level >= 5 ? 54f : (level >= 3 ? 63f : 72f);
        scratchRect.set(cx - radius, cy - radius, cx + radius, cy + radius);
        p.setShader(null);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeCap(Paint.Cap.ROUND);
        p.setStrokeWidth(level >= 5 ? 9f : 7f);
        p.setColor(color);
        c.drawArc(scratchRect, start, sweep, false, p);
        p.setStrokeCap(Paint.Cap.BUTT);
    }

    /**
     * 회전 카운트다운 팝업. carrot-wip carrot_serv.py `_update_countdown_alert()` 와
     * 같은 규칙이다. 다만 EON 이 아니라 앱에서 계산하므로 remote_hud.py 는 손대지 않는다.
     *
     *   남은초 = (turnDist - v) / v          (v: m/s)
     *   시작초 = clamp(kph/10 + 1, 6, 11)    (빠를수록 일찍 시작)
     *
     * 거리(turnDist)는 티맵 guidance_current.distance_m 이 그대로 온 값이라
     * 튀는 경우가 있어 단조감소로 묶는다.
     */
    private JSONObject turnCountdownAlert(JSONObject s) {
        JSONObject navi = s.optJSONObject("navi");
        if (navi == null || !navi.optBoolean("active", false)
                || !navi.optBoolean("guidanceLive", false)) {
            resetCountdown();
            return null;
        }

        // NOO 게이트. 티맵이 회전을 안내하는 것만으로는 뜨지 않고,
        // desire_helper 가 실제로 회전 조향을 잡았을 때(nooTurnDirection != 0)
        // 또는 lateral_planner 가 지도경로를 섞기 시작했을 때(nooMapBlend > 0) 뜬다.
        // 운전자가 반대 토크/지시등/브레이크로 취소하면 noo_driver_cancel 로
        // 방향이 0 이 되므로 팝업도 같이 사라진다.
        boolean nooOn = s.optBoolean("nooMode", false) || s.optInt("atcMode", 0) != 0;
        int nooDir = s.optInt("atcDirection", 0);
        double nooBlend = s.optDouble("atcBlend", 0d);
        if (!nooOn || (nooDir == 0 && nooBlend <= 0.005d)) {
            resetCountdown();
            return null;
        }
        int dist = navi.optInt("turnDist", -1);
        if (dist <= 0) {
            resetCountdown();
            return null;
        }
        int kph = Math.max(0, s.optInt("speed", 0));
        float v = kph / 3.6f;
        if (v < 1.0f) {
            // 정차 중엔 "남은초"가 무한대로 튄다. 카운트 자체를 하지 않는다.
            resetCountdown();
            return null;
        }

        int sec = (int) ((Math.max(dist - v, 1f) / v) + 0.5f);
        if (sec > 11) {
            resetCountdown();
            return null;
        }
        if (sec > countdownLastSec) {
            sec = countdownLastSec;
        }
        countdownLastSec = sec;

        int maxSec = Math.min(11, Math.max(6, kph / 10 + 1));
        if (sec > maxSec) {
            return null;
        }

        long now = SystemClock.elapsedRealtime();
        if (sec <= 0) {
            if (countdownZeroAtMs == 0L) {
                countdownZeroAtMs = now;
            }
            if (now - countdownZeroAtMs > 800L) {
                return null;
            }
        } else {
            countdownZeroAtMs = 0L;
        }

        // 방향은 NOO 가 실제로 잡은 쪽을 우선한다(티맵 turn_type 보다 신뢰도가 높다).
        String turn = nooDir < 0 ? lang("좌회전", "LEFT")
                : nooDir > 0 ? lang("우회전", "RIGHT")
                : turnWord(navi.optInt("turnType", 0), navi.optString("title", ""));
        String head = sec <= 0 ? turn
                : (Integer.toString(sec) + lang("초 후 ", "s  ") + turn);
        String title = navi.optString("title", "");
        if (title.length() > 16) {
            title = title.substring(0, 16);
        }
        String detail = title.length() > 0 ? title + "  ·  " + distanceText(dist) : distanceText(dist);
        // 지도경로 혼합률. 조향을 얼마나 지도에 맡기고 있는지 바로 보인다.
        if (nooBlend > 0.005d) {
            detail = detail + "  ·  NOO " + Math.round(nooBlend * 100d) + "%";
        } else {
            detail = detail + "  ·  NOO";
        }

        JSONObject box = new JSONObject();
        try {
            box.put("text1", head);
            box.put("text2", detail);
            // 3초 이하면 주황 테두리(drawAlert 의 prompt 색), 그 전엔 흰색.
            box.put("status", sec <= 3 ? "prompt" : "");
            box.put("size", "small");
        } catch (JSONException e) {
            return null;
        }
        return box;
    }

    private void resetCountdown() {
        countdownLastSec = 100;
        countdownZeroAtMs = 0L;
    }

    /** 카운트다운 문구용 회전 이름. 화살표는 이미 TBT 행에 있으니 글자만 쓴다. */
    private String turnWord(int type, String label) {
        switch (turnDirection(type, label)) {
            case TURN_LEFT:
                return lang("좌회전", "LEFT");
            case TURN_RIGHT:
                return lang("우회전", "RIGHT");
            case TURN_SLIGHT_LEFT:
            case TURN_STRAIGHT_LEFT:
                return lang("좌측방향", "KEEP LEFT");
            case TURN_SLIGHT_RIGHT:
            case TURN_STRAIGHT_RIGHT:
                return lang("우측방향", "KEEP RIGHT");
            case TURN_LEFT_RIGHT:
                return lang("좌우분기", "SPLIT");
            case TURN_UTURN:
                return lang("유턴", "U-TURN");
            case TURN_ARRIVE:
                return lang("목적지", "ARRIVE");
            default:
                return lang("직진", "STRAIGHT");
        }
    }

    /** 연속 공백을 한 칸으로 (renderer.py 의 " ".join(split()) 과 동일) */
    private static String collapse(String value) {
        if (value == null) {
            return "";
        }
        return value.trim().replaceAll("\\s+", " ");
    }

    private void drawBlinkers(Canvas c, Paint p, JSONObject s, boolean stale) {
        if (stale || ((SystemClock.elapsedRealtime() / 500L) & 1L) != 0L) {
            return;
        }
        if (s.optBoolean("leftBlinker", false)) {
            drawBlinker(c, p, 336f, 386f, true);
        }
        if (s.optBoolean("rightBlinker", false)) {
            drawBlinker(c, p, 616f, 386f, false);
        }
    }

    private void drawBlinker(Canvas c, Paint p, float x, float y, boolean left) {
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.rgb(72, 226, 118));
        scratchPath.rewind();
        if (left) {
            scratchPath.moveTo(x - 16f, y);
            scratchPath.lineTo(x + 10f, y - 13f);
            scratchPath.lineTo(x + 10f, y + 13f);
        } else {
            scratchPath.moveTo(x + 16f, y);
            scratchPath.lineTo(x - 10f, y - 13f);
            scratchPath.lineTo(x - 10f, y + 13f);
        }
        scratchPath.close();
        c.drawPath(scratchPath, p);
    }

    private int ink() {
        if (skyBand) {
            return skyLightInk ? Color.rgb(245, 248, 250) : Color.rgb(18, 18, 18);
        }
        return frameDark ? Color.rgb(238, 242, 246) : Color.rgb(18, 18, 18);
    }

    private int dim() {
        if (skyBand) {
            return skyLightInk ? Color.rgb(196, 206, 214) : Color.rgb(86, 92, 98);
        }
        return frameDark ? Color.rgb(140, 152, 163) : Color.rgb(104, 111, 116);
    }

    private int muted() {
        if (skyBand) {
            return skyLightInk ? Color.rgb(150, 160, 174) : Color.rgb(140, 148, 158);
        }
        return frameDark ? Color.rgb(88, 98, 108) : Color.rgb(174, 179, 182);
    }

    private int cardBg() {
        return frameDark ? Color.rgb(24, 30, 38) : Color.rgb(232, 235, 237);
    }

    private int cardEdge() {
        return frameDark ? Color.rgb(60, 70, 82) : Color.rgb(158, 166, 171);
    }

    private int hairline() {
        if (skyBand) {
            return skyLightInk ? Color.argb(110, 255, 255, 255)
                    : Color.argb(70, 0, 0, 0);
        }
        return frameDark ? Color.rgb(58, 66, 76) : Color.rgb(202, 207, 210);
    }

    /** 속도 숫자. 기준선을 84 -> SPEED_BASELINE(118, 예전 KM 라벨 자리)까지 내려서
     *  위쪽에 RPM 아크가 잘리지 않고 들어갈 공간을 만든다. 단위(KM) 라벨은 쓰지
     *  않는다. 세 자리(100km/h 이상)에서는 72px 로 줄여 아크 안쪽에 들어가게 한다. */
    private void drawSpeed(Canvas c, Paint p, int speed) {
        String value = speed < 0 ? "--" : Integer.toString(speed);
        text(c, p, value, DRIVE_CX, SPEED_BASELINE, value.length() < 3 ? 88f : 72f,
                ink(), Paint.Align.CENTER);
    }

    /** 속도 숫자를 위에서 감싸는 엔진 회전수 아크.
     *  예전 값(cy=112, r=110, 굵기 10)은 아크 바깥선이 y = 112-110-5 = -3 이라
     *  패널 위쪽에서 3px 잘렸다. cy=118 / r=106 이면 바깥선이 y=7 로 들어온다.
     *  구분선(y=129) 바로 아래는 SET 원(반지름 36+테두리 3, y=132부터)이 차지해서
     *  라벨을 아크 아래로 더 내릴 수 없다. 그래서 "RPM" 과 회전수 숫자는 아래가
     *  아니라 아크 양 끝에서 좌우 바깥으로 LABEL_GAP 만큼 띄운다.
     *  rpm < 0 (신호 없음 / EV) 이면 아무것도 그리지 않는다. */
    private void drawRpm(Canvas c, Paint p, int rpm, float redline) {
        if (rpm < 0) {
            return;
        }
        float cx = DRIVE_CX;
        float cy = 118f;
        float r = 106f;
        float start = 181f;
        float sweep = 178f;
        float limit = redline > 100f ? redline : 6500f;
        float redFrac = Math.max(0f, Math.min(1f, 5000f / limit));
        float frac = Math.max(0f, Math.min(1f, rpm / limit));

        p.setShader(null);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeCap(Paint.Cap.BUTT);

        // 밝은 실내광에서 라이트테마 레일(150,158,166)이 흰 배경에 묻혀 안 보였다.
        // 순정 계기판 링 색을 추출한 값으로 양 테마 통일한다.
        int railColor = Color.rgb(72, 96, 104);
        int railRed = Color.rgb(226, 72, 77);

        // 바깥/안쪽 레일. 바깥면은 예전 아크(r=106, 굵기 10)와 같은 111 이라
        // 위쪽 잘림(y=6)이나 RPM 라벨 간격에는 영향이 없다.
        p.setStrokeWidth(RPM_RAIL_W);
        float ro = r + RPM_RAIL_OFF;
        float ri = r - RPM_BAR_W / 2f - 3f;
        p.setColor(railColor);
        scratchRect.set(cx - ro, cy - ro, cx + ro, cy + ro);
        c.drawArc(scratchRect, start, sweep, false, p);
        scratchRect.set(cx - ri, cy - ri, cx + ri, cy + ri);
        c.drawArc(scratchRect, start, sweep, false, p);

        p.setColor(railRed);
        float redFrom = start + sweep * redFrac;
        float redSweep = start + sweep - redFrom;
        scratchRect.set(cx - ro, cy - ro, cx + ro, cy + ro);
        c.drawArc(scratchRect, redFrom, redSweep, false, p);
        scratchRect.set(cx - ri, cy - ri, cx + ri, cy + ri);
        c.drawArc(scratchRect, redFrom, redSweep, false, p);

        // 레일 사이를 채우는 인셋 바. 값 이하 구간만 파랑->시안 그라데이션으로
        // 채우고, 레드존은 값이 못 미쳐도 어둡게 남겨 최대치를 알 수 있게 한다.
        float rb = r - 2f;
        float step = sweep / RPM_BARS;
        float gap = 0.55f;
        p.setStrokeWidth(RPM_BAR_W);
        scratchRect.set(cx - rb, cy - rb, cx + rb, cy + rb);
        for (int i = 0; i < RPM_BARS; i++) {
            float mid = (i + 0.5f) / RPM_BARS;
            int color;
            if (mid <= frac) {
                color = mid >= redFrac
                        ? Color.rgb(222, 67, 70)
                        : mixColor(Color.rgb(40, 150, 255), Color.rgb(46, 211, 224),
                                frac > 0.001f ? mid / frac : 0f);
            } else if (mid >= redFrac) {
                color = frameDark ? Color.rgb(96, 40, 44) : Color.rgb(232, 186, 188);
            } else {
                continue;
            }
            p.setColor(color);
            c.drawArc(scratchRect, start + step * i + gap, step - gap * 2f, false, p);
        }

        text(c, p, "RPM", cx - r - RPM_LABEL_GAP, 122f, 13f, dim(), Paint.Align.RIGHT);
        text(c, p, String.format(Locale.US, "%,d", rpm), cx + r + RPM_LABEL_GAP, 124f, 22f,
                ink(), Paint.Align.LEFT);
    }

    /** 두 색을 t(0~1) 비율로 섞는다. RPM 인셋 바 그라데이션용. */
    private static int mixColor(int a, int b, float t) {
        float u = Math.max(0f, Math.min(1f, t));
        return Color.rgb(
                (int) (Color.red(a) + (Color.red(b) - Color.red(a)) * u),
                (int) (Color.green(a) + (Color.green(b) - Color.green(a)) * u),
                (int) (Color.blue(a) + (Color.blue(b) - Color.blue(a)) * u));
    }

    /** Single active-gear tile followed by a compact OEM-style coolant bar. */
    private void drawGearAndCoolant(Canvas c, Paint p, JSONObject s) {
        String gear = s.optString("gear", "--");
        boolean parkingBrake = s.optBoolean("parkingBrake", false);
        float boxLeft = 22f;
        float boxTop = 82f;
        float boxSize = 42f;

        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(parkingBrake ? Color.rgb(210, 42, 52)
                : (frameDark ? Color.rgb(42, 49, 58) : Color.rgb(248, 249, 250)));
        scratchRect.set(boxLeft, boxTop, boxLeft + boxSize, boxTop + boxSize);
        c.drawRoundRect(scratchRect, 6f, 6f, p);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(2f);
        p.setColor(parkingBrake ? Color.rgb(145, 22, 31)
                : (frameDark ? Color.rgb(112, 125, 136) : Color.rgb(176, 184, 190)));
        c.drawRoundRect(scratchRect, 6f, 6f, p);
        text(c, p, gear, boxLeft + boxSize * 0.5f, 114f, 28f,
                parkingBrake ? Color.WHITE
                        : (frameDark ? Color.rgb(240, 243, 246) : Color.rgb(28, 32, 36)),
                Paint.Align.CENTER);

        JSONObject system = s.optJSONObject("system");
        double coolant = system == null ? Double.NaN
                : system.optDouble("coolantTemp", Double.NaN);

        // Enlarge the accepted gauge by 20% and add a small gap from the gear
        // tile.  Scaling stays anchored to the tile bottom so the lower edge
        // remains aligned exactly as approved on the vehicle preview.
        final float coolantScale = 0.924f;
        int coolantSave = c.save();
        c.translate(90f, boxTop + boxSize);
        c.scale(coolantScale, coolantScale);

        final float railLeft = 23f;
        final float railRight = 130f;
        final float railY = -4f;
        text(c, p, "C", 0f, -16f, 18f, ink(), Paint.Align.CENTER);
        text(c, p, "H", 153f, -16f, 18f, ink(), Paint.Align.CENTER);
        drawCoolantThermometer(c, p, 76f, -27f);

        p.setStyle(Paint.Style.STROKE);
        p.setStrokeCap(Paint.Cap.ROUND);
        p.setStrokeWidth(4f);
        p.setColor(frameDark ? Color.rgb(110, 122, 132) : Color.rgb(164, 174, 181));
        c.drawLine(railLeft, railY, railRight, railY, p);
        for (int i = 0; i <= 4; i++) {
            float x = railLeft + (railRight - railLeft) * i / 4f;
            c.drawLine(x, railY - 4f, x, railY + 4f, p);
        }
        if (Double.isFinite(coolant) && coolant > -50d && coolant < 200d) {
            float fraction = (float) Math.max(0d, Math.min(1d, (coolant - 50d) / 70d));
            p.setStrokeWidth(7f);
            p.setColor(fraction > 0.86f
                    ? Color.rgb(226, 67, 70) : Color.rgb(42, 199, 218));
            c.drawLine(railLeft, railY, railLeft + (railRight - railLeft) * fraction, railY, p);
        }
        p.setStrokeCap(Paint.Cap.BUTT);
        c.restoreToCount(coolantSave);
    }

    private void drawCoolantThermometer(Canvas c, Paint p, float x, float y) {
        p.setShader(null);
        p.setColor(dim());
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(2.5f);
        c.drawLine(x, y - 9f, x, y + 7f, p);
        c.drawCircle(x, y + 11f, 5f, p);
        c.drawLine(x - 4f, y - 7f, x + 4f, y - 7f, p);
        c.drawLine(x - 4f, y - 2f, x + 2f, y - 2f, p);
        c.drawLine(x - 4f, y + 3f, x + 4f, y + 3f, p);
    }

    private void drawModeAndEta(Canvas c, Paint p, JSONObject s) {
        JSONObject l = layout(s);
        int mode = s.optInt("drivingMode", 3);
        String label = "NORM";
        int color = frameDark ? Color.rgb(196, 206, 214) : Color.rgb(68, 76, 82);
        if (mode == 1) {
            label = "SAFE";
            color = Color.rgb(226, 144, 38);
        } else if (mode == 2) {
            label = "ECO";
            color = Color.rgb(20, 160, 92);
        } else if (mode == 4) {
            label = "FAST";
            color = Color.rgb(222, 67, 70);
        }
        text(c, p, label, lv(l, "modeX", 938f), lv(l, "modeY", 116f), lv(l, "modeSize", 29f),
                color, Paint.Align.RIGHT);

        Date now = new Date();
        String clock = new SimpleDateFormat("h:mm", Locale.KOREA).format(now);
        String period = new SimpleDateFormat("a", Locale.KOREA).format(now);
        float etaRight = lv(l, "etaRight", 832f);
        float etaY = lv(l, "etaY", 116f);
        float etaTimeSize = lv(l, "etaTimeSize", 27f);
        float etaLabelSize = lv(l, "etaLabelSize", 14f);
        float gap = lv(l, "etaGap", 8f);
        p.setTypeface(Typeface.create("sans", Typeface.BOLD));
        p.setTextSize(etaTimeSize);
        float clockWidth = p.measureText(clock);
        text(c, p, clock, etaRight, etaY, etaTimeSize, ink(), Paint.Align.RIGHT);
        text(c, p, period, etaRight - clockWidth - gap, etaY - 1f, etaLabelSize,
                dim(), Paint.Align.RIGHT);
    }

    /** 주행가능거리. "주행/RANGE" 글자 대신 주유기 아이콘을 거리 왼쪽에 붙인다.
     *  숫자 오른쪽 끝(932)은 그대로 두고 아이콘 위치만 숫자 폭에 맞춰 계산한다. */
    private void drawRange(Canvas c, Paint p, JSONObject s) {
        double km = s.optDouble("distanceToEmpty", -1d);
        if (!Double.isFinite(km) || km < 0d) {
            return;
        }
        String value = String.format(Locale.US, "%.0f km", km);
        float size = 22f;
        float baseline = 44f;
        float iconH = 26f;
        float gap = 9f;

        p.setShader(null);
        p.setTypeface(Typeface.create("sans", Typeface.BOLD));
        p.setTextAlign(Paint.Align.LEFT);
        p.setTextSize(size);
        float valueWidth = p.measureText(value);

        drawFuelIcon(c, p, 932f - valueWidth - gap - fuelIconWidth(iconH), baseline, iconH);
        text(c, p, value, 932f, baseline, size, ink(), Paint.Align.RIGHT);
    }

    private static float fuelIconWidth(float h) {
        return h * 0.80f;
    }

    private void drawOutsideTemp(Canvas c, Paint p, JSONObject s) {
        feedWeather(s);
        double temp = s.optDouble("outsideTemp", -1000d);
        if (!Double.isFinite(temp) || temp < -50d || temp > 80d) {
            return;
        }
        String label = String.format(Locale.US, "%.0f°C", temp);
        // 우측 끝 790 — 오른쪽 주유기 아이콘(932 기준 유동)과 40px 를 남긴다.
        text(c, p, label, 790f, 44f, 22f, ink(), Paint.Align.RIGHT);
        int icon = weather == null ? WeatherService.ICON_NONE : weather.icon();
        if (icon == WeatherService.ICON_NONE) {
            return;
        }
        p.setTextSize(22f);
        p.setTypeface(Typeface.create("sans", Typeface.BOLD));
        float labelWidth = p.measureText(label);
        drawWeatherIcon(c, p, 790f - labelWidth - 10f - 13f, 36f, 26f,
                icon, weather.isDay());
    }

    /**
     * 상단 밴드 배경. 날씨 상태가 바뀔 때만 비트맵을 만들고 매 프레임은
     * 얹기만 한다. 하늘이 어두우면 밴드 안 글자색을 밝은 쪽으로 뒤집는다.
     */
    private void drawSkyBand(Canvas c, Paint p) {
        skyBand = false;
        if (weather == null) {
            return;
        }
        int icon = weather.icon();
        if (icon == WeatherService.ICON_NONE) {
            return;
        }
        SkyBackground.Sky sky = SkyBackground.get(icon, weather.isDay(),
                weather.cloudPercent(), DRIVE_RIGHT, ModelWorldGL.TOP);
        if (sky == null || sky.bitmap == null || sky.bitmap.isRecycled()) {
            return;
        }
        p.setShader(null);
        p.setColorFilter(null);
        p.setAlpha(255);
        c.drawBitmap(sky.bitmap, 0f, 0f, p);
        skyBand = true;
        skyLightInk = sky.lightInk;
    }

    /** 티맵이 주는 좌표를 날씨 조회에 넘긴다. 실제 요청은 서비스가 판단한다. */
    private void feedWeather(JSONObject s) {
        if (weather == null || s == null) {
            return;
        }
        // EON 은 정밀 GPS 를 전송하지 않고, TMAP 안내가 꺼져 있으면 navi 자체가
        // 비어서 내려온다. 그래서 날씨 좌표는 최상위 wxPos 를 먼저 본다.
        JSONArray pos = s.optJSONArray("wxPos");
        if (pos == null || pos.length() < 2) {
            JSONObject navi = s.optJSONObject("navi");
            JSONObject scene = navi == null ? null : navi.optJSONObject("scene");
            pos = scene == null ? null : scene.optJSONArray("pos");
        }
        if (pos == null || pos.length() < 2) {
            return;
        }
        weather.onPosition(pos.optDouble(0, Double.NaN), pos.optDouble(1, Double.NaN));
    }

    /**
     * 하늘상태 아이콘. PNG 없이 벡터로 그린다.
     * 구름은 테마색, 해·달·비·눈·번개만 컬러다.
     */
    private void drawWeatherIcon(Canvas c, Paint p, float cx, float cy,
                                 float size, int icon, boolean day) {
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        float u = size / 26f;
        switch (icon) {
            case WeatherService.ICON_CLEAR:
                if (day) {
                    drawSunDisc(c, p, cx, cy, 7.5f * u, true);
                } else {
                    drawMoon(c, p, cx, cy, 8f * u);
                }
                break;
            case WeatherService.ICON_FEW:
                if (day) {
                    drawSunDisc(c, p, cx + 5f * u, cy - 5f * u, 5.5f * u, true);
                } else {
                    drawMoon(c, p, cx + 5f * u, cy - 5f * u, 5.5f * u);
                }
                drawCloud(c, p, cx - 1f * u, cy + 3f * u, u, ink());
                break;
            case WeatherService.ICON_OVERCAST:
                drawCloud(c, p, cx + 3f * u, cy - 3f * u, u * 0.78f, dim());
                drawCloud(c, p, cx - 2f * u, cy + 2f * u, u, ink());
                break;
            case WeatherService.ICON_FOG:
                drawCloud(c, p, cx, cy - 3f * u, u * 0.9f, ink());
                p.setColor(dim());
                p.setStrokeWidth(2f * u);
                p.setStyle(Paint.Style.STROKE);
                p.setStrokeCap(Paint.Cap.ROUND);
                c.drawLine(cx - 8f * u, cy + 6f * u, cx + 8f * u, cy + 6f * u, p);
                c.drawLine(cx - 5f * u, cy + 10f * u, cx + 7f * u, cy + 10f * u, p);
                p.setStyle(Paint.Style.FILL);
                break;
            case WeatherService.ICON_RAIN:
                drawCloud(c, p, cx, cy - 3f * u, u, ink());
                p.setColor(wxRain());
                p.setStrokeWidth(2.2f * u);
                p.setStyle(Paint.Style.STROKE);
                p.setStrokeCap(Paint.Cap.ROUND);
                for (int i = -1; i <= 1; i++) {
                    float x = cx + i * 5.5f * u;
                    c.drawLine(x + 1.5f * u, cy + 5f * u,
                            x - 1.5f * u, cy + 10f * u, p);
                }
                p.setStyle(Paint.Style.FILL);
                break;
            case WeatherService.ICON_SNOW:
                drawCloud(c, p, cx, cy - 3f * u, u, ink());
                p.setColor(wxSnow());
                for (int i = -1; i <= 1; i++) {
                    c.drawCircle(cx + i * 5.5f * u, cy + 8f * u, 1.9f * u, p);
                }
                break;
            case WeatherService.ICON_THUNDER:
                drawCloud(c, p, cx, cy - 3f * u, u, ink());
                p.setColor(wxBolt());
                scratchPath.rewind();
                scratchPath.moveTo(cx + 1.5f * u, cy + 3f * u);
                scratchPath.lineTo(cx - 3.5f * u, cy + 11f * u);
                scratchPath.lineTo(cx - 0.5f * u, cy + 11f * u);
                scratchPath.lineTo(cx - 2.5f * u, cy + 16f * u);
                scratchPath.lineTo(cx + 4f * u, cy + 8f * u);
                scratchPath.lineTo(cx + 0.5f * u, cy + 8f * u);
                scratchPath.close();
                c.drawPath(scratchPath, p);
                break;
            default:
                break;
        }
    }

    private void drawCloud(Canvas c, Paint p, float cx, float cy, float u, int color) {
        p.setColor(color);
        c.drawCircle(cx - 4.5f * u, cy + 0.5f * u, 4.2f * u, p);
        c.drawCircle(cx + 0.5f * u, cy - 2.2f * u, 5.4f * u, p);
        c.drawCircle(cx + 5.5f * u, cy + 0.5f * u, 4.0f * u, p);
        c.drawRect(cx - 4.5f * u, cy + 0.2f * u, cx + 5.5f * u, cy + 4.4f * u, p);
    }

    private void drawSunDisc(Canvas c, Paint p, float cx, float cy, float r,
                             boolean rays) {
        p.setColor(wxSun());
        c.drawCircle(cx, cy, r, p);
        if (!rays) {
            return;
        }
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(r * 0.30f);
        p.setStrokeCap(Paint.Cap.ROUND);
        for (int i = 0; i < 8; i++) {
            double a = Math.PI * i / 4.0;
            float sx = cx + (float) Math.cos(a) * r * 1.45f;
            float sy = cy + (float) Math.sin(a) * r * 1.45f;
            float ex = cx + (float) Math.cos(a) * r * 1.95f;
            float ey = cy + (float) Math.sin(a) * r * 1.95f;
            c.drawLine(sx, sy, ex, ey, p);
        }
        p.setStyle(Paint.Style.FILL);
    }

    private void drawMoon(Canvas c, Paint p, float cx, float cy, float r) {
        Path full = new Path();
        full.addCircle(cx, cy, r, Path.Direction.CW);
        Path bite = new Path();
        bite.addCircle(cx + r * 0.55f, cy - r * 0.42f, r * 0.88f, Path.Direction.CW);
        full.op(bite, Path.Op.DIFFERENCE);
        p.setColor(wxMoon());
        c.drawPath(full, p);
    }

    private int wxSun() {
        return frameDark ? Color.rgb(255, 196, 64) : Color.rgb(240, 164, 24);
    }

    private int wxMoon() {
        return frameDark ? Color.rgb(154, 168, 188) : Color.rgb(122, 138, 160);
    }

    private int wxRain() {
        return frameDark ? Color.rgb(96, 170, 255) : Color.rgb(38, 120, 214);
    }

    private int wxSnow() {
        return frameDark ? Color.rgb(198, 224, 255) : Color.rgb(96, 150, 200);
    }

    private int wxBolt() {
        return frameDark ? Color.rgb(255, 206, 84) : Color.rgb(232, 160, 20);
    }

    /** 주유기 아이콘. PNG 없이 벡터로 그린다.
     *  x = 아이콘 왼쪽, baseline = 옆 숫자의 baseline(아이콘 아랫변), h = 높이. */
    private void drawFuelIcon(Canvas c, Paint p, float x, float baseline, float h) {
        float top = baseline - h;
        float bodyW = h * 0.58f;
        float stroke = Math.max(1.8f, h * 0.105f);
        int color = dim();

        p.setShader(null);
        p.setColor(color);

        // 펌프 본체
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(stroke);
        scratchRect.set(x + stroke / 2f, top + stroke / 2f,
                x + bodyW - stroke / 2f, baseline - stroke / 2f);
        c.drawRoundRect(scratchRect, h * 0.13f, h * 0.13f, p);

        // 표시창
        p.setStyle(Paint.Style.FILL);
        scratchRect.set(x + h * 0.15f, top + h * 0.16f,
                x + bodyW - h * 0.15f, top + h * 0.42f);
        c.drawRoundRect(scratchRect, h * 0.05f, h * 0.05f, p);

        // 주유 호스 : 본체 오른쪽에서 나와 위로 꺾이는 파이프
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(stroke);
        scratchPath.rewind();
        scratchPath.moveTo(x + bodyW, baseline - h * 0.18f);
        scratchPath.lineTo(x + bodyW + h * 0.16f, baseline - h * 0.18f);
        scratchPath.quadTo(x + bodyW + h * 0.28f, baseline - h * 0.18f,
                x + bodyW + h * 0.28f, baseline - h * 0.34f);
        scratchPath.lineTo(x + bodyW + h * 0.28f, top + h * 0.30f);
        scratchPath.quadTo(x + bodyW + h * 0.28f, top + h * 0.14f,
                x + bodyW + h * 0.14f, top + h * 0.14f);
        c.drawPath(scratchPath, p);

        // 받침
        c.drawLine(x - h * 0.05f, baseline, x + bodyW + h * 0.05f, baseline, p);
    }

    private void drawLights(Canvas c, Paint p, JSONObject s) {
        // 상태 아이콘 줄의 원래 시작 위치는 유지한다. 순정 계기판 사진과 같은
        // 래스터 자산을 축소해 그리며, 자산 누락 때만 기존 벡터를 사용한다.
        final float y = 36f;
        float x = 21f;
        boolean raster = statusIcons != null && !statusIcons.isRecycled();
        if (s.optBoolean("lowBeam", false)) {
            // lowBeam is already the vehicle's combined tail/headlamp state.
            // Drawing both sprite slots here made a low-beam-only state look
            // as if low and high beams were active together.  Slot 1 is the
            // low-beam icon; high beam remains controlled only by highBeam.
            if (raster) {
                drawStatusIcon(c, p, 1, x, y, 38f, 27f);
                x += 43f;
            } else {
                drawLamp(c, p, x, y, 0);
                x += 45f;
            }
        }
        if (s.optBoolean("highBeam", false)) { drawLamp(c, p, x, y, 1); x += 45f; }
        if (s.optBoolean("frontFog", false)) { drawLamp(c, p, x, y, 2); x += 45f; }
        if (s.optBoolean("seatbeltUnlatched", false)) {
            if (raster) drawStatusIcon(c, p, 2, x, y, 28f, 31f);
            else drawSeatbeltIcon(c, p, x + 7f, y);
            x += 34f;
        }
        int wiperMode = visibleWiperMode(s.optInt("wiperMode", 0));
        if (wiperMode != 0) {
            boolean highlighted = SystemClock.elapsedRealtime() - wiperModeChangedElapsed
                    <= WIPER_MODE_HIGHLIGHT_MS;
            drawWiperStatus(c, p, x, y, wiperMode, highlighted);
        }
    }

    /**
     * OFF 는 즉시 숨기고, 순간 입력인 MIST 만 2.5초 동안 유지한다. AUTO/INT/LOW/HIGH
     * 는 선택된 동안 계속 표시해 운전자가 현재 레버 위치를 바로 확인할 수 있게 한다.
     */
    private int visibleWiperMode(int rawMode) {
        int mode = Math.max(0, Math.min(5, rawMode));
        long now = SystemClock.elapsedRealtime();
        if (mode != lastRawWiperMode) {
            int previous = lastRawWiperMode;
            lastRawWiperMode = mode;
            if (mode != 0) {
                heldWiperMode = mode;
                wiperModeChangedElapsed = now;
            } else if (previous != 5) {
                heldWiperMode = 0;
            }
        }
        if (mode != 0) return mode;
        if (heldWiperMode == 5
                && now - wiperModeChangedElapsed <= WIPER_MODE_HIGHLIGHT_MS) {
            return heldWiperMode;
        }
        heldWiperMode = 0;
        return 0;
    }

    /** 문 열림 상태 바로 오른쪽에 붙는 소형 와이퍼 아이콘과 모드명. */
    private void drawWiperStatus(Canvas c, Paint p, float left, float cy, int mode,
                                 boolean highlighted) {
        String label;
        switch (mode) {
            case 1: label = "AUTO"; break;
            case 2: label = "INT"; break;
            case 3: label = "LOW"; break;
            case 4: label = "HIGH"; break;
            case 5: label = "MIST"; break;
            default: return;
        }

        int color = mode == 1 ? Color.rgb(42, 145, 235) : ink();
        float right = left + ("HIGH".equals(label) ? 83f : 77f);
        if (highlighted) {
            p.setShader(null);
            p.setStyle(Paint.Style.FILL);
            p.setColor(frameDark ? Color.rgb(47, 56, 66) : Color.rgb(222, 229, 234));
            scratchRect.set(left - 5f, cy - 20f, right, cy + 20f);
            c.drawRoundRect(scratchRect, 8f, 8f, p);
        }

        // Windshield outline and a single swept blade, kept vector-only so it
        // remains crisp at the small status-row size.
        p.setShader(null);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeCap(Paint.Cap.ROUND);
        p.setStrokeJoin(Paint.Join.ROUND);
        p.setStrokeWidth(2.6f);
        p.setColor(color);
        scratchRect.set(left, cy - 13f, left + 28f, cy + 14f);
        c.drawArc(scratchRect, 205f, 130f, false, p);
        c.drawLine(left + 4f, cy + 7f, left + 24f, cy - 5f, p);
        c.drawLine(left + 4f, cy + 7f, left + 1f, cy + 12f, p);
        p.setStrokeCap(Paint.Cap.BUTT);
        text(c, p, label, left + 34f, cy + 6f, 15f, color, Paint.Align.LEFT);
    }

    /** 투명 PNG 스프라이트에서 아이콘 한 개만 잘라 원래 상태표시 줄에 그린다. */
    private void drawStatusIcon(Canvas c, Paint p, int index, float left, float cy,
                                float width, float height) {
        if (statusIcons == null || statusIcons.isRecycled()) return;
        final float[][] bounds = {
                {0.04f, 0.30f, 0.25f, 0.68f},
                {0.32f, 0.30f, 0.53f, 0.68f},
                {0.56f, 0.24f, 0.73f, 0.76f},
                {0.77f, 0.22f, 0.95f, 0.78f}
        };
        if (index < 0 || index >= bounds.length) return;
        float[] b = bounds[index];
        int sw = statusIcons.getWidth();
        int sh = statusIcons.getHeight();
        scratchIRect.set(Math.round(sw * b[0]), Math.round(sh * b[1]),
                Math.round(sw * b[2]), Math.round(sh * b[3]));
        scratchRect.set(left, cy - height * 0.5f, left + width, cy + height * 0.5f);
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setAlpha(255);
        // 주간의 흰 배경에서는 흰 차체가 사라지므로 문 열림 아이콘에만
        // 진회색 변환을 건다. 빨간 문은 행렬상 원래 색을 유지한다.
        p.setColorFilter(index == 3 && !frameDark ? dayDoorFilter : null);
        p.setFilterBitmap(true);
        c.drawBitmap(statusIcons, scratchIRect, scratchRect, p);
        p.setColorFilter(null);
    }

    /**
     * 순정 PNG의 차체만 사용하고 실제 FL/FR/RL/RR 신호에 해당하는 문을
     * 빨간색으로 표시한다. 기존 PNG에 고정된 운전석 문은 소스 크롭에서 제외한다.
     */
    private void drawDoorStatus(Canvas c, Paint p, float left, float cy, JSONObject doors) {
        if (statusIcons == null || statusIcons.isRecycled()) {
            drawDoorWarning(c, p, left + 9f, cy, doors);
            return;
        }

        int sw = statusIcons.getWidth();
        int sh = statusIcons.getHeight();
        scratchIRect.set(Math.round(sw * 0.835f), Math.round(sh * 0.22f),
                Math.round(sw * 0.95f), Math.round(sh * 0.78f));
        float bodyLeft = left + 6f;
        float bodyRight = left + 29f;
        scratchRect.set(bodyLeft, cy - 15.5f, bodyRight, cy + 15.5f);
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setAlpha(255);
        p.setColorFilter(!frameDark ? dayDoorFilter : null);
        p.setFilterBitmap(true);
        c.drawBitmap(statusIcons, scratchIRect, scratchRect, p);
        p.setColorFilter(null);

        p.setColor(Color.rgb(238, 45, 55));
        if (doors.optBoolean("fl", false)) {
            drawOpenDoorLeaf(c, p, bodyLeft + 1f, cy - 9f, cy - 3f,
                    left - 1f, cy - 12f, cy - 6f);
        }
        if (doors.optBoolean("fr", false)) {
            drawOpenDoorLeaf(c, p, bodyRight - 1f, cy - 9f, cy - 3f,
                    left + 36f, cy - 12f, cy - 6f);
        }
        if (doors.optBoolean("rl", false)) {
            drawOpenDoorLeaf(c, p, bodyLeft + 1f, cy + 3f, cy + 9f,
                    left - 1f, cy + 6f, cy + 12f);
        }
        if (doors.optBoolean("rr", false)) {
            drawOpenDoorLeaf(c, p, bodyRight - 1f, cy + 3f, cy + 9f,
                    left + 36f, cy + 6f, cy + 12f);
        }
    }

    private void drawOpenDoorLeaf(Canvas c, Paint p, float hingeX, float hingeTop,
                                  float hingeBottom, float outerX, float outerTop,
                                  float outerBottom) {
        float inward = outerX < hingeX ? 1.8f : -1.8f;
        scratchPath.rewind();
        scratchPath.moveTo(hingeX, hingeTop);
        scratchPath.lineTo(outerX, outerTop);
        scratchPath.lineTo(outerX + inward, outerBottom);
        scratchPath.lineTo(hingeX, hingeBottom);
        scratchPath.close();
        p.setStyle(Paint.Style.FILL);
        c.drawPath(scratchPath, p);
    }

    private boolean hasOpenDoor(JSONObject doors) {
        return doors != null && (doors.optBoolean("fl", false)
                || doors.optBoolean("fr", false) || doors.optBoolean("rl", false)
                || doors.optBoolean("rr", false));
    }

    private boolean hasOpenWindow(JSONObject windows) {
        return windows != null && (windows.optBoolean("fl", false)
                || windows.optBoolean("fr", false) || windows.optBoolean("rl", false)
                || windows.optBoolean("rr", false));
    }

    private boolean hasVehicleOpening(JSONObject doors, JSONObject windows) {
        return hasOpenDoor(doors) || hasOpenWindow(windows)
                || (doors != null && (doors.optBoolean("hood", false)
                || doors.optBoolean("trunk", false)));
    }

    private void drawSeatbeltIcon(Canvas c, Paint p, float x, float y) {
        int red = Color.rgb(230, 48, 58);
        p.setShader(null);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeCap(Paint.Cap.ROUND);
        p.setStrokeJoin(Paint.Join.ROUND);
        p.setStrokeWidth(4f);
        p.setColor(red);
        c.drawCircle(x, y - 10f, 4f, p);
        scratchPath.rewind();
        scratchPath.moveTo(x - 2f, y - 5f);
        scratchPath.lineTo(x - 7f, y + 11f);
        scratchPath.lineTo(x + 8f, y + 11f);
        scratchPath.lineTo(x + 3f, y - 3f);
        c.drawPath(scratchPath, p);
        c.drawLine(x - 6f, y - 4f, x + 8f, y + 13f, p);
        p.setStrokeCap(Paint.Cap.BUTT);
    }

    private void drawDoorWarning(Canvas c, Paint p, float cx, float cy, JSONObject doors) {
        final float left = cx - 7f, right = cx + 7f;
        final float top = cy - 13f, bottom = cy + 13f;
        p.setShader(null);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeJoin(Paint.Join.ROUND);
        p.setStrokeCap(Paint.Cap.ROUND);
        p.setStrokeWidth(2.5f);
        p.setColor(ink());
        scratchRect.set(left, top, right, bottom);
        c.drawRoundRect(scratchRect, 4f, 4f, p);
        c.drawLine(left + 2f, cy - 5f, right - 2f, cy - 5f, p);
        c.drawLine(left + 2f, cy + 6f, right - 2f, cy + 6f, p);
        p.setStrokeWidth(4f);
        p.setColor(Color.rgb(230, 48, 58));
        if (doors.optBoolean("fl", false)) c.drawLine(left, cy - 7f, left - 8f, cy - 12f, p);
        if (doors.optBoolean("fr", false)) c.drawLine(right, cy - 7f, right + 8f, cy - 12f, p);
        if (doors.optBoolean("rl", false)) c.drawLine(left, cy + 4f, left - 8f, cy + 10f, p);
        if (doors.optBoolean("rr", false)) c.drawLine(right, cy + 4f, right + 8f, cy + 10f, p);
        p.setStrokeCap(Paint.Cap.BUTT);
    }

    private void drawLamp(Canvas c, Paint p, float x, float y, int kind) {
        int color = kind == 1 ? Color.rgb(44, 128, 238) : Color.rgb(39, 177, 89);
        p.setShader(null);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(3f);
        p.setColor(color);
        scratchRect.set(x + 22f, y - 10f, x + 36f, y + 10f);
        c.drawArc(scratchRect, 90f, 180f, false, p);
        for (int i = -1; i <= 1; i++) {
            float yy = y + i * 7f;
            if (kind == 2) {
                c.drawLine(x, yy + 4f, x + 18f, yy, p);
            } else {
                c.drawLine(x, yy, x + 18f, yy, p);
            }
        }
        if (kind == 2) {
            scratchPath.rewind();
            scratchPath.moveTo(x + 9f, y - 12f);
            scratchPath.lineTo(x + 13f, y - 6f);
            scratchPath.lineTo(x + 9f, y);
            scratchPath.lineTo(x + 13f, y + 6f);
            scratchPath.lineTo(x + 9f, y + 12f);
            c.drawPath(scratchPath, p);
        }
    }

    private void drawSteeringWheel(Canvas c, Paint p, float cx, float cy, float angle,
                                   boolean enabled, int steerWarning) {
        if (wheelImage != null && !wheelImage.isRecycled()) {
            drawWheelImage(c, p, cx, cy, angle, enabled, steerWarning);
            return;
        }
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        int wheelBg = steerWarning >= 2 ? Color.rgb(210, 42, 52)
                : (steerWarning == 1 ? Color.rgb(242, 177, 38)
                : (enabled ? Color.rgb(18, 95, 225) : Color.rgb(92, 101, 107)));
        p.setColor(wheelBg);
        c.drawCircle(cx, cy, 36f, p);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(4f);
        p.setColor(Color.rgb(246, 248, 249));
        c.drawCircle(cx, cy, 28f, p);
        double rad = Math.toRadians(-angle);
        double[] spokes = {-90d, 30d, 150d};
        for (double deg : spokes) {
            double a = rad + Math.toRadians(deg);
            c.drawLine(cx + (float) Math.cos(a) * 7f, cy + (float) Math.sin(a) * 7f,
                    cx + (float) Math.cos(a) * 24f, cy + (float) Math.sin(a) * 24f, p);
        }
        p.setStyle(Paint.Style.FILL);
        c.drawCircle(cx, cy, 6f, p);
    }

    /** hud_wheel.png 을 지름 72px 원 안에 맞춰 조향각만큼 회전시켜 그린다.
     *  해제 상태에서는 회색조 + 반투명으로 낮춰 기존 벡터 핸들과 같은 인상을 유지한다. */
    private void drawWheelImage(Canvas c, Paint p, float cx, float cy, float angle,
                                boolean enabled, int steerWarning) {
        float target = 72f;
        int w = wheelImage.getWidth();
        int h = wheelImage.getHeight();
        if (w <= 0 || h <= 0) {
            return;
        }
        float scale = target / Math.max(w, h);

        wheelMatrix.reset();
        wheelMatrix.postTranslate(-w * 0.5f, -h * 0.5f);
        wheelMatrix.postScale(scale, scale);
        wheelMatrix.postRotate(-angle);
        wheelMatrix.postTranslate(cx, cy);

        // 핸들 사진이 검정이라 어두운 주행 패널에서 묻힌다.
        // 뒤에 상태색 원을 깔아 스포크 사이로 비치게 하고, 바깥 링으로 한 번 더 구분한다.
        p.setShader(null);
        p.setColorFilter(null);
        p.setAlpha(255);
        p.setStyle(Paint.Style.FILL);
        int wheelBg = steerWarning >= 2 ? Color.rgb(210, 42, 52)
                : (steerWarning == 1 ? Color.rgb(242, 177, 38)
                : (enabled ? Color.rgb(18, 95, 225) : Color.rgb(92, 101, 107)));
        p.setColor(wheelBg);
        c.drawCircle(cx, cy, 34f, p);

        boolean vivid = enabled || steerWarning > 0;
        p.setFilterBitmap(true);
        p.setColorFilter(vivid ? null : wheelGrayFilter());
        p.setAlpha(vivid ? 255 : 170);
        c.drawBitmap(wheelImage, wheelMatrix, p);
        p.setColorFilter(null);
        p.setAlpha(255);

        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(3f);
        p.setColor(wheelBg);
        c.drawCircle(cx, cy, 36f, p);
    }

    private ColorMatrixColorFilter wheelGrayFilter() {
        if (wheelGray == null) {
            ColorMatrix m = new ColorMatrix();
            m.setSaturation(0f);
            wheelGray = new ColorMatrixColorFilter(m);
        }
        return wheelGray;
    }

    /**
     * NOO·곡선·카메라 감속으로 실제 적용 중인 상한과 그 원인.
     * SET 원 오른쪽에 EON 화면과 같은 황토색으로 세운다. 설정속도와 차이가
     * 없으면(EON 기준 0.5 km/h 이내) EON 이 0 을 보내므로 아무것도 안 그린다.
     */
    private void drawApplySpeed(Canvas c, Paint p, JSONObject s) {
        int applySpeed = s.optInt("applySpeed", 0);
        if (applySpeed <= 0) {
            return;
        }
        String source = s.optString("applySource", "");
        float cx = DRIVE_CX + APPLY_DX;
        text(c, p, Integer.toString(applySpeed), cx, 171f + 12f, 40f, APPLY_OCHRE,
                Paint.Align.LEFT);
        if (!source.isEmpty()) {
            text(c, p, source.toUpperCase(Locale.US), cx, 171f + 36f, 18f, APPLY_OCHRE,
                    Paint.Align.LEFT);
        }
    }

    /** SET 원이 전방충돌 경고도 함께 표시한다. AEB(빨강)가 FCW(노랑)보다 우선한다. */
    private void drawSetSpeed(Canvas c, Paint p, float cx, float cy, int set,
                              boolean enabled, JSONObject s) {
        boolean valid = enabled && set > 0 && set < 255;
        boolean aeb = s.optBoolean("stockAeb", false);
        boolean fcw = !aeb && s.optBoolean("stockFcw", false);
        int inactiveAccent = frameDark
                ? Color.rgb(139, 147, 152) : Color.rgb(88, 98, 106);
        int accent = aeb ? Color.rgb(210, 42, 52)
                : (fcw ? Color.rgb(214, 151, 20)
                : (valid ? Color.rgb(18, 149, 224) : inactiveAccent));
        int fill = aeb ? Color.rgb(230, 48, 58)
                : (fcw ? Color.rgb(255, 205, 52)
                : (frameDark ? Color.rgb(26, 32, 40) : Color.rgb(232, 235, 237)));
        // This value sits inside its own opaque badge. Using sky-band ink here
        // made cloudy daytime skies select white text on the white daytime badge.
        int valueColor = aeb ? Color.WHITE : (fcw ? Color.rgb(28, 30, 32)
                : (frameDark ? Color.rgb(238, 242, 246) : Color.rgb(18, 18, 18)));

        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(fill);
        c.drawCircle(cx, cy, 36f, p);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(6f);
        p.setColor(accent);
        c.drawCircle(cx, cy, 36f, p);
        text(c, p, valid ? Integer.toString(set) : "--", cx, cy + 9f, 29f,
                valueColor, Paint.Align.CENTER);
        text(c, p, "SET", cx, cy + 55f, 14f, accent, Paint.Align.CENTER);
    }

    /**
     * 과속방지턱 아이콘.
     *
     * 2026-08-19: EON 이 쓰는 것과 같은 삼각 경고표지 그림
     * (selfdrive/assets/images/speed_bump.png 사본 = res/drawable-nodpi/hud_speed_bump.png)
     * 을 그대로 쓴다. 리소스가 없으면 예전 벡터 아이콘으로 떨어진다.
     */
    private void drawBumpIcon(Canvas c, Paint p, float cx, float cy, int dist) {
        p.setShader(null);
        if (speedBumpImage != null && !speedBumpImage.isRecycled()) {
            float h = 86f;
            float w = h * speedBumpImage.getWidth() / (float) speedBumpImage.getHeight();
            scratchRect.set(cx - w / 2f, cy - h / 2f + 2f, cx + w / 2f, cy + h / 2f + 2f);
            p.setStyle(Paint.Style.FILL);
            p.setAlpha(255);
            p.setFilterBitmap(true);
            c.drawBitmap(speedBumpImage, null, scratchRect, p);
            // 남은거리는 테마색(낮 검정 / 밤 흰색)
            text(c, p, distanceText(dist), cx, cy + 66f, 18f, ink(), Paint.Align.CENTER);
            return;
        }
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.rgb(250, 250, 250));
        c.drawCircle(cx, cy, 36f, p);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(6f);
        p.setColor(Color.rgb(240, 192, 32));
        c.drawCircle(cx, cy, 36f, p);

        // 둔덕 실루엣 + 줄무늬
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.rgb(46, 48, 52));
        scratchRect.set(cx - 24f, cy - 8f, cx + 24f, cy + 32f);
        c.drawArc(scratchRect, 180f, 180f, false, p);
        c.drawRect(cx - 24f, cy + 10f, cx + 24f, cy + 13f, p);
        p.setColor(Color.rgb(240, 192, 32));
        for (int i = -1; i <= 1; i++) {
            float bx = cx + i * 11f;
            float half = 24f;
            float t = Math.abs(bx - cx) / half;
            float h = 19f * (float) Math.sqrt(Math.max(0f, 1f - t * t));
            c.drawRect(bx - 2.5f, cy + 11f - h, bx + 2.5f, cy + 11f, p);
        }

        text(c, p, distanceText(dist), cx, cy + 60f, 18f, ink(), Paint.Align.CENTER);
    }

    private void drawCamera(Canvas c, Paint p, float cx, float cy, int limit, int dist, boolean section) {
        if (limit <= 0) {
            return;
        }
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.rgb(250, 250, 250));
        c.drawCircle(cx, cy, 36f, p);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(6f);
        p.setColor(Color.rgb(220, 45, 45));
        c.drawCircle(cx, cy, 36f, p);
        text(c, p, Integer.toString(limit), cx, cy + 9f, 29f, Color.rgb(20, 20, 20), Paint.Align.CENTER);
        if (dist > 0) {
            text(c, p, (section ? lang("구간 ", "ZONE ") : "") + distanceText(dist), cx, cy + 60f, 18f,
                    ink(), Paint.Align.CENTER);
        }
    }

    private void drawLeadCard(Canvas c, Paint p, JSONObject lead) {
        if (configuredRadarInfo == 0) {
            return;
        }
        scratchRect.set(8f, 376f, 156f, 454f);
        drawCard(c, p, scratchRect);
        double d = lead == null ? 0d : lead.optDouble("d", 0d);
        double v = lead == null ? 0d : lead.optDouble("v", 0d);
        boolean showDistance = configuredRadarInfo == 2 || configuredRadarInfo == 4;
        text(c, p, lang("앞차", "LEAD"), 18f, 400f, 13f, dim(), Paint.Align.LEFT);
        text(c, p, (!showDistance || d <= 0d) ? "--" : String.format(Locale.US, "%.0f m", d),
                145f, 400f, 19f, ink(), Paint.Align.RIGHT);
        p.setStyle(Paint.Style.STROKE);
        p.setColor(hairline());
        p.setStrokeWidth(1f);
        c.drawLine(16f, 414f, 148f, 414f, p);
        text(c, p, lang("상대", "REL"), 18f, 440f, 13f, dim(), Paint.Align.LEFT);
        text(c, p, d > 0d ? String.format(Locale.US, "%+.0f km/h", v) : "--",
                145f, 440f, 17f, ink(), Paint.Align.RIGHT);
    }

    private static final float TPMS_LOW_PSI = 30f;

    private void drawTpms(Canvas c, Paint p, JSONObject s) {
        JSONObject tpms = s.optJSONObject("tpms");
        float fl = tpmsValue(tpms, "fl"), fr = tpmsValue(tpms, "fr");
        float rl = tpmsValue(tpms, "rl"), rr = tpmsValue(tpms, "rr");
        boolean lowFl = tpmsLow(fl), lowFr = tpmsLow(fr);
        boolean lowRl = tpmsLow(rl), lowRr = tpmsLow(rr);
        boolean warning = lowFl || lowFr || lowRl || lowRr;
        // 카드 배경은 항상 기본. 경고는 가운데 차체 표시만 노란색으로 바꾼다.
        scratchRect.set(791f, 376f, 939f, 454f);
        drawCard(c, p, scratchRect);
        int normalText = ink();
        int titleColor = dim();
        int lowText = Color.rgb(205, 30, 42);
        text(c, p, "TPMS", 865f, 394f, 12f, titleColor, Paint.Align.CENTER);
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(warning
                ? (frameDark ? Color.rgb(232, 176, 32) : Color.rgb(245, 190, 40))
                : Color.rgb(95, 102, 107));
        scratchRect.set(852f, 404f, 878f, 446f);
        c.drawRoundRect(scratchRect, 4f, 4f, p);
        text(c, p, tpmsText(fl), 840f, 417f, 16f, lowFl ? lowText : normalText, Paint.Align.RIGHT);
        text(c, p, tpmsText(fr), 890f, 417f, 16f, lowFr ? lowText : normalText, Paint.Align.LEFT);
        text(c, p, tpmsText(rl), 840f, 443f, 16f, lowRl ? lowText : normalText, Paint.Align.RIGHT);
        text(c, p, tpmsText(rr), 890f, 443f, 16f, lowRr ? lowText : normalText, Paint.Align.LEFT);
    }

    private boolean tpmsLow(float value) {
        return value >= 5f && value <= 60f && value < TPMS_LOW_PSI;
    }

    private String tpmsText(float v) {
        return (v < 5f || v > 60f) ? "--" : Integer.toString(Math.round(v));
    }

    private float tpmsValue(JSONObject t, String key) {
        return t == null ? -1f : (float) t.optDouble(key, -1d);
    }

    private Bitmap decodeUnscaled(String name) {
        int id = getResources().getIdentifier(name, "drawable", getPackageName());
        if (id == 0) {
            return null;
        }
        BitmapFactory.Options opts = new BitmapFactory.Options();
        opts.inScaled = false;
        return BitmapFactory.decodeResource(getResources(), id, opts);
    }

    /**
     * 회전 그림 고르기.
     * 1순위 = 티맵이 보내준 현재 회전 아이콘(모든 회전종류 지원, current=true 일 때만).
     * 2순위 = 앱에 넣은 hud_turn_* 그림(직진·좌우회전·유턴·분기·방향·도착).
     * 둘 다 없으면 null 을 돌려주어 호출부가 벡터 화살표로 폴백한다.
     */
    private Bitmap turnIcon(int type, String label, boolean current) {
        if (current && tmapIconEnabled) {
            Bitmap tmap = tbtCompactFrame.get();
            // 자산 교체(replaceAsset)도 assetLock 안에서 일어나므로 렌더 중 재활용되지 않는다.
            if (tmap != null && !tmap.isRecycled() && tmap.getWidth() > 0 && tmap.getHeight() > 0) {
                return tmap;
            }
        }
        int dir = turnDirection(type, label);
        Bitmap icon = (dir >= 0 && dir < turnImages.length) ? turnImages[dir] : null;
        return (icon != null && !icon.isRecycled()) ? icon : null;
    }

    /**
     * NOO 안내 — 지도 패널의 카드를 없애고 주행 패널 한가운데에 표시한다.
     * 화살표와 남은거리는 안내가 살아 있을 때만, 상태줄은 항상 표시한다.
     * 색은 동작 중이면 테마색(낮 검정 / 밤 흰색), 대기 중이면 흐린색.
     */
    private void drawNooTurn(Canvas c, Paint p, JSONObject s) {
        JSONObject navi = s.optJSONObject("navi");
        int nooMode = s.optInt("nooMode", s.optInt("atcMode", 0));  // legacy wire key fallback
        boolean guidance = navi != null && navi.optBoolean("active", false)
                && navi.optBoolean("guidanceLive", false)
                && navi.optInt("remainDist", 0) > 0;
        int dist = guidance ? navi.optInt("turnDist", -1) : -1;
        boolean armed = nooMode >= 1 && guidance && dist >= 0;
        int color = armed ? ink() : dim();

        // 화살표는 안내가 살아 있을 때만, 깜박이지 않고 계속 떠 있는다.
        if (armed) {
            if (!drawTurnIcon(c, p, NOO_CX, NOO_CY, NOO_ICON_H, navi.optInt("turnType", 0),
                    navi.optString("title", ""), color, true)) {
                drawScaledArrow(c, p, NOO_CX, NOO_CY, navi.optInt("turnType", 0),
                        NOO_ARROW_SCALE, navi.optString("title", ""), color);
            }
            text(c, p, distanceText(dist), NOO_CX, NOO_CY + NOO_TEXT_DY, 28f, color,
                    Paint.Align.CENTER);
        }
        // 상태 줄은 안내가 없어도 항상 뜬다. NOO 가 무엇을 할 참인지(몇 m 앞
        // 어느 방향) 또는 왜 못 하는지를 한 줄로 알려준다.
        text(c, p, nooEventText(s, nooMode, guidance, dist), NOO_CX, NOO_CY + NOO_LANE_DY,
                NOO_LANE_SIZE, color, Paint.Align.CENTER);
    }

    private String nooSide(int direction) {
        return direction < 0 ? lang("좌", "L") : lang("우", "R");
    }

    /**
     * NOO 상태 한 줄. 우선순위는 실제로 무엇을 하고 있는지 → 무엇을 할 참인지 →
     * 왜 못 하는지 순이다.
     */
    private String nooEventText(JSONObject s, int nooMode, boolean guidance, int dist) {
        if (nooMode < 1) {
            return lang("NOO 꺼짐", "NOO OFF");
        }
        JSONObject noo = s.optJSONObject("noo");
        int laneDir = noo == null ? 0 : noo.optInt("dir", 0);
        int cur = noo == null ? 0 : noo.optInt("cur", 0);
        int tgt = noo == null ? 0 : noo.optInt("tgt", 0);
        int cam = noo == null ? 0 : noo.optInt("cam", 0);
        int map = noo == null ? 0 : noo.optInt("map", 0);
        int turnDir = s.optInt("atcDirection", 0);
        double blend = s.optDouble("atcBlend", 0d);
        // 거리는 바로 윗줄에 크게 떠 있으므로 여기서는 방향과 사유만 짧게 쓴다.
        // 주행패널 오른쪽 끝(952)까지 여유가 70px 뿐이라 길면 잘린다.
        // 1) 차선변경을 이미 요청한 상태.
        if (laneDir != 0) {
            return nooSide(laneDir) + lang(" 차선변경", " LANE CHG");
        }
        // 2) 회전 조향을 잡았거나 지도경로를 섞기 시작한 상태.
        if (turnDir != 0 || blend > 0.005d) {
            return nooSide(turnDir) + lang("회전 조향", " TURN");
        }
        if (!guidance) {
            return lang("안내 없음", "NO ROUTE");
        }
        // 3) 계획은 섰지만 아직 실행 전 — 어느 차로로 갈지.
        if (cur > 0 && tgt > 0 && cur != tgt) {
            return cur + "→" + tgt + lang("차로 ", "L ") + nooSide(tgt - cur);
        }
        if (cur > 0 && tgt > 0) {
            return lang("차로 유지", "KEEP LANE");
        }
        // 4) 계획이 안 서는 이유.
        if (cam > 0 && map > 0) {
            return lang("차로 c", "LANE c") + cam + "/m" + map;
        }
        if (map > 0) {
            return lang("차로판정 실패", "NO CAM LANES");
        }
        return lang("차로안내 없음", "NO LANE GUIDE");
    }

    private static final int TBT_GREEN = Color.rgb(31, 122, 72);
    private static final int TBT_GREEN_DARK = Color.rgb(20, 98, 57);
    /** 분기 실사 이미지 폭과, 도착정보 바를 침범하지 않는 아래 한계. */
    private static final float JUNCTION_W = 340f;
    private static final float JUNCTION_LEFT = MAP_LEFT + 2f;
    /** 도착정보 바(위끝 396) 바로 위까지. */
    private static final float JUNCTION_BOTTOM_MAX = 400f;
    /** 도착정보 바는 실사 이미지와 같은 폭·같은 왼쪽 기준. */
    private static final float ETA_H = 58f;
    /** 패널 아래끝(462)에 딱 붙인다. */
    private static final float ETA_TOP = HEIGHT - ETA_H;

    /**
     * 실제 회전 그림(turn_l/turn_r)을 중심 (cx, cy) 에 height 픽셀로 그린다.
     * 흰색 원본이라 color 가 흰색이면 그대로, 아니면 그 색으로 물들인다.
     * 그림이 없는 회전종류면 false 를 돌려주어 호출부가 벡터로 폴백한다.
     */
    private boolean drawTurnIcon(Canvas c, Paint p, float cx, float cy, float height,
                                 int type, String label, int color, boolean current) {
        Bitmap icon = turnIcon(type, label, current);
        if (icon == null) {
            return false;
        }
        float w = height * icon.getWidth() / (float) icon.getHeight();
        scratchRect.set(cx - w / 2f, cy - height / 2f, cx + w / 2f, cy + height / 2f);
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setAlpha(255);
        p.setFilterBitmap(true);
        p.setColorFilter(color == Color.WHITE ? null
                : new PorterDuffColorFilter(color, PorterDuff.Mode.SRC_IN));
        c.drawBitmap(icon, null, scratchRect, p);
        p.setColorFilter(null);
        return true;
    }

    /**
     * 1행(현재 회전)은 티맵 PNG 를 그대로 쓴다 — 고가차도·복잡분기 아이콘이
     * 살아 있어야 하기 때문. 2행(다음 회전)만 1행과 같은 녹색으로 직접 그려
     * 폭을 줄인다. 패킷의 navi.next 가 없으면 그리지 않는다.
     */
    /**
     * TBT 1행 — 폰 티맵 화면의 초록 배너와 같은 형태로 직접 그린다.
     * 티맵이 보내주는 tbt_current_full.png 는 "157m 교차로" 가 한 줄로 붙은
     * 가로형이라, 거리 아래에 도로명을 넣는 폰 화면 모양이 나오지 않는다.
     * 패킷에 turnType / turnDist / title 이 있으므로 직접 그린다.
     * 그린 아래쪽 y 를 돌려주어 2행을 바로 밑에 붙인다.
     */
    private float drawTbtBanner(Canvas c, Paint p, JSONObject navi, float left, float top) {
        if (navi == null || !navi.optBoolean("active", false)
                || navi.optInt("remainDist", 0) <= 0) {
            return top;
        }
        int dist = navi.optInt("turnDist", -1);
        if (dist < 0) {
            return top;
        }
        final float w = 342f;
        final float h = 126f;
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(TBT_GREEN);
        scratchRect.set(left, top, left + w, top + h);
        c.drawRoundRect(scratchRect, 8f, 8f, p);
        String label = navi.optString("title", "");
        int turnType = navi.optInt("turnType", 0);
        // 실제 티맵 화면처럼 굵은 흰 화살표 그림을 쓴다. 그림이 없는 회전종류는 벡터.
        if (!drawTurnIcon(c, p, left + 52f, top + 50f, 68f, turnType, label, Color.WHITE, true)) {
            drawScaledArrow(c, p, left + 46f, top + 50f, turnType, 1.25f, label);
        }
        text(c, p, distanceText(dist), left + 104f, top + 74f, 50f, Color.WHITE, Paint.Align.LEFT);
        String title = label;
        if (title.length() > 7) {
            title = title.substring(0, 7);
        }
        if (title.length() > 0) {
            // 도로명은 한 단 어두운 녹색 박스에 넣는다(티맵과 같은 형태).
            p.setStyle(Paint.Style.FILL);
            p.setColor(TBT_GREEN_DARK);
            scratchRect.set(left + 100f, top + 84f, left + w - 8f, top + h - 6f);
            c.drawRoundRect(scratchRect, 6f, 6f, p);
            text(c, p, title, left + 110f, top + h - 14f, 26f, Color.WHITE, Paint.Align.LEFT);
        }
        return top + h;
    }

    /**
     * (미사용 · 필요하면 되돌리기용) TBT 1행을 티맵 PNG 로 그린다. drawNativeOverlay 처럼
     * 박스 안에서 세로 가운데 정렬을 하면 2행을 바로 밑에 붙일 수 없어서
     * 실제로 그린 높이(아래쪽 y)를 돌려준다.
     */
    private float drawTbtImage(Canvas c, Paint p, Bitmap b, float left, float top,
                               float maxWidth, float maxHeight) {
        if (b == null || b.isRecycled() || b.getWidth() <= 0 || b.getHeight() <= 0) {
            return top;
        }
        float scale = Math.min(maxWidth / b.getWidth(), maxHeight / b.getHeight());
        float w = b.getWidth() * scale;
        float h = b.getHeight() * scale;
        p.setAlpha(255);
        p.setFilterBitmap(true);
        scratchRect.set(left, top, left + w, top + h);
        c.drawBitmap(b, null, scratchRect, p);
        return top + h;
    }

    private void drawTbtNext(Canvas c, Paint p, JSONObject navi, float left, float top) {
        if (navi == null || !navi.optBoolean("active", false)
                || navi.optInt("remainDist", 0) <= 0) {
            return;
        }
        JSONObject next = navi.optJSONObject("next");
        if (next == null) {
            return;
        }
        int nextDist = next.optInt("turnDist", -1);
        if (nextDist < 0) {
            return;
        }
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(TBT_GREEN);
        // 폭 352 -> 176 (절반). 안쪽 화살표/글자도 같이 줄인다.
        scratchRect.set(left, top, left + 176f, top + 52f);
        c.drawRoundRect(scratchRect, 7f, 7f, p);
        int nextType = next.optInt("turnType", 0);
        String nextLabel = next.optString("title", "");
        if (!drawTurnIcon(c, p, left + 28f, top + 25f, 30f, nextType, nextLabel, Color.WHITE, false)) {
            drawScaledArrow(c, p, left + 28f, top + 26f, nextType, 0.62f, nextLabel);
        }
        text(c, p, distanceText(nextDist), left + 56f, top + 36f, 24f, Color.WHITE, Paint.Align.LEFT);
    }

    // 회전 방향 종류. EON onroad_navi.inc 의 CarrotTurnDirection 과 1:1 대응.
    private static final int TURN_STRAIGHT = 0;
    private static final int TURN_LEFT = 1;
    private static final int TURN_RIGHT = 2;
    private static final int TURN_STRAIGHT_LEFT = 3;
    private static final int TURN_STRAIGHT_RIGHT = 4;
    private static final int TURN_LEFT_RIGHT = 5;
    private static final int TURN_UTURN = 6;
    private static final int TURN_SLIGHT_LEFT = 7;
    private static final int TURN_SLIGHT_RIGHT = 8;
    private static final int TURN_ARRIVE = 9;

    /**
     * 티맵 turn_type + 안내문구 → 회전 방향.
     *
     * 2026-08-19: 예전 매핑({12,16,20,3,5} 좌 / {13,18,21,4,6} 우)은 근거 없는
     * 추정이라 티맵 화면과 자주 어긋났다. EON onroad_navi.inc 의
     * carrotTurnDirection() 과 완전히 같은 규칙으로 교체한다(코드 우선,
     * 못 맞추면 안내문구로 보정).
     */
    private static int turnDirection(int type, String label) {
        String v = label == null ? "" : label.toLowerCase(Locale.US);
        if (type == 20) return TURN_STRAIGHT_LEFT;
        if (type == 21) return TURN_STRAIGHT_RIGHT;
        if (type == 22) return TURN_LEFT_RIGHT;
        if (v.contains("\uc720\ud134") || v.contains("u-turn") || type == 14) return TURN_UTURN;
        if (v.contains("\ubaa9\uc801\uc9c0") || v.contains("\ub3c4\ucc29") || type == 2) return TURN_ARRIVE;
        if (v.contains("\uc67c\ucabd \ubc29\ud5a5") || v.contains("\uc88c\uce21 \ubc29\ud5a5")
                || v.contains("\ube44\uc2a4\ub4ec\ud788 \uc67c\ucabd")
                || type == 16 || type == 17) return TURN_SLIGHT_LEFT;
        if (v.contains("\uc624\ub978\ucabd \ubc29\ud5a5") || v.contains("\uc6b0\uce21 \ubc29\ud5a5")
                || v.contains("\ube44\uc2a4\ub4ec\ud788 \uc624\ub978\ucabd")
                || type == 18 || type == 19) return TURN_SLIGHT_RIGHT;
        if (v.contains("\uc88c\ud68c\uc804") || v.contains("\uc67c\ucabd") || type == 12) return TURN_LEFT;
        if (v.contains("\uc6b0\ud68c\uc804") || v.contains("\uc624\ub978\ucabd") || type == 13) return TURN_RIGHT;
        return TURN_STRAIGHT;
    }

    private void drawScaledArrow(Canvas c, Paint p, float cx, float cy, int type,
                                 float scale, String label) {
        drawScaledArrow(c, p, cx, cy, type, scale, label, Color.WHITE);
    }

    private void drawScaledArrow(Canvas c, Paint p, float cx, float cy, int type,
                                 float scale, String label, int color) {
        int save = c.save();
        c.scale(scale, scale, cx, cy);
        drawAtcArrow(c, p, cx, cy, type, label, color);
        c.restoreToCount(save);
    }

    /** EON drawCarrotTurnArrow() 의 도형을 그대로 옮긴 것(기준 s=22px). */
    private void drawAtcArrow(Canvas c, Paint p, float cx, float cy, int type, String label) {
        drawAtcArrow(c, p, cx, cy, type, label, Color.WHITE);
    }

    private void drawAtcArrow(Canvas c, Paint p, float cx, float cy, int type, String label,
                              int color) {
        int dir = turnDirection(type, label);
        float s = 22f;
        p.setShader(null);
        p.setStrokeWidth(6f);
        p.setStrokeCap(Paint.Cap.ROUND);
        p.setStrokeJoin(Paint.Join.ROUND);
        p.setColor(color);

        if (dir == TURN_ARRIVE) {
            p.setStyle(Paint.Style.FILL);
            c.drawCircle(cx, cy, s * 0.45f, p);
            p.setStyle(Paint.Style.STROKE);
            c.drawCircle(cx, cy, s * 0.85f, p);
            p.setStyle(Paint.Style.FILL);
            return;
        }

        p.setStyle(Paint.Style.STROKE);
        scratchPath.rewind();
        if (dir == TURN_STRAIGHT_LEFT || dir == TURN_STRAIGHT_RIGHT || dir == TURN_LEFT_RIGHT) {
            boolean drawStraight = dir != TURN_LEFT_RIGHT;
            boolean drawLeft = dir == TURN_STRAIGHT_LEFT || dir == TURN_LEFT_RIGHT;
            boolean drawRight = dir == TURN_STRAIGHT_RIGHT || dir == TURN_LEFT_RIGHT;
            if (drawStraight) {
                scratchPath.moveTo(cx, cy + s);
                scratchPath.lineTo(cx, cy - s);
                scratchPath.moveTo(cx - s * 0.35f, cy - s * 0.6f);
                scratchPath.lineTo(cx, cy - s);
                scratchPath.lineTo(cx + s * 0.35f, cy - s * 0.6f);
            }
            if (drawLeft) {
                scratchPath.moveTo(cx, cy + s * 0.25f);
                scratchPath.lineTo(cx, cy - s * 0.15f);
                scratchPath.lineTo(cx - s, cy - s * 0.15f);
                scratchPath.moveTo(cx - s * 0.55f, cy - s * 0.5f);
                scratchPath.lineTo(cx - s, cy - s * 0.15f);
                scratchPath.lineTo(cx - s * 0.55f, cy + s * 0.2f);
            }
            if (drawRight) {
                scratchPath.moveTo(cx, cy + s * 0.25f);
                scratchPath.lineTo(cx, cy - s * 0.15f);
                scratchPath.lineTo(cx + s, cy - s * 0.15f);
                scratchPath.moveTo(cx + s * 0.55f, cy - s * 0.5f);
                scratchPath.lineTo(cx + s, cy - s * 0.15f);
                scratchPath.lineTo(cx + s * 0.55f, cy + s * 0.2f);
            }
        } else if (dir == TURN_UTURN) {
            scratchPath.moveTo(cx + s * 0.55f, cy + s);
            scratchPath.lineTo(cx + s * 0.55f, cy - s * 0.25f);
            scratchPath.cubicTo(cx + s * 0.55f, cy - s, cx - s * 0.65f, cy - s,
                    cx - s * 0.65f, cy - s * 0.2f);
            scratchPath.moveTo(cx - s, cy - s * 0.35f);
            scratchPath.lineTo(cx - s * 0.65f, cy + s * 0.05f);
            scratchPath.lineTo(cx - s * 0.3f, cy - s * 0.35f);
        } else {
            float dx = 0f;
            if (dir == TURN_LEFT) dx = -s;
            else if (dir == TURN_RIGHT) dx = s;
            else if (dir == TURN_SLIGHT_LEFT) dx = -s * 0.7f;
            else if (dir == TURN_SLIGHT_RIGHT) dx = s * 0.7f;

            float tipX;
            float tipY;
            scratchPath.moveTo(cx, cy + s);
            if (dir == TURN_LEFT || dir == TURN_RIGHT) {
                scratchPath.lineTo(cx, cy - s * 0.2f);
                scratchPath.lineTo(cx + dx, cy - s * 0.2f);
                tipX = cx + dx;
                tipY = cy - s * 0.2f;
            } else {
                scratchPath.lineTo(cx + dx, cy - s);
                tipX = cx + dx;
                tipY = cy - s;
            }
            float side = dx < 0f ? 1f : (dx > 0f ? -1f : 0f);
            if (dx == 0f) {
                scratchPath.moveTo(tipX - s * 0.35f, tipY + s * 0.4f);
                scratchPath.lineTo(tipX, tipY);
                scratchPath.lineTo(tipX + s * 0.35f, tipY + s * 0.4f);
            } else {
                scratchPath.moveTo(tipX + side * s * 0.05f, tipY + s * 0.45f);
                scratchPath.lineTo(tipX, tipY);
                scratchPath.lineTo(tipX + side * s * 0.45f, tipY + s * 0.05f);
            }
        }
        c.drawPath(scratchPath, p);
        p.setStyle(Paint.Style.FILL);
    }

    private void drawCard(Canvas c, Paint p, RectF box) {
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(cardBg());
        c.drawRoundRect(box, 9f, 9f, p);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(2f);
        p.setColor(cardEdge());
        c.drawRoundRect(box, 9f, 9f, p);
    }

    private String distanceText(int meters) {
        return meters >= 1000
                ? String.format(Locale.US, "%.1f km", meters / 1000f)
                : meters + " m";
    }

    // ── 우측 패널 ─────────────────────────────────────────────────────────

    private String systemValue(JSONObject system, String key, String unit) {
        if (system == null || system.isNull(key)) {
            return "--";
        }
        double value = system.optDouble(key, Double.NaN);
        return (Double.isNaN(value) || Double.isInfinite(value))
                ? "--" : String.format(Locale.US, "%.0f%s", value, unit);
    }

    /** 폭 192px 패널이라 라벨을 위, 값을 아래로 쌓는다. */
    private void drawSystemMetric(Canvas c, Paint p, float top, String label, String value) {
        float bottom = top + 84f;
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.rgb(16, 23, 32));
        scratchRect.set(1734f, top, 1914f, bottom);
        c.drawRoundRect(scratchRect, 9f, 9f, p);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(2f);
        p.setColor(Color.rgb(55, 68, 80));
        c.drawRoundRect(scratchRect, 9f, 9f, p);
        text(c, p, label, 1824f, top + 26f, 15f, Color.rgb(145, 158, 168), Paint.Align.CENTER);
        text(c, p, value, 1824f, top + 66f, 30f, Color.rgb(235, 240, 245), Paint.Align.CENTER);
    }

    private void drawSystem(Canvas c, Paint p, JSONObject s) {
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.rgb(7, 12, 18));
        c.drawRect(SYSTEM_LEFT, 0f, SYSTEM_RIGHT, HEIGHT, p);
        text(c, p, "SYSTEM", 1824f, 26f, 17f, Color.rgb(235, 240, 245), Paint.Align.CENTER);

        JSONObject system = s.optJSONObject("system");
        if (system == null) {
            system = s;
        }
        drawSystemMetric(c, p, 42f, "CPU", systemValue(system, "cpu", "%"));
        drawSystemMetric(c, p, 132f, "TEMP", systemValue(system, "temp", "°C"));
        drawSystemMetric(c, p, 222f, "ENGINE", systemValue(system, "engineTemp", "°C"));
        drawSystemMetric(c, p, 312f, "COOLANT", systemValue(system, "coolantTemp", "°C"));

        double accel = s.optDouble("accel", Double.NaN);
        if (Double.isFinite(accel)) {
            text(c, p, "ACCEL", 1824f, 424f, 13f, Color.rgb(120, 132, 142), Paint.Align.CENTER);
            text(c, p, String.format(Locale.US, "%+.2f", accel), 1824f, 448f, 20f,
                    Color.rgb(173, 184, 192), Paint.Align.CENTER);
        }
    }

    /** 출력모드 2 — 폭 192px 자리에 들어가는 실시간 디버그. */
    private void drawSystemDebug(Canvas c, Paint p, JSONObject s) {
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.rgb(7, 12, 18));
        c.drawRect(SYSTEM_LEFT, 0f, SYSTEM_RIGHT, HEIGHT, p);
        text(c, p, "DEBUG", 1824f, 26f, 17f, Color.rgb(255, 190, 120), Paint.Align.CENTER);

        JSONObject sys = s.optJSONObject("system");
        if (sys == null) {
            sys = s;
        }
        JSONObject lead = s.optJSONObject("lead");
        String leadText = lead == null ? "--" : String.format(Locale.US, "%.0fm", lead.optDouble("d", 0d));
        String relText = lead == null ? "--" : String.format(Locale.US, "%+.0f", lead.optDouble("v", 0d));

        String[][] rows = {
                {"CPU", String.format(Locale.US, "%.0f%%", sys.optDouble("cpu", 0d))},
                {"TEMP", String.format(Locale.US, "%.0f°C", sys.optDouble("temp", 0d))},
                {"SPEED", Integer.toString(s.optInt("speed", 0))},
                {"SET", Integer.toString(s.optInt("set", 0))},
                {"GAP", Integer.toString(s.optInt("gap", 0))},
                {"LEAD", leadText},
                {"REL", relText},
                {"FPS", String.format(Locale.US, "%.1f", measuredFps)},
                {"JPEG", String.format(Locale.US, "%.0fK", lastJpegBytes / 1024.0f)},
        };
        // NOO 차선변경 진단은 주행패널의 회전화살표 위로 옮겼다(drawNooTurn).
        // 9 rows: 38 + 8*42 + 38 = 412 < HEIGHT(462).
        float top = 38f;
        for (String[] row : rows) {
            p.setStyle(Paint.Style.FILL);
            p.setColor(Color.rgb(16, 23, 32));
            scratchRect.set(1734f, top, 1914f, top + 38f);
            c.drawRoundRect(scratchRect, 6f, 6f, p);
            text(c, p, row[0], 1744f, top + 26f, 14f, Color.rgb(140, 152, 162), Paint.Align.LEFT);
            text(c, p, row[1], 1904f, top + 27f, 19f, Color.rgb(235, 240, 245), Paint.Align.RIGHT);
            top += 42f;
        }
    }

    /** 출력모드 3 — S9(폰) 자신의 상태와 USB 경로 진단. */
    private void drawS9Remote(Canvas c, Paint p) {
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.rgb(7, 12, 18));
        c.drawRect(SYSTEM_LEFT, 0f, SYSTEM_RIGHT, HEIGHT, p);
        text(c, p, lang("S9 리모트", "S9 REMOTE"), 1824f, 26f, 17f,
                Color.rgb(140, 210, 255), Paint.Align.CENTER);

        long now = SystemClock.elapsedRealtime();
        long silence = display == null ? -1L : display.silenceMs();
        long linkAge = lastReconnectElapsed == 0L ? -1L : now - lastReconnectElapsed;
        Runtime rt = Runtime.getRuntime();
        long usedMb = (rt.totalMemory() - rt.freeMemory()) / 1048576L;

        String[][] rows = {
                {"SoC", s9TempC < 0f ? "--" : String.format(Locale.US, "%.0f°C", s9TempC)},
                {"CPU", s9CpuPercent < 0f ? "--" : String.format(Locale.US, "%.0f%%", s9CpuPercent)},
                {"MEM", usedMb + "M"},
                {"USB ERR", Integer.toString(usbErrorStreak)},
                {"PANEL", silence < 0L ? "--" : String.format(Locale.US, "%.0fs", silence / 1000f)},
                {"LINK", linkAge < 0L ? "--" : durationText(linkAge)},
        };
        float top = 46f;
        for (String[] row : rows) {
            p.setStyle(Paint.Style.FILL);
            p.setColor(Color.rgb(16, 23, 32));
            scratchRect.set(1734f, top, 1914f, top + 54f);
            c.drawRoundRect(scratchRect, 8f, 8f, p);
            text(c, p, row[0], 1824f, top + 20f, 13f, Color.rgb(140, 152, 162), Paint.Align.CENTER);
            text(c, p, row[1], 1824f, top + 45f, 24f, Color.rgb(235, 240, 245), Paint.Align.CENTER);
            top += 60f;
        }
        text(c, p, lang("USB 오류 누적 / 패널 무응답", "USB ERRORS / PANEL SILENCE"),
                1824f, 424f, 10f, Color.rgb(110, 122, 132), Paint.Align.CENTER);
        text(c, p, lang("LINK = 마지막 재연결 경과", "LINK = SINCE RECONNECT"),
                1824f, 442f, 10f, Color.rgb(110, 122, 132), Paint.Align.CENTER);
    }

    private String durationText(long ms) {
        long sec = ms / 1000L;
        if (sec < 100L) {
            return sec + "s";
        }
        if (sec < 6000L) {
            return (sec / 60L) + "m";
        }
        return (sec / 3600L) + "h";
    }

    // ── S9 자체 상태 샘플러 ───────────────────────────────────────────
    //
    // 안드로이드 12+ 는 일반 앱의 /proc/stat, /sys/class/thermal 접근을
    // 막는다(hidepid + SELinux). 직접 읽기를 먼저 시도하고, 막히면 su 로
    // 넘어간다. 폰이 루팅돼 있지 않으면 그대로 "--" 로 남는다.
    //
    // 프로세스를 띄우는 일이라 렌더 스레드가 아니라 전용 스레드에서
    // 3초마다 돌린다.

    private static final long STATS_PERIOD_MS = 3000L;

    private void statsLoop() {
        while (running.get()) {
            try {
                sampleS9Stats();
            } catch (Exception ignored) {
            }
            SystemClock.sleep(STATS_PERIOD_MS);
        }
    }

    private void sampleS9Stats() {
        String thermal = readThermalDump();
        String stat = readProcStat();
        if ((thermal == null || stat == null) && !suUnavailable) {
            String dump = shellRead(
                    "for z in /sys/class/thermal/thermal_zone*; do "
                            + "echo \"T:$(cat $z/type 2>/dev/null):$(cat $z/temp 2>/dev/null)\"; done; "
                            + "echo \"S:$(head -1 /proc/stat)\"");
            if (dump == null) {
                suUnavailable = true;
            } else {
                StringBuilder zones = new StringBuilder();
                for (String line : dump.split("\n")) {
                    if (line.startsWith("T:")) {
                        zones.append(line.substring(2)).append('\n');
                    } else if (line.startsWith("S:")) {
                        stat = line.substring(2);
                    }
                }
                if (thermal == null && zones.length() > 0) {
                    thermal = zones.toString();
                }
            }
        }
        applyThermal(thermal);
        applyCpu(stat);
    }

    /** 권한이 있으면 직접 읽는다. "type:temp" 줄들을 돌려준다. */
    private String readThermalDump() {
        java.io.File base = new java.io.File("/sys/class/thermal");
        java.io.File[] zones = base.listFiles();
        if (zones == null) {
            return null;
        }
        StringBuilder sb = new StringBuilder();
        for (java.io.File zone : zones) {
            if (!zone.getName().startsWith("thermal_zone")) {
                continue;
            }
            String type = readFirstLine(new java.io.File(zone, "type"));
            String temp = readFirstLine(new java.io.File(zone, "temp"));
            if (temp != null) {
                sb.append(type == null ? "" : type).append(':').append(temp).append('\n');
            }
        }
        return sb.length() > 0 ? sb.toString() : null;
    }

    private String readProcStat() {
        String line = readFirstLine(new java.io.File("/proc/stat"));
        return (line != null && line.startsWith("cpu ")) ? line : null;
    }

    /** cpu/big/soc 존을 우선하고, 없으면 가장 높은 온도를 쓴다. */
    private void applyThermal(String dump) {
        if (dump == null) {
            return;
        }
        float best = -1f;
        float hottest = -1f;
        for (String line : dump.split("\n")) {
            int sep = line.lastIndexOf(':');
            if (sep < 0) {
                continue;
            }
            float value;
            try {
                value = Float.parseFloat(line.substring(sep + 1).trim());
            } catch (NumberFormatException e) {
                continue;
            }
            if (value > 1000f) {
                value /= 1000f;
            }
            if (value <= 0f || value >= 150f) {
                continue;
            }
            String type = line.substring(0, sep).toLowerCase(Locale.US);
            if (type.contains("cpu") || type.contains("big") || type.contains("soc")
                    || type.contains("apollo") || type.contains("atlas")) {
                best = Math.max(best, value);
            }
            hottest = Math.max(hottest, value);
        }
        s9TempC = best > 0f ? best : hottest;
    }

    private void applyCpu(String line) {
        if (line == null) {
            return;
        }
        String[] parts = line.trim().split("\\s+");
        long total = 0L;
        long idle = 0L;
        try {
            for (int i = 1; i < parts.length && i <= 8; i++) {
                long v = Long.parseLong(parts[i]);
                total += v;
                if (i == 4 || i == 5) {
                    idle += v;
                }
            }
        } catch (NumberFormatException e) {
            return;
        }
        if (cpuLastTotal > 0L && total > cpuLastTotal) {
            long dt = total - cpuLastTotal;
            long di = idle - cpuLastIdle;
            s9CpuPercent = Math.max(0f, Math.min(100f, (dt - di) * 100f / dt));
        }
        cpuLastTotal = total;
        cpuLastIdle = idle;
    }

    private static String shellRead(String command) {
        Process proc = null;
        try {
            proc = Runtime.getRuntime().exec(new String[]{"su", "-c", command});
            java.io.BufferedReader reader = new java.io.BufferedReader(
                    new java.io.InputStreamReader(proc.getInputStream()), 2048);
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) {
                sb.append(line).append('\n');
            }
            reader.close();
            return sb.length() > 0 ? sb.toString() : null;
        } catch (Exception e) {
            return null;
        } finally {
            if (proc != null) {
                proc.destroy();
            }
        }
    }

    private static String readFirstLine(java.io.File file) {
        java.io.BufferedReader reader = null;
        try {
            reader = new java.io.BufferedReader(new java.io.FileReader(file), 256);
            return reader.readLine();
        } catch (Exception e) {
            return null;
        } finally {
            if (reader != null) {
                try {
                    reader.close();
                } catch (Exception ignored) {
                }
            }
        }
    }

    private String lang(String ko, String en) {
        return configuredLanguage == 1 ? en : ko;
    }

    private boolean darkTheme() {
        if (configuredTheme == 1) {
            return true;
        }
        if (configuredTheme == 2) {
            return false;
        }
        int h = Calendar.getInstance().get(Calendar.HOUR_OF_DAY);
        return h < 7 || h >= 19;
    }

    private void applyThemeOverlay(Canvas c, Paint p) {
        if (!darkTheme()) {
            return;
        }
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.argb(28, 0, 0, 0));
        c.drawRect(0f, 0f, WIDTH, HEIGHT, p);
    }

    private void updateTrip(JSONObject s, long now) {
        if (tripStartElapsed == 0L) {
            tripStartElapsed = now;
            tripLastElapsed = now;
            return;
        }
        long dt = Math.max(0L, Math.min(2000L, now - tripLastElapsed));
        tripLastElapsed = now;
        double speed = Math.max(0d, s.optDouble("speed", 0d));
        tripDistanceKm += dt * speed / 3600000d;
    }

    private int mapRight() {
        return configuredLayoutMode == 2 ? WIDTH : MAP_RIGHT;
    }

    private float mapCenterX() {
        return (MAP_LEFT + mapRight()) * 0.5f;
    }

    private void drawRightBase(Canvas c, Paint p, String title) {
        boolean dark = darkTheme();
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(dark ? Color.rgb(8, 13, 19) : Color.rgb(232, 235, 237));
        c.drawRect(MAP_LEFT, 0f, mapRight(), HEIGHT, p);
        text(c, p, title, mapCenterX(), 42f, 27f, dark ? Color.WHITE : Color.rgb(25, 30, 34), Paint.Align.CENTER);
    }

    private void drawDebugRight(Canvas c, Paint p, JSONObject s) {
        drawRightBase(c, p, lang("실시간 디버그", "LIVE DEBUG"));
        JSONObject sys = s.optJSONObject("system");
        if (sys == null) {
            sys = s;
        }
        boolean dark = darkTheme();
        int fg = dark ? Color.rgb(235, 240, 245) : Color.rgb(25, 30, 34);
        int sub = dark ? Color.rgb(160, 172, 182) : Color.rgb(90, 100, 108);

        float y = 95f;
        text(c, p, String.format(Locale.US, "CPU %.0f%%   TEMP %.0f°C",
                sys.optDouble("cpu", 0d), sys.optDouble("temp", 0d)), 1000f, y, 25f, fg, Paint.Align.LEFT);
        y += 52f;
        text(c, p, String.format(Locale.US, "SPEED %d   SET %d   GAP %d",
                s.optInt("speed", 0), s.optInt("set", 0), s.optInt("gap", 0)),
                1000f, y, 23f, fg, Paint.Align.LEFT);
        y += 52f;
        JSONObject lead = s.optJSONObject("lead");
        if (lead != null) {
            text(c, p, String.format(Locale.US, "LEAD %.0fm  %+.0fkm/h",
                    lead.optDouble("d", 0d), lead.optDouble("v", 0d)), 1000f, y, 23f, fg, Paint.Align.LEFT);
        } else {
            text(c, p, "LEAD --", 1000f, y, 23f, sub, Paint.Align.LEFT);
        }
        y += 52f;
        text(c, p, String.format(Locale.US, "FPS %d   MAP %dfps   JPEG %d",
                configuredFps, Math.max(2, Math.min(5, s.optInt("hudMapFps", 5))), jpegQuality),
                1000f, y, 22f, fg, Paint.Align.LEFT);
        y += 52f;
        text(c, p, lang("S9 렌더링 / USB 출력", "S9 RENDER / USB OUTPUT"),
                1000f, y, 20f, sub, Paint.Align.LEFT);
    }

    private void drawTripRight(Canvas c, Paint p, JSONObject s) {
        drawRightBase(c, p, lang("주행 리포트", "TRIP REPORT"));
        long elapsed = tripStartElapsed != 0L ? SystemClock.elapsedRealtime() - tripStartElapsed : 0L;
        double hours = elapsed / 3600000d;
        double avg = hours > 1.0e-4d ? tripDistanceKm / hours : 0d;
        boolean dark = darkTheme();
        int fg = dark ? Color.rgb(235, 240, 245) : Color.rgb(25, 30, 34);
        int sub = dark ? Color.rgb(160, 172, 182) : Color.rgb(90, 100, 108);

        text(c, p, String.format(Locale.US, "%.1f km", tripDistanceKm), mapCenterX(), 145f, 54f, fg, Paint.Align.CENTER);
        text(c, p, lang("주행거리", "DISTANCE"), mapCenterX(), 178f, 18f, sub, Paint.Align.CENTER);
        text(c, p, String.format(Locale.US, "%02d:%02d", elapsed / 3600000L, (elapsed / 60000L) % 60L),
                mapCenterX(), 255f, 48f, fg, Paint.Align.CENTER);
        text(c, p, lang("주행시간", "DRIVE TIME"), mapCenterX(), 286f, 18f, sub, Paint.Align.CENTER);
        text(c, p, String.format(Locale.US, "AVG %.0f km/h", avg), mapCenterX(), 363f, 35f, fg, Paint.Align.CENTER);
    }

    private void drawMap(Canvas c, Paint p, JSONObject s, Bitmap map, Bitmap tbtCurrent,
                         Bitmap tbtNext, Bitmap lane) {
        scratchIRect.set(MAP_LEFT, 0, mapRight(), HEIGHT);
        if (map == null || map.isRecycled()) {
            p.setShader(null);
            p.setStyle(Paint.Style.FILL);
            p.setColor(Color.BLACK);
            c.drawRect(scratchIRect, p);
            JSONObject l = layout(s);
            int waitSave = beginElement(c, l, "mapWait", mapCenterX(), 240f);
            text(c, p, lang("TMAP 화면 대기", "WAITING FOR TMAP"), mapCenterX(), 240f, 34f,
                    Color.GRAY, Paint.Align.CENTER);
            c.restoreToCount(waitSave);
            return;
        }
        p.setFilterBitmap(true);
        int mapSave = c.save();
        c.drawBitmap(map, null, scratchIRect, p);
        c.restoreToCount(mapSave);

        // TMAP 캡처는 앱의 주간 지도가 그대로 들어오므로 전체 HUD 야간
        // 오버레이만으로는 흰 배경이 지나치게 밝다. 야간 테마일 때 지도
        // 영역에만 짙은 남청색 마스크를 추가한다. TBT 배너는 이 다음에
        // 그리므로 안내 정보의 원래 밝기와 색상은 유지된다.
        if (frameDark) {
            p.setShader(null);
            p.setStyle(Paint.Style.FILL);
            p.setColor(Color.argb(155, 2, 9, 20));
            c.drawRect(scratchIRect, p);
        }

        int overlaySave = c.save();
        JSONObject l = layout(s);
        // 배너 위끝(0)이 곧 기준점이어야 패널 최상단에 딱 붙는다. 기준점이 71 이면
        // 배율이 1 이 아닐 때 71x(1-배율) 만큼 아래로 밀린다.
        int save = beginElement(c, l, "tbt1", 1139f, 0f);
        float tbtBottom = drawTbtBanner(c, p, s.optJSONObject("navi"), 962f, 0f);
        c.restoreToCount(save);
        // 세로 기준점(py)을 1행과 같은 71 로 맞춘다. 순정 화면에서는
        // beginElement 가 py 를 중심으로 위젯 배율을 역보정하는데, 1행(71)과
        // 2행(190)의 기준점이 119px 떨어져 있어서 배율이 1 이 아닌 순간
        // 그 차이만큼(119 x (1-배율)) 두 배너 사이가 벌어졌다.
        // 기준점을 1행과 같은 0 으로 맞춘 뒤로는 별도 보정이 필요 없다.
        // 예전 보정(-18px)이 남아 있으면 1행 아래끝을 파고들어 겹친다.
        int save2 = beginElement(c, l, "tbt2", 1144f, 0f);
        drawTbtNext(c, p, s.optJSONObject("navi"), 962f, tbtBottom);
        c.restoreToCount(save2);
        // 분기 실사도 1행 바로 아래에 붙어야 하므로 같은 기준점을 쓴다.
        int junctionSave = beginElement(c, l, "junction", MAP_LEFT + 172f, 0f);
        // 분기 실사는 1행 배너 바로 아래에서 시작해 2행(다음 회전) 배너를 덮는다.
        // 그리기 순서상 2행보다 뒤라 별도 처리 없이 위에 얹힌다.
        drawJunction(c, p, tbtBottom);
        c.restoreToCount(junctionSave);
        // 기준점을 패널 아래끝으로 두면 배율이 어떻든 아래끝에 붙은 채로 남는다.
        int etaSave = beginElement(c, l, "eta", MAP_LEFT + 172f, (float) HEIGHT);
        drawNaviEta(c, p, s);
        c.restoreToCount(etaSave);

        int save3 = beginElement(c, l, "lane", 1395f, (float) HEIGHT);
        drawNativeOverlay(c, p, lane, 1130f, 378f, 1660f, (float) HEIGHT, Paint.Align.CENTER);
        c.restoreToCount(save3);

        // NOO 안내는 지도 패널이 아니라 주행 패널 중앙에 그린다(drawNooTurn).
        c.restoreToCount(overlaySave);
    }

    /**
     * 티맵 분기 실사 이미지. TBT 배너 바로 아래에 폰과 같은 순서로 붙인다.
     * 파일이 사라지면(안내 종료) EON 이 빈 자산을 보내 비트맵이 null 이 되므로
     * 별도의 표시 조건이 필요 없다.
     */
    private void drawJunction(Canvas c, Paint p, float top) {
        if (junctionMode == 0) {
            return;
        }
        Bitmap image = crossroadFrame.get();
        if (image == null || image.isRecycled() || image.getWidth() <= 0 || image.getHeight() <= 0) {
            return;
        }
        float width = JUNCTION_W;
        float height = width * image.getHeight() / (float) image.getWidth();
        if (top + height > JUNCTION_BOTTOM_MAX) {
            height = JUNCTION_BOTTOM_MAX - top;
            width = height * image.getWidth() / (float) image.getHeight();
        }
        if (width < 40f || height < 30f) {
            return;
        }
        float left = JUNCTION_LEFT;
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setAlpha(255);
        p.setFilterBitmap(true);
        scratchRect.set(left, top, left + width, top + height);
        c.drawBitmap(image, null, scratchRect, p);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(2f);
        p.setColor(TBT_GREEN_DARK);
        c.drawRect(left, top, left + width, top + height, p);
    }

    /** 폰 티맵 하단처럼 도착시각 · 남은 분 · 남은 km 를 한 줄로. */
    private void drawNaviEta(Canvas c, Paint p, JSONObject s) {
        if (junctionMode < 2) {
            return;
        }
        JSONObject navi = s.optJSONObject("navi");
        if (navi == null || !navi.optBoolean("active", false)) {
            return;
        }
        int remainTime = navi.optInt("remainTime", 0);
        int remainDist = navi.optInt("remainDist", 0);
        if (remainTime <= 0 && remainDist <= 0) {
            return;
        }
        float left = JUNCTION_LEFT;
        float top = ETA_TOP;
        float right = left + JUNCTION_W;
        float bottom = top + ETA_H;
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.rgb(248, 249, 250));
        scratchRect.set(left, top, right, bottom);
        c.drawRoundRect(scratchRect, 8f, 8f, p);

        String eta = "--:--";
        if (remainTime > 0) {
            eta = new SimpleDateFormat("HH:mm", Locale.KOREA)
                    .format(new Date(System.currentTimeMillis() + remainTime * 1000L));
        }
        String minutes = remainTime > 0 ? Integer.toString(Math.max(1, remainTime / 60)) : "--";
        String kilometres = remainDist >= 1000
                ? String.format(Locale.US, "%.0f", remainDist / 1000.0)
                : (remainDist > 0 ? String.format(Locale.US, "%.1f", remainDist / 1000.0) : "--");
        int value = Color.rgb(20, 24, 28);
        int label = Color.rgb(82, 90, 98);
        float[] columns = {left + JUNCTION_W * 0.24f, left + JUNCTION_W * 0.55f,
                           left + JUNCTION_W * 0.80f};
        String[][] cells = {{eta, lang("도착", "ETA")},
                            {minutes, lang("분", "MIN")},
                            {kilometres, "km"}};
        for (int i = 0; i < columns.length; i++) {
            text(c, p, cells[i][0], columns[i], top + 30f, 30f, value, Paint.Align.CENTER);
            text(c, p, cells[i][1], columns[i], top + 52f, 21f, label, Paint.Align.CENTER);
        }
    }

    private void drawNativeOverlay(Canvas c, Paint p, Bitmap bitmap,
                                   float left, float top, float right, float bottom, Paint.Align align) {
        if (bitmap == null || bitmap.isRecycled() || bitmap.getWidth() <= 0 || bitmap.getHeight() <= 0) {
            return;
        }
        float boundsW = right - left;
        float boundsH = bottom - top;
        float scale = Math.min(boundsW / bitmap.getWidth(), boundsH / bitmap.getHeight());
        float w = bitmap.getWidth() * scale;
        float h = bitmap.getHeight() * scale;
        float x;
        if (align == Paint.Align.RIGHT) {
            x = right - w;
        } else if (align == Paint.Align.CENTER) {
            x = (left + right) * 0.5f - w / 2f;
        } else {
            x = left;
        }
        float y = top + (boundsH - h) / 2f;
        p.setAlpha(255);
        p.setFilterBitmap(true);
        scratchRect.set(x, y, x + w, y + h);
        c.drawBitmap(bitmap, null, scratchRect, p);
    }

    // ── 레이아웃 헬퍼 ─────────────────────────────────────────────────────

    private JSONObject layout(JSONObject s) {
        JSONObject l = s == null ? null : s.optJSONObject("layout");
        return l == null ? new JSONObject() : l;
    }

    private float lv(JSONObject l, String key, float def) {
        double v = l.optDouble(key, def);
        return Double.isFinite(v) ? (float) v : def;
    }

    private int lc(JSONObject l, String key, int def) {
        int v = l.optInt(key, -1);
        return v < 0 ? def : Color.rgb((v >> 16) & 255, (v >> 8) & 255, v & 255);
    }

    private int beginElement(Canvas c, JSONObject l, String name, float px, float py) {
        int save = c.save();
        float dx = lv(l, name + "Dx", 0f);
        float dy = lv(l, name + "Dy", 0f);
        float scale = Math.max(0.5f, Math.min(2f, lv(l, name + "Scale", 1f)));
        c.translate(dx, dy);
        c.scale(scale, scale, px, py);
        return save;
    }

    private static void text(Canvas c, Paint p, String value, float x, float y,
                             float size, int color, Paint.Align align) {
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setTypeface(Typeface.create("sans", Typeface.BOLD));
        p.setTextSize(size);
        p.setTextAlign(align);
        p.setColor(color);
        c.drawText(value, x, y, p);
    }

    // ── 종료 ──────────────────────────────────────────────────────────────

    private void recycleRef(AtomicReference<Bitmap> ref) {
        Bitmap old = ref.getAndSet(null);
        if (old != null) {
            old.recycle();
        }
    }

    @Override
    public void onDestroy() {
        running.set(false);
        workersStarted = false;
        serviceRunning = false;
        mapConnected = false;
        usbConnected = false;
        usbError = false;
        usbStatus = "서비스 중지됨";
        measuredFps = 0.0f;
        lastJpegBytes = 0;
        lastRenderElapsed = 0L;
        if (activeInstance == this) {
            activeInstance = null;
        }
        if (display != null) {
            display.close();
        }
        if (egoCar != null) {
            egoCar.recycle();
            egoCar = null;
        }
        if (wheelImage != null) {
            wheelImage.recycle();
            wheelImage = null;
        }
        if (statusIcons != null) {
            statusIcons.recycle();
            statusIcons = null;
        }
        if (outFrame != null) {
            outFrame.recycle();
            outFrame = null;
            outCanvas = null;
        }
        synchronized (phoneFrameLock) {
            if (phoneFrame != null) {
                phoneFrame.recycle();
                phoneFrame = null;
                phoneCanvas = null;
            }
        }
        synchronized (assetLock) {
            recycleRef(mapFrame);
            recycleRef(tbtCurrentFrame);
            recycleRef(tbtNextFrame);
            recycleRef(laneFrame);
        }
        if (usbReceiverRegistered) {
            try {
                unregisterReceiver(usbReceiver);
            } catch (Exception ignored) {
            }
            usbReceiverRegistered = false;
        }
        releaseWakeLock();
        super.onDestroy();
    }

    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }
}
