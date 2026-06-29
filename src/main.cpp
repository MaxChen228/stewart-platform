#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include <stdarg.h>
#include <esp_system.h>            // esp_reset_reason() / ESP.restart()：開機診斷 + wedge 自救
#include <WiFi.h>                  // HTTP-pull OTA 用 WiFiClient
#include <HTTPUpdate.h>            // HTTP-pull OTA：ESP32 outbound 抓 firmware（繞 host 防火牆）
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "servo42d.h"
#include "kinematics.h"
#include "encoder.h"
#include "forward_kinematics.h"
#include "control.h"
#include "net_transport.h"

// 韌體源碼身分：build 時由 sysid/fw_hash.js 烤入 src/fw_identity.h（gitignored）。
// 缺檔/未經 stamp 時退化為 "unknown" 仍可編譯。server.js /api/firmware 以此判「燒錄 vs 最新」。
#if defined(__has_include)
#  if __has_include("fw_identity.h")
#    include "fw_identity.h"
#  endif
#endif
#ifndef FW_SRC_HASH
#  define FW_SRC_HASH "unknown"
#endif
#ifndef FW_GIT_REV
#  define FW_GIT_REV "unknown"
#endif
#ifndef FW_DIRTY
#  define FW_DIRTY 0
#endif
#ifndef FW_BUILD_EPOCH
#  define FW_BUILD_EPOCH 0UL
#endif

// esp_reset_reason → 人類可讀字串（開機診斷 brownout/panic/watchdog 用）。
static const char* resetReasonStr(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:   return "poweron";
        case ESP_RST_EXT:       return "ext";
        case ESP_RST_SW:        return "sw";
        case ESP_RST_PANIC:     return "panic";
        case ESP_RST_INT_WDT:   return "int_wdt";
        case ESP_RST_TASK_WDT:  return "task_wdt";
        case ESP_RST_WDT:       return "wdt";
        case ESP_RST_BROWNOUT:  return "brownout";
        case ESP_RST_DEEPSLEEP: return "deepsleep";
        case ESP_RST_SDIO:      return "sdio";
        default:                return "unknown";
    }
}

Servo42D servos;
EncoderState enc;

// ===== 位置模式控制 =====
Pose targetPose = {0, 0, NEUTRAL_Z, 0, 0, 0};

// ===== Auto-return（item 1）：馬達定時主動上報，ESP32 連續排空、控制環只取快照 =====
// MCP2515 只有 2 RX buffer，6 顆 streaming 必須高頻排空；core0 canRxTask 專職接收，
// core1 控制環只在 CAN mutex 下取 latestRaw 快照，避免 int64 tear 與 SPI 競態。
// 預設開啟：行為仍可用 A0 切回輪詢版。
int64_t latestRaw[NUM_MOTORS];
uint32_t latestRawAtMs[NUM_MOTORS] = {0};
volatile bool autoReturnMode = false;
volatile bool canRxPaused = false;
constexpr uint16_t AR_PERIOD_MIN_MS = 1;   // SERVO42D 0x01 timer 整數 ms 硬體下限（=1000Hz）。人為上限應使用者要求移除。
                                           // ⚠️ 匯流排權衡（使用者自負）：6×0x35 與 F5 共用 500kbps；AR<~10ms(>100Hz) 會與 F5 搶頻寬
                                           // → 控制變鈍（非 brick、非亂動，馬達內環會保持位置）。boot 預設 arPeriodMs=20(50Hz) 仍安全，
                                           // 只有明確設高值才持久化。AR>telemetry率(~20-33Hz) 無觀測實益，純供進階/原始編碼器擷取用途。
constexpr uint16_t AR_PERIOD_MAX_MS = 100;
uint16_t arPeriodMs = 20;                  // boot 預設 50Hz（不綁 MIN，否則開機即 1ms×6 灌爆 500kbps 匯流排）；馬達主動上報週期，整數 ms
constexpr uint32_t AR_FRESH_MS = 120; // 啟動/重啟 auto-return 時用來判斷「近期收到過」的軟門檻
constexpr uint32_t AR_WARN_MS = 1000; // 長時間沒新 frame 才警告；短缺包沿用 last-known-good，不停機
bool autoReturnWarnLatched = false;

// ===== 可調速率（item 2 of 本輪）=====
volatile uint32_t loopPeriodUs = 10000;       // 控制迴圈週期，可調（L 指令，單位 ms→µs）；預設 100Hz（10ms）
const uint32_t TELE_PERIOD_MS = 30;  // 遙測輸出週期（與控制解耦，固定 ~33Hz，免塞爆 115200 序列埠）
bool posEnabled = false;
float enableAngle[NUM_MOTORS] = {0};

uint16_t posSpeed = 120;
uint8_t  posAcc   = 100;
uint16_t workingCurrentMa = 1600;
constexpr uint32_t HOLD_PROFILE_MAX_MS = 20000;
float holdProfileMaxVelDps = 0.0f;    // 0=respect requested P duration; >0 caps min-jerk target peak velocity

// ===== 匯流排佔用統計（每遙測窗計數，print 後歸零）：實測幀數，client 換算 500kbps 佔用% =====
uint16_t busTxF5  = 0;   // F5 指令幀（DLC 8）
uint16_t busTxQ   = 0;   // 0x35 輪詢查詢幀（DLC 2，每幀必有一筆 DLC 8 回覆在線上）
uint16_t busRxEnc = 0;   // auto-return 實收 0x35 廣播幀（DLC 8）
// [Phase0 診斷] 量完移除：EFLG RX overflow(0xC0) 出現的遙測窗次數 + 最近一次 6×F5 burst µs
uint16_t busOvr   = 0;
uint32_t f5us     = 0;
uint32_t f5usMax  = 0;

// ===== Telemetry v2 window counters =====
uint32_t ctlIkFailWin = 0;
uint32_t ctlFkFailWin = 0;
uint32_t ctlCoordLimitWin = 0;
int32_t  ctlCoordMaxAbs = 0;

TaskHandle_t canRxTaskHandle = nullptr;
TaskHandle_t controlTaskHandle = nullptr;
TaskHandle_t telemetryTaskHandle = nullptr;

struct QueuedCommand {
    char cmd[160];
    bool fromNet;
};

enum TelemetryExtra : uint8_t { TELEM_EXTRA_NONE = 0, TELEM_EXTRA_HOLD = 1, TELEM_EXTRA_TASK = 2, TELEM_EXTRA_JOINT = 3 };

struct TelemetrySnapshot {
    float angles[NUM_MOTORS];
    int64_t raw[NUM_MOTORS];
    int ok;
    int zeroed;
    int pos;
    int controlMode;
    uint8_t eflg, tec, rec;
    uint32_t t, dtMs, canReadUs;
    float lhz;
    uint16_t arMs;
    uint16_t busF5, busQ, busRx, busMs, busPer[NUM_MOTORS], busOvr, arAge[NUM_MOTORS];
    uint32_t f5us, f5max;
    uint8_t maxDrain;
    uint32_t teleCycles, dtMinMs, dtMaxMs;
    float dtAvgMs;
    uint32_t ctlIkFail, ctlFkFail, ctlCoordLimit;
    int32_t ctlCoordMaxAbs;
    int fkStreak;
    uint16_t pid[4];
    int pidSaved;
    uint16_t savedPid[4];
    int nvsPid;
    int followMode;
    uint8_t profile;
    float followCurPose[6], followTgtPose[6], followVelPose[6];
    float followVmax[2], followAmax[2], followJmax[2];
    float followTight;
    TelemetryExtra extra;
    float hold[NUM_MOTORS], herr[NUM_MOTORS], hmax;
    float holdDampGain, holdDampMax, holdDampCorrMax;
    float holdIntC[NUM_MOTORS], holdKiSnap[NUM_MOTORS];   // 積分輸出（角度域）+ 當前增益
    float holdIClamp, holdIDead, holdISettle;             // 護欄快照（供前端反映真值）
    int holdIntSaved;                                     // dirty flag（已存=1）
    float fkPose[6], poseErr[6], motorTgt[NUM_MOTORS];
    int fkIterations;
    float jointGain;
    uint32_t ctrlUs, ctrlMaxUs, jitterUs, missed, teleDrop, cmdDrop, snapAgeMs;
};

static QueueHandle_t commandQueue = nullptr;
static QueueHandle_t telemetryQueue = nullptr;
static volatile uint32_t cmdDropCount = 0;
static volatile uint32_t teleDropCount = 0;
static volatile uint32_t controlMissedCount = 0;
static volatile uint32_t controlMaxUs = 0;
static volatile bool otaStopRequested = false;

static bool enqueueCommand(const String& cmd, bool fromNet);
static void emitTelemetry(const TelemetrySnapshot& s);
static void controlTick();
void handleSerial();

// ===== HOLD 模式 =====
// 純 passthrough：snapshot 當前 enc.angles，每 cycle 直送 F5，跳過 PD/IK/平滑
// 用來測馬達內環硬度
bool holdMode = false;
float holdAngles[NUM_MOTORS] = {0};
float maxHoldErr = 0;

// Runtime output damping for HOLD/min-jerk/FOLLOW targets.
// OD <gain_sec> [max_deg] [deadband_dps]; OD 0 disables.
float holdDampGainSec = 0.0f;
float holdDampMaxDeg = 1.5f;
float holdDampDeadbandDps = 1.0f;
constexpr float HOLD_DAMP_TAU = 0.06f;
float holdDampPrevAngle[NUM_MOTORS] = {0};
float holdDampVel[NUM_MOTORS] = {0};
float holdDampLastCorr[NUM_MOTORS] = {0};
float holdDampCorrMaxWin = 0.0f;
bool holdDampInit = false;

// ===== HOLD joint-domain 積分（路線甲：消重力 droop）=====
// 積分搬到 ESP32 上層、角度域、每腿各補；與 applyHoldOutputDamping 同域、互不爭速度閘。
// 誤差 err = holdAngles - enc.angles（命令−實際）：droop（實際低於命令）→ err>0 → corr>0 → 命令加大 → enc 追上。
// BOOT Ki=0 → 輸出恆 0 → 行為與純P 死咬完全相同（向後相容）。
constexpr float BOOT_HOLD_KI = 0.0f;             // BOOT 預設積分增益（0=關，向後相容）
constexpr float BOOT_HOLD_CLAMP_DEG = 3.0f;      // 積分輸出角度域硬上限（anti-windup 最終安全閥）
constexpr float BOOT_HOLD_DEADBAND_DEG = 0.15f;  // 誤差死區
constexpr float BOOT_HOLD_SETTLE_DPS = 8.0f;     // 安定速度閾值
float holdKi[NUM_MOTORS] = {0};          // 每軸積分增益（0=關）；單一真相源，K/UI 改它、telemetry 上報
float holdIClampDeg = BOOT_HOLD_CLAMP_DEG;       // |vel|>此值不積分（避免運動/振盪峰 windup）
float holdIDeadbandDeg = BOOT_HOLD_DEADBAND_DEG; // |err|<此值不積分（避免噪聲 windup）
float holdISettleDps = BOOT_HOLD_SETTLE_DPS;
ScalarIntegrator holdInt[NUM_MOTORS];
float holdIntPrevAngle[NUM_MOTORS] = {0};
float holdIntCorr[NUM_MOTORS] = {0};     // 遙測：每軸積分輸出（角度域）
bool  holdIntInit = false;
// 持久化（對齊 vFOC PID 模式）：saved 值 + dirty flag
float savedHoldKi[NUM_MOTORS] = {0};
float savedHoldClampDeg = BOOT_HOLD_CLAMP_DEG;
float savedHoldDeadbandDeg = BOOT_HOLD_DEADBAND_DEG;
float savedHoldSettleDps = BOOT_HOLD_SETTLE_DPS;
bool  nvsHoldIntPresent = false;
bool  holdIntCfgSaved = false;

// 擾動注入器（U/W 指令）：對 F5 目標短暫加偏移持續 ms 後歸零。疊加在 F5 匯流口 → 全控制模式通用
// （HOLD 保持中或運動中皆可注入）。U=單軸、W=六軸協同。
float    bumpDeg[NUM_MOTORS] = {0};   // 六軸擾動偏移（enc 慣例度）
uint32_t bumpUntilMs = 0;            // 0=無擾動，否則=結束時刻(millis)；過期由 F5 匯流口 clearBump 歸零
constexpr uint32_t BUMP_MAX_MS = 60000;  // 擾動時長上限（放寬自 5s→60s，支援「持續一段時間」）

// 姿態軌跡（P 指令）：HOLD 下把 holdAngles 從 holdStart 平滑移到 holdTarget=IK(絕對 pose)。
// 保留 HOLD 直送機制 → 馬達純 P 死咬移動中的目標 = 穩定性不變，只是目標會動。
// holdTarget 由韌體 IK 解絕對 pose 而來，保證對應有效剛體姿態（非相對增量，杜絕 over-constrain 較勁）。
float    holdTarget[NUM_MOTORS] = {0};   // 移動目標 = IK(目標絕對 pose)
float    holdStart[NUM_MOTORS]  = {0};   // 本次移動起點（發 P 當下的 holdAngles）
uint32_t holdMoveStart = 0, holdMoveMs = 0;   // 移動起始時刻、總時長（0=不在移動）

static float minJerk01(float t) {
    t = constrain(t, 0.0f, 1.0f);
    return t * t * t * (10.0f + t * (-15.0f + 6.0f * t));
}

static uint32_t constrainedHoldMoveMs(float requestedMs, const float startAngles[NUM_MOTORS], const float targetAngles[NUM_MOTORS]) {
    float maxDelta = 0.0f;
    for (int i = 0; i < NUM_MOTORS; i++) {
        float d = fabsf(targetAngles[i] - startAngles[i]);
        if (d > maxDelta) maxDelta = d;
    }
    uint32_t requested = (uint32_t)constrain((int)requestedMs, 1, (int)HOLD_PROFILE_MAX_MS);
    if (holdProfileMaxVelDps <= 0.0f) return requested;
    float maxVel = fmaxf(1.0f, holdProfileMaxVelDps);
    uint32_t byVel = (uint32_t)ceilf(1000.0f * 1.875f * maxDelta / maxVel);
    return constrain((int)max(requested, byVel), 1, (int)HOLD_PROFILE_MAX_MS);
}

static void resetHoldDampingState() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        holdDampPrevAngle[i] = enc.angles[i];
        holdDampVel[i] = 0.0f;
        holdDampLastCorr[i] = 0.0f;
    }
    holdDampCorrMaxWin = 0.0f;
    holdDampInit = true;
}

static void resetHoldIntegral() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        holdInt[i].reset();
        holdIntPrevAngle[i] = enc.angles[i];   // seed prevAngle → 首個 vel 不是 spike
        holdIntCorr[i] = 0.0f;
    }
    holdIntInit = true;
}

static void applyHoldOutputDamping(float targets[NUM_MOTORS], float dt) {
    if (holdDampGainSec <= 0.0f || holdDampMaxDeg <= 0.0f || dt <= 0.0f) {
        for (int i = 0; i < NUM_MOTORS; i++) holdDampLastCorr[i] = 0.0f;
        holdDampInit = false;
        return;
    }
    if (!holdDampInit) resetHoldDampingState();

    float alpha = constrain(dt / (HOLD_DAMP_TAU + dt), 0.0f, 1.0f);
    for (int i = 0; i < NUM_MOTORS; i++) {
        float rawVel = (enc.angles[i] - holdDampPrevAngle[i]) / dt;
        rawVel = constrain(rawVel, -720.0f, 720.0f);
        holdDampVel[i] += alpha * (rawVel - holdDampVel[i]);
        holdDampPrevAngle[i] = enc.angles[i];

        float v = fabsf(holdDampVel[i]) < holdDampDeadbandDps ? 0.0f : holdDampVel[i];
        float corr = constrain(-holdDampGainSec * v, -holdDampMaxDeg, holdDampMaxDeg);
        targets[i] += corr;
        holdDampLastCorr[i] = corr;
        float ac = fabsf(corr);
        if (ac > holdDampCorrMaxWin) holdDampCorrMaxWin = ac;
    }
}

// HOLD 積分外環：每腿在角度域慢速消 droop。必須在 motorTargets=holdAngles 之後、
// applyHoldOutputDamping 之前呼叫（積分先疊命令，阻尼再看 enc 速度）。
// active = 主閘（純死咬靜止 + 編碼器新鮮），由呼叫端用既有旗標組成；false 則凍結並輸出 0 修正。
// 函式只管積分數學與次要閘（物理靜止 + 死區），gate 條件外部決定 → 不依賴 follow 全域宣告順序。
static void applyHoldIntegral(float targets[NUM_MOTORS], float dt, bool active) {
    if (dt <= 0.0f) return;
    if (!holdIntInit) resetHoldIntegral();

    for (int i = 0; i < NUM_MOTORS; i++) {
        float vel = (enc.angles[i] - holdIntPrevAngle[i]) / dt;
        holdIntPrevAngle[i] = enc.angles[i];               // 每 cycle 更新，gate 關也更新避免 vel spike
        float err = holdAngles[i] - enc.angles[i];          // 命令 − 實際；droop→err>0
        bool gated = active
                  && fabsf(vel) < holdISettleDps            // 物理靜止（次要閘）
                  && fabsf(err) > holdIDeadbandDeg;          // 死區外才積分
        float corr = holdInt[i].update(err, dt, holdKi[i], holdIClampDeg, gated);
        holdIntCorr[i] = corr;
        targets[i] += corr;
    }
}

// 跟隨模式（FOLLOW/PF）：HOLD 下接 host 的「最新 pose 目標」，ESP32 在控制 tick 內產生唯一物理 reference。
// 與 P minimum-jerk profile 互斥（follow on 時 followStep 接管 holdAngles，忽略 holdMoveMs）。
// 設計：host/browser/server 只更新 followTgt；followStep 以固定 control loop 做 acceleration/jerk limited
// online trajectory。這避免低頻/抖動 PF 目標直接變成馬達階梯，也避免 server/browser 成為物理時鐘。
bool  followMode = false;
float followCur[6] = {0, 0, NEUTRAL_Z, 0, 0, 0};   // 當前濾波後 pose（IK 對象）
float followTgt[6] = {0, 0, NEUTRAL_Z, 0, 0, 0};   // 串流進來的目標 pose（PF 設）
float followVel[6] = {0, 0, 0, 0, 0, 0};            // pose reference velocity（mm/s 或 deg/s）
float followAcc[6] = {0, 0, 0, 0, 0, 0};            // pose reference acceleration
float followTight  = 0.5f;     // 跟隨「緊度」單參數 ∈[0,1]：越大跟越緊（FE 可調，runtime，不存 NVS）
float followVmaxT  = 77.5f;    // 平移速度上限 mm/s（tight 派生；VF 可底層覆寫）
float followVmaxR  = 57.5f;    // 旋轉速度上限 deg/s（tight 派生）
float followAccelT = 0.1925f;  // 加速時間常數 s（tight 派生；越小起步越猛）= 0.7·τ
float followJerkT  = 0.09625f; // jerk 時間常數 s（tight 派生；越小切向越敏捷）= 0.35·τ
float followApproachK = 3.6364f; // first-order 趨近增益 1/s = 1/τ；v 上限隨剩餘誤差線性收斂 →「越近越慢」ease-out
constexpr float FOLLOW_EPS_T = 0.003f;        // mm，到目標後 snap，避免微小尾巴造成 keep-alive 抖動。
constexpr float FOLLOW_EPS_R = 0.002f;        // deg
// ↑ 這些靜態 init = applyFollowTight(0.5) 之結果；但 setup() 開機會呼 applyFollowTight(0.7) 覆寫成預設緊度 0.7。

static inline float followVmaxForAxis(int i) {
    return i < 3 ? followVmaxT : followVmaxR;
}

static inline float followAmaxForAxis(int i) {
    return followVmaxForAxis(i) / followAccelT;
}

static inline float followJmaxForAxis(int i) {
    return followAmaxForAxis(i) / followJerkT;
}

// 跟隨緊度單參數 tight∈[0,1] → 一鈕派生四個物理量（全部單調隨 tight，越大跟越緊）：
//   τ       = 0.45→0.10 s   趨近時間常數（緊→短）
//   vmaxT/R 線性插值          速度上限（緊→快）
//   accelT  = 0.7·τ          加速時間（緊→短，起步更猛）
//   jerkT   = 0.35·τ         jerk 時間（緊→短，切向更敏捷）
//   K       = 1/τ            趨近增益（緊→大，收尾更緊、誤差容忍更小）
// accelT 取 0.7·τ 保證 amax=vmax/accelT ≥ K·vmax（headroom 1/0.7≈1.43）→ ease-out 永不被 accel 限幅切掉。
static void applyFollowTight(float tight) {
    tight = constrain(tight, 0.0f, 1.0f);
    followTight = tight;
    const float tau = 0.45f - (0.45f - 0.10f) * tight;
    followVmaxT = 35.0f + (120.0f - 35.0f) * tight;
    followVmaxR = 25.0f + (90.0f  - 25.0f) * tight;
    followAccelT = 0.7f  * tau;
    followJerkT  = 0.35f * tau;
    followApproachK = 1.0f / tau;
}

static void resetFollowDynamics(const float init[6]) {
    for (int i = 0; i < 6; i++) {
        followCur[i] = init[i];
        followTgt[i] = init[i];
        followVel[i] = 0.0f;
        followAcc[i] = 0.0f;
    }
}

static inline Pose poseFromArr(const float a[6]) {
    return Pose{a[0], a[1], a[2], a[3], a[4], a[5]};
}

static void followAxisCandidate(int i, float dt, float cand[6], float velNext[6], float accNext[6]) {
    const float vmax = followVmaxForAxis(i);
    const float amax = followAmaxForAxis(i);
    const float jmax = followJmaxForAxis(i);
    const float eps = i < 3 ? FOLLOW_EPS_T : FOLLOW_EPS_R;

    const float x = followCur[i];
    const float err = followTgt[i] - x;
    const float v = followVel[i];
    const float a = followAcc[i];

    if (fabsf(err) <= eps && fabsf(v) <= eps / fmaxf(dt, 0.001f) && fabsf(a) <= amax * 0.01f) {
        cand[i] = followTgt[i];
        velNext[i] = 0.0f;
        accNext[i] = 0.0f;
        return;
    }

    const float dir = err >= 0.0f ? 1.0f : -1.0f;
    const float vBrake = sqrtf(fmaxf(0.0f, 2.0f * amax * fabsf(err)));
    // first-order 趨近項：速度上限隨剩餘誤差線性收斂（K·|err|）→ 接近目標自動放慢（ease-out 曲線感）；
    // 遠離時 K·|err|≫vmax 被 vmax 蓋過 → 全速撲。三者取 min：vmax(撲速)/vBrake(不過衝)/K·err(柔順收尾)。
    const float vApproach = followApproachK * fabsf(err);
    const float vDesired = dir * fminf(fminf(vmax, vBrake), vApproach);
    float aDesired = constrain((vDesired - v) / fmaxf(dt, 0.001f), -amax, amax);
    // jerk-aware stop：煞車加速度上限 |a| ≤ √(2·jmax·|v|)，保證能在 v 歸零前把 a ramp 回 0。
    // 否則 a 釘在 -amax、jerk 限制來不及收力，v 被推穿 0 → 反向過衝 ring（v 倒退、彈回）。
    // 離線確定性模擬驗證（sysid/follow_sim.js §9）：中段 tight 過衝 ring -70~86%，rise/settle 零代價。
    if (aDesired * v < 0.0f && fabsf(v) > 1e-6f) {
        const float aStop = sqrtf(2.0f * jmax * fabsf(v));
        if (fabsf(aDesired) > aStop) aDesired = (aDesired >= 0.0f ? aStop : -aStop);
    }
    const float da = constrain(aDesired - a, -jmax * dt, jmax * dt);
    float nextA = constrain(a + da, -amax, amax);
    float nextV = constrain(v + nextA * dt, -vmax, vmax);
    float nextX = x + nextV * dt;

    if ((followTgt[i] - x) * (followTgt[i] - nextX) <= 0.0f) {
        nextX = followTgt[i];
        nextV = 0.0f;
        nextA = 0.0f;
    }

    cand[i] = nextX;
    velNext[i] = nextV;
    accNext[i] = nextA;
}

// 跟隨生成器一步：acceleration/jerk limited pose reference → IK → holdAngles。
// IK 無效（拖出工作空間）→ 維持上一有效點，並清速度/加速度，避免重新進可達區時帶著舊動量衝出去。
static void followStep(float dt) {
    dt = constrain(dt, 0.001f, 0.02f);
    float cand[6], velNext[6], accNext[6];
    for (int i = 0; i < 6; i++) followAxisCandidate(i, dt, cand, velNext, accNext);

    IKResult ik = inverse_kinematics(poseFromArr(cand));
    if (ik.valid) {
        for (int i = 0; i < 6; i++) {
            followCur[i] = cand[i];
            followVel[i] = velNext[i];
            followAcc[i] = accNext[i];
            holdAngles[i] = ik.angles[i];
        }
    } else {
        ctlIkFailWin++;
        for (int i = 0; i < 6; i++) {
            followVel[i] = 0.0f;
            followAcc[i] = 0.0f;
        }
    }
}

static void controlTask(void*) {
    uint32_t nextUs = micros();
    for (;;) {
        controlTick();
        uint32_t periodUs = loopPeriodUs;
        if (periodUs < 1000) periodUs = 1000;
        nextUs += periodUs;

        uint32_t nowUs = micros();
        int32_t waitUs = (int32_t)(nextUs - nowUs);
        if (waitUs < 0 || waitUs > (int32_t)periodUs) {
            nextUs = nowUs + periodUs;
            waitUs = periodUs;
        }

        while (waitUs > 0) {
            if (waitUs > 2000) {
                vTaskDelay(pdMS_TO_TICKS((waitUs - 1000) / 1000));
            } else if (waitUs > 1100) {
                // 高控制率（小週期）下，剩餘閒置不足 2 tick 但仍 >1ms：強制讓出 1 tick，
                // 否則一路 busy-wait(delayMicroseconds 不 yield) 會餓死同核低優先 loopTask
                // （serial/net 指令吞吐）→ PF 在 UART RX 堆積成秒級 backlog。實測 400Hz wedge 根因。
                vTaskDelay(1);
            } else {
                delayMicroseconds(waitUs > 100 ? 100 : waitUs);
            }
            waitUs = (int32_t)(nextUs - micros());
        }
    }
}

static void telemetryTask(void*) {
    TelemetrySnapshot snap;
    for (;;) {
        if (telemetryQueue && xQueueReceive(telemetryQueue, &snap, portMAX_DELAY) == pdTRUE) {
            uint32_t now = millis();
            snap.snapAgeMs = now >= snap.t ? now - snap.t : 0;
            emitTelemetry(snap);
        }
    }
}

static void startRealtimeTasks() {
    if (!commandQueue) commandQueue = xQueueCreate(32, sizeof(QueuedCommand));
    if (!telemetryQueue) telemetryQueue = xQueueCreate(1, sizeof(TelemetrySnapshot));
    if (!telemetryTaskHandle)
        xTaskCreatePinnedToCore(telemetryTask, "telemetry", 8192, nullptr, 1, &telemetryTaskHandle, 0);
    if (!controlTaskHandle)
        xTaskCreatePinnedToCore(controlTask, "control", 8192, nullptr, 4, &controlTaskHandle, 1);
}

void loop() {
    // OTA 已移至 core0 netTask 服務（避免被高優先 controlTask 餓死）；此處不再呼叫 netOtaHandle()。
    handleSerial();

    String netCmd;
    while (netNextCommand(netCmd)) {
        enqueueCommand(netCmd, true);
    }

    vTaskDelay(pdMS_TO_TICKS(1));
}

// 開機預設 vFOC PID：純P工作點 [1024,0,0,0]（2026-06-26 定案）。
// Ki=0 滅自持極限環根因（積分撞死區）；maxKp 補剛性；F5 posSpeed 提供阻尼；
// Kd=0 因離散微分=高頻放大器→嘯叫，Kv=0 不需。
// 不靠馬達 EEPROM：ESP32 開機把「已保存 PID」下發到六顆馬達；若 NVS 無保存值才用 BOOT。
// K = 暫時套用；KS = 明確保存；KRESET = 清保存值並回 BOOT。詳見記憶 project_pid_working_point。
constexpr uint16_t BOOT_KP = 1024, BOOT_KI = 0, BOOT_KD = 0, BOOT_KV = 0;

// 當前 vFOC PID（單一真相源）：開機=NVS 已保存值或 BOOT，收 K 指令更新，telemetry 上報 → 前端反映真值不寫死
uint16_t curKp = BOOT_KP, curKi = BOOT_KI, curKd = BOOT_KD, curKv = BOOT_KV;
uint16_t savedKp = BOOT_KP, savedKi = BOOT_KI, savedKd = BOOT_KD, savedKv = BOOT_KV;
bool nvsPidPresent = false;
bool pidSaved = false;

struct VFOCPidNVS {
    uint32_t magic;
    uint16_t version;
    uint16_t kp, ki, kd, kv;
};

constexpr uint32_t VFOC_PID_MAGIC = 0x56464350; // "VFCP"
constexpr uint16_t VFOC_PID_VERSION = 1;

static bool validPidValue(uint16_t v) {
    return v <= 1024;
}

static bool validPidConfig(const VFOCPidNVS& cfg) {
    return cfg.magic == VFOC_PID_MAGIC &&
           cfg.version == VFOC_PID_VERSION &&
           validPidValue(cfg.kp) && validPidValue(cfg.ki) &&
           validPidValue(cfg.kd) && validPidValue(cfg.kv);
}

static bool loadVFOCPidFromNVS() {
    Preferences prefs;
    VFOCPidNVS cfg;
    prefs.begin("stewart", true);
    size_t len = prefs.getBytes("vfocPidV1", &cfg, sizeof(cfg));
    prefs.end();
    if (len != sizeof(cfg) || !validPidConfig(cfg)) return false;
    curKp = cfg.kp; curKi = cfg.ki; curKd = cfg.kd; curKv = cfg.kv;
    savedKp = cfg.kp; savedKi = cfg.ki; savedKd = cfg.kd; savedKv = cfg.kv;
    nvsPidPresent = true;
    return true;
}

static bool currentPidMatchesSaved() {
    return nvsPidPresent &&
           curKp == savedKp && curKi == savedKi &&
           curKd == savedKd && curKv == savedKv;
}

static bool saveVFOCPidToNVS() {
    VFOCPidNVS cfg = {VFOC_PID_MAGIC, VFOC_PID_VERSION, curKp, curKi, curKd, curKv};
    Preferences prefs;
    prefs.begin("stewart", false);
    size_t written = prefs.putBytes("vfocPidV1", &cfg, sizeof(cfg));
    prefs.end();
    if (written != sizeof(cfg)) return false;
    savedKp = curKp; savedKi = curKi; savedKd = curKd; savedKv = curKv;
    nvsPidPresent = true;
    return true;
}

static void clearVFOCPidNVS() {
    Preferences prefs;
    prefs.begin("stewart", false);
    prefs.remove("vfocPidV1");
    prefs.end();
    nvsPidPresent = false;
}

// ===== HOLD 積分組態持久化（對齊 vFOC PID 模式）=====
constexpr uint32_t HOLD_INT_MAGIC = 0x484F4C49; // "HOLI"
constexpr uint16_t HOLD_INT_VERSION = 1;
constexpr float HOLD_KI_MAX = 200.0f;          // 每軸積分增益上限
constexpr float HOLD_CLAMP_MAX = 10.0f;        // 積分輸出角度上限的上限
constexpr float HOLD_DEADBAND_MAX = 5.0f;
constexpr float HOLD_SETTLE_MIN = 0.1f, HOLD_SETTLE_MAX = 100.0f;

struct HoldIntNVS {
    uint32_t magic;
    uint16_t version;
    uint16_t _pad;
    float ki[NUM_MOTORS];
    float clampDeg, deadbandDeg, settleDps;
};

static bool validHoldIntConfig(const HoldIntNVS& c) {
    if (c.magic != HOLD_INT_MAGIC || c.version != HOLD_INT_VERSION) return false;
    for (int i = 0; i < NUM_MOTORS; i++)
        if (!isfinite(c.ki[i]) || c.ki[i] < 0.0f || c.ki[i] > HOLD_KI_MAX) return false;
    return isfinite(c.clampDeg)    && c.clampDeg    >= 0.0f && c.clampDeg    <= HOLD_CLAMP_MAX &&
           isfinite(c.deadbandDeg) && c.deadbandDeg >= 0.0f && c.deadbandDeg <= HOLD_DEADBAND_MAX &&
           isfinite(c.settleDps)   && c.settleDps   >= HOLD_SETTLE_MIN && c.settleDps <= HOLD_SETTLE_MAX;
}

static bool currentHoldIntMatchesSaved() {
    if (!nvsHoldIntPresent) return false;
    for (int i = 0; i < NUM_MOTORS; i++) if (holdKi[i] != savedHoldKi[i]) return false;
    return holdIClampDeg == savedHoldClampDeg &&
           holdIDeadbandDeg == savedHoldDeadbandDeg &&
           holdISettleDps == savedHoldSettleDps;
}

static bool loadHoldIntFromNVS() {
    Preferences prefs;
    HoldIntNVS c;
    prefs.begin("stewart", true);
    size_t len = prefs.getBytes("holdIntV1", &c, sizeof(c));
    prefs.end();
    if (len != sizeof(c) || !validHoldIntConfig(c)) return false;
    for (int i = 0; i < NUM_MOTORS; i++) { holdKi[i] = c.ki[i]; savedHoldKi[i] = c.ki[i]; }
    holdIClampDeg    = savedHoldClampDeg    = c.clampDeg;
    holdIDeadbandDeg = savedHoldDeadbandDeg = c.deadbandDeg;
    holdISettleDps   = savedHoldSettleDps   = c.settleDps;
    nvsHoldIntPresent = true;
    return true;
}

static bool saveHoldIntToNVS() {
    HoldIntNVS c = {HOLD_INT_MAGIC, HOLD_INT_VERSION, 0, {0}, holdIClampDeg, holdIDeadbandDeg, holdISettleDps};
    for (int i = 0; i < NUM_MOTORS; i++) c.ki[i] = holdKi[i];
    Preferences prefs;
    prefs.begin("stewart", false);
    size_t w = prefs.putBytes("holdIntV1", &c, sizeof(c));
    prefs.end();
    if (w != sizeof(c)) return false;
    for (int i = 0; i < NUM_MOTORS; i++) savedHoldKi[i] = holdKi[i];
    savedHoldClampDeg    = holdIClampDeg;
    savedHoldDeadbandDeg = holdIDeadbandDeg;
    savedHoldSettleDps   = holdISettleDps;
    nvsHoldIntPresent = true;
    return true;
}

static void clearHoldIntNVS() {
    Preferences prefs;
    prefs.begin("stewart", false);
    prefs.remove("holdIntV1");
    prefs.end();
    nvsHoldIntPresent = false;
}

// KI/KIC/KIS/KIRESET 共用 ack（單一輸出格式，避免重複）
static void emitHoldIntAck(bool reset = false) {
    Out.printf("{\"tune\":\"holdi\",\"saved\":%d,\"reset\":%d,"
        "\"ki\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],"
        "\"clamp\":%.3f,\"deadband\":%.3f,\"settle\":%.3f}\n",
        holdIntCfgSaved ? 1 : 0, reset ? 1 : 0,
        holdKi[0],holdKi[1],holdKi[2],holdKi[3],holdKi[4],holdKi[5],
        holdIClampDeg, holdIDeadbandDeg, holdISettleDps);
}

struct RuntimeTuningNVS {
    uint32_t magic;
    uint16_t version;
    uint16_t currentMa;
    uint16_t f5Speed;
    uint8_t f5Acc;
    uint16_t arMs;
    uint32_t loopUs;
    float odGain;
    float odMaxDeg;
    float odDeadbandDps;
};

constexpr uint32_t RUNTIME_TUNE_MAGIC = 0x5254554E; // "RTUN"
constexpr uint16_t RUNTIME_TUNE_VERSION = 1;

static bool validRuntimeTuning(const RuntimeTuningNVS& cfg) {
    return cfg.magic == RUNTIME_TUNE_MAGIC &&
           cfg.version == RUNTIME_TUNE_VERSION &&
           cfg.currentMa >= 100 && cfg.currentMa <= 3000 &&
           cfg.f5Speed >= 1 && cfg.f5Speed <= 200 &&
           cfg.f5Acc >= 1 && cfg.f5Acc <= 255 &&
           cfg.arMs >= 1 && cfg.arMs <= AR_PERIOD_MAX_MS &&
           cfg.loopUs >= 1000 && cfg.loopUs <= 100000 &&
           isfinite(cfg.odGain) && cfg.odGain >= 0.0f && cfg.odGain <= 0.08f &&
           isfinite(cfg.odMaxDeg) && cfg.odMaxDeg >= 0.0f && cfg.odMaxDeg <= 5.0f &&
           isfinite(cfg.odDeadbandDps) && cfg.odDeadbandDps >= 0.0f && cfg.odDeadbandDps <= 30.0f;
}

static uint16_t clampAutoReturnPeriodMs(float requestedMs) {
    float clampedMs = constrain(requestedMs, (float)AR_PERIOD_MIN_MS, (float)AR_PERIOD_MAX_MS);
    return (uint16_t)constrain((int)floorf(clampedMs + 0.5f), (int)AR_PERIOD_MIN_MS, (int)AR_PERIOD_MAX_MS);
}

static bool loadRuntimeTuningFromNVS() {
    Preferences prefs;
    RuntimeTuningNVS cfg;
    prefs.begin("stewart", true);
    size_t len = prefs.getBytes("runtimeTuneV1", &cfg, sizeof(cfg));
    prefs.end();
    if (len != sizeof(cfg) || !validRuntimeTuning(cfg)) return false;
    workingCurrentMa = cfg.currentMa;
    posSpeed = cfg.f5Speed;
    posAcc = cfg.f5Acc;
    arPeriodMs = clampAutoReturnPeriodMs((float)cfg.arMs);
    loopPeriodUs = cfg.loopUs;
    holdDampGainSec = cfg.odGain;
    holdDampMaxDeg = cfg.odMaxDeg;
    holdDampDeadbandDps = cfg.odDeadbandDps;
    return true;
}

static bool saveRuntimeTuningToNVS() {
    RuntimeTuningNVS cfg = {
        RUNTIME_TUNE_MAGIC,
        RUNTIME_TUNE_VERSION,
        workingCurrentMa,
        posSpeed,
        posAcc,
        arPeriodMs,
        loopPeriodUs,
        holdDampGainSec,
        holdDampMaxDeg,
        holdDampDeadbandDps,
    };
    Preferences prefs;
    prefs.begin("stewart", false);
    size_t written = prefs.putBytes("runtimeTuneV1", &cfg, sizeof(cfg));
    prefs.end();
    return written == sizeof(cfg);
}

static bool applyVFOCPid(uint16_t kp, uint16_t ki, uint16_t kd, uint16_t kv, int okMask[NUM_MOTORS], int& okCnt) {
    okCnt = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
        bool a = servos.setVFOC_KpKi(MOTOR_ADDR[i], kp, ki);
        delay(8);
        servos.flushReceiveBuffer();
        bool b = servos.setVFOC_KdKv(MOTOR_ADDR[i], kd, kv);
        delay(8);
        servos.flushReceiveBuffer();
        okMask[i] = (a && b) ? 1 : 0;
        if (okMask[i]) okCnt++;
    }
    curKp = kp; curKi = ki; curKd = kd; curKv = kv;
    return okCnt == NUM_MOTORS;
}

static bool applyWorkingCurrent(uint16_t mA, int okMask[NUM_MOTORS], int& okCnt) {
    workingCurrentMa = constrain((int)mA, 100, 3000);
    okCnt = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
        bool ok = servos.setWorkingCurrent(MOTOR_ADDR[i], workingCurrentMa);
        okMask[i] = ok ? 1 : 0;
        if (ok) okCnt++;
        delay(8);
        servos.flushReceiveBuffer();
    }
    return okCnt == NUM_MOTORS;
}

static void clearAutoReturnSnapshots() {
    CanGuard _g(servos.can);
    for (int i = 0; i < NUM_MOTORS; i++) {
        latestRaw[i] = INT64_MIN;
        latestRawAtMs[i] = 0;
    }
}

static int waitForFreshAutoReturn(uint32_t timeoutMs) {
    uint32_t waitStart = millis();
    int fresh = 0;
    while (millis() - waitStart < timeoutMs) {
        fresh = 0;
        uint32_t now = millis();
        {
            CanGuard _g(servos.can);
            for (int i = 0; i < NUM_MOTORS; i++) {
                if (latestRaw[i] != INT64_MIN &&
                    latestRawAtMs[i] != 0 &&
                    (now - latestRawAtMs[i]) <= AR_FRESH_MS) fresh++;
            }
        }
        if (fresh == NUM_MOTORS) break;
        delay(5);
    }
    return fresh;
}

static int armAutoReturn(bool waitFresh) {
    canRxPaused = true;
    delay(2);
    clearAutoReturnSnapshots();
    int okc = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (servos.setAutoReturn(MOTOR_ADDR[i], 0x35, arPeriodMs)) okc++;
        delay(5);
    }
    servos.flushReceiveBuffer();
    autoReturnMode = true;
    autoReturnWarnLatched = false;
    canRxPaused = false;
    if (waitFresh) {
        uint32_t waitMs = (uint32_t)arPeriodMs * 4;
        if (waitMs < AR_FRESH_MS) waitMs = AR_FRESH_MS;
        waitForFreshAutoReturn(waitMs);
    }
    return okc;
}

static int rearmAutoReturnIfEnabled() {
    return autoReturnMode ? armAutoReturn(true) : 0;
}

static void disarmAutoReturn() {
    canRxPaused = true;
    for (int i = 0; i < NUM_MOTORS; i++) {
        servos.setAutoReturn(MOTOR_ADDR[i], 0x35, 0);
        delay(5);
    }
    servos.flushReceiveBuffer();
    clearAutoReturnSnapshots();
    autoReturnMode = false;
    autoReturnWarnLatched = false;
    canRxPaused = false;
}

static int applyVFOCPidGuarded(uint16_t kp, uint16_t ki, uint16_t kd, uint16_t kv, int okMask[NUM_MOTORS], int& okCnt) {
    if (autoReturnMode) canRxPaused = true;
    applyVFOCPid(kp, ki, kd, kv, okMask, okCnt);
    return rearmAutoReturnIfEnabled();
}

// ===== 控制模式 =====
// 0 = joint-space（舊版自適應追蹤）
// 1 = task-space PD（FK + IK）
int controlMode = 0;
int silentBootTotal = 0;
int silentBootPerId[7] = {0};

// Task-space controller
TaskSpacePD tsController;

// Joint-space 備用參數
Pose smoothedTarget = {0, 0, NEUTRAL_Z, 0, 0, 0};
constexpr float SMOOTH_TIME = 0.5f;
float prevAngles[NUM_MOTORS] = {0};
float trackingMu = 0.5f;
float trackingKd = 3.0f;
float maxGain = 0.3f;
float smoothKinetic = 0;
bool adaptiveFirstCycle = true;

// 擾動歸零：清六軸偏移 + 計時，F5 匯流口偵測脈衝過期時呼叫。
static void clearBump() {
    for (int i = 0; i < NUM_MOTORS; i++) bumpDeg[i] = 0;
    bumpUntilMs = 0;
}

void posStop() {
    posEnabled = false;
    holdMode = false;
    followMode = false;
    holdDampInit = false;
    holdIntInit = false;          // 下次進 HOLD 由 applyHoldIntegral lazy-reset，不殘留 windup
    servos.stopAllPosition(5);
}

void posDisable() {
    posEnabled = false;
    holdMode = false;
    followMode = false;
    holdDampInit = false;
    holdIntInit = false;          // 下次進 HOLD 由 applyHoldIntegral lazy-reset，不殘留 windup
    servos.stopAllPosition(5);
    delay(50);
    servos.disableAll();
}

static void otaSafetyStop() {
    posEnabled = false;
    holdMode = false;
    followMode = false;
    holdDampInit = false;
    holdIntInit = false;          // 下次進 HOLD 由 applyHoldIntegral lazy-reset，不殘留 windup
    servos.stopAllPosition(5);
    delay(50);
    servos.disableAll();
}

static void requestOtaSafetyStop() {
    otaStopRequested = true;
}

// HTTP-pull OTA 下載進度（每 ~10% 印一次；otaPullProgressPct 於每次 OTA 前重置為 -1）。
static volatile int otaPullProgressPct = -1;
static void otaPullOnProgress(int cur, int total) {
    int pct = total > 0 ? (int)((int64_t)cur * 100 / total) : 0;
    if (pct >= otaPullProgressPct + 10 || pct >= 100) {
        otaPullProgressPct = pct;
        Out.printf("{\"ota\":\"http_progress\",\"pct\":%d}\n", pct);
    }
}

// netTask wedge 自救逾時：netTask >此時間沒更新心跳（且非 OTA 中）→ 網路堆疊掛死 → ESP.restart()。
constexpr uint32_t NET_WEDGE_REBOOT_MS = 8000;

// ===== 失效保護（P4）=====
// 只有「以 WiFi 為控制來源」時才 arm：TCP 下控制指令 →true，USB 下指令 →false。
bool netIsControlSource = false;

// HOLD-current：snapshot 當前實際角度保持。馬達已 enable（failsafe 僅在 posEnabled 觸發），
// 故無需重新 enable/設零點；holdAngles=當前角度 → angleToCoord 命令「留在原地」，無跳變。
static void enterHoldCurrent() {
    for (int i = 0; i < NUM_MOTORS; i++) holdAngles[i] = enc.angles[i];
    holdMoveMs = 0;        // 取消進行中的姿態移動
    followMode = false;    // 失效保護凍結當前姿態，不殘留跟隨
    maxHoldErr = 0;
    resetHoldDampingState();
    resetHoldIntegral();   // failsafe 凍結當前姿態 → 積分從零起，不帶入舊 windup
    holdMode = true;       // 切 HOLD 直送，脫離會發散的 PD/joint 外環
    // posEnabled 維持 true
}

// 斷線偵測 + 安全態切換（core1 執行；netTask 只設旗標、不碰馬達）。
static void checkFailsafe() {
    if (!netIsControlSource || !posEnabled) return;
    // hbTimeout 心跳僅在 client 週期上行時有效（lastNetRxMs 只認入站 byte）；
    // 只收不送的 client 會誤觸，故預設 hbTimeoutMs=0 停用、僅靠 socket-close/WiFi-down。
    bool linkLost = !netConnected ||
        (netCfg.hbTimeoutMs > 0 && (millis() - lastNetRxMs > netCfg.hbTimeoutMs));
    if (!linkLost) return;

    netIsControlSource = false;   // 先解除，避免反覆觸發
    if (netCfg.failsafe == FS_DISABLE) {
        posDisable();
        Out.println("{\"failsafe\":\"disable\"}");
    } else {
        enterHoldCurrent();
        Out.printf("{\"failsafe\":\"hold\",\"hold\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]}\n",
            holdAngles[0],holdAngles[1],holdAngles[2],holdAngles[3],holdAngles[4],holdAngles[5]);
    }
}

static bool enqueueCommand(const String& cmd, bool fromNet) {
    if (!commandQueue || cmd.length() == 0) return false;
    QueuedCommand q{};
    size_t n = cmd.length();
    if (n >= sizeof(q.cmd)) n = sizeof(q.cmd) - 1;
    memcpy(q.cmd, cmd.c_str(), n);
    q.cmd[n] = '\0';
    q.fromNet = fromNet;
    if (xQueueSend(commandQueue, &q, 0) == pdTRUE) return true;
    cmdDropCount++;
    return false;
}

static const char* ctlModeName(const TelemetrySnapshot& s) {
    if (!s.pos) return "off";
    if (s.extra == TELEM_EXTRA_HOLD) return "hold";
    return s.controlMode == 1 ? "task_pd" : "joint";
}

static const char* profileName(const TelemetrySnapshot& s) {
    if (s.profile == 1) return "follow";
    if (s.profile == 2) return "minjerk";
    return "none";
}

static void teleAppend(char* dst, size_t cap, size_t& pos, const char* fmt, ...) {
    if (!dst || cap == 0 || pos >= cap) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(dst + pos, cap - pos, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    size_t written = (size_t)n;
    pos = written >= cap - pos ? cap - 1 : pos + written;
}

static void emitTelemetry(const TelemetrySnapshot& s) {
    char line[NET_OUT_LINE_BYTES];
    size_t pos = 0;
    line[0] = '\0';
    teleAppend(line, sizeof(line), pos, "{\"a\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],"
                  "\"r\":[%lld,%lld,%lld,%lld,%lld,%lld],"
                  "\"ok\":%d,\"z\":%d,\"pos\":%d,\"cm\":%d,"
                  "\"ef\":%u,\"tx\":%u,\"rx\":%u,"
                  "\"t\":%u,\"dt\":%u,\"cus\":%u,\"lhz\":%.0f,\"ar\":%d,"
                  "\"arAge\":[%u,%u,%u,%u,%u,%u],"
                  "\"bus\":{\"f5\":%u,\"q\":%u,\"rx\":%u,\"ms\":%u,"
                  "\"per\":[%u,%u,%u,%u,%u,%u],\"ovr\":%u,\"f5us\":%u,\"mxd\":%u},"
                  "\"tim\":{\"cycles\":%u,\"dtMin\":%u,\"dtMax\":%u,\"dtAvg\":%.2f,"
                  "\"teleMs\":%u,\"f5us\":%u,\"f5max\":%u,\"canReadUs\":%u},"
                  "\"can\":{\"backend\":\"mcp2515\",\"bitrate\":500000,\"ef\":%u,\"tx\":%u,\"rx\":%u,"
                  "\"rxDrop\":%u,\"txFail\":%u,\"qDepth\":0,\"qMax\":0},"
                  "\"ctl\":{\"mode\":\"%s\",\"ikFail\":%u,\"fkFail\":%u,\"coordLimit\":%u,\"coordMax\":%d,"
                  "\"fkStreak\":%d,\"profile\":\"%s\"},"
                  "\"pid\":[%u,%u,%u,%u],\"pids\":%d,"
                  "\"spid\":[%u,%u,%u,%u],\"pidn\":%d,\"fw\":\"%s\",\"fl\":%d,"
                  "\"sched\":{\"ctrl_us\":%u,\"ctrl_max_us\":%u,\"jitter_us\":%u,\"missed\":%u,"
                  "\"tele_drop\":%u,\"cmd_drop\":%u,\"snap_age_ms\":%u}",
        s.angles[0], s.angles[1], s.angles[2],
        s.angles[3], s.angles[4], s.angles[5],
        s.raw[0], s.raw[1], s.raw[2], s.raw[3], s.raw[4], s.raw[5],
        s.ok, s.zeroed, s.pos, s.controlMode,
        s.eflg, s.tec, s.rec,
        s.t, s.dtMs, s.canReadUs, s.lhz, s.arMs,
        s.arAge[0], s.arAge[1], s.arAge[2],
        s.arAge[3], s.arAge[4], s.arAge[5],
        s.busF5, s.busQ, s.busRx, s.busMs,
        s.busPer[0], s.busPer[1], s.busPer[2],
        s.busPer[3], s.busPer[4], s.busPer[5],
        s.busOvr, s.f5us, s.maxDrain,
        s.teleCycles, s.dtMinMs, s.dtMaxMs, s.dtAvgMs,
        s.busMs, s.f5us, s.f5max, s.canReadUs,
        s.eflg, s.tec, s.rec,
        s.busOvr, 0,
        ctlModeName(s), s.ctlIkFail, s.ctlFkFail, s.ctlCoordLimit, s.ctlCoordMaxAbs,
        s.fkStreak, profileName(s),
        s.pid[0], s.pid[1], s.pid[2], s.pid[3], s.pidSaved,
        s.savedPid[0], s.savedPid[1], s.savedPid[2], s.savedPid[3], s.nvsPid, FW_SRC_HASH, s.followMode,
        s.ctrlUs, s.ctrlMaxUs, s.jitterUs, s.missed, s.teleDrop, s.cmdDrop, s.snapAgeMs);

    if (s.extra == TELEM_EXTRA_HOLD) {
        teleAppend(line, sizeof(line), pos, ",\"hold\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],"
                      "\"herr\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],"
                      "\"hmax\":%.2f,"
                      "\"od\":[%.4f,%.2f,%.2f],"
                      "\"hi\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],"
                      "\"hki\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],"
                      "\"hig\":[%.3f,%.3f,%.3f],\"hisaved\":%d",
            s.hold[0],s.hold[1],s.hold[2],
            s.hold[3],s.hold[4],s.hold[5],
            s.herr[0],s.herr[1],s.herr[2],s.herr[3],s.herr[4],s.herr[5],
            s.hmax,
            s.holdDampGain, s.holdDampMax, s.holdDampCorrMax,
            s.holdIntC[0],s.holdIntC[1],s.holdIntC[2],s.holdIntC[3],s.holdIntC[4],s.holdIntC[5],
            s.holdKiSnap[0],s.holdKiSnap[1],s.holdKiSnap[2],s.holdKiSnap[3],s.holdKiSnap[4],s.holdKiSnap[5],
            s.holdIClamp, s.holdIDead, s.holdISettle, s.holdIntSaved);
    } else if (s.extra == TELEM_EXTRA_TASK) {
        teleAppend(line, sizeof(line), pos, ",\"fk\":[%.1f,%.1f,%.1f,%.2f,%.2f,%.2f],"
                      "\"err\":[%.1f,%.1f,%.1f,%.2f,%.2f,%.2f],"
                      "\"tgt\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
                      "\"fki\":%d",
            s.fkPose[0], s.fkPose[1], s.fkPose[2], s.fkPose[3], s.fkPose[4], s.fkPose[5],
            s.poseErr[0], s.poseErr[1], s.poseErr[2], s.poseErr[3], s.poseErr[4], s.poseErr[5],
            s.motorTgt[0],s.motorTgt[1],s.motorTgt[2],
            s.motorTgt[3],s.motorTgt[4],s.motorTgt[5],
            s.fkIterations);
    } else if (s.extra == TELEM_EXTRA_JOINT) {
        teleAppend(line, sizeof(line), pos, ",\"tgt\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
                      "\"g\":%.3f",
            s.motorTgt[0],s.motorTgt[1],s.motorTgt[2],
            s.motorTgt[3],s.motorTgt[4],s.motorTgt[5],
            s.jointGain);
    }
    if (s.followMode) {
        float errT = 0.0f, errR = 0.0f, velT = 0.0f, velR = 0.0f;
        for (int i = 0; i < 3; i++) {
            errT = fmaxf(errT, fabsf(s.followTgtPose[i] - s.followCurPose[i]));
            velT = fmaxf(velT, fabsf(s.followVelPose[i]));
        }
        for (int i = 3; i < 6; i++) {
            errR = fmaxf(errR, fabsf(s.followTgtPose[i] - s.followCurPose[i]));
            velR = fmaxf(velR, fabsf(s.followVelPose[i]));
        }
        teleAppend(line, sizeof(line), pos, ",\"fref\":{\"g\":1,\"e\":[%.2f,%.3f],\"v\":[%.2f,%.3f],\"lim\":[%.1f,%.1f],\"tight\":%.2f}",
            errT, errR, velT, velR, s.followVmax[0], s.followVmax[1], s.followTight);
    }
    teleAppend(line, sizeof(line), pos, "}");
    Out.writeLine(line);
}

static void smoothPose(Pose& sm, const Pose& tgt, float dt) {
    float alpha = fminf(1.0f, dt / SMOOTH_TIME);
    sm.x    += alpha * (tgt.x    - sm.x);
    sm.y    += alpha * (tgt.y    - sm.y);
    sm.z    += alpha * (tgt.z    - sm.z);
    sm.roll += alpha * (tgt.roll - sm.roll);
    sm.pitch+= alpha * (tgt.pitch- sm.pitch);
    sm.yaw  += alpha * (tgt.yaw  - sm.yaw);
}

void canRxTask(void*) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
        if (autoReturnMode && !canRxPaused) {
            servos.drainInto(latestRaw, latestRawAtMs);
        }
    }
}

void startCanRxTask() {
    if (canRxTaskHandle) return;
    xTaskCreatePinnedToCore(canRxTask, "canRx", 4096, nullptr, 3, &canRxTaskHandle, 0);
}

void setup() {
    Serial.setTxBufferSize(1024);   // 大 TX ring buffer：遙測整幀瞬入緩衝、UART ISR 背景排空 → 不卡控制 loop
    Serial.begin(460800);           // 460800：406B 幀 ≈9ms（≪30ms 週期，不飽和）；921600 此 CP2102 訊號完整性不穩→改用此
    Serial.setTimeout(20);          // handleSerial 排空時，半行最壞只卡 20ms（預設 1000ms 太長）
    while (!Serial) delay(10);
    delay(200); // 給 FreeRTOS / SPI mutex 完全初始化的時間，避免 paramLock NULL
    netSetOtaStartCallback(requestOtaSafetyStop);

    // 明確 init SPI bus（VSPI 預設腳位 SCK=18 MISO=19 MOSI=23 SS=5）
    // 必須在任何 MCP_CAN SPI 操作前呼叫，確保 SPI 的 paramLock mutex 已建立
    SPI.begin(18, 19, 23, 5);
    SPI.setFrequency(10000000);
    delay(50);

    loadRuntimeTuningFromNVS();
    loadHoldIntFromNVS();            // HOLD 積分組態：有保存值則載入，否則維持 BOOT 預設（Ki=0）
    applyFollowTight(0.7f);          // 跟隨預設緊度 0.5→0.7：tau 0.275→0.205s，lag 顯著降而 step 過衝仍低（FE 可 runtime 再調）
    enc.init();
    pidSaved = loadVFOCPidFromNVS();

    Out.printf("{\"status\":\"init\",\"calibrated\":%s}\n",
                  enc.zeroed ? "true" : "false");

    while (!servos.begin()) {
        Out.println("{\"error\":\"CAN init failed, retrying...\"}");
        delay(2000);
    }

    // DIAGNOSTIC: 不送任何 cmd，只 begin CAN + listen 5s，看 motor 是否從通電就自發 spam
    {
        uint32_t st = millis();
        while (millis() - st < 5000) {
            if (servos.can.checkReceive() == CAN_MSGAVAIL) {
                unsigned long id; uint8_t len; uint8_t buf[8];
                servos.can.readMsgBuf(&id, &len, buf);
                silentBootTotal++;
                if (id >= 1 && id <= 6) silentBootPerId[id]++;
            }
        }
    }
    // 之後恢復一般流程
    servos.disableAll();
    for (int i = 0; i < NUM_MOTORS; i++) {
        servos.setResponseMode(MOTOR_ADDR[i], 1, 0);
        delay(10);
    }

    // 開機持久化 runtime tuning：0x83 工作電流存在 ESP32 NVS，由 ESP32 每次開機下發。
    int currentOk[NUM_MOTORS] = {0}, currentOkCnt = 0;
    applyWorkingCurrent(workingCurrentMa, currentOk, currentOkCnt);

    // 開機持久化 vFOC PID：馬達重開會回自己的預設，ESP32 每次開機下發本機保存值。
    // 若 NVS 尚未保存，cur* 仍是 BOOT_* 純P工作點。
    int pidOk[NUM_MOTORS] = {0}, pidOkCnt = 0;
    applyVFOCPid(curKp, curKi, curKd, curKv, pidOk, pidOkCnt);

    delay(200);
    servos.flushReceiveBuffer();

    // 預設啟用 auto-return：MCP2515/500K 下保守用 20ms(50Hz)，避免 6 顆 streaming + F5 壓爆 2 格 RX buffer。
    // ESP32 連續排空、控制環只取最新快照 → cus≈0、無輪詢 8ms backstop（runtime A0 可切回輪詢）。
    armAutoReturn(false);
    startCanRxTask();

    // WiFi bring-up（P1）：載入 NVS 憑證，若已啟用則自動連線並印 IP。
    // 失敗不致命——USB Serial 仍是主路徑。WiFi 細節封裝在 net_transport。
    netBoot();

    // 啟動 core0 netTask + 出站遙測 queue（P2）。此後 Out 的每行同時鏡像到 TCP。
    netInit();
    startRealtimeTasks();

    // 開機診斷：上報 reset 原因（brownout/panic/watchdog/poweron）+ heap，解「莫名其妙斷線」盲區。
    // 走 Out → USB + TCP queue；server 於 (重)連線時收到此行即知上次為何重啟。
    esp_reset_reason_t _rr = esp_reset_reason();
    Out.printf("{\"boot\":{\"reset\":%d,\"reset_str\":\"%s\",\"heap\":%u,\"minheap\":%u,"
               "\"fw\":\"%s\",\"rev\":\"%s\",\"dirty\":%d,\"built\":%lu}}\n",
               (int)_rr, resetReasonStr(_rr),
               (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
               FW_SRC_HASH, FW_GIT_REV, FW_DIRTY, (unsigned long)FW_BUILD_EPOCH);
    Out.println("{\"status\":\"ready\"}");
}

// 指令分派：source-agnostic。USB（handleSerial）與 TCP（loop 取 qIn）共用同一條，
// 指令語意一致、ack 自動經 Out 鏡像回兩路。原本分支內的 return 改為從 dispatch 返回，
// 行為等價（跳過其餘分支）。
void dispatch(const String& cmd, bool fromNet = false) {
    // WIFI/FS/HB 系列指令先攔截，命中即短路
    if (netHandleCommand(cmd, fromNet, posEnabled)) return;

    // TCP 來源禁校正類（Z/Z0~Z5 寫 NVS 不可逆；校正須實體 home 姿態 = 本就手邊 USB 操作）
    if (fromNet && cmd.startsWith("Z")) {
        Out.println("{\"error\":\"Z* (calibration) is USB-only\"}");
        return;
    }

    if (cmd == "Z") {
        if (enc.zeroAll(servos)) {
            Out.println("{\"status\":\"zeroed all\",\"saved\":true}");
        } else {
            Out.println("{\"error\":\"encoder read failed during zeroing\"}");
        }
    } else if (cmd.startsWith("Z") && cmd.length() == 2) {
        int idx = cmd.charAt(1) - '0';
        if (idx >= 0 && idx < NUM_MOTORS) {
            if (enc.zeroMotor(idx, servos)) {
                Out.printf("{\"status\":\"zeroed M%d\",\"saved\":true}\n", idx + 1);
            } else {
                Out.printf("{\"error\":\"M%d read failed\"}\n", idx + 1);
            }
        }
    } else if (cmd == "A1") {
        // 啟用 auto-return：6 顆每 arPeriodMs 主動上報 0x35 位置
        int okc = armAutoReturn(true);
        Out.printf("{\"status\":\"auto-return ON\",\"period_ms\":%d,\"hz\":%.3f,\"cfg_ok\":%d}\n",
                      arPeriodMs, 1000.0f / arPeriodMs, okc);
    } else if (cmd.startsWith("AR ")) {
        // 調整上報週期（ms）；SERVO42D 協定 timer 為 uint16 ms，小數輸入會量化到最接近的整數 ms。
        float requestedMs = 0.0f;
        if (sscanf(cmd.c_str(), "AR %f", &requestedMs) == 1 && requestedMs > 0.0f) {
            uint16_t prevArPeriodMs = arPeriodMs;
            arPeriodMs = clampAutoReturnPeriodMs(requestedMs);
            int okc = rearmAutoReturnIfEnabled();
            bool saved = saveRuntimeTuningToNVS();
            int quantized = fabsf(requestedMs - (float)arPeriodMs) > 0.001f ? 1 : 0;
            int clamped = (requestedMs < (float)AR_PERIOD_MIN_MS || requestedMs > (float)AR_PERIOD_MAX_MS) ? 1 : 0;
            Out.printf("{\"status\":\"ar period\",\"period_ms\":%d,\"hz\":%.3f,"
                       "\"requested_period_ms\":%.3f,\"requested_hz\":%.3f,"
                       "\"min_period_ms\":%d,\"max_period_ms\":%d,"
                       "\"quantized\":%d,\"clamped\":%d,\"prev_period_ms\":%d,\"cfg_ok\":%d,\"saved\":%d}\n",
                       arPeriodMs, 1000.0f / arPeriodMs,
                       requestedMs, 1000.0f / requestedMs,
                       AR_PERIOD_MIN_MS, AR_PERIOD_MAX_MS,
                       quantized, clamped, prevArPeriodMs, okc, saved ? 1 : 0);
        } else {
            Out.println("{\"error\":\"usage AR period_ms\"}");
        }
    } else if (cmd.startsWith("C ")) {
        // 設回覆模式：C 0 0=不回覆 / C 1 0=只即時 / C 1 1=預設全回
        int sp = cmd.indexOf(' ', 2);
        int xx = cmd.substring(2, sp < 0 ? cmd.length() : sp).toInt();
        int yy = sp < 0 ? 0 : cmd.substring(sp + 1).toInt();
        for (int i = 0; i < NUM_MOTORS; i++) { servos.setResponseMode(MOTOR_ADDR[i], xx, yy); delay(5); }
        servos.flushReceiveBuffer();
        Out.printf("{\"status\":\"resp mode\",\"xx\":%d,\"yy\":%d}\n", xx, yy);
    } else if (cmd.startsWith("L ")) {
        // 調整控制迴圈週期（ms，可小數；內部以 µs 排程）
        float requestedMs = 0.0f;
        if (sscanf(cmd.c_str(), "L %f", &requestedMs) == 1 && requestedMs > 0.0f) {
            float clampedMs = constrain(requestedMs, 1.0f, 100.0f);
            uint32_t periodUs = (uint32_t)floorf(clampedMs * 1000.0f + 0.5f);
            if (periodUs < 1000) periodUs = 1000;
            if (periodUs > 100000) periodUs = 100000;
            loopPeriodUs = periodUs;
            bool saved = saveRuntimeTuningToNVS();
            Out.printf("{\"status\":\"loop period\",\"period_ms\":%.3f,\"period_us\":%u,\"hz\":%.3f,\"saved\":%d}\n",
                periodUs / 1000.0f, periodUs, 1000000.0f / periodUs, saved ? 1 : 0);
        } else {
            Out.println("{\"error\":\"usage L period_ms\"}");
        }
    } else if (cmd == "HP" || cmd == "HP?") {
        Out.printf("{\"status\":\"hold profile\",\"maxVelDps\":%.1f,\"enabled\":%d,\"maxMs\":%lu}\n",
            holdProfileMaxVelDps, holdProfileMaxVelDps > 0.0f ? 1 : 0, (unsigned long)HOLD_PROFILE_MAX_MS);
    } else if (cmd.startsWith("HP ")) {
        float maxVel = 0.0f;
        if (sscanf(cmd.c_str(), "HP %f", &maxVel) == 1) {
            holdProfileMaxVelDps = maxVel <= 0.0f ? 0.0f : constrain(maxVel, 5.0f, 360.0f);
            Out.printf("{\"status\":\"hold profile\",\"maxVelDps\":%.1f,\"enabled\":%d,\"maxMs\":%lu}\n",
                holdProfileMaxVelDps, holdProfileMaxVelDps > 0.0f ? 1 : 0, (unsigned long)HOLD_PROFILE_MAX_MS);
        } else {
            Out.println("{\"error\":\"usage HP max_vel_dps (0 disables)\"}");
        }
    } else if (cmd == "OD" || cmd == "OD?") {
        Out.printf("{\"status\":\"output damping\",\"gain\":%.4f,\"maxDeg\":%.2f,\"deadbandDps\":%.2f}\n",
            holdDampGainSec, holdDampMaxDeg, holdDampDeadbandDps);
    } else if (cmd.startsWith("OD ")) {
        float gain = 0, maxDeg = holdDampMaxDeg, deadband = holdDampDeadbandDps;
        int n = sscanf(cmd.c_str(), "OD %f %f %f", &gain, &maxDeg, &deadband);
        if (n >= 1) {
            holdDampGainSec = constrain(gain, 0.0f, 0.08f);
            if (n >= 2) holdDampMaxDeg = constrain(maxDeg, 0.0f, 5.0f);
            if (n >= 3) holdDampDeadbandDps = constrain(deadband, 0.0f, 30.0f);
            resetHoldDampingState();
            bool saved = saveRuntimeTuningToNVS();
            Out.printf("{\"status\":\"output damping\",\"gain\":%.4f,\"maxDeg\":%.2f,\"deadbandDps\":%.2f,\"saved\":%d}\n",
                holdDampGainSec, holdDampMaxDeg, holdDampDeadbandDps, saved ? 1 : 0);
        } else {
            Out.println("{\"error\":\"usage OD gain_sec [max_deg] [deadband_dps]\"}");
        }
    } else if (cmd == "A0") {
        disarmAutoReturn();
        Out.println("{\"status\":\"auto-return OFF (回輪詢)\"}");
    } else if (cmd == "D") {
        posDisable();
        Out.println("{\"status\":\"disabled all\"}");
    } else if (cmd == "E") {
        if (!enc.zeroed) {
            Out.println("{\"error\":\"not calibrated, run Z first\"}");
            return;
        }

        uint8_t enMask = servos.enableAll();
        int enCnt = __builtin_popcount(enMask);
        delay(20);
        for (int i = 0; i < NUM_MOTORS; i++) {
            servos.setZeroPoint(MOTOR_ADDR[i]);
            delay(10);
            servos.flushReceiveBuffer();
        }

        int64_t raw[NUM_MOTORS];
        servos.readAllRawEncoders(raw);
        enc.updateAngles(raw);   // 0x35 不受 0x92 影響，先刷新 enc.angles 當 fallback 基準
        for (int i = 0; i < NUM_MOTORS; i++) {
            // 讀失敗 fallback 用新鮮 enc.angles（真實當前角），不用寫死 90°，
            // 否則 0x92 已把零點設在真實位置 → 首個 F5 coord 算錯 → 該軸跳動（與 H handler 對齊）
            enableAngle[i] = (raw[i] != INT64_MIN)
                ? enc.rawToAngle(i, raw[i]) : enc.angles[i];
        }

        // 初始化控制器：target 自動對齊當前 FK 姿態（避免 Enable 瞬間暴衝）
        if (controlMode == 1) {
            tsController.reset(enc.angles);
            targetPose = tsController.currentPose; // Enable 時 target = 當前位置
        } else {
            smoothedTarget = targetPose;
            for (int i = 0; i < NUM_MOTORS; i++)
                prevAngles[i] = enableAngle[i];
            adaptiveFirstCycle = true;
        }

        followMode = false;   // E 啟動 mode0/1 控制 → 清跟隨，避免 follow-HOLD 下殘留追 followTgt
        holdDampInit = false;
        holdIntInit = false;  // 離開 HOLD → 凍結積分（mode0/1 不走 applyHoldIntegral）
        posEnabled = true;
        autoReturnWarnLatched = false;
        Out.printf("{\"status\":\"pos enabled\",\"mode\":%d,\"speed\":%d,\"acc\":%d,"
            "\"enOk\":[%d,%d,%d,%d,%d,%d],\"enCnt\":%d}\n",
            controlMode, posSpeed, posAcc,
            (enMask>>0)&1,(enMask>>1)&1,(enMask>>2)&1,(enMask>>3)&1,(enMask>>4)&1,(enMask>>5)&1,
            enCnt);
    } else if (cmd == "S") {
        posStop();
        Out.println("{\"status\":\"pos stopped\"}");
    } else if (cmd == "H") {
        // HOLD: snapshot 當前角度，enable + 0x92 + 鎖位
        if (!enc.zeroed) {
            Out.println("{\"error\":\"not calibrated, run Z first\"}");
            return;
        }
        // 先確保 enc.angles 是新鮮的
        servos.flushReceiveBuffer();
        int64_t raw0[NUM_MOTORS];
        servos.readAllRawEncoders(raw0);
        enc.updateAngles(raw0);
        for (int i = 0; i < NUM_MOTORS; i++) holdAngles[i] = enc.angles[i];
        holdMoveMs = 0;                                                     // 取消任何進行中的姿態移動

        // Enable 馬達 + 設零點
        uint8_t enMask = servos.enableAll();
        int enCnt = __builtin_popcount(enMask);
        delay(20);
        for (int i = 0; i < NUM_MOTORS; i++) {
            servos.setZeroPoint(MOTOR_ADDR[i]);
            delay(10);
            servos.flushReceiveBuffer();
        }

        // 重抓 enableAngle（0x92 之後的當前角度）
        int64_t raw[NUM_MOTORS];
        servos.readAllRawEncoders(raw);
        for (int i = 0; i < NUM_MOTORS; i++) {
            enableAngle[i] = (raw[i] != INT64_MIN)
                ? enc.rawToAngle(i, raw[i]) : holdAngles[i];
        }

        holdMode = true;
        followMode = false;   // 進 HOLD 一律純死咬，跟隨須另送 FOLLOW 1
        posEnabled = true;
        autoReturnWarnLatched = false;
        maxHoldErr = 0;
        resetHoldDampingState();
        resetHoldIntegral();   // 進 HOLD → 積分從零起，seed prevAngle 防首 cycle vel spike
        Out.printf("{\"status\":\"hold\",\"hold\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],"
            "\"enOk\":[%d,%d,%d,%d,%d,%d],\"enCnt\":%d}\n",
            holdAngles[0],holdAngles[1],holdAngles[2],
            holdAngles[3],holdAngles[4],holdAngles[5],
            (enMask>>0)&1,(enMask>>1)&1,(enMask>>2)&1,(enMask>>3)&1,(enMask>>4)&1,(enMask>>5)&1,
            enCnt);
    } else if (cmd.startsWith("J ")) {
        int mA;
        if (sscanf(cmd.c_str(), "J %d", &mA) == 1) {
            int okMask[NUM_MOTORS] = {0};
            int okCnt = 0;
            bool allOk = applyWorkingCurrent((uint16_t)mA, okMask, okCnt);
            bool saved = allOk && saveRuntimeTuningToNVS();
            Out.printf("{\"tune\":\"current\",\"mA\":%d,\"saved\":%d,\"ok\":[%d,%d,%d,%d,%d,%d],\"okCnt\":%d}\n",
                workingCurrentMa, saved ? 1 : 0,
                okMask[0],okMask[1],okMask[2],okMask[3],okMask[4],okMask[5], okCnt);
        }
    } else if (cmd.startsWith("P ")) {
        // 絕對姿態目標 `P x y z r p y [ms]`。前 6 值=絕對 pose（mm/度）。
        //  - n>=6：一律更新 targetPose（mode0/1 控制器讀它）。
        //  - HOLD 下：把絕對 pose 用 IK 解成有效馬達角度，重用 holdMoveMs minimum-jerk 平滑死咬過去。
        //    用 IK 解（非相對增量）保證 6 角度對應真實剛體姿態 → 不 over-constrain 較勁。
        //    第 7 值 ms = 軌跡時長（缺省 1500）。holdAngles==enc==IK 同為下腿絕對幾何角度，可直接插值。
        float v[6]; float ms_f = -1;
        int n = sscanf(cmd.c_str(), "P %f %f %f %f %f %f %f",
                       &v[0],&v[1],&v[2],&v[3],&v[4],&v[5], &ms_f);
        if (n >= 6) {
            targetPose = {v[0], v[1], v[2], v[3], v[4], v[5]};
            if (holdMode) {
                IKResult ik = inverse_kinematics(targetPose);
                if (!ik.valid) {
                    Out.println("{\"error\":\"P pose unreachable (IK invalid)\"}");
                } else {
                    uint32_t requestedMs = (n >= 7) ? (uint32_t)constrain((int)ms_f, 1, (int)HOLD_PROFILE_MAX_MS) : 1500;
                    for (int i = 0; i < NUM_MOTORS; i++) {
                        holdStart[i]  = holdAngles[i];     // 從當下死咬目標起步（多段可接續）
                        holdTarget[i] = ik.angles[i];      // IK 解 = 有效幾何角度
                    }
                    uint32_t ms = constrainedHoldMoveMs((float)requestedMs, holdStart, holdTarget);
                    holdMoveStart = millis();
                    holdMoveMs = ms;
                    resetHoldIntegral();   // 新目標 → 舊 droop 積分作廢，軌跡完成後對新點重新消差
                    Out.printf("{\"status\":\"pose goto\",\"t\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],\"ms\":%lu,\"reqMs\":%lu,\"profile\":\"minjerk\",\"maxVelDps\":%.1f,\"limitEnabled\":%d,\"limited\":%d}\n",
                        v[0],v[1],v[2],v[3],v[4],v[5],
                        (unsigned long)ms, (unsigned long)requestedMs,
                        holdProfileMaxVelDps, holdProfileMaxVelDps > 0.0f ? 1 : 0, ms > requestedMs ? 1 : 0);
                }
            } else {
                Out.printf("{\"status\":\"target set\",\"t\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]}\n",
                    v[0],v[1],v[2],v[3],v[4],v[5]);
            }
        }
    } else if (cmd.startsWith("FOLLOW")) {
        // FOLLOW 1 / FOLLOW 0：進/出跟隨模式（僅 HOLD 有意義）。
        // 進入時用 FK(當前角度) 當濾波器起點 → IK(起點)≈當前 holdAngles，不跳；回 pose 供前端同步滑桿。
        int on = 0;
        sscanf(cmd.c_str(), "FOLLOW %d", &on);
        if (on) {
            if (!holdMode) {
                Out.println("{\"error\":\"FOLLOW needs HOLD (send H first)\"}");
            } else {
                FKSolver fk;
                Pose p = fk.solve(enc.angles);
                float init[6] = { p.x, p.y, p.z, p.roll, p.pitch, p.yaw };
                if (!fk.converged) {   // FK 沒收斂 → 退回最後指令 pose，仍給有效起點
                    init[0]=targetPose.x; init[1]=targetPose.y; init[2]=targetPose.z;
                    init[3]=targetPose.roll; init[4]=targetPose.pitch; init[5]=targetPose.yaw;
                }
                resetFollowDynamics(init);
                holdMoveMs = 0;          // 與 P minimum-jerk profile 互斥
                followMode = true;
                Out.printf("{\"status\":\"follow on\",\"pose\":[%.2f,%.2f,%.2f,%.3f,%.3f,%.3f],"
                           "\"fk\":%d,\"gen\":\"esp32_jerk_limited\"}\n",
                    init[0],init[1],init[2],init[3],init[4],init[5], fk.converged?1:0);
            }
        } else {
            followMode = false;        // holdAngles 凍結在當前 → 純死咬續持
            resetHoldIntegral();        // 新死咬點（follow 終點）→ 積分從零起
            Out.println("{\"status\":\"follow off\"}");
        }
    } else if (cmd.startsWith("PF ")) {
        // 跟隨目標更新 `PF x y z r p y`：只更新最新意圖，不回 ack（避免塞爆接收）。
        // 物理 reference 由 ESP32 followStep 每 control tick 產生；非 followMode 忽略。
        float v[6];
        int n = sscanf(cmd.c_str(), "PF %f %f %f %f %f %f", &v[0],&v[1],&v[2],&v[3],&v[4],&v[5]);
        if (n >= 6 && followMode) { for (int i = 0; i < 6; i++) followTgt[i] = v[i]; }
    } else if (cmd.startsWith("VF ")) {
        // 跟隨速度上限 `VF vmaxT vmaxR`（mm/s, deg/s）：所有 host 動作的硬體安全閥。
        // accel/jerk 時間常數由 FE(tight) 設；VF 只覆寫 vmax（amax=vmax/accelT 隨之變）。
        float vt, vr;
        int n = sscanf(cmd.c_str(), "VF %f %f", &vt, &vr);
        if (n >= 2) {
            followVmaxT = constrain(vt, 1.0f, 500.0f);
            followVmaxR = constrain(vr, 1.0f, 360.0f);
            Out.printf("{\"status\":\"follow vmax\",\"t\":%.1f,\"r\":%.1f,"
                       "\"at\":%.1f,\"ar\":%.1f,\"jt\":%.1f,\"jr\":%.1f,"
                       "\"gen\":\"esp32_jerk_limited\"}\n",
                followVmaxT, followVmaxR,
                followAmaxForAxis(0), followAmaxForAxis(3),
                followJmaxForAxis(0), followJmaxForAxis(3));
        }
    } else if (cmd.startsWith("FE ")) {
        // 跟隨緊度 `FE tight`（0..1）：單參數同時設 vmax/accelTime/jerkTime/趨近增益 K，越大跟越緊。
        // 對應前端「跟隨緊度」滑桿；VF 仍可事後底層覆寫 vmax。
        float tg;
        int n = sscanf(cmd.c_str(), "FE %f", &tg);
        if (n >= 1) {
            applyFollowTight(tg);
            Out.printf("{\"status\":\"follow tight\",\"tight\":%.3f,\"vt\":%.1f,\"vr\":%.1f,"
                       "\"accelT\":%.4f,\"jerkT\":%.4f,\"K\":%.2f,"
                       "\"at\":%.1f,\"ar\":%.1f,\"gen\":\"esp32_jerk_limited\"}\n",
                followTight, followVmaxT, followVmaxR,
                followAccelT, followJerkT, followApproachK,
                followAmaxForAxis(0), followAmaxForAxis(3));
        }
    } else if (cmd == "OTA" || cmd.startsWith("OTA ")) {
        // HTTP-pull OTA：ESP32 主動 outbound 抓 firmware（繞 host 防火牆，比 ArduinoOTA 回連可靠、可全自動）。
        // 用法：OTA <url>（host 提供 http://<ip>:3000/firmware.bin）。下載在 controlTick(core1) 內阻塞 ~10s，
        // 故先直接 otaSafetyStop() 禁能馬達（平台 flash 時 limp 屬預期，與 USB flash 同）。成功寫非作用分割區後自動 reboot（雙分割原子回退）。
        String url = cmd.length() > 4 ? cmd.substring(4) : String("");
        url.trim();
        if (url.length() == 0) {
            Out.println("{\"ota\":\"error\",\"reason\":\"usage: OTA http://host:port/firmware.bin\"}");
        } else {
            Out.printf("{\"ota\":\"http_pull\",\"url\":\"%s\"}\n", url.c_str());
            otaSafetyStop();                   // 同步禁能馬達（在 controlTick 內，與 CAN 同核安全）
            otaPullProgressPct = -1;
            WiFiClient otaClient;
            httpUpdate.rebootOnUpdate(true);
            httpUpdate.onProgress(otaPullOnProgress);
            t_httpUpdate_return r = httpUpdate.update(otaClient, url);
            if (r == HTTP_UPDATE_FAILED) {
                Out.printf("{\"ota\":\"http_failed\",\"code\":%d,\"err\":\"%s\"}\n",
                           httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
            } else if (r == HTTP_UPDATE_NO_UPDATES) {
                Out.println("{\"ota\":\"http_no_updates\"}");
            }
            // HTTP_UPDATE_OK 不返回（已 reboot）
        }
    } else if (cmd.startsWith("CM ")) {
        // 控制模式切換：CM 0 = joint-space, CM 1 [kp kd] = task-space PD
        // （前綴從 "C " 改為 "CM "：避開上方 0x8C 回覆模式 "C " 的撞名遮蔽）
        float mode_f, kp = -1, kd = -1;
        int n = sscanf(cmd.c_str(), "CM %f %f %f", &mode_f, &kp, &kd);
        if (n >= 1) {
            controlMode = (int)mode_f;
            if (controlMode == 1 && n >= 2) {
                for (int i = 0; i < 6; i++) tsController.Kp[i] = fmaxf(0.01f, fminf(1.0f, kp));
                if (n >= 3) for (int i = 0; i < 6; i++) tsController.Kd[i] = fmaxf(0.0f, kd);
            }
            Out.printf("{\"status\":\"control mode\",\"mode\":%d,\"kp\":%.3f,\"kd\":%.3f}\n",
                controlMode, tsController.Kp[0], tsController.Kd[0]);
        }
    } else if (cmd == "KS" || cmd.startsWith("KS ")) {
        int kp, ki, kd, kv;
        int n = sscanf(cmd.c_str(), "KS %d %d %d %d", &kp, &ki, &kd, &kv);
        if (n == 4) {
            if (kp < 0 || kp > 1024 || ki < 0 || ki > 1024 || kd < 0 || kd > 1024 || kv < 0 || kv > 1024) {
                Out.println("{\"error\":\"PID range 0-1024\"}");
                return;
            }
            int okMask[NUM_MOTORS] = {0};
            int okCnt = 0;
            int arOk = applyVFOCPidGuarded(kp, ki, kd, kv, okMask, okCnt);
            pidSaved = (okCnt == NUM_MOTORS) && saveVFOCPidToNVS();
            Out.printf("{\"tune\":\"pid\",\"saved\":%d,\"kp\":%u,\"ki\":%u,\"kd\":%u,\"kv\":%u,"
                "\"ok\":[%d,%d,%d,%d,%d,%d],\"okCnt\":%d,\"ar_ok\":%d}\n",
                pidSaved ? 1 : 0, curKp, curKi, curKd, curKv,
                okMask[0],okMask[1],okMask[2],okMask[3],okMask[4],okMask[5], okCnt, arOk);
        } else {
            pidSaved = saveVFOCPidToNVS();
            Out.printf("{\"tune\":\"pid\",\"saved\":%d,\"kp\":%u,\"ki\":%u,\"kd\":%u,\"kv\":%u,"
                "\"ok\":[1,1,1,1,1,1],\"okCnt\":6,\"local\":1}\n",
                pidSaved ? 1 : 0, curKp, curKi, curKd, curKv);
        }
    } else if (cmd == "KRESET") {
        clearVFOCPidNVS();
        pidSaved = false;
        int okMask[NUM_MOTORS] = {0};
        int okCnt = 0;
        int arOk = applyVFOCPidGuarded(BOOT_KP, BOOT_KI, BOOT_KD, BOOT_KV, okMask, okCnt);
        Out.printf("{\"tune\":\"pid\",\"reset\":1,\"saved\":0,\"kp\":%u,\"ki\":%u,\"kd\":%u,\"kv\":%u,"
            "\"ok\":[%d,%d,%d,%d,%d,%d],\"okCnt\":%d,\"ar_ok\":%d}\n",
            curKp, curKi, curKd, curKv,
            okMask[0],okMask[1],okMask[2],okMask[3],okMask[4],okMask[5], okCnt, arOk);
    } else if (cmd.startsWith("KIC ")) {
        // HOLD 積分護欄 `KIC clampDeg deadbandDeg settleDps`。
        float c, d, s;
        if (sscanf(cmd.c_str(), "KIC %f %f %f", &c, &d, &s) == 3) {
            if (!isfinite(c) || c < 0.0f || c > HOLD_CLAMP_MAX ||
                !isfinite(d) || d < 0.0f || d > HOLD_DEADBAND_MAX ||
                !isfinite(s) || s < HOLD_SETTLE_MIN || s > HOLD_SETTLE_MAX) {
                Out.println("{\"error\":\"KIC clamp 0-10, deadband 0-5, settle 0.1-100\"}");
                return;
            }
            holdIClampDeg = c; holdIDeadbandDeg = d; holdISettleDps = s;
            holdIntCfgSaved = currentHoldIntMatchesSaved();
            emitHoldIntAck();
        } else {
            Out.println("{\"error\":\"usage: KIC clampDeg deadbandDeg settleDps\"}");
        }
    } else if (cmd == "KIS") {
        // 保存當前 HOLD 積分組態到 NVS。
        holdIntCfgSaved = saveHoldIntToNVS();
        emitHoldIntAck();
    } else if (cmd == "KIRESET") {
        // 清 NVS + 回 BOOT 預設（Ki=0 → 積分關閉）。
        clearHoldIntNVS();
        for (int i = 0; i < NUM_MOTORS; i++) { holdKi[i] = BOOT_HOLD_KI; holdInt[i].reset(); }
        holdIClampDeg = BOOT_HOLD_CLAMP_DEG;
        holdIDeadbandDeg = BOOT_HOLD_DEADBAND_DEG;
        holdISettleDps = BOOT_HOLD_SETTLE_DPS;
        holdIntCfgSaved = false;
        emitHoldIntAck(true);
    } else if (cmd.startsWith("KI ")) {
        // HOLD 積分增益 `KI val`（全軸）或 `KI axis val`（單軸 0-5）。
        // 設 ki<=0 時 reset 該軸積分態 → 關閉即清零，避免 ki 再開啟時帶入殘留 windup（修 review nit）。
        float a, b;
        int n = sscanf(cmd.c_str(), "KI %f %f", &a, &b);
        if (n == 2) {
            int axis = (int)a;
            if (axis < 0 || axis >= NUM_MOTORS || !isfinite(b) || b < 0.0f || b > HOLD_KI_MAX) {
                Out.println("{\"error\":\"KI axis 0-5, ki 0-200\"}");
                return;
            }
            holdKi[axis] = b;
            if (b <= 0.0f) holdInt[axis].reset();
        } else if (n == 1) {
            if (!isfinite(a) || a < 0.0f || a > HOLD_KI_MAX) {
                Out.println("{\"error\":\"KI ki 0-200\"}");
                return;
            }
            for (int i = 0; i < NUM_MOTORS; i++) { holdKi[i] = a; if (a <= 0.0f) holdInt[i].reset(); }
        } else {
            Out.println("{\"error\":\"usage: KI val | KI axis val\"}");
            return;
        }
        holdIntCfgSaved = currentHoldIntMatchesSaved();
        emitHoldIntAck();
    } else if (cmd.startsWith("K ")) {
        int kp, ki, kd, kv;
        if (sscanf(cmd.c_str(), "K %d %d %d %d", &kp, &ki, &kd, &kv) == 4) {
            if (kp < 0 || kp > 1024 || ki < 0 || ki > 1024 || kd < 0 || kd > 1024 || kv < 0 || kv > 1024) {
                Out.println("{\"error\":\"PID range 0-1024\"}");
                return;
            }
            int okMask[NUM_MOTORS] = {0};
            int okCnt = 0;
            int arOk = applyVFOCPidGuarded(kp, ki, kd, kv, okMask, okCnt);
            pidSaved = currentPidMatchesSaved();
            Out.printf("{\"tune\":\"pid\",\"saved\":%d,\"kp\":%u,\"ki\":%u,\"kd\":%u,\"kv\":%u,"
                "\"ok\":[%d,%d,%d,%d,%d,%d],\"okCnt\":%d,\"ar_ok\":%d}\n",
                pidSaved ? 1 : 0, curKp, curKi, curKd, curKv,
                okMask[0],okMask[1],okMask[2],okMask[3],okMask[4],okMask[5], okCnt, arOk);
        }
    } else if (cmd.startsWith("V ")) {
        int spd, ac;
        if (sscanf(cmd.c_str(), "V %d %d", &spd, &ac) == 2) {
            posSpeed = constrain(spd, 1, 200);
            posAcc = constrain(ac, 1, 255);
            bool saved = saveRuntimeTuningToNVS();
            // V 是 ESP32 本地變數，下個 cycle 才會用到 → 永遠 ok（無 CAN 動作）
            Out.printf("{\"tune\":\"motion\",\"speed\":%d,\"acc\":%d,\"saved\":%d,"
                "\"ok\":[1,1,1,1,1,1],\"okCnt\":6,\"local\":1}\n", posSpeed, posAcc, saved ? 1 : 0);
        }
    } else if (cmd.startsWith("M ")) {
        // Joint-space 參數（C 0 模式用）
        float mu, kd = -1, mg = -1;
        int n = sscanf(cmd.c_str(), "M %f %f %f", &mu, &kd, &mg);
        if (n >= 1) {
            trackingMu = fmaxf(0.0f, mu);
            if (n >= 2) trackingKd = fmaxf(0.0f, kd);
            if (n >= 3) maxGain = fmaxf(0.05f, fminf(1.0f, mg));
            Out.printf("{\"status\":\"tracking params\",\"mu\":%.2f,\"kd\":%.2f,\"maxGain\":%.2f}\n",
                trackingMu, trackingKd, maxGain);
        }
    } else if (cmd.startsWith("T")) {
        int idx = cmd.charAt(1) - '0';
        if (idx >= 0 && idx < NUM_MOTORS) {
            servos.setEnable(MOTOR_ADDR[idx], true);
            delay(10);
            int64_t rawBefore = servos.readRawEncoderValue(MOTOR_ADDR[idx]);
            servos.setSpeed(MOTOR_ADDR[idx], 3, 0, 0);
            delay(500);
            servos.setSpeed(MOTOR_ADDR[idx], 0, 0, 0);
            int64_t rawAfter = servos.readRawEncoderValue(MOTOR_ADDR[idx]);
            servos.setEnable(MOTOR_ADDR[idx], false);
            int64_t delta = rawAfter - rawBefore;
            float degChange = MOTOR_SIGN[idx] * (float)delta * 360.0f / 16384.0f;
            Out.printf("{\"status\":\"test M%d\",\"raw_delta\":%lld,\"deg_change\":%.2f}\n",
                idx + 1, delta, degChange);
        }
    } else if (cmd.startsWith("U ")) {
        // 擾動脈衝（單軸）：U <motor 0-5> <deg> <ms> — 對該馬達 F5 目標加 deg 偏移持續 ms 後歸零。全模式通用。
        int idx, ms; float deg;
        if (sscanf(cmd.c_str(), "U %d %f %d", &idx, &deg, &ms) == 3 && idx >= 0 && idx < NUM_MOTORS) {
            for (int i = 0; i < NUM_MOTORS; i++) bumpDeg[i] = 0;
            bumpDeg[idx] = constrain(deg, -30.0f, 30.0f);   // 對齊 W 幅度上限（安全：避免手滑大角度甩出）
            bumpUntilMs = millis() + constrain(ms, 1, BUMP_MAX_MS);
            Out.printf("{\"status\":\"bump\",\"motor\":%d,\"deg\":%.2f,\"ms\":%d,\"hold\":%d}\n",
                idx + 1, bumpDeg[idx], ms, holdMode ? 1 : 0);   // 回報 clamp 後實際值，不謊報
        }
    } else if (cmd.startsWith("W ")) {
        // 擾動脈衝（六軸協同）：W <d0> <d1> <d2> <d3> <d4> <d5> <ms> — 六軸同時加各自偏移持續 ms 後歸零。全模式通用。
        // host 端 disturb.js 用 task-space IK 算各軸 deg（已乘 MOTOR_SIGN 轉 enc 慣例），這裡只照單全收
        float d[NUM_MOTORS]; int ms;
        if (sscanf(cmd.c_str(), "W %f %f %f %f %f %f %d",
                   &d[0], &d[1], &d[2], &d[3], &d[4], &d[5], &ms) == 7) {
            for (int i = 0; i < NUM_MOTORS; i++) bumpDeg[i] = constrain(d[i], -30.0f, 30.0f);
            bumpUntilMs = millis() + constrain(ms, 1, BUMP_MAX_MS);
            Out.printf("{\"status\":\"bumpW\",\"deg\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],\"ms\":%d,\"hold\":%d}\n",
                bumpDeg[0], bumpDeg[1], bumpDeg[2], bumpDeg[3], bumpDeg[4], bumpDeg[5], ms, holdMode ? 1 : 0);
        }
    } else if (cmd.startsWith("R ")) {
        float v[6];
        if (sscanf(cmd.c_str(), "R %f %f %f %f %f %f", &v[0],&v[1],&v[2],&v[3],&v[4],&v[5]) == 6) {
            Pose testPose = {v[0], v[1], v[2], v[3], v[4], v[5]};
            IKResult ik = inverse_kinematics(testPose);
            if (!ik.valid) {
                Out.println("{\"error\":\"IK invalid for test pose\"}");
            } else {
                FKSolver fk;
                Pose r = fk.solve(ik.angles);
                Out.printf("{\"roundtrip\":{\"err\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],\"iter\":%d}}\n",
                    r.x-v[0], r.y-v[1], r.z-v[2], r.roll-v[3], r.pitch-v[4], r.yaw-v[5],
                    fk.iterations);
            }
        }
    } else if (cmd == "F") {
        FKSolver fk;
        Pose p = fk.solve(enc.angles);
        Out.printf("{\"fk\":[%.2f,%.2f,%.2f,%.3f,%.3f,%.3f],\"iter\":%d,\"err\":%.3f,\"ok\":%d}\n",
            p.x, p.y, p.z, p.roll, p.pitch, p.yaw,
            fk.iterations, fk.residual, fk.converged ? 1 : 0);
    } else if (cmd == "I") {
        Out.print("{\"diag\":{\"zeroRaw\":[");
        for (int i = 0; i < NUM_MOTORS; i++) Out.printf("%s%lld", i?",":"", enc.zeroRaw[i]);
        Out.printf("],\"mode\":%d,\"posEnabled\":%d}}\n", controlMode, posEnabled ? 1 : 0);
    } else if (cmd == "B") {
        Out.printf("{\"silent_boot\":{\"total\":%d,\"per_id\":[%d,%d,%d,%d,%d,%d]}}\n",
                      silentBootTotal,
                      silentBootPerId[1],silentBootPerId[2],silentBootPerId[3],
                      silentBootPerId[4],silentBootPerId[5],silentBootPerId[6]);
    } else if (cmd == "X") {
        // 深度診斷已完成（根因 = 某顆 motor firmware auto-broadcast bug，~78% reply rate）
        // 保留輕量化版供日後快速健診：只跑 listen + 6 顆 baseline
        bool savedPos = posEnabled, savedHold = holdMode;
        canRxPaused = true;
        posEnabled = false; holdMode = false;
        delay(100);
        servos.flushReceiveBuffer();
        Out.println("{\"xdiag\":\"start\"}");

        // 三段對比：normal/active(ACK) vs listen-only(no-ACK) vs filter to id=99
        auto countListen = [&](uint32_t ms) -> int {
            uint32_t st = millis();
            int n = 0;
            while (millis() - st < ms) {
                if (servos.can.checkReceive() == CAN_MSGAVAIL) {
                    unsigned long id; uint8_t len; uint8_t buf[8];
                    servos.can.readMsgBuf(&id, &len, buf);
                    n++;
                }
            }
            return n;
        };

        // (a) normal mode 基準
        int normalRx = countListen(1500);

        // (b) listen-only mode — 我們不 ACK 任何東西，看 spam 是否消失
        bool lo = servos.can.setListenOnly();
        int listenOnlyRx = countListen(1500);

        // SANITY 3: 5 次 write-read 看是否穩定 + mode change 行為
        servos.can.setNormal();
        servos.can.rawSetMode(0x80);
        for (int t = 0; t < 5; t++) {
            uint8_t pat = 0x55 << (t & 1);  // 0x55 / 0xAA alternating
            servos.can.rawWriteReg(0x20, pat);
            uint8_t r1 = servos.can.rawReadReg(0x20);
            uint8_t r2 = servos.can.rawReadReg(0x20);  // 再讀一次
            Out.printf("{\"san3_inconfig\":{\"t\":%d,\"wrote\":\"0x%02X\",\"r1\":\"0x%02X\",\"r2\":\"0x%02X\"}}\n",
                          t, pat, r1, r2);
        }
        // 寫 0xAA，然後切 normal，再讀
        servos.can.rawWriteReg(0x20, 0xAA);
        uint8_t conf_r = servos.can.rawReadReg(0x20);
        servos.can.rawSetMode(0x00);
        uint8_t norm_r1 = servos.can.rawReadReg(0x20);
        uint8_t norm_r2 = servos.can.rawReadReg(0x20);
        Out.printf("{\"san3_modechange\":{\"conf\":\"0x%02X\",\"norm1\":\"0x%02X\",\"norm2\":\"0x%02X\"}}\n",
                      conf_r, norm_r1, norm_r2);

        // TEST 1: dynamic filter to id=3, listen 1.5s, check id distribution
        servos.can.rawSetHardwareFilter(3);
        uint32_t lstart = millis();
        int idCnt[8] = {0};
        int filterRx = 0;
        while (millis() - lstart < 1500) {
            if (servos.can.checkReceive() == CAN_MSGAVAIL) {
                unsigned long id; uint8_t len; uint8_t buf[8];
                servos.can.readMsgBuf(&id, &len, buf);
                filterRx++;
                if (id < 8) idCnt[id]++;
            }
        }
        Out.printf("{\"dyn_filter_to_3\":{\"total\":%d,\"per_id\":[%d,%d,%d,%d,%d,%d,%d,%d]}}\n",
                      filterRx, idCnt[0],idCnt[1],idCnt[2],idCnt[3],idCnt[4],idCnt[5],idCnt[6],idCnt[7]);

        // TEST 2: per-motor 100 query with per-query dynamic filter
        // (filter changes before each query to target motor)
        Out.println("{\"xdiag\":\"per_motor_with_dyn_filter\"}");
        for (int i = 0; i < NUM_MOTORS; i++) {
            servos.can.rawSetHardwareFilter(MOTOR_ADDR[i]);
            servos.flushReceiveBuffer();
            int ok=0;
            for (int k = 0; k < 100; k++) {
                if (servos.readRawEncoderValue(MOTOR_ADDR[i], 20) != INT64_MIN) ok++;
            }
            Out.printf("{\"xdiag\":\"dyn_m%d\",\"ok\":%d}\n", i+1, ok);
        }
        servos.can.rawSetFilterAcceptAll();

        Out.printf("{\"xdiag\":\"bus_test\",\"normal_rx\":%d,\"listen_only_rx\":%d,\"filter99_rx\":%d,"
                      "\"lo_ok\":%d,\"f_ok\":%d}\n",
                      normalRx, listenOnlyRx, filterRx, lo?1:0, 1);
        Out.printf("{\"xdiag\":\"spam\",\"baseline\":%d}\n", normalRx);

        // 失敗分類：nf=完全沒收到 frame, wm=收到別顆 frame, wc=對的 motor 錯 cmd
        servos.flushReceiveBuffer();
        for (int i = 0; i < NUM_MOTORS; i++) {
            int ok=0, nf=0, wm=0, wc=0, cf=0;
            for (int k = 0; k < 100; k++) {
                uint8_t txCmd[2] = {0x35, (uint8_t)((MOTOR_ADDR[i]+0x35) & 0xFF)};
                if (servos.can.sendMsgBuf(MOTOR_ADDR[i], 0, 2, txCmd) != CAN_OK) { nf++; continue; }
                uint32_t t0 = micros();
                bool got=false, sawAny=false;
                int lwm=0, lwc=0, lcf=0;
                while ((micros()-t0) < 30000) {
                    if (servos.can.checkReceive() == CAN_MSGAVAIL) {
                        sawAny = true;
                        unsigned long rxId; uint8_t rxLen; uint8_t rxBuf[8];
                        servos.can.readMsgBuf(&rxId, &rxLen, rxBuf);
                        if ((rxId & 0x7FF) != MOTOR_ADDR[i]) { lwm++; continue; }
                        if (rxLen < 1 || rxBuf[0] != 0x35) { lwc++; continue; }
                        if (rxLen < 8) { lcf++; continue; }
                        uint8_t crc = MOTOR_ADDR[i];
                        for (int j = 0; j < 7; j++) crc += rxBuf[j];
                        if ((crc & 0xFF) != rxBuf[7]) { lcf++; continue; }
                        ok++; got=true; break;
                    }
                }
                if (!got) {
                    if (!sawAny) nf++;
                    else if (lwm) wm++;
                    else if (lwc) wc++;
                    else cf++;
                }
            }
            Out.printf("{\"xdiag\":\"m%d\",\"ok\":%d,\"nf\":%d,\"wm\":%d,\"wc\":%d,\"cf\":%d}\n",
                          i+1, ok, nf, wm, wc, cf);
        }

        // 再 listen 1s 看 query 過程是否誘發 spam
        servos.flushReceiveBuffer();
        uint32_t lstart2 = millis();
        int idCnt2[7] = {0};
        while (millis() - lstart2 < 1000) {
            if (servos.can.checkReceive() == CAN_MSGAVAIL) {
                unsigned long id; uint8_t len; uint8_t buf[8];
                servos.can.readMsgBuf(&id, &len, buf);
                if (id >= 1 && id <= 6) idCnt2[id]++;
            }
        }
        Out.printf("{\"xdiag\":\"spam_after_query\",\"per_id\":[%d,%d,%d,%d,%d,%d]}\n",
                      idCnt2[1],idCnt2[2],idCnt2[3],idCnt2[4],idCnt2[5],idCnt2[6]);
        Out.println("{\"xdiag\":\"done\"}");
        posEnabled = savedPos; holdMode = savedHold;
        servos.flushReceiveBuffer();
        canRxPaused = false;
    }
}

// USB 指令來源：讀一行 → dispatch。語意同舊版 handleSerial。
void handleSerial() {
    // 每次排空所有待處理行（上限 16），避免 loopTask 被稀疏排程時 PF 突發在 UART RX 累積延遲。
    // Serial.setTimeout(20)（setup）使半行最壞只卡 20ms 而非預設 1000ms。
    for (int i = 0; i < 16 && Serial.available(); i++) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() == 0) continue;
        if (enqueueCommand(cmd, false)) netIsControlSource = false;    // USB 任何指令（含純查詢如 WIFI?）→ 解除 WiFi failsafe arm
    }
}

static void controlTick() {
    uint32_t ctrlStartUs = micros();

    // netTask wedge 自救：netTask 每輪更新 netTaskHeartbeatMs；>NET_WEDGE_REBOOT_MS 沒更新（且非 OTA 中）
    // = core0 網路堆疊掛死（觀察到：telemetry 凍結+TCP 不回應+不自我重連）→ 整機重啟。
    // core1 always-running 故由此監看 core0；reset_reason 將顯示 sw。心跳=0 表 netTask 尚未起跑，不誤判。
    if (netTaskHeartbeatMs != 0 && !netOtaBusy() &&
        millis() - netTaskHeartbeatMs > NET_WEDGE_REBOOT_MS) {
        Out.println("{\"fatal\":\"netTask wedge, reboot\"}");
        delay(20);
        ESP.restart();
    }

    if (otaStopRequested) {
        otaSafetyStop();
        otaStopRequested = false;
    }

    QueuedCommand qc;
    for (int i = 0; i < 8 && commandQueue && xQueueReceive(commandQueue, &qc, 0) == pdTRUE; i++) {
        if (qc.fromNet) netIsControlSource = true;
        dispatch(String(qc.cmd), qc.fromNet);
    }

    // 斷線失效保護（P4）：以 WiFi 控制且 posEnabled 時，連線中斷即進安全態
    checkFailsafe();

    if (netOtaBusy()) return;

    static uint32_t lastReadUs = 0;
    static uint32_t ctrlCycles = 0;
    static uint32_t dtMinMs = UINT32_MAX;
    static uint32_t dtMaxMs = 0;
    static uint32_t dtSumMs = 0;
    uint32_t nowUs = micros();
    uint32_t periodUs = loopPeriodUs;
    uint32_t elapsedUs = lastReadUs ? (nowUs - lastReadUs) : periodUs;
    uint32_t elapsed = elapsedUs / 1000;  // ms
    lastReadUs = nowUs;
    ctrlCycles++;
    if (elapsedUs > periodUs + periodUs / 2) controlMissedCount++;
    if (elapsed < 1000) {  // ignore the first post-boot interval if lastReadUs was 0
        if (elapsed < dtMinMs) dtMinMs = elapsed;
        if (elapsed > dtMaxMs) dtMaxMs = elapsed;
        dtSumMs += elapsed;
    }

    // 真實取樣間隔（秒），餵控制器的時間導數/平滑項。clamp 到標稱的 [0.5×, 3×]：
    // 過短 → vel=noise/dt 把 D 項炸成高頻放大器；stall 後一個巨大 Δ 配正常 dt 會 spike。
    float nominalDt = periodUs / 1e6f;
    float dt = fmaxf(0.5f * nominalDt, fminf(3.0f * nominalDt, (float)elapsedUs / 1000000.0f));

    int64_t raw[NUM_MOTORS];
    int ok = 0;
    uint32_t canReadUs = 0;
    uint16_t arAgeMs[NUM_MOTORS] = {0};
    bool encStale = false;   // 編碼器資料不新鮮 → HOLD 積分凍結（盲積分 = 隱形 windup）
    if (autoReturnMode) {
        // core0 canRxTask 連續排空；控制環只在同一把 CAN 鎖下取最新快照，避免 int64 tear。
        // auto-return 是冗餘串流，短暫缺一兩個 frame 屬正常；沿用 last-known-good，不把它當故障停機。
        bool warnStale[NUM_MOTORS] = {false};
        {
            CanGuard _g(servos.can);
            uint32_t nowMsForFreshness = millis();
            for (int i = 0; i < NUM_MOTORS; i++) {
                raw[i] = latestRaw[i];
                if (raw[i] != INT64_MIN && latestRawAtMs[i] != 0) {
                    ok++;
                    uint32_t age = nowMsForFreshness - latestRawAtMs[i];
                    arAgeMs[i] = age > 65535 ? 65535 : (uint16_t)age;
                    warnStale[i] = age > AR_WARN_MS;
                } else {
                    raw[i] = INT64_MIN;
                    arAgeMs[i] = 65535;
                    warnStale[i] = true;
                }
            }
        }
        bool anyWarn = false;
        for (int i = 0; i < NUM_MOTORS; i++) anyWarn = anyWarn || warnStale[i];
        encStale = anyWarn;
        if (!anyWarn) autoReturnWarnLatched = false;
        if (posEnabled && anyWarn && !autoReturnWarnLatched) {
                autoReturnWarnLatched = true;
                Out.printf("{\"warn\":\"auto-return old\",\"ok\":%d,\"age\":[%u,%u,%u,%u,%u,%u],\"action\":\"holding last-known-good\"}\n",
                    ok, arAgeMs[0], arAgeMs[1], arAgeMs[2], arAgeMs[3], arAgeMs[4], arAgeMs[5]);
        }
    } else {
        servos.flushReceiveBuffer();
        uint32_t _canT0 = micros();
        ok = servos.readAllRawEncoders(raw);
        canReadUs = micros() - _canT0;   // 讀 6 顆編碼器的純 CAN 來回時間（頻寬指紋核心）
        busTxQ += NUM_MOTORS;            // 匯流排佔用：每次輪詢送 6 筆 0x35 查詢（各保證一筆回覆）
        encStale = (ok < NUM_MOTORS);    // 任一顆讀失敗 → enc.angles 保持舊值 → 凍結積分
    }
    enc.updateAngles(raw);

    float motorTargets[NUM_MOTORS] = {0};
    int32_t coords[NUM_MOTORS] = {0};

    if (posEnabled) {
        bool controlOk = false;

        if (holdMode) {
            // ===== HOLD：直送 snapshot，跳過所有 PD/IK/平滑 =====
            // 跟隨模式（FOLLOW/PF/動作庫）：ESP32 線上 reference generator 接管 holdAngles，與 minimum-jerk profile 互斥。
            if (followMode) {
                followStep(dt);
            } else if (holdMoveMs > 0) {
            // 姿態移動（P）：minimum-jerk 過渡 holdAngles → holdTarget，端點速度/加速度皆為 0。
                float t = (float)(millis() - holdMoveStart) / holdMoveMs;
                if (t >= 1.0f) {
                    for (int i = 0; i < NUM_MOTORS; i++) holdAngles[i] = holdTarget[i];
                    holdMoveMs = 0;
                } else {
                    float s = minJerk01(t);
                    for (int i = 0; i < NUM_MOTORS; i++)
                        holdAngles[i] = holdStart[i] + (holdTarget[i] - holdStart[i]) * s;
                }
            }
            // 擾動疊加已移至 F5 共同匯流口（plant-input，全模式通用）→ 此處只設 HOLD 目標。
            for (int i = 0; i < NUM_MOTORS; i++) motorTargets[i] = holdAngles[i];
            // 積分主閘：唯有「純死咬靜止 + 編碼器新鮮」才積分；運動/跟隨/stale 一律凍結。
            bool intActive = !followMode && holdMoveMs == 0 && !encStale;
            applyHoldIntegral(motorTargets, dt, intActive);   // 積分外環先疊命令（消 droop）
            applyHoldOutputDamping(motorTargets, dt);          // 阻尼後處理（看 enc 速度）
            float maxAbs = 0;
            for (int i = 0; i < NUM_MOTORS; i++) {
                float e = fabsf(enc.angles[i] - holdAngles[i]);
                if (e > maxAbs) maxAbs = e;
            }
            if (maxAbs > maxHoldErr) maxHoldErr = maxAbs;
            controlOk = true;
        } else if (controlMode == 1) {
            // ===== Task-space PD =====
            controlOk = tsController.update(enc.angles, targetPose, motorTargets, dt);
            if (!controlOk) ctlFkFailWin++;

            // FK 連續失敗 5 次 → 自動降回 joint-space
            if (tsController.fkFailCount >= 5) {
                controlMode = 0;
                smoothedTarget = targetPose;
                for (int i = 0; i < NUM_MOTORS; i++)
                    prevAngles[i] = enc.angles[i];
                adaptiveFirstCycle = true;
                Out.println("{\"warn\":\"FK fail x5, fallback to joint-space\"}");
            }
        }

        if (!holdMode && controlMode == 0) {
            // ===== Joint-space 備用 =====
            // 平滑時間常數用真實 dt（改 L 時 smoothing 速率才正確）；
            // 注意：下方自適應追蹤的 vel 是 per-cycle Δ（不除 dt）為刻意設計，
            // 該公式本質與迴圈率耦合，未在此正規化（見 audit Gate A confound 說明）。
            smoothPose(smoothedTarget, targetPose, dt);

            IKResult target = inverse_kinematics(smoothedTarget);
            if (!target.valid) {
                ctlIkFailWin++;
                posStop();
                Out.println("{\"error\":\"IK invalid, pos stopped\"}");
            } else {
                if (adaptiveFirstCycle) {
                    for (int i = 0; i < NUM_MOTORS; i++)
                        prevAngles[i] = enc.angles[i];
                    smoothKinetic = 0;
                    adaptiveFirstCycle = false;
                }

                float vel[NUM_MOTORS];
                float kinetic = 0;
                for (int i = 0; i < NUM_MOTORS; i++) {
                    vel[i] = enc.angles[i] - prevAngles[i];
                    kinetic += vel[i] * vel[i];
                }

                smoothKinetic = 0.9f * smoothKinetic + 0.1f * kinetic;
                float gain = fminf(maxGain, 1.0f / (1.0f + trackingMu * smoothKinetic));

                for (int i = 0; i < NUM_MOTORS; i++) {
                    float error = target.angles[i] - enc.angles[i];
                    float dampRatio = (error * vel[i] < 0) ? 1.0f : 0.3f;
                    float damp = trackingKd * vel[i] * dampRatio;
                    motorTargets[i] = enc.angles[i] + gain * error - damp;
                    prevAngles[i] = enc.angles[i];
                }
                controlOk = true;
            }
        }

        // 發送 F5 指令
        if (controlOk) {
            // 擾動注入（plant-input 疊加層）：HOLD / task-space / joint 算完 motorTargets 後在此統一疊加，
            // 故運動中也能注入；脈衝過期自動歸零。bumpUntilMs==0 表無擾動。
            bool bumpActive = (bumpUntilMs != 0 && millis() < bumpUntilMs);
            if (bumpUntilMs != 0 && !bumpActive) clearBump();   // 脈衝結束
            uint32_t _f5t0 = micros();      // [Phase0] 量 6×F5 burst 是否卡 core1（預期 sub-ms）
            for (int i = 0; i < NUM_MOTORS; i++) {
                float cmd = motorTargets[i] + (bumpActive ? bumpDeg[i] : 0.0f);
                coords[i] = enc.angleToCoord(i, cmd, enableAngle[i]);
                if (abs(coords[i]) > 8192) {
                    ctlCoordLimitWin++;
                    posStop();
                    Out.println("{\"error\":\"coord out of range, pos stopped\"}");
                    controlOk = false;
                    break;
                }
                int32_t absCoord = abs(coords[i]);
                if (absCoord > ctlCoordMaxAbs) ctlCoordMaxAbs = absCoord;
                servos.setAbsoluteCoord(MOTOR_ADDR[i], posSpeed, posAcc, coords[i]);
                busTxF5++;                 // 匯流排佔用：F5 指令幀（DLC 8）
            }
            f5us = micros() - _f5t0;        // [Phase0]
            if (f5us > f5usMax) f5usMax = f5us;
        }
    }

    // 遙測與控制解耦：每 TELE_PERIOD_MS 才輸出（高速迴圈下全遙測會塞爆 115200 序列埠）
    static uint32_t lastTeleMs = 0;
    uint32_t nowMs = millis();
    if (nowMs - lastTeleMs < TELE_PERIOD_MS) return;
    uint32_t teleWin = nowMs - lastTeleMs;                    // 遙測窗長（匯流排佔用分母）
    uint32_t teleCycles = ctrlCycles;
    uint32_t teleDtMin = (dtMinMs == UINT32_MAX) ? 0 : dtMinMs;
    uint32_t teleDtMax = dtMaxMs;
    float teleDtAvg = teleCycles ? (float)dtSumMs / teleCycles : 0.0f;
    float lhz = teleCycles * 1000.0f / teleWin;               // 實測控制迴圈頻率
    ctrlCycles = 0;
    dtMinMs = UINT32_MAX;
    dtMaxMs = 0;
    dtSumMs = 0;
    lastTeleMs = nowMs;

    uint16_t rxPerId[NUM_MOTORS];
    uint8_t maxDrain = 0;
    servos.snapshotAndResetRxStats(rxPerId, &maxDrain);
    for (int i = 0; i < NUM_MOTORS; i++) busRxEnc += rxPerId[i];

    // JSON 輸出資料只做 snapshot；實際格式化/USB/TCP 輸出交給 telemetryTask。
    uint8_t eflg = servos.can.getEFLG();
    uint8_t tec  = servos.can.getTEC();
    uint8_t rec  = servos.can.getREC();
    if (eflg & 0xC0) { busOvr++; servos.can.clearRXOverflow(); }  // [Phase0] 先計次再清

    uint32_t ctrlUs = micros() - ctrlStartUs;
    if (ctrlUs > controlMaxUs) controlMaxUs = ctrlUs;
    uint32_t jitterUs = (elapsedUs > periodUs) ? (elapsedUs - periodUs) : (periodUs - elapsedUs);

    TelemetrySnapshot snap{};
    for (int i = 0; i < NUM_MOTORS; i++) {
        snap.angles[i] = enc.angles[i];
        snap.raw[i] = raw[i];
        snap.busPer[i] = rxPerId[i];
        snap.arAge[i] = arAgeMs[i];
        snap.motorTgt[i] = motorTargets[i];
    }
    snap.ok = ok;
    snap.zeroed = enc.zeroed ? 1 : 0;
    snap.pos = posEnabled ? 1 : 0;
    snap.controlMode = controlMode;
    snap.eflg = eflg; snap.tec = tec; snap.rec = rec;
    snap.t = nowMs; snap.dtMs = elapsed; snap.canReadUs = canReadUs;
    snap.lhz = lhz;
    snap.arMs = autoReturnMode ? arPeriodMs : 0;
    snap.busF5 = busTxF5; snap.busQ = busTxQ; snap.busRx = busRxEnc; snap.busMs = teleWin;
    snap.busOvr = busOvr; snap.f5us = f5us; snap.f5max = f5usMax; snap.maxDrain = maxDrain;
    snap.teleCycles = teleCycles; snap.dtMinMs = teleDtMin; snap.dtMaxMs = teleDtMax; snap.dtAvgMs = teleDtAvg;
    snap.ctlIkFail = ctlIkFailWin; snap.ctlFkFail = ctlFkFailWin; snap.ctlCoordLimit = ctlCoordLimitWin;
    snap.ctlCoordMaxAbs = ctlCoordMaxAbs;
    snap.fkStreak = tsController.fkFailCount;
    snap.pid[0] = curKp; snap.pid[1] = curKi; snap.pid[2] = curKd; snap.pid[3] = curKv;
    snap.pidSaved = pidSaved ? 1 : 0;
    snap.savedPid[0] = savedKp; snap.savedPid[1] = savedKi; snap.savedPid[2] = savedKd; snap.savedPid[3] = savedKv;
    snap.nvsPid = nvsPidPresent ? 1 : 0;
    snap.followMode = followMode ? 1 : 0;
    snap.profile = followMode ? 1 : (holdMoveMs > 0 ? 2 : 0);
    for (int i = 0; i < 6; i++) {
        snap.followCurPose[i] = followCur[i];
        snap.followTgtPose[i] = followTgt[i];
        snap.followVelPose[i] = followVel[i];
    }
    snap.followVmax[0] = followVmaxT;
    snap.followVmax[1] = followVmaxR;
    snap.followTight = followTight;
    snap.followAmax[0] = followAmaxForAxis(0);
    snap.followAmax[1] = followAmaxForAxis(3);
    snap.followJmax[0] = followJmaxForAxis(0);
    snap.followJmax[1] = followJmaxForAxis(3);
    snap.ctrlUs = ctrlUs;
    snap.ctrlMaxUs = controlMaxUs;
    snap.jitterUs = jitterUs;
    snap.missed = controlMissedCount;
    snap.teleDrop = teleDropCount;
    snap.cmdDrop = cmdDropCount;
    snap.snapAgeMs = 0;

    if (posEnabled && holdMode) {
        snap.extra = TELEM_EXTRA_HOLD;
        for (int i = 0; i < NUM_MOTORS; i++) {
            snap.hold[i] = holdAngles[i];
            snap.herr[i] = enc.angles[i] - holdAngles[i];
        }
        snap.hmax = maxHoldErr;
        snap.holdDampGain = holdDampGainSec;
        snap.holdDampMax = holdDampMaxDeg;
        snap.holdDampCorrMax = holdDampCorrMaxWin;
        for (int i = 0; i < NUM_MOTORS; i++) {
            snap.holdIntC[i] = holdIntCorr[i];
            snap.holdKiSnap[i] = holdKi[i];
        }
        snap.holdIClamp = holdIClampDeg;
        snap.holdIDead = holdIDeadbandDeg;
        snap.holdISettle = holdISettleDps;
        snap.holdIntSaved = holdIntCfgSaved ? 1 : 0;
    } else if (posEnabled && controlMode == 1) {
        const Pose& fk = tsController.currentPose;
        const Pose& er = tsController.poseError;
        snap.extra = TELEM_EXTRA_TASK;
        snap.fkPose[0] = fk.x; snap.fkPose[1] = fk.y; snap.fkPose[2] = fk.z;
        snap.fkPose[3] = fk.roll; snap.fkPose[4] = fk.pitch; snap.fkPose[5] = fk.yaw;
        snap.poseErr[0] = er.x; snap.poseErr[1] = er.y; snap.poseErr[2] = er.z;
        snap.poseErr[3] = er.roll; snap.poseErr[4] = er.pitch; snap.poseErr[5] = er.yaw;
        snap.fkIterations = tsController.fk.iterations;
    } else if (posEnabled && controlMode == 0) {
        snap.extra = TELEM_EXTRA_JOINT;
        snap.jointGain = maxGain;
    }

    busTxF5 = busTxQ = busRxEnc = 0;   // 匯流排計數每窗歸零
    busOvr = 0;                                                // [Phase0]
    f5usMax = 0;
    holdDampCorrMaxWin = 0.0f;
    ctlIkFailWin = ctlFkFailWin = ctlCoordLimitWin = 0;
    ctlCoordMaxAbs = 0;

    if (telemetryQueue) {
        if (uxQueueMessagesWaiting(telemetryQueue) > 0) {
            teleDropCount++;
            snap.teleDrop = teleDropCount;
        }
        xQueueOverwrite(telemetryQueue, &snap);
    }
}
