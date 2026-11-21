/**
  ******************************************************************************
  * @file    handheld.c
  * @brief   手持控制器业务逻辑
  *          JY901S 姿态采集 → 组包校验 → NRF24L01 无线发送
  ******************************************************************************
  */
#include "handheld.h"

/* ============================ 参数配置 ============================ */

/* 数据包协议(与机械臂端完全一致) */
#define PKT_SYNC1             0xAA
#define PKT_SYNC2             0x55
#define PKT_CHECKSUM_OFFSET   9
#define PKT_ANGLE_UNIT        10.0f   /* 角度换算: 度 → 0.1° 单位(×10) */

/* 夹爪手势控制(可根据需要改为外接按键) */
#define GRIP_GESTURE_THRESHOLD_DEG  45.0f   /* 横滚角超过 ±45° 触发 */
#define GRIP_GESTURE_HOLD_MS        300     /* 保持 300ms 消抖 */
#define GRIP_OPEN_ANGLE             40      /* 夹爪张开(发送角度) */
#define GRIP_CLOSE_ANGLE            150     /* 夹爪闭合(发送角度) */

/* 发送周期(ms), 太快会塞满发送 FIFO, 太慢则跟随迟缓 */
#define TX_INTERVAL_MS              20

/* ============================ 内部变量 ============================ */

static uint8_t  s_gripClosed   = 0;      /* 0=张开 1=闭合 */
static uint8_t  s_gestureActive= 0;      /* 手势是否正在保持 */
static uint8_t  s_gestureDone  = 0;      /* 本次手势是否已触发 */
static uint32_t s_gestureStart = 0;
static uint32_t s_lastTxTick   = 0;
static uint8_t  s_hasAngleData = 0;      /* 是否已收到有效角度 */
static uint8_t  s_rfOk         = 0;      /* NRF24L01 模块自检结果 */
static float    s_dispRoll    = 0.0f;    /* 供 OLED 显示的最近角度(度) */
static float    s_dispPitch   = 0.0f;
static float    s_dispYaw     = 0.0f;
static uint8_t  s_dispGripClosed = 0;    /* 供 OLED 显示的夹爪状态 */
static uint8_t  s_dispValid   = 0;       /* 是否已有可显示数据 */

/* ============================ 内部函数 ============================ */

/* 夹爪手势检测: 横滚角超过阈值并保持一段时间后切换开/合 */
static void Handheld_UpdateGrip(float rollDeg)
{
    uint32_t now = HAL_GetTick();

    if ((rollDeg > GRIP_GESTURE_THRESHOLD_DEG) || (rollDeg < -GRIP_GESTURE_THRESHOLD_DEG))
    {
        if (!s_gestureActive)
        {
            s_gestureActive = 1;
            s_gestureDone   = 0;
            s_gestureStart  = now;
        }
        else if (!s_gestureDone && ((now - s_gestureStart) >= GRIP_GESTURE_HOLD_MS))
        {
            s_gripClosed = (s_gripClosed == 0) ? 1 : 0;   /* 切换开/合 */
            s_gestureDone = 1;                            /* 本次手势只触发一次 */
        }
    }
    else
    {
        s_gestureActive = 0;
        s_gestureDone   = 0;
    }
}

/* ============================ 接口函数 ============================ */

/* 系统初始化: 解析器、无线发送模块 */
void Handheld_Init(void)
{
    Jy901s_Init();
    NRF24_Init();
    /* NRF24_Check() 返回 0 表示模块通信异常(SPI 不通/接线错误), 供 OLED 显示 */
    s_rfOk = NRF24_Check();
    s_lastTxTick = HAL_GetTick();
}

/* 主循环处理: 采集 → 组包 → 发送 */
void Handheld_Process(void)
{
    uint8_t  buf[NRF24_PAYLOAD_WIDTH];
    int16_t  roll, pitch, yaw;
    uint8_t  grip, sum, i;
    float    r, p, y;
    uint32_t now;

    /* 1. 轮询读取并解析 JY901S */
    Jy901s_Poll();

    if (Jy901s_IsAngleUpdated())
    {
        s_hasAngleData = 1;   /* 收到至少一帧有效角度 */
    }

    /* 2. 按固定周期发送, 避免过快 */
    now = HAL_GetTick();
    if ((now - s_lastTxTick) < TX_INTERVAL_MS)
    {
        return;
    }
    s_lastTxTick = now;

    /* 尚未收到有效角度前不发送 */
    if (!s_hasAngleData)
    {
        return;
    }

    /* 3. 读取最新姿态角度(度) */
    Jy901s_GetAngle(&r, &p, &y);

    /* 4. 更新夹爪手势 */
    Handheld_UpdateGrip(r);

    /* 5. 角度(度) → 0.1° 单位(与机械臂端协议一致) */
    roll  = (int16_t)(r * PKT_ANGLE_UNIT);
    pitch = (int16_t)(p * PKT_ANGLE_UNIT);
    yaw   = (int16_t)(y * PKT_ANGLE_UNIT);

    grip = s_gripClosed ? GRIP_CLOSE_ANGLE : GRIP_OPEN_ANGLE;

    /* 保存最近数据, 供 OLED 等外设显示 */
    s_dispRoll       = r;
    s_dispPitch      = p;
    s_dispYaw        = y;
    s_dispGripClosed = s_gripClosed;
    s_dispValid      = 1;

    /* 6. 组包(32 字节):
     *    [0]0xAA [1]0x55
     *    [2..3]roll [4..5]pitch [6..7]yaw (int16 小端, 0.1°)
     *    [8]grip(0~180)
     *    [9]校验和 = 字节2~8 累加低8位
     *    [10..31]保留 0 */
    for (i = 0; i < NRF24_PAYLOAD_WIDTH; i++)
    {
        buf[i] = 0;
    }
    buf[0] = PKT_SYNC1;
    buf[1] = PKT_SYNC2;
    buf[2] = (uint8_t)(roll & 0xFF);
    buf[3] = (uint8_t)((roll >> 8) & 0xFF);
    buf[4] = (uint8_t)(pitch & 0xFF);
    buf[5] = (uint8_t)((pitch >> 8) & 0xFF);
    buf[6] = (uint8_t)(yaw & 0xFF);
    buf[7] = (uint8_t)((yaw >> 8) & 0xFF);
    buf[8] = grip;

    sum = 0;
    for (i = 2; i < PKT_CHECKSUM_OFFSET; i++)
    {
        sum += buf[i];
    }
    buf[PKT_CHECKSUM_OFFSET] = sum;

    /* 7. 无线发送 */
    NRF24_TxPacket(buf, NRF24_PAYLOAD_WIDTH);
}

/* 获取最近一次姿态数据(供 OLED 显示), 返回 0 表示尚无有效数据 */
uint8_t Handheld_GetDisplayData(float *pRoll, float *pPitch, float *pYaw, uint8_t *pGripClosed)
{
    if (!s_dispValid)
    {
        return 0;
    }

    if (pRoll)       *pRoll       = s_dispRoll;
    if (pPitch)      *pPitch      = s_dispPitch;
    if (pYaw)        *pYaw        = s_dispYaw;
    if (pGripClosed) *pGripClosed = s_dispGripClosed;

    return 1;
}

/* 获取 NRF24L01 模块自检结果(供 OLED 显示诊断) */
uint8_t Handheld_GetRfStatus(void)
{
    return s_rfOk;
}
