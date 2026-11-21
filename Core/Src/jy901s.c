/**
  ******************************************************************************
  * @file    jy901s.c
  * @brief   JY901S 姿态传感器 I2C 读取实现
  * @note    通过软件 I2C(PB6=SCL, PB7=SDA) 读取 JY901S 角度寄存器
  *          JY901S I2C 从机地址: 0x50 (7 位)
  *          角度寄存器 0x3D: Roll(2) Pitch(2) Yaw(2) 版本号(2), int16 小端
  *          角度换算: 原始值 / 32768 * 180 (度)
  ******************************************************************************
  */
#include "jy901s.h"

/* 引脚定义: PB6=SCL, PB7=SDA (软件 I2C) */
#define JY901S_SCL_PORT      GPIOB
#define JY901S_SCL_PIN       GPIO_PIN_6
#define JY901S_SDA_PORT      GPIOB
#define JY901S_SDA_PIN       GPIO_PIN_7

/* I2C 从机地址(7位 0x50, 8位写 0xA0 / 读 0xA1) */
#define JY901S_I2C_ADDR      0x50
/* 角度寄存器地址(0x3D 为官方 I2C 角度寄存器; 若仍全 0 再试 0x53) */
#define JY901S_REG_ANGLE     0x3D

/* 角度分辨率: 原始值 / 32768 * 180 (度) */
#define JY901S_ANGLE_SCALE   (180.0f / 32768.0f)

/* 引脚控制宏 */
#define SCL_H()    HAL_GPIO_WritePin(JY901S_SCL_PORT, JY901S_SCL_PIN, GPIO_PIN_SET)
#define SCL_L()    HAL_GPIO_WritePin(JY901S_SCL_PORT, JY901S_SCL_PIN, GPIO_PIN_RESET)
#define SDA_H()    HAL_GPIO_WritePin(JY901S_SDA_PORT, JY901S_SDA_PIN, GPIO_PIN_SET)
#define SDA_L()    HAL_GPIO_WritePin(JY901S_SDA_PORT, JY901S_SDA_PIN, GPIO_PIN_RESET)
#define SDA_READ() HAL_GPIO_ReadPin(JY901S_SDA_PORT, JY901S_SDA_PIN)

/* SDA 方向切换: 推挽输出(写) / 上拉输入(读), 无需外部上拉电阻 */
/* PB7 对应 GPIOB->CRL 的 bit[31:28] */
#define SDA_OUT()  MODIFY_REG(GPIOB->CRL, 0xF0000000U, 0x30000000U)  /* 推挽输出 50MHz */
#define SDA_IN()   MODIFY_REG(GPIOB->CRL, 0xF0000000U, 0x80000000U)  /* 上拉输入 */

/* 最新角度(度) */
static float   s_roll    = 0.0f;
static float   s_pitch   = 0.0f;
static float   s_yaw     = 0.0f;
static uint8_t s_updated = 0;

/* 微秒级近似延时(72MHz 主频) */
/* 注意: 原 us*6 在 72MHz 下偏快(实际每 us 约 0.4us), 会把 SCL 拉到 600kHz 以上,
   超过 JY901S 的 I2C 规范(≤400kHz), 读 0x3D 角度寄存器时容易 NACK 或读到全 0,
   表现为手持端收不到有效角度、机械臂不跟随。加大系数把 SCL 降到 ~300kHz 以内。 */
static void I2C_DelayUs(uint32_t us)
{
    volatile uint32_t cnt;
    for (cnt = 0; cnt < us * 12u; cnt++)
    {
        __NOP();
    }
}

/* I2C 起始条件: SCL 高时 SDA 由高变低 */
static void I2C_Start(void)
{
    SDA_OUT();
    SDA_H();
    I2C_DelayUs(2);
    SCL_H();
    I2C_DelayUs(2);
    SDA_L();
    I2C_DelayUs(2);
    SCL_L();
    I2C_DelayUs(1);
}

/* I2C 停止条件: SCL 高时 SDA 由低变高 */
static void I2C_Stop(void)
{
    SDA_OUT();
    SCL_L();
    I2C_DelayUs(1);
    SDA_L();
    I2C_DelayUs(1);
    SCL_H();
    I2C_DelayUs(2);
    SDA_H();
    I2C_DelayUs(2);
}

/* 写一个字节, 返回从机应答(0=ACK, 1=NACK) */
static uint8_t I2C_WriteByte(uint8_t dat)
{
    uint8_t i, ack;

    SDA_OUT();
    for (i = 0; i < 8; i++)
    {
        if (dat & 0x80)
        {
            SDA_H();
        }
        else
        {
            SDA_L();
        }
        dat <<= 1;
        I2C_DelayUs(1);
        SCL_H();
        I2C_DelayUs(2);
        SCL_L();
        I2C_DelayUs(1);
    }

    /* 切换 SDA 为输入, 读取从机 ACK */
    SDA_IN();
    I2C_DelayUs(1);
    SCL_H();
    I2C_DelayUs(2);
    ack = SDA_READ();
    SCL_L();
    I2C_DelayUs(1);
    SDA_OUT();

    return ack;
}

/* 读一个字节, ack=0 表示读完后发送 ACK, ack=1 发送 NACK(最后一字节) */
static uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t i, dat = 0;

    SDA_IN();
    for (i = 0; i < 8; i++)
    {
        dat <<= 1;
        SCL_H();
        I2C_DelayUs(2);
        if (SDA_READ())
        {
            dat |= 0x01;
        }
        SCL_L();
        I2C_DelayUs(1);
    }

    /* 主机发送 ACK/NACK */
    SDA_OUT();
    if (ack)
    {
        SDA_H();    /* NACK */
    }
    else
    {
        SDA_L();    /* ACK */
    }
    I2C_DelayUs(1);
    SCL_H();
    I2C_DelayUs(2);
    SCL_L();
    I2C_DelayUs(1);
    SDA_H();

    return dat;
}

/* 初始化: PB6=SCL 推挽输出, PB7=SDA 推挽输出(读时动态切输入, 无需外部上拉) */
void Jy901s_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin   = JY901S_SCL_PIN | JY901S_SDA_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;   /* 推挽输出 */
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    SCL_H();
    SDA_H();

    s_updated = 0;
    s_roll    = 0.0f;
    s_pitch   = 0.0f;
    s_yaw     = 0.0f;
}

/* 读取角度寄存器 0x3D(8 字节), 成功返回 1, 失败返回 0 */
static uint8_t Jy901s_ReadAngle(uint8_t *buf)
{
    uint8_t i;

    I2C_Start();
    if (I2C_WriteByte((uint8_t)(JY901S_I2C_ADDR << 1)))       /* 从机地址+写 */
    {
        I2C_Stop();
        return 0;
    }
    if (I2C_WriteByte(JY901S_REG_ANGLE))                       /* 寄存器地址 */
    {
        I2C_Stop();
        return 0;
    }

    I2C_Start();                                               /* 重复起始 */
    if (I2C_WriteByte((uint8_t)((JY901S_I2C_ADDR << 1) | 1))) /* 从机地址+读 */
    {
        I2C_Stop();
        return 0;
    }

    for (i = 0; i < 8; i++)
    {
        buf[i] = I2C_ReadByte((i == 7) ? 1 : 0);   /* 最后一字节 NACK */
    }

    I2C_Stop();
    return 1;
}

/* 轮询读取角度(主循环中反复调用) */
void Jy901s_Poll(void)
{
    uint8_t buf[8];
    int16_t roll, pitch, yaw;

    if (!Jy901s_ReadAngle(buf))
    {
        return;
    }

    roll  = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    pitch = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    yaw   = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));

    s_roll  = (float)roll  * JY901S_ANGLE_SCALE;
    s_pitch = (float)pitch * JY901S_ANGLE_SCALE;
    s_yaw   = (float)yaw   * JY901S_ANGLE_SCALE;
    s_updated = 1;
}

/* 是否有新的有效角度 */
uint8_t Jy901s_IsAngleUpdated(void)
{
    return s_updated;
}

/* 读取最新角度(度), 并清除更新标志 */
void Jy901s_GetAngle(float *pRoll, float *pPitch, float *pYaw)
{
    *pRoll  = s_roll;
    *pPitch = s_pitch;
    *pYaw   = s_yaw;
    s_updated = 0;
}
