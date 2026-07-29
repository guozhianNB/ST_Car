#include "i2c_soft.h"
#include "main.h"

static GPIO_TypeDef* SCL_Port;
static uint16_t SCL_Pin;
static GPIO_TypeDef* SDA_Port;
static uint16_t SDA_Pin;

// crude delay for bit-banging (tunable)
static void I2C_Delay(void)
{
    volatile int i = 120; // slower and safer for weak pull-ups
    while (i--) __NOP();
}

static void SDA_Release(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SDA_Port, &GPIO_InitStruct);
}

static void SDA_Low(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SDA_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SDA_Port, SDA_Pin, GPIO_PIN_RESET);
}

static void SCL_Release(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SCL_Port, &GPIO_InitStruct);
}

static void SCL_Low(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SCL_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_RESET);
}

void I2C_Soft_Init(GPIO_TypeDef* sclPort, uint16_t sclPin, GPIO_TypeDef* sdaPort, uint16_t sdaPin)
{
    SCL_Port = sclPort;
    SCL_Pin = sclPin;
    SDA_Port = sdaPort;
    SDA_Pin = sdaPin;

    if (SCL_Port == GPIOA || SDA_Port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    if (SCL_Port == GPIOB || SDA_Port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    if (SCL_Port == GPIOC || SDA_Port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    if (SCL_Port == GPIOD || SDA_Port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    if (SCL_Port == GPIOE || SDA_Port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();

    SCL_Release();
    SDA_Release();

    HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SDA_Port, SDA_Pin, GPIO_PIN_SET);
    I2C_Delay();
}

static void I2C_Start(void)
{
    SDA_Release();
    SCL_Release();
    HAL_GPIO_WritePin(SDA_Port, SDA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_SET);
    I2C_Delay();
    SDA_Low();
    I2C_Delay();
    SCL_Low();
    I2C_Delay();
}

static void I2C_Stop(void)
{
    SDA_Low();
    I2C_Delay();
    SCL_Release();
    HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_SET);
    I2C_Delay();
    SDA_Release();
    HAL_GPIO_WritePin(SDA_Port, SDA_Pin, GPIO_PIN_SET);
    I2C_Delay();
}

static uint8_t I2C_WriteBit(uint8_t bit)
{
    if (bit)
        SDA_Release();
    else
        SDA_Low();

    I2C_Delay();
    SCL_Release();
    HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_SET);
    I2C_Delay();
    SCL_Low();
    I2C_Delay();
    return 0;
}

static uint8_t I2C_ReadBit(void)
{
    uint8_t bit;
    SDA_Release();
    I2C_Delay();
    SCL_Release();
    HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_SET);
    I2C_Delay();
    bit = (uint8_t)(HAL_GPIO_ReadPin(SDA_Port, SDA_Pin) != GPIO_PIN_RESET);
    SCL_Low();
    I2C_Delay();
    return bit;
}

static uint8_t I2C_WriteByte(uint8_t data)
{
    for (int i = 0; i < 8; i++)
    {
        I2C_WriteBit((data & 0x80) != 0);
        data <<= 1;
    }
    // read ack
    SDA_Release();
    I2C_Delay();
    SCL_Release();
    HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_SET);
    I2C_Delay();
    uint8_t ack = (HAL_GPIO_ReadPin(SDA_Port, SDA_Pin) == GPIO_PIN_RESET) ? 0 : 1;
    SCL_Low();
    I2C_Delay();
    return ack; // 0 means ACK
}

static uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t data = 0;
    for (int i = 0; i < 8; i++)
    {
        data <<= 1;
        data |= I2C_ReadBit();
    }
    // send ack/nack
    if (ack == 0)
    {
        SDA_Low();
    }
    else
    {
        SDA_Release();
    }

    I2C_Delay();
    SCL_Release();
    HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_SET);
    I2C_Delay();
    SCL_Low();
    I2C_Delay();
    return data;
}

uint8_t I2C_Soft_WriteBytes(uint8_t devAddr, uint8_t regAddr, uint8_t* data, uint16_t len)
{
    I2C_Start();
    if (I2C_WriteByte((devAddr << 1) | 0x00) != 0) { I2C_Stop(); return 1; }
    if (I2C_WriteByte(regAddr) != 0) { I2C_Stop(); return 1; }
    for (uint16_t i = 0; i < len; i++)
    {
        if (I2C_WriteByte(data[i]) != 0) { I2C_Stop(); return 1; }
    }
    I2C_Stop();
    return 0;
}

uint8_t I2C_Soft_ReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t* buf, uint16_t len)
{
    I2C_Start();
    if (I2C_WriteByte((devAddr << 1) | 0x00) != 0) { I2C_Stop(); return 1; }
    if (I2C_WriteByte(regAddr) != 0) { I2C_Stop(); return 1; }

    // repeated start
    I2C_Start();
    if (I2C_WriteByte((devAddr << 1) | 0x01) != 0) { I2C_Stop(); return 1; }

    for (uint16_t i = 0; i < len; i++)
    {
        if (i == (len - 1))
            buf[i] = I2C_ReadByte(1); // NACK on last byte
        else
            buf[i] = I2C_ReadByte(0); // ACK
    }
    I2C_Stop();
    return 0;
}
