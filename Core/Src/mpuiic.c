/*
 * @Date         : 2022-04-01 19:37:31
 * @LastEditors  : liu wei
 * @LastEditTime : 2022-04-01 21:51:31
 * @brief        : Do not edit
 * @FilePath     : \LED\Core\Src\mpuiic.c
 * @Github       : https://github.com/Blackerrr
 * @Coding       : utf-8
 */

#include "mpuiic.h"
/**
 * @description: MPU IIC delay function
 * @param {cnt}  延时的us数
 * @return {*}
 */
void MPU_IIC_Delay(uint16_t cnt)
{
    delay_us(cnt);
}

// static void IIC_SDA_OUT(void)
// {

//     GPIOC->CRL &= 0XFFFFFF0F;
//     GPIOC->CRL |= 3 << 4;

//     // GPIO_InitTypeDef GPIO_Initure;
//     // GPIO_Initure.Pin = GPIO_PIN_1;
//     // GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
//     // GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
//     // HAL_GPIO_Init(GPIOC, &GPIO_Initure);
// }

// static void IIC_SDA_IN(void)
// {
//     // 使用寄存器操作比较迅速
//     GPIOC->CRL &= 0XFFFFFF0F;
//     GPIOC->CRL |= 8 << 4;


//     // 使用下面的代码切换的太慢了，虽然能读出数据，但是数据不对。一直在跳变
//     // GPIO_InitTypeDef GPIO_Initure;
//     // GPIO_Initure.Pin = GPIO_PIN_1;
//     // GPIO_Initure.Mode = GPIO_MODE_INPUT;
//     // GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
//     // HAL_GPIO_Init(GPIOC, &GPIO_Initure);
// }
/**
 * @description:
 * @param {*}
 * @return {*}
 */
void MPU_IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_Initure.Pin = GPIO_PIN_0 | GPIO_PIN_1;

#if USING_PP
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull = GPIO_PULLUP;
#else
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_OD;    // 开漏输出模式
#endif
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOC, &GPIO_Initure);

    MPU_IIC_SCL = 1;
    MPU_IIC_SDA = 1;    // 总线空闲状态
}

/**
 * @description:  检测I2C总线设备，CPU向发送设备地址，然后读取设备应答来判断该设备是否存在
 * @param {u8} addr 设备的I2C总线地址
 * @return {*}
 */
u8 MPU_IIC_Check_Device(u8 addr)
{
    u8 res;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 0);   // 发送设备地址+写指令
    res = MPU_IIC_Wait_Ack();
    MPU_IIC_Stop();
    return res;
}


/**
 * @description:  Send IIC start signal
 * @param {*}
 * @return {*}
 */
void MPU_IIC_Start(void)
{
#if USING_PP
    IIC_SDA_OUT();
#endif
    MPU_IIC_SDA = 1;
    MPU_IIC_SCL = 1;
    MPU_IIC_Delay(2);
    MPU_IIC_SDA = 0;    // START:when CLK is high,DATA change form high to low
    MPU_IIC_Delay(2);
    MPU_IIC_SCL = 0;    // Clamp the I2C bus, ready to send or receive data
    // MPU_IIC_Delay(2);

}

/**
 * @description: send iic stop signal
 * @param {*}
 * @return {*}
 */
void MPU_IIC_Stop(void)
{
#if USING_PP
    IIC_SDA_OUT();
#endif
    MPU_IIC_SCL = 0;
    MPU_IIC_SDA = 0;  //STOP:when CLK is high DATA change form low to high
    MPU_IIC_Delay(2);
    MPU_IIC_SCL = 1;
    // MPU_IIC_Delay(2);
    MPU_IIC_SDA = 1;  // Send I2C bus end signal
    MPU_IIC_Delay(2);
}

// Wait for the answer signal to arrive
// Return value: 1, fail
//               0, success
/**
 * @description: 等待应答信号到来， Return value: 1, fail，0, success
 * @param {*}
 * @return {*} Return value: 1, fail，0, success
 */
u8 MPU_IIC_Wait_Ack(void)
{
    u8 ucErrTime = 0;
#if USING_PP
    IIC_SDA_IN();
#endif
    MPU_IIC_SDA = 1;
    MPU_IIC_Delay(2);
    MPU_IIC_SCL = 1;
    MPU_IIC_Delay(2);
    while (MPU_READ_SDA)   // SDA 高表示非应答
    {
        ucErrTime++;
        if (ucErrTime > 250)
        {
            MPU_IIC_Stop();
            return 1;
        }
    }
    MPU_IIC_SCL = 0; // clock out 0
    return 0;
}

/**
 * @description: generate ACK response
 * @param {*}
 * @return {*}
 */
void MPU_IIC_Ack(void)
{
#if USING_PP
    IIC_SDA_OUT();
#endif
    MPU_IIC_SDA = 0;    // master控制SDA为低表示应答
    MPU_IIC_SCL = 0;
    MPU_IIC_Delay(2);
    MPU_IIC_SCL = 1;    // master产生一个时钟
    MPU_IIC_Delay(2);
    MPU_IIC_SCL = 0;
    MPU_IIC_SDA = 1;   // master释放SDA总线
}

/**
 * @description: No ACK response is generated
 * @param {*}
 * @return {*}
 */
void MPU_IIC_NAck(void)
{
#if USING_PP
    IIC_SDA_OUT();
#endif
    MPU_IIC_SDA = 1;   // master控制SDA为高表示应答
    MPU_IIC_SCL = 0;
    MPU_IIC_Delay(2);
    MPU_IIC_SCL = 1;
    MPU_IIC_Delay(2);
    MPU_IIC_SCL = 0;
}

/**
 * @description: IIC发送一个字节
 * @param {u8} txd
 * @return {*}
 */
void MPU_IIC_Send_Byte(u8 txd)
{
#if USING_PP
    IIC_SDA_OUT();
#endif
    u8 t;
    MPU_IIC_SCL = 0;   // Pull the clock low to start data transfer
    for (t = 0; t < 8; t++)
    {
        MPU_IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        MPU_IIC_SCL = 1;
        MPU_IIC_Delay(2);
        MPU_IIC_SCL = 0;
        MPU_IIC_Delay(2);
        // if (t == 7)
            // MPU_IIC_SDA = 1;   // 释放总线
    }
}

/**
 * @description: Read 1 byte, when ack=1, send ACK, when ack=0, send nACK
 * @param {*}
 * @return {*}
 */
u8 MPU_IIC_Read_Byte(unsigned char ack)
{
#if USING_PP
    IIC_SDA_IN();
#endif
    u8 i, receive = 0;
    for (i = 0; i < 8; i++)
    {
        MPU_IIC_SCL = 0;
        MPU_IIC_Delay(2);
        MPU_IIC_SCL = 1;
        receive <<= 1;
        if (MPU_READ_SDA)
            receive++;
        MPU_IIC_Delay(2);
    }
    if (!ack)
        MPU_IIC_NAck(); // No ACK
    else
        MPU_IIC_Ack(); // ACK
    return receive;
}
