/*
 * @Date         : 2022-01-19 16:28:47
 * @LastEditors  : liu wei
 * @LastEditTime : 2022-02-22 22:26:48
 * @FilePath     : \LED\Core\Src\mpu6050.c
 * @Github       : https://github.com/Blackerrr
 * @Coding       : utf-8
 */

#include "mpu6050.h"

Magnetometer mag;
MPU6050_InitDefine mpu6050 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//MPU6050_Original mpu6050_original = {0, 0, 0, 0, 0, 0, 295, -201, -116, 0, 0, 0};
MPU6050_Original mpu6050_original = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;  //
float exInt = 0, eyInt = 0, ezInt = 0; //
unsigned int a_LSB = 16384;            // acceleration least significant bit
float g_LSB = 32.8;                    // Gyro least significant bit

/**
 * @description: MPU IIC delay function
 * @param {*}
 * @return {*}
 */
void MPU_IIC_Delay(void)
{
    delay_us(2);
}

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
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOC, &GPIO_Initure);

    MPU_IIC_SCL = 1;
    MPU_IIC_SDA = 1;
}


/**
 * @description:  Send IIC start signal
 * @param {*}
 * @return {*}
 */
void MPU_IIC_Start(void)
{
    MPU_SDA_OUT();
    MPU_IIC_SDA = 1;
    MPU_IIC_SCL = 1;
    MPU_IIC_Delay();
    MPU_IIC_SDA = 0;    // START:when CLK is high,DATA change form high to low
    MPU_IIC_Delay();
    MPU_IIC_SCL = 0;    // Clamp the I2C bus, ready to send or receive data
}

/**
 * @description: send iic stop signal
 * @param {*}
 * @return {*}
 */
void MPU_IIC_Stop(void)
{
    MPU_SDA_OUT();
    MPU_IIC_SCL = 0;
    MPU_IIC_SDA = 0;  //STOP:when CLK is high DATA change form low to high
    MPU_IIC_Delay();
    MPU_IIC_SCL = 1;
    MPU_IIC_SDA = 1;  // Send I2C bus end signal
    MPU_IIC_Delay();
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
    MPU_SDA_IN();
    MPU_IIC_SDA = 1;
    MPU_IIC_Delay();
    MPU_IIC_SCL = 1;
    MPU_IIC_Delay();
    while (MPU_READ_SDA)
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
    MPU_IIC_SCL = 0;
    MPU_SDA_OUT();
    MPU_IIC_SDA = 0;
    MPU_IIC_Delay();
    MPU_IIC_SCL = 1;
    MPU_IIC_Delay();
    MPU_IIC_SCL = 0;
}

/**
 * @description: No ACK response is generated
 * @param {*}
 * @return {*}
 */
void MPU_IIC_NAck(void)
{
    MPU_IIC_SCL = 0;
    MPU_SDA_OUT();
    MPU_IIC_SDA = 1;
    MPU_IIC_Delay();
    MPU_IIC_SCL = 1;
    MPU_IIC_Delay();
    MPU_IIC_SCL = 0;
}

/**
 * @description: IIC发送一个字节，返回从机有无应答，1，有应答，0，无应答
 * @param {u8} txd
 * @return {*}
 */
void MPU_IIC_Send_Byte(u8 txd)
{
    u8 t;
    MPU_SDA_OUT();
    MPU_IIC_SCL = 0;   // Pull the clock low to start data transfer
    for (t = 0; t < 8; t++)
    {
        MPU_IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        MPU_IIC_SCL = 1;
        MPU_IIC_Delay();
        MPU_IIC_SCL = 0;
        MPU_IIC_Delay();
    }
}

/**
 * @description: Read 1 byte, when ack=1, send ACK, when ack=0, send nACK
 * @param {*}
 * @return {*}
 */
u8 MPU_IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    MPU_SDA_IN();
    for (i = 0; i < 8; i++)
    {
        MPU_IIC_SCL = 0;
        MPU_IIC_Delay();
        MPU_IIC_SCL = 1;
        receive <<= 1;
        if (MPU_READ_SDA)
            receive++;
        MPU_IIC_Delay();
    }
    if (!ack)
        MPU_IIC_NAck(); // No ACK
    else
        MPU_IIC_Ack(); // ACK
    return receive;
}

/**
 * @description: initialize mpu6050,  return value: 0, success, other, error code
 * @param {*}
 * @return {*}
 */
u8 MPU_Init(void)
{
    // u8 res;

    // GPIO_InitTypeDef GPIO_Initure;

    // __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable GPIOA clock

    // GPIO_Initure.Pin = GPIO_PIN_15;            //PA15
    // GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;   // Push-pull output
    // GPIO_Initure.Pull = GPIO_PULLUP;           // pull up
    // GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH; // high speed
    // HAL_GPIO_Init(GPIOA, &GPIO_Initure);

    // __HAL_AFIO_REMAP_SWJ_DISABLE();
    // // JTAG is prohibited, so that PA15 can be used as normal IO, otherwise PA15 cannot be used as normal IO!!!
    // __HAL_AFIO_REMAP_SWJ_DISABLE();

    // MPU_AD0_CTRL = 0; // AD0 pin is low level, the slave address is: 0X68

    MPU_IIC_Init();                          // initialize mpuiic
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); // reset mpu6050
    HAL_Delay(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); // wake up MPU6050
    MPU_Set_Gyro_Fsr(3);                     // Gyro sensor, ±2000dps
    MPU_Set_Accel_Fsr(0);                    // Accelerometer, ±2g
    MPU_Set_Rate(50);                        // Set the sample rate to 50Hz
    MPU_Write_Byte(MPU_INT_EN_REG, 0X00);    // turn off all interrupts
    MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); // I2C master mode off
    MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);   // turn off FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X82); // INT pin is active low and enable bypass mode

    // res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
    // if (res == MPU_ADDR) // if Device ID is correct
    // {
    //     MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01);	//Set CLKSEL and PLL, X axis as reference
    //     MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00);	// Both accelerometer and gyroscope work
    //     MPU_Set_Rate(50);						// Set the sample rate to 50Hz
    // }
    //    else return 1;
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01);	//Set CLKSEL and PLL, X axis as reference
    MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00);	// Both accelerometer and gyroscope work
    MPU_Set_Rate(50);					    	// Set the sample rate to 50Hz

    /***********************AK8963 magnetometer Sensor Initialization*********************/

#if USING_MAGNETOMETER
    AK8963_Write_Byte(AK8963_Control_1, 0x00);  // Power-down mode
    HAL_Delay(100);
    AK8963_Write_Byte(AK8963_Control_1, 0x11);  //  16-bit output and single  measurement mode 1
    HAL_Delay(100);
    mag.AK8963_ID = AK8963_Read_Byte(AK8963_WIA);
#endif
    return 0;
}


/**
 * @description: set MPU6050 Gyro Sensor Full Scale Range,
 *               fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
 * @param {u8} fsr
 * @return {u8} 0, set successfully; other, Setup failed
 */
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3);
}

// Set the full-scale range of the MPU6050 accelerometer
// fsr:0,±2g;1,±4g;2,±8g;3,±16g
// return value: 0, set successfully
//           other, Setup failed
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3);
}

// Setting up the digital low-pass filter of the MPU6050
// lpf: digital low-pass filter frequency (Hz)
// return value: 0, set successfully
//           other, Setup failed
u8 MPU_Set_LPF(u16 lpf)
{
    u8 data = 0;
    if (lpf >= 188)
        data = 1;
    else if (lpf >= 98)
        data = 2;
    else if (lpf >= 42)
        data = 3;
    else if (lpf >= 20)
        data = 4;
    else if (lpf >= 10)
        data = 5;
    else
        data = 6;
    return MPU_Write_Byte(MPU_CFG_REG, data);
}

/**
 * @description: Set the sampling rate of MPU6050 (assume Fs=1KHz)
 * @param {u16} rate 4~1000(Hz)
 * @return {u8} 0, set successfully, other, Setup failed
 */
u8 MPU_Set_Rate(u16 rate)
{
    u8 data;
    if (rate > 1000)
        rate = 1000;
    if (rate < 4)
        rate = 4;
    data = 1000 / rate - 1;
    data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data); // set digital low-pass filter
    return MPU_Set_LPF(rate / 2);                     // Automatically set LPF to half the sample rate
}

/**
 * @description: get temperature value
 * @param {*}
 * @return {short} temperature value (expanded 100 times)
 */
short MPU_Get_Temperature(void)
{
    u8 buf[2];
    short raw;
    float temp;
    MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    raw = ((u16)buf[0] << 8) | buf[1];
    temp = 36.53 + ((double)raw) / 340;
    return temp * 100;
    ;
}

/**
 * @description: get gyroscope value (original value)
 *               gx,gy,gz: original readings of gyroscope x, y, z axes (with sign)
 * @param {*}
 * @return {u8} 0, success,  other, error code
 */
u8 MPU_Get_Gyroscope()
{
    u8 buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        mpu6050_original.gx = ((u16)buf[0] << 8) | buf[1];
        mpu6050_original.gy = ((u16)buf[2] << 8) | buf[3];
        mpu6050_original.gz = ((u16)buf[4] << 8) | buf[5];
    }
    return res;
}

/**
 * @description: get acceleration value (raw value)
 *               ax, ay, az: readings of accelerometer x, y, z axes (with sign)
 * @param {*}
 * @return {u8}  0, success, other, error code
 */
u8 MPU_Get_Accelerometer()
{
    u8 buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        mpu6050_original.ax = ((u16)buf[0] << 8) | buf[1];
        mpu6050_original.ay = ((u16)buf[2] << 8) | buf[3];
        mpu6050_original.az = ((u16)buf[4] << 8) | buf[5];
    }
    return res;
}

// IIC continuous write
// addr: device address
// reg: register address
// len: length of write
// buf: data area
// return value: 0, normal
// other, error code
/**
 * @description: IIC continuous write
 * @param {u8} addr device address
 * @param {u8} reg  register address
 * @param {u8} len  length of write
 * @param {u8} *buf data area
 * @return {u8} 0, normal, other, error code
 */
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    u8 i;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 0); // Send device address + write command
    if (MPU_IIC_Wait_Ack())             // wait ack
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg); // register address to write
    MPU_IIC_Wait_Ack();     // wait ack
    for (i = 0; i < len; i++)
    {
        MPU_IIC_Send_Byte(buf[i]); // send data
        if (MPU_IIC_Wait_Ack())    // wait ack
        {
            MPU_IIC_Stop();
            return 1;
        }
    }
    MPU_IIC_Stop();
    return 0;
}

/**
 * @description: IIC continuous read
 * @param {u8} addr  device address
 * @param {u8} reg   register address to read
 * @param {u8} len   length to read
 * @param {u8} *buf  read data storage area
 * @return {u8} 0, normal,  other, error code
 */
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 0); // send device address + order to wirite
    if (MPU_IIC_Wait_Ack())             // wait ack
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg); // register address to write
    MPU_IIC_Wait_Ack();     // wait ack
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 1); // send device address + order to read
    MPU_IIC_Wait_Ack();                 // wait ack
    while (len)
    {
        if (len == 1)
            *buf = MPU_IIC_Read_Byte(0); // read data and secd nACK
        else
            *buf = MPU_IIC_Read_Byte(1); // read data and send ack
        len--;
        buf++;
    }
    MPU_IIC_Stop(); // generate a stop condition
    return 0;
}

//IIC writes a byte
//reg: register address
//data: data
//return value: 0, normal
//          other, error code
/**
 * @description: IIC writes a byte
 * @param {u8} reg  register address
 * @param {u8} data 要写入的数据
 * @return {u8}     0, normal,  other, error codes
 */
u8 MPU_Write_Byte(u8 reg, u8 data)
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR << 1) | 0); // send device address + write command
    if (MPU_IIC_Wait_Ack())                 // wait response
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg);  // register address to write
    MPU_IIC_Wait_Ack();      // wait response
    MPU_IIC_Send_Byte(data); // send data
    if (MPU_IIC_Wait_Ack())  // wait response
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Stop();
    return 0;
}

//IIC reads a byte
//reg: register address
//return value: read data
/**
 * @description:  IIC reads a byte
 * @param {u8} reg  register address
 * @return {u8}     read data
 */
u8 MPU_Read_Byte(u8 reg)
{
    u8 res;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR << 1) | 0); // send device address + write command
    MPU_IIC_Wait_Ack();                     // wait response
    MPU_IIC_Send_Byte(reg);                 // register address to write
    MPU_IIC_Wait_Ack();                     // wait response
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR << 1) | 1); // send device address + read command
    MPU_IIC_Wait_Ack();                     // wait response
    res = MPU_IIC_Read_Byte(0);             // read data and send ack
    MPU_IIC_Stop();                         // generate a stop condition
    return res;
}

// fast square root
// static float invSqrt(float x)
// {
//     float halfx = 0.5f * x;
//     float y = x;
//     long i = *(long *)&y;
//     i = 0x5f3759df - (i >> 1);
//     y = *(float *)&i;
//     y = y * (1.5f - (halfx * y * y));
//     return y;
// }

float dt_roll = 0.02;  // Note: The value of dt is the sampling time of the kalman filter
float angle_roll, angle_dot_roll; // angle and Angular velocity
float P_roll[2][2] =
{
    { 1, 0 },
    { 0, 1 }
};
float Pdot_roll[4] = { 0, 0, 0, 0};
float Q_angle_roll = 0.001, Q_gyro_roll = 0.005; // Confidence of angle data, Confidence of angular velocity data
float R_angle_roll = 0.5, C_0_roll = 1;
float q_bias_roll, angle_err_roll, PCt_0_roll, PCt_1_roll, E_roll, K_0_roll, K_1_roll, t_0_roll, t_1_roll;


/**
 * @description:  kalman filter
 * @param {*}
 * @return {float} 解算出来的角度
 */
float Kalman_Filter_Roll(void)
{
    float angle_m = atan2(mpu6050.Ay, mpu6050.Az) * 180 / 3.14;
    float gyro_m =  mpu6050.Gx;
    angle_roll += (gyro_m - q_bias_roll) * dt_roll;
    angle_err_roll = angle_m - angle_roll;
    Pdot_roll[0] = Q_angle_roll - P_roll[0][1] - P_roll[1][0];
    Pdot_roll[1] = - P_roll[1][1];
    Pdot_roll[2] = - P_roll[1][1];
    Pdot_roll[3] = Q_gyro_roll;
    P_roll[0][0] += Pdot_roll[0] * dt_roll;
    P_roll[0][1] += Pdot_roll[1] * dt_roll;
    P_roll[1][0] += Pdot_roll[2] * dt_roll;
    P_roll[1][1] += Pdot_roll[3] * dt_roll;
    PCt_0_roll = C_0_roll * P_roll[0][0];
    PCt_1_roll = C_0_roll * P_roll[1][0];
    E_roll = R_angle_roll + C_0_roll * PCt_0_roll;
    K_0_roll = PCt_0_roll / E_roll;
    K_1_roll = PCt_1_roll / E_roll;
    t_0_roll = PCt_0_roll;
    t_1_roll = C_0_roll * P_roll[0][1];
    P_roll[0][0] -= K_0_roll * t_0_roll;
    P_roll[0][1] -= K_0_roll * t_1_roll;
    P_roll[1][0] -= K_1_roll * t_0_roll;
    P_roll[1][1] -= K_1_roll * t_1_roll;
    angle_roll += K_0_roll * angle_err_roll; // optimal angle
    q_bias_roll += K_1_roll * angle_err_roll;
    angle_dot_roll = gyro_m - q_bias_roll; // optimal angular velocity
    return angle_roll;
}

float dt_pitch = 0.0035; // Note: The value of dt is the sampling time of the kalman filter
float angle_pitch, angle_dot_pitch;   // angle and angular velocity
float P_pitch[2][2] = {{ 1, 0 },
    { 0, 1 }
};
float Pdot_pitch[4] = { 0, 0, 0, 0};
float Q_angle_pitch = 3, Q_gyro_pitch = 1;  // confidence of angle, confidence of angular velocity
float R_angle_pitch = 0.03, C_0_pitch = 1;
float q_bias_pitch, angle_err_pitch, PCt_0_pitch, PCt_1_pitch, E_pitch, K_0_pitch, K_1_pitch, t_0_pitch, t_1_pitch;

/**
 * @description:  kalman filter
 * @param {*}
 * @return {float} 解算出来的角度
 */
float Kalman_Filter_Pitch(void)
{
    float angle_m = atan2(mpu6050.Ax, mpu6050.Az) * -180 / 3.14;
    float gyro_m =  mpu6050.Gy;
    angle_pitch += (gyro_m - q_bias_pitch) * dt_pitch;
    angle_err_pitch = angle_m - angle_pitch;
    Pdot_pitch[0] = Q_angle_pitch - P_pitch[0][1] - P_pitch[1][0];
    Pdot_pitch[1] = - P_pitch[1][1];
    Pdot_pitch[2] = - P_pitch[1][1];
    Pdot_pitch[3] = Q_gyro_pitch;
    P_pitch[0][0] += Pdot_pitch[0] * dt_pitch;
    P_pitch[0][1] += Pdot_pitch[1] * dt_pitch;
    P_pitch[1][0] += Pdot_pitch[2] * dt_pitch;
    P_pitch[1][1] += Pdot_pitch[3] * dt_pitch;
    PCt_0_pitch = C_0_pitch * P_pitch[0][0];
    PCt_1_pitch = C_0_pitch * P_pitch[1][0];
    E_pitch = R_angle_pitch + C_0_pitch * PCt_0_pitch;
    K_0_pitch = PCt_0_pitch / E_pitch;
    K_1_pitch = PCt_1_pitch / E_pitch;
    t_0_pitch = PCt_0_pitch;
    t_1_pitch = C_0_pitch * P_pitch[0][1];
    P_pitch[0][0] -= K_0_pitch * t_0_pitch;
    P_pitch[0][1] -= K_0_pitch * t_1_pitch;
    P_pitch[1][0] -= K_1_pitch * t_0_pitch;
    P_pitch[1][1] -= K_1_pitch * t_1_pitch;
    angle_pitch += K_0_pitch * angle_err_pitch;  // optimal angle
    q_bias_pitch += K_1_pitch * angle_err_pitch;
    angle_dot_pitch = gyro_m - q_bias_pitch;    // optimal angular velocity
    return angle_pitch;
}

float dt_yaw = 0.0035; //Note：the value of dt is the sampling time of filter
float angle_yaw, angle_dot_yaw;  // angle and angular velocity
float P_yaw[2][2] = {{ 1, 0 },
    { 0, 1 }
};
float Pdot_yaw[4] = { 0, 0, 0, 0};
float Q_angle_yaw = 5, Q_gyro_yaw = 3; // confidence of angle, confidence of angular velocity
float R_angle_yaw = 0.03, C_0_yaw = 1;
float q_bias_yaw, angle_err_yaw, PCt_0_yaw, PCt_1_yaw, E_yaw, K_0_yaw, K_1_yaw, t_0_yaw, t_1_yaw;

/**
 * @description:  kalman filter 航向角
 * @param {*}
 * @return {float} 解算出来的角度
 */
float Kalman_Filter_Yaw(void)
{
    float angle_m = atan2(mpu6050.Ax, mpu6050.Ay) * 180 / 3.14;
    float gyro_m =  mpu6050.Gz;
    angle_yaw += (gyro_m - q_bias_yaw) * dt_yaw;
    angle_err_yaw = angle_m - angle_yaw;
    Pdot_yaw[0] = Q_angle_yaw - P_yaw[0][1] - P_yaw[1][0];
    Pdot_yaw[1] = - P_yaw[1][1];
    Pdot_yaw[2] = - P_yaw[1][1];
    Pdot_yaw[3] = Q_gyro_yaw;
    P_yaw[0][0] += Pdot_yaw[0] * dt_yaw;
    P_yaw[0][1] += Pdot_yaw[1] * dt_yaw;
    P_yaw[1][0] += Pdot_yaw[2] * dt_yaw;
    P_yaw[1][1] += Pdot_yaw[3] * dt_yaw;
    PCt_0_yaw = C_0_yaw * P_yaw[0][0];
    PCt_1_yaw = C_0_yaw * P_yaw[1][0];
    E_yaw = R_angle_yaw + C_0_yaw * PCt_0_yaw;
    K_0_yaw = PCt_0_yaw / E_yaw;
    K_1_yaw = PCt_1_yaw / E_yaw;
    t_0_yaw = PCt_0_yaw;
    t_1_yaw = C_0_yaw * P_yaw[0][1];
    P_yaw[0][0] -= K_0_yaw * t_0_yaw;
    P_yaw[0][1] -= K_0_yaw * t_1_yaw;
    P_yaw[1][0] -= K_1_yaw * t_0_yaw;
    P_yaw[1][1] -= K_1_yaw * t_1_yaw;
    angle_yaw += K_0_yaw * angle_err_yaw; // optimal angle
    q_bias_yaw += K_1_yaw * angle_err_yaw;
    angle_dot_yaw = gyro_m - q_bias_yaw; // optimal angular velocity
    return angle_yaw;
}

// Quaternion solution
// void MPU_Update(void)
// {
//     float Kp = 1.5f;
//     float Ki = 0.1f;
//     float vx, vy, vz;
//     float ex, ey, ez;
//     float norm;

//     float q0q0 = q0 * q0;
//     float q0q1 = q0 * q1;
//     float q0q2 = q0 * q2;
//     float q0q3 = q0 * q3;
//     float q1q1 = q1 * q1;
//     float q1q2 = q1 * q2;
//     float q1q3 = q1 * q3;
//     float q2q2 = q2 * q2;
//     float q2q3 = q2 * q3;
//     float q3q3 = q3 * q3;

//     MPU_Get_Gyroscope();
//     MPU_Get_Accelerometer();

//     mpu6050.Gx = ((double)mpu6050_original.gx / g_LSB) / 57.2957795;
//     mpu6050.Gy = ((double)mpu6050_original.gy / g_LSB) / 57.2957795;
//     mpu6050.Gz = ((double)mpu6050_original.gz / g_LSB) / 57.2957795;

//     mpu6050.Ax = ((double)mpu6050_original.ax / a_LSB) * G;
//     mpu6050.Ay = ((double)mpu6050_original.ay / a_LSB) * G;
//     mpu6050.Az = ((double)mpu6050_original.az / a_LSB) * G;

//     if (mpu6050.Ax * mpu6050.Ay * mpu6050.Az == 0)
//         return;

//
//     norm = invSqrt(mpu6050.Ax * mpu6050.Ax + mpu6050.Ay * mpu6050.Ay + mpu6050.Az * mpu6050.Az);
//     mpu6050.Ax = mpu6050.Ax * norm;
//     mpu6050.Ay = mpu6050.Ay * norm;
//     mpu6050.Az = mpu6050.Az * norm;

//
//     vx = 2 * (q1q3 - q0q2);
//     vy = 2 * (q0q1 + q2q3);
//     vz = q0q0 - q1q1 - q2q2 + q3q3;

//
//     ex = (mpu6050.Ay * vz - mpu6050.Az * vy);
//     ey = (mpu6050.Az * vx - mpu6050.Ax * vz);
//     ez = (mpu6050.Ax * vy - mpu6050.Ay * vx);

//
//     exInt = exInt + ex * Ki;
//     eyInt = eyInt + ey * Ki;
//     ezInt = ezInt + ez * Ki;

//
//     mpu6050.Gx = mpu6050.Gx + Kp * ex + exInt;
//     mpu6050.Gy = mpu6050.Gy + Kp * ey + eyInt;
//     mpu6050.Gz = mpu6050.Gz + Kp * ez + ezInt;

//
//     q0 = q0 + (-q1 * mpu6050.Gx - q2 * mpu6050.Gy - q3 * mpu6050.Gz) * halfT;
//     q1 = q1 + (q0 * mpu6050.Gx + q2 * mpu6050.Gz - q3 * mpu6050.Gy) * halfT;
//     q2 = q2 + (q0 * mpu6050.Gy - q1 * mpu6050.Gz + q3 * mpu6050.Gx) * halfT;
//     q3 = q3 + (q0 * mpu6050.Gz + q1 * mpu6050.Gy - q2 * mpu6050.Gx) * halfT;

//
//     norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//     q0 = q0 * norm;
//     q1 = q1 * norm;
//     q2 = q2 * norm;
//     q3 = q3 * norm;

//
//     mpu6050.Yaw = atan2(2.f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3) * 57.3f;
//     mpu6050.Pitch = -asin(2.f * (q1q3 - q0q2)) * 57.3f;
//     mpu6050.Roll = atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3) * 57.3f;
// }

/**
 * @description: Kalman solution
 * @param {*}
 * @return {*}
 */
void MPU_Update(void)
{
    // MPU_Get_Gyroscope();
    // MPU_Get_Accelerometer();

#if USING_MAGNETOMETER
    MPU_Get_Magnetometer();
#endif

    mpu6050.Gx = (double)(mpu6050_original.gx + mpu6050_original.gx_e) / g_LSB;
    mpu6050.Gy = (double)(mpu6050_original.gy + mpu6050_original.gy_e) / g_LSB;
    mpu6050.Gz = (double)(mpu6050_original.gz + mpu6050_original.gz_e) / g_LSB;

    mpu6050.Ax = (double)mpu6050_original.ax / a_LSB;
    mpu6050.Ay = (double)mpu6050_original.ay / a_LSB;
    mpu6050.Az = (double)mpu6050_original.az / a_LSB;

    mpu6050.Roll = Kalman_Filter_Roll();
    mpu6050.Pitch = Kalman_Filter_Pitch();
    mpu6050.Yaw = Kalman_Filter_Yaw();
}




/*********************MPU9250 AK8963 magnetometer Sensor****************************/
u8 MPU_Get_Magnetometer(void)
{
    u8 x_axis, y_axis, z_axis;
    u8 buf[6];
    static u8 count = 0;

    // x_axis = AK8963_Read_Byte(AK8963_ASAX_REG); // X轴灵敏度调整值
    // y_axis = AK8963_Read_Byte(AK8963_ASAY_REG);
    // z_axis = AK8963_Read_Byte(AK8963_ASAZ_REG);

    if (++count == 20)
    {
        count = 0;
        MPU_Read_Len(AK8963_ADDR, AK8963_Mag_XOUTL_REG, 6, buf);
        // // mag.mz = AK8963_Read_Byte(AK8963_Mag_ZOUTL_REG);
        mag.mx = (buf[1] << 8) | buf[0];
        // HAL_Delay(5);

        // MPU_Read_Len(AK8963_ADDR, AK8963_Mag_YOUTL_REG, 2, buf + 2);
        mag.my = (buf[3] << 8) | buf[2];
        // HAL_Delay(5);

        // MPU_Read_Len(AK8963_ADDR, AK8963_Mag_ZOUTL_REG, 2, buf + 4);
        mag.mz = (buf[5] << 8) | buf[4];

        AK8963_Write_Byte(AK8963_Control_1, 0x11);  //  16-bit output and single  measurement mode 1
    }

    // // if ((AK8963_Read_Byte(AK8963_ST1_REG))) //data ready
    // if (++count == 2)
    // {
    //     // count = 0;
    //     //读取计算X轴数据
    //     buf[0] = AK8963_Read_Byte(AK8963_Mag_XOUTL_REG); //Low data
    //     if ((AK8963_Read_Byte(AK8963_ST2_REG) & 0x08) == 1) // data reading end register & check Magnetic sensor overflow occurred
    //     {
    //         buf[0] = AK8963_Read_Byte(AK8963_Mag_XOUTL_REG); //reload data
    //     }
    //     buf[1] = AK8963_Read_Byte(AK8963_Mag_XOUTH_REG); //Low data
    //     if ((AK8963_Read_Byte(AK8963_ST2_REG) & 0x08) == 1) // data reading end register & check Magnetic sensor overflow occurred
    //     {
    //         buf[1] = AK8963_Read_Byte(AK8963_Mag_XOUTH_REG); //reload data
    //     }
    //     // mag.mx = ((buf[1] << 8) | buf[0]) * (((x_axis - 128) >> 8) + 1);		//灵敏度纠正 公式见/RM-MPU-9250A-00 PDF/ 5.13
    //     mag.mx = (buf[1] << 8) | buf[0];

    //     //读取计算Y轴数据
    //     buf[2] = AK8963_Read_Byte(AK8963_Mag_YOUTL_REG); //Low data
    //     if ((AK8963_Read_Byte(AK8963_ST2_REG) & 0x08) == 1) // data reading end register
    //     {
    //         buf[2] = AK8963_Read_Byte(AK8963_Mag_YOUTL_REG);
    //     }
    //     buf[3] = AK8963_Read_Byte(AK8963_Mag_YOUTH_REG); //High data
    //     if ((AK8963_Read_Byte(AK8963_ST2_REG) & 0x08) == 1) // data reading end register
    //     {
    //         buf[3] = AK8963_Read_Byte(AK8963_Mag_YOUTH_REG);
    //     }
    //     // mag.my = ((buf[3] << 8) | buf[2]) * (((y_axis - 128) >> 8) + 1);
    //     mag.my = (buf[3] << 8) | buf[2];

    //     //读取计算Z轴数据
    //     buf[4] = AK8963_Read_Byte(AK8963_Mag_ZOUTL_REG); //Low data
    //     if ((AK8963_Read_Byte(AK8963_ST2_REG) & 0x08) == 1) // data reading end register
    //     {
    //         buf[4] = AK8963_Read_Byte(AK8963_Mag_ZOUTL_REG);
    //     }
    //     buf[5] = AK8963_Read_Byte(AK8963_Mag_ZOUTH_REG); //High data
    //     if ((AK8963_Read_Byte(AK8963_ST2_REG) & 0x08) == 1) // data reading end register
    //     {
    //         buf[5] = AK8963_Read_Byte(AK8963_Mag_ZOUTH_REG);
    //     }
    //     // mag.mz = ((buf[5] << 8) | buf[4]) * (((z_axis - 128) >> 8) + 1);
    //     mag.mz = (buf[5] << 8) | buf[4];
    //     // AK8963_Write_Byte(AK8963_Control_1, 0x11);  //  16-bit output and continuous measurement mode 1
    // }
    return 0;

}


u8 AK8963_Write_Byte(u8 reg, u8 data)
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((AK8963_ADDR << 1) | 0);
    if (MPU_IIC_Wait_Ack())                 // wait response
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg);  // register address to write
    MPU_IIC_Wait_Ack();      // wait response
    MPU_IIC_Send_Byte(data); // send data
    if (MPU_IIC_Wait_Ack())  // wait response
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Stop();
    return 0;
}

u8 AK8963_Read_Byte(u8 reg)
{
    u8 res;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((AK8963_ADDR << 1) | 0); // send device address + write command
    MPU_IIC_Wait_Ack();                     // wait response
    MPU_IIC_Send_Byte(reg);                 // register address to write
    MPU_IIC_Wait_Ack();                     // wait response
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((AK8963_ADDR << 1) | 1); // send device address + read command
    MPU_IIC_Wait_Ack();                     // wait response
    res = MPU_IIC_Read_Byte(0);             // read data and send ack
    MPU_IIC_Stop();                         // generate a stop condition
    return res;
}


// 版本回退练习