/*
 * @Date         : 2022-01-19 16:29:37
 * @LastEditors  : liu wei
 * @LastEditTime : 2022-04-02 12:39:49
 * @FilePath     : \LED\Core\Inc\mpu.h
 * @Github       : https://github.com/Blackerrr
 * @Coding       : utf-8
 */

#ifndef __MPU_H
#define __MPU_H
#include "main.h"
#include "math.h"
#include "stdlib.h"
#include "mpuiic.h"


/********************************* MPU6050 ***************************************/

// MPU6050 AD0 control pin
// #define MPU_AD0_CTRL PAout(15) // Control the AD0 level, thereby controlling the MPU address

//#define MPU_ACCEL_OFFS_REG		0X06	// The accel_offs register, which can read the version number, is not mentioned in the register manual
//#define MPU_PROD_ID_REG			0X0C	// prod id register, not mentioned in the register manual
#define MPU_SELF_TESTX_REG 0X0D   // Self-Test Register X
#define MPU_SELF_TESTY_REG 0X0E   // Self-Test Register Y
#define MPU_SELF_TESTZ_REG 0X0F   // Self-Test Register Z
#define MPU_SELF_TESTA_REG 0X10   // Self-Test Register A
#define MPU_SAMPLE_RATE_REG 0X19  // Sampling frequency divider

#define MPU_CFG_REG 0X1A          // configuration register
#define MPU_GYRO_CFG_REG 0X1B     // Gyroscope Configuration Register
#define MPU_ACCEL_CFG_REG 0X1C    // Accelerometer Configuration Register
#define MPU_MOTION_DET_REG 0X1F   // Motion Detection Threshold Setting Register
#define MPU_FIFO_EN_REG 0X23      // FIFO enable register
#define MPU_I2CMST_CTRL_REG 0X24  // IIC master control register
#define MPU_I2CSLV0_ADDR_REG 0X25 // IIC Slave 0 Device Address Register
#define MPU_I2CSLV0_REG 0X26      // IIC Slave 0 Data Address Register
#define MPU_I2CSLV0_CTRL_REG 0X27 // IIC slave 0 control register
#define MPU_I2CSLV1_ADDR_REG 0X28 // IIC Slave 1 Device Address Register
#define MPU_I2CSLV1_REG 0X29      // IIC Slave 1 Data Address Register
#define MPU_I2CSLV1_CTRL_REG 0X2A // IIC slave 1 control register
#define MPU_I2CSLV2_ADDR_REG 0X2B // IIC Slave 2 Device Address Register
#define MPU_I2CSLV2_REG 0X2C      // IIC Slave 2 Data Address Register
#define MPU_I2CSLV2_CTRL_REG 0X2D // IIC slave 2 control register
#define MPU_I2CSLV3_ADDR_REG 0X2E // IIC Slave 3 Device Address Register
#define MPU_I2CSLV3_REG 0X2F      // IIC Slave 3 Data Address Register
#define MPU_I2CSLV3_CTRL_REG 0X30 // IIC slave 3 control register
#define MPU_I2CSLV4_ADDR_REG 0X31 // IIC Slave 4 Device Address Register
#define MPU_I2CSLV4_REG 0X32      // IIC Slave 4 Data Address Register
#define MPU_I2CSLV4_DO_REG 0X33   // IIC slave 4 write data register
#define MPU_I2CSLV4_CTRL_REG 0X34 // IIC slave 4 control register
#define MPU_I2CSLV4_DI_REG 0X35   // IIC slave 4 read data register

#define MPU_I2CMST_STA_REG 0X36 // IIC Master Status register
#define MPU_INTBP_CFG_REG 0X37  // Interrupt/Bypass Setting Register
#define MPU_INT_EN_REG 0X38     // Interrupt Enable Register
#define MPU_INT_STA_REG 0X3A    // Interrupt Status Register

#define MPU_ACCEL_XOUTH_REG 0X3B // Acceleration value, X-axis high 8-bit register
#define MPU_ACCEL_XOUTL_REG 0X3C // Acceleration value, X-axis low 8-bit register
#define MPU_ACCEL_YOUTH_REG 0X3D // Acceleration value, Y-axis high 8-bit register
#define MPU_ACCEL_YOUTL_REG 0X3E // Acceleration value, Y-axis low 8-bit register
#define MPU_ACCEL_ZOUTH_REG 0X3F // Acceleration value, Z-axis high 8-bit register
#define MPU_ACCEL_ZOUTL_REG 0X40 // Acceleration value, Z-axis low 8-bit register

#define MPU_TEMP_OUTH_REG 0X41 // temperature value, high 8-big register
#define MPU_TEMP_OUTL_REG 0X42 // temperature value, low 8-bit register

#define MPU_GYRO_XOUTH_REG 0X43 // Gyro value, X-axis high 8-bit register
#define MPU_GYRO_XOUTL_REG 0X44 // Gyro value, X-axis low 8-bit register
#define MPU_GYRO_YOUTH_REG 0X45 // Gyro value, Y-axis high 8-bit register
#define MPU_GYRO_YOUTL_REG 0X46 // Gyro value, Y-axis low 8-bit register
#define MPU_GYRO_ZOUTH_REG 0X47 // Gyro value, Z-axis high 8-bit register
#define MPU_GYRO_ZOUTL_REG 0X48 // Gyro value, Z-axis low 8-bit register

#define MPU_I2CSLV0_DO_REG 0X63 // IIC slave 0 data register
#define MPU_I2CSLV1_DO_REG 0X64 // IIC slave 1 data register
#define MPU_I2CSLV2_DO_REG 0X65 // IIC slave 2 data register
#define MPU_I2CSLV3_DO_REG 0X66 // IIC slave 3 data register

#define MPU_I2CMST_DELAY_REG 0X67 // IIC Master Delay Management Register
#define MPU_SIGPATH_RST_REG 0X68  // Signal Channel Reset Register
#define MPU_MDETECT_CTRL_REG 0X69 // Motion Detection Control Register
#define MPU_USER_CTRL_REG 0X6A    // User Control Register 
#define MPU_PWR_MGMT1_REG 0X6B    // Power Management Register 1
#define MPU_PWR_MGMT2_REG 0X6C    // Power Management Register 2
#define MPU_FIFO_CNTH_REG 0X72    // FIFO count register upper eight bits
#define MPU_FIFO_CNTL_REG 0X73    // FIFO count register lower eight bits
#define MPU_FIFO_RW_REG 0X74      // FIFO read and write registers
#define MPU_DEVICE_ID_REG 0X75    // Device ID Register    WHO AM I


#define EXT_SENS_DATA_00     0x49
#define EXT_SENS_DATA_01     0x4A
#define EXT_SENS_DATA_02     0x4B
#define EXT_SENS_DATA_03     0x4C
#define EXT_SENS_DATA_04     0x4D
#define EXT_SENS_DATA_05     0x4E
#define EXT_SENS_DATA_06     0x4F
#define EXT_SENS_DATA_07     0x50
#define EXT_SENS_DATA_08     0x51
#define EXT_SENS_DATA_09     0x52

#define EXT_SENS_DATA_10     0x53
#define EXT_SENS_DATA_11     0x54
#define EXT_SENS_DATA_12     0x55
#define EXT_SENS_DATA_13     0x56
#define EXT_SENS_DATA_14     0x57
#define EXT_SENS_DATA_15     0x58
#define EXT_SENS_DATA_16     0x59
#define EXT_SENS_DATA_17     0x5A
#define EXT_SENS_DATA_18     0x5B
#define EXT_SENS_DATA_19     0x5C

#define EXT_SENS_DATA_20     0x5D
#define EXT_SENS_DATA_21     0x5E
#define EXT_SENS_DATA_22     0x5F
#define EXT_SENS_DATA_23     0x60



// If the AD0 pin (9 pin) is grounded, the IIC address is 0X68 (does not include the lowest(least) bit).
// If connected to V3.3, the IIC address is 0X69 (not including the lowest bit).
#define MPU_ADDR 0X68    // mpu6050  

// Because the module AD0 is connected to GND by default, it is 0XD1 and 0XD0
// if it is connected to VCC, it is 0XD3 and 0XD2) after switching to the read-write address.
// #define MPU_READ    0XD1
// #define MPU_WRITE   0XD0



// MPU9250 AK8963 registers
#define AK8963_ADDR           0x0C
#define AK8963_Device_ID      0x48   //   this is not a register
#define AK8963_WIA            0x00
#define AK8963_Mag_XOUTH_REG  0x04
#define AK8963_Mag_XOUTL_REG  0x03
#define AK8963_Mag_YOUTH_REG  0x06
#define AK8963_Mag_YOUTL_REG  0x05
#define AK8963_Mag_ZOUTH_REG  0x08
#define AK8963_Mag_ZOUTL_REG  0x07
#define AK8963_Control_1      0x0A
#define AK8963_Control_2      0x0B
#define AK8963_ST1_REG        0x02
#define AK8963_ST2_REG        0x09  
#define AK8963_ASAX_REG       0x10
#define AK8963_ASAY_REG       0x11
#define AK8963_ASAZ_REG       0x12



// #define G 9.7947  // Hefei gravity acceleration
#define G 9.7914    // Chongqing gravity acceleration
#define halfT 0.01f // Half of the sampling time, in seconds

typedef struct
{
    uint8_t  ID;    // 读出来的ID
    uint8_t  ACK;   // MPU设备是否应答 
} MPU_Status;
extern MPU_Status mpu_status; 

typedef struct
{
    double Roll;
    double Pitch;
    double Yaw;
    double Gx;
    double Gy;
    double Gz;
    double Ax;
    double Ay;
    double Az;
} MPU6050_InitDefine;

typedef struct
{
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx_e;
    int16_t gy_e;
    int16_t gz_e;
    int16_t ax_e;
    int16_t ay_e;
    int16_t az_e;
} MPU6050_Original;

extern MPU6050_InitDefine mpu6050;
extern MPU6050_Original mpu6050_original;
extern float q0, q1, q2, q3;      //
extern float exInt, eyInt, ezInt; //
extern unsigned int a_LSB;        //
extern float g_LSB;               //

u8 MPU_Init(void);                                  // Initialize MPU6050
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf); // IIC continuous write
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf);  // IIC continuous read
u8 MPU_Write_Byte(u8 reg, u8 data);                 // IIC writes a byte
u8 MPU_Read_Byte(u8 reg);                           // IIC reads a byte

u8 MPU_Set_Gyro_Fsr(u8 fsr);
u8 MPU_Set_Accel_Fsr(u8 fsr);
u8 MPU_Set_LPF(u16 lpf);
u8 MPU_Set_Rate(u16 rate);
u8 MPU_Set_Fifo(u8 sens);

short MPU_Get_Temperature(void);
u8 MPU_Get_Gyroscope(void);
u8 MPU_Get_Accelerometer(void);


void MPU_Update(void);
float Kalman_Filter_Roll(void);
float Kalman_Filter_Pitch(void);
float Kalman_Filter_Yaw(void);


/*************************MPU9250 AK8963 Magnetometer Sensor******************************/
#define USING_MAGNETOMETER 0
typedef struct
{
    u8 AK8963_ID;
    int16_t mx;
    int16_t my;
    int16_t mz;
} Magnetometer;
extern Magnetometer mag;
u8 MPU_Get_Magnetometer(void);
u8 AK8963_Write_Byte(u8 reg, u8 data);
u8 AK8963_Read_Byte(u8 reg);



#endif
