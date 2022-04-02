/*
 * @Date         : 2022-04-01 19:39:42
 * @LastEditors  : liu wei
 * @LastEditTime : 2022-04-02 12:30:33
 * @brief        : Do not edit
 * @FilePath     : \LED\Core\Inc\mpuiic.h
 * @Github       : https://github.com/Blackerrr
 * @Coding       : utf-8
 */
#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "main.h"


/********************************* MPUIIC ***************************************/


#define MPU_IIC_SCL PCout(0) //SCL
#define MPU_IIC_SDA PCout(1) //SDA
#define MPU_READ_SDA PCin(1) // input SDA

#define USING_PP    0         // 1 IIC 使用推挽   0  IIC 使用开漏



//IIC All function operations
// void MPU_IIC_Delay(uint16_t cnt);        // MPU IIC delay function
void MPU_IIC_Init(void);                 // Initialize the IO port of the IIC
void MPU_IIC_Start(void);                // Send IIC start signal
void MPU_IIC_Stop(void);                 // send IIC stop signal
void MPU_IIC_Send_Byte(u8 txd);          // IIC sends a byte
u8 MPU_IIC_Read_Byte(unsigned char ack); // IIC reads a byte
u8 MPU_IIC_Wait_Ack(void);               // IIC waits for ACK signal
void MPU_IIC_Ack(void);                  // IIC sends ACK signal
void MPU_IIC_NAck(void);                 // IIC does not send ACK signal


u8 MPU_IIC_Check_Device(u8 addr);       // 检查设备是否存在

#endif