/*
 * @Date         : 2022-01-24 19:51:02
 * @LastEditors  : liu wei
 * @LastEditTime : 2022-01-29 13:23:05
 * @brief        : 向匿名地面站发送自定义数据, 发送的是不定长数据
 * @FilePath     : \LED\Core\Src\report.c
 * @Github       : https://github.com/Blackerrr
 * @Coding       : utf-8
 */
#include "report.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

uint8_t DataScope_OutPut_Buffer[100] = {0};   //串口发送缓冲区,

/**
 * @description:
 * @param {float} *target        转换前的数据
 * @param {unsigned char} *buf   转换后的存放的首地址
 * @param {unsigned char} beg
 * @param {uint8_t} flag         1 大端模式  0 小端模式
 * @return {*}
 */
void Float2Byte(float *target, unsigned char *buf, unsigned char beg, uint8_t flag)
{
    unsigned char *point;
    point = (unsigned char *)target; //得到float的地址
    if (flag)                        // 大端模式
    {
        buf[beg] = point[3];
        buf[beg + 1] = point[2];
        buf[beg + 2] = point[1];
        buf[beg + 3] = point[0];
    }
    else // 小端模式
    {
        buf[beg] = point[0];
        buf[beg + 1] = point[1];
        buf[beg + 2] = point[2];
        buf[beg + 3] = point[3];
    }
}

/**
 * @description: 将待发送通道的单精度浮点数据写入发送缓冲区
 * @param {float} Data              通道数据
 * @param {unsigned char} Channel   0 - 20
 * @param {uint8_t} flag            1 大端模式  0 小端模式
 * @return {*}
 */
void DataScope_Get_Channel_Data(float Data, unsigned char Channel, uint8_t flag)
{
    if ((Channel > 20) || (Channel == 0))
        return;             //通道个数大于10或等于0，直接跳出，不执行函数
    else
    {
        switch (Channel)
        {
            case 1:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 1, flag);
                break;
            case 2:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 5, flag);
                break;
            case 3:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 9, flag);
                break;
            case 4:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 13, flag);
                break;
            case 5:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 17, flag);
                break;
            case 6:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 21, flag);
                break;
            case 7:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 25, flag);
                break;
            case 8:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 29, flag);
                break;
            case 9:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 33, flag);
                break;
            case 10:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 37, flag);
                break;
            case 11:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 41, flag);
                break;
            case 12:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 45, flag);
                break;
            case 13:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 49, flag);
                break;
            case 14:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 53, flag);
                break;
            case 15:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 57, flag);
                break;
            case 16:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 61, flag);
                break;
            case 17:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 65, flag);
                break;
            case 18:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 69, flag);
                break;
            case 19:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 73, flag);
                break;
            case 20:
                Float2Byte(&Data, DataScope_OutPut_Buffer, 77, flag);
                break;
        }
    }
}

/*******************匿名四轴地面站移植********************/

/**
 * @description: 传送数据到匿名四轴地面站 v4.22
 * @param {uint8_t} fun     功能字  0XF1~0XFA
 * @param {uint8_t} *data   数据缓存区
 * @param {uint8_t} len     有效数据个数
 * @return {*}
 */
void Usart_Send_Data(uint8_t fun, uint8_t *data, uint8_t len)
{
    uint8_t send_buf[100];
    uint8_t cnt = 0;

    send_buf[cnt++] = 0xAA;
    send_buf[cnt++] = 0xAA;
    send_buf[cnt++] = fun;
    send_buf[cnt++] = len;

    send_buf[len + 4] = 0;                  //校验数置
    memcpy(send_buf + 4, data, len);           //复制数据
    for (cnt = 0; cnt < len + 4; cnt++)
        send_buf[len + 4] += send_buf[cnt]; //计算校验和
    HAL_UART_Transmit(&huart1, send_buf, len + 5, 1000);
}

/**
 * @description: 发送用户自定义数据
 * @param {*}
 * @return {*}
 */
void ANO_DT_UserData_Report(void)
{
    uint8_t channel_number = 0;
    /*****************************生成通道数据************************************/
    DataScope_Get_Channel_Data(mag.AK8963_ID, 1, 1);
    DataScope_Get_Channel_Data(10, 2, 1);
    DataScope_Get_Channel_Data(0, 3, 1);
    DataScope_Get_Channel_Data(0, 4, 1);
    DataScope_Get_Channel_Data(0, 5, 1);
    DataScope_Get_Channel_Data(0, 6, 1);
    DataScope_Get_Channel_Data(0, 7, 1);
    DataScope_Get_Channel_Data(0, 8, 1);
    DataScope_Get_Channel_Data(0, 9, 1);
    DataScope_Get_Channel_Data(0, 10, 1);
    DataScope_Get_Channel_Data(0, 11, 1);
    DataScope_Get_Channel_Data(0, 12, 1);
    DataScope_Get_Channel_Data(0, 13, 1);
    DataScope_Get_Channel_Data(0, 14, 1);
    
    DataScope_Get_Channel_Data(0, 15, 1);
    DataScope_Get_Channel_Data(0, 16, 1);
    DataScope_Get_Channel_Data(0, 17, 1);
    DataScope_Get_Channel_Data(0, 18, 1);
    DataScope_Get_Channel_Data(0, 19, 1);
    DataScope_Get_Channel_Data(0, 20, 1);
    channel_number = 2;

    Usart_Send_Data(0xF1, DataScope_OutPut_Buffer + 1, channel_number * 4);
}

void ANO_DT_Send_Senser(void)
{
    u8 _cnt = 0;
    vs16 _temp;

    DataScope_OutPut_Buffer[_cnt++] = 0xAA;
    DataScope_OutPut_Buffer[_cnt++] = 0xAA;
    DataScope_OutPut_Buffer[_cnt++] = 0x02;
    DataScope_OutPut_Buffer[_cnt++] = 0;

    _temp = mpu6050_original.ax;
    DataScope_OutPut_Buffer[_cnt++] = BYTE1(_temp);
    DataScope_OutPut_Buffer[_cnt++] = BYTE0(_temp);
    _temp = mpu6050_original.ay;
    DataScope_OutPut_Buffer[_cnt++] = BYTE1(_temp);
    DataScope_OutPut_Buffer[_cnt++] = BYTE0(_temp);
    _temp = mpu6050_original.az;
    DataScope_OutPut_Buffer[_cnt++] = BYTE1(_temp);
    DataScope_OutPut_Buffer[_cnt++] = BYTE0(_temp);

    _temp = mpu6050_original.gx;
    DataScope_OutPut_Buffer[_cnt++] = BYTE1(_temp);
    DataScope_OutPut_Buffer[_cnt++] = BYTE0(_temp);
    _temp = mpu6050_original.gy;
    DataScope_OutPut_Buffer[_cnt++] = BYTE1(_temp);
    DataScope_OutPut_Buffer[_cnt++] = BYTE0(_temp);
    _temp = mpu6050_original.gz;
    DataScope_OutPut_Buffer[_cnt++] = BYTE1(_temp);
    DataScope_OutPut_Buffer[_cnt++] = BYTE0(_temp);

    /*****************磁力计*********************/
    _temp = mag.mx;
    DataScope_OutPut_Buffer[_cnt++] = BYTE1(_temp);
    DataScope_OutPut_Buffer[_cnt++] = BYTE0(_temp);
    _temp = mag.my;
    DataScope_OutPut_Buffer[_cnt++] = BYTE1(_temp);
    DataScope_OutPut_Buffer[_cnt++] = BYTE0(_temp);
    _temp = mag.mz;
    DataScope_OutPut_Buffer[_cnt++] = BYTE1(_temp);
    DataScope_OutPut_Buffer[_cnt++] = BYTE0(_temp);

    DataScope_OutPut_Buffer[3] = _cnt - 4;

    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
        sum += DataScope_OutPut_Buffer[i];
    DataScope_OutPut_Buffer[_cnt++] = sum;

    HAL_UART_Transmit(&huart1, DataScope_OutPut_Buffer, _cnt, 1000);
}