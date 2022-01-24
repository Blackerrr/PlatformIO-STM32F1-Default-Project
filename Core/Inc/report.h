/*
 * @Date         : 2022-01-24 19:51:35
 * @LastEditors  : liu wei
 * @LastEditTime : 2022-01-24 22:15:24
 * @brief        : Do not edit
 * @FilePath     : \LED\Core\Inc\report.h
 * @Github       : https://github.com/Blackerrr
 * @Coding       : utf-8
 */
#ifndef __REPORT_H
#define __REPORT_H
#include "main.h"
#include "usart.h"

/*取字节*/
#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))


void wireless_niming_report(uint8_t fun, uint8_t *data, uint8_t len);
void niming_report(void);
#endif
