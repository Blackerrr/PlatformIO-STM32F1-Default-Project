/*
 * @Date         : 2022-01-20 13:24:32
 * @LastEditors  : liu wei
 * @LastEditTime : 2022-01-20 13:33:36
 * @FilePath     : \LED\Core\Inc\retarget.h
 * @Github       : https://github.com/Blackerrr
 * @Coding       : utf-8
 */
#ifndef _RETARGET_H__
#define _RETARGET_H__
#include "stm32f1xx_hal.h"
#include <sys/stat.h>
#include <stdio.h>
void RetargetInit(UART_HandleTypeDef *huart);
int _isatty(int fd);
int _write(int fd, char *ptr, int len);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _read(int fd, char *ptr, int len);
int _fstat(int fd, struct stat *st);
void *_sbrk(int incr);

#endif