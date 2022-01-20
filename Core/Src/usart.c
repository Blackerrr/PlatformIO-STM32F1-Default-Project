/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "stdio.h"
#include <stdarg.h>
#include <string.h>

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /*使能串口接收中断*/
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

    /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (uartHandle->Instance == USART1)
    {
        /* USER CODE BEGIN USART1_MspInit 0 */

        /* USER CODE END USART1_MspInit 0 */
        /* USART1 clock enable */
        __HAL_RCC_USART1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USART1 GPIO Configuration
        PA9     ------> USART1_TX
        PA10     ------> USART1_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART1 interrupt Init */
        HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
        /* USER CODE BEGIN USART1_MspInit 1 */

        /* USER CODE END USART1_MspInit 1 */
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

    if (uartHandle->Instance == USART1)
    {
        /* USER CODE BEGIN USART1_MspDeInit 0 */

        /* USER CODE END USART1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART1_CLK_DISABLE();

        /**USART1 GPIO Configuration
        PA9     ------> USART1_TX
        PA10     ------> USART1_RX
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);

        /* USART1 interrupt Deinit */
        HAL_NVIC_DisableIRQ(USART1_IRQn);
        /* USER CODE BEGIN USART1_MspDeInit 1 */

        /* USER CODE END USART1_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

// #ifdef __GNUC__
// /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
//    set to 'Yes') calls __io_putchar() */
// #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
// #else
// #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
// #endif /* __GNUC__ */
// /**
//   * @brief  Retargets the C library printf function to the USART.
//   * @param  None
//   * @retval None
//   */
// PUTCHAR_PROTOTYPE
// {
//     /* Place your implementation of fputc here */
//     /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
//     HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);

//     return ch;
// }


/**
 * @description: 串口发送字符串
 * @param {UART_HandleTypeDef*} uartHandle
 * @param {uint8_t*} str
 * @return {*}
 */
void Usart_SendString(UART_HandleTypeDef* uartHandle, uint8_t* str)
{
    unsigned int k = 0;
    do
    {
        HAL_UART_Transmit(uartHandle, (uint8_t*)(str + k), 1, 1000);
        k++;
    } while (*(str + k) != '\0');
}

/**
 * @description:
 * @param {uint8_t} c
 * @return {*}
 */
void usart1_send_char(uint8_t c)
{
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET)
    {
    };
    HAL_UART_Transmit(&huart1, &c, 1, 1000);
}

// void vprint(const char *fmt, va_list argp)
// {
//     char string[200];
//     if(0 < vsprintf(string,fmt,argp)) // build string
//     {
//         HAL_UART_Transmit(&huart1, (uint8_t*)string, strlen(string), 0xffffff); // send message via UART
//     }
// }

// void my_printf(const char *fmt, ...) // custom printf() function
// {
//     va_list argp;
//     va_start(argp, fmt);
//     vprint(fmt, argp);
//     va_end(argp);
// }


// arm-none-eabi 编译器下重定向 fputs, fgets失效
// /**
//  * @description: 重定向 c 库函数 printf 到串口 USARTx，重定向后可使用 printf 函数
//  * @param {int} ch
//  * @param {FILE} *f
//  * @return {*}
//  */
// int fputc(int ch, FILE *f)
// {
//     /* 发送一个字节数据到串口 USARTx */
//     HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
//     return (ch);
// }

// /**
//  * @description: 重定向 c 库函数 scanf 到串口 USARTx，重写向后可使用 scanf、getchar 等函数
//  * @param {FILE} *f
//  * @return {*}
//  */
// int fgetc(FILE *f)
// {
//     int ch;
//     /* 等待串口输入数据 */
//     while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET);
//     HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
//     return (ch);
// }
/* USER CODE END 1 */
