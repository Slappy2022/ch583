/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2020/08/06
 * Description        : 外设从机应用主函数及任务系统初始化
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */

#include "compat.h"
#include "CH58xBLE_LIB.h"
#include "peripheral.h"

/*******************************************************************************
 * Function Name  : main
 * Description    : 主函数
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
int main(void)
{
    // compat.h
    compat_Init();
    // peripheral.h
    Peripheral_Init();

    while(1)
    {
        // CH58xBLE_LIB.h
        TMOS_SystemProcess();
    }
}

/******************************** endfile @ main ******************************/
