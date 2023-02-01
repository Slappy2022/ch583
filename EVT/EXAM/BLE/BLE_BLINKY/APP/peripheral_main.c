/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2020/08/06
 * Description        : ����ӻ�Ӧ��������������ϵͳ��ʼ��
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
/* ͷ�ļ����� */
//#include "config.h"
//#include "HAL.h"
//#include "gattprofile.h"
//#include "CH58x_common.h"

// Needed for sys
#ifndef  SAFEOPERATE
#define  SAFEOPERATE   __nop();__nop()
#endif
#include <stdint.h>      // Needed for gpio and sys
#include "compat.h"
#include "CH58xBLE_LIB.h"
#include "peripheral.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
#ifndef BLE_MEMHEAP_SIZE
#define BLE_MEMHEAP_SIZE                    (1024*6)
#endif
__attribute__((aligned(4))) u32 MEM_BUF[BLE_MEMHEAP_SIZE / 4];

/*******************************************************************************
 * Function Name  : main
 * Description    : ������
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
int main(void)
{
    compat_SetSysClock();
		compat_LedInit();
		compat_LedToggle();

    compat_CH58X_BLEInit();
    compat_HAL_TimeInit();

    // CH58xBLE_LIB.h
    GAPRole_PeripheralInit();

    // peripheral.h
    Peripheral_Init();

    while(1)
    {
        // CH58xBLE_LIB.h
        TMOS_SystemProcess();
    }
}

/******************************** endfile @ main ******************************/
