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

#include "compat.h"
#include "CH58xBLE_LIB.h"
#include "peripheral.h"

/*******************************************************************************
 * Function Name  : main
 * Description    : ������
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
