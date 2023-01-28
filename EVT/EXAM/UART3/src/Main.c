/********************************** (C) COPYRIGHT
 ******************************** File Name          : Main.c Author : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : 串口1收发演示
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH58x_common.h"

uint8_t TxBuff[] = "This is a tx exam for UART3\r\n";
uint8_t RxBuff[100];

/*********************************************************************
 * @fn      main
 *
 * @brief   主函数
 *
 * @return  none
 */
int main() {
  uint8_t len;

  SetSysClock(CLK_SOURCE_PLL_60MHz);

  GPIOA_SetBits(GPIO_Pin_5);
  GPIOA_ModeCfg(GPIO_Pin_4, GPIO_ModeIN_PU);      // RX
  GPIOA_ModeCfg(GPIO_Pin_5, GPIO_ModeOut_PP_5mA); // TX
  UART3_DefInit();

  UART3_SendString(TxBuff, sizeof(TxBuff));

  while (1) {
    len = UART3_RecvString(RxBuff);
    if (len) {
      UART3_SendString(RxBuff, len);
      UART3_SendString(RxBuff, len);
    }
  }
}
