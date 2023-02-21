/******************************************************************************
 * @file     uart_transfer.h
 * @brief    General UART ISP slave header file
 * @version  1.0.0
 * @date     22, Sep, 2014
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __UART_TRANS_H__
#define __UART_TRANS_H__
#include <stdint.h>

/*-------------------------------------------------------------*/

#define ISP_UART_PORT					(UART1)
#define ISP_UART_PORT_IRQn				(UART1_IRQn)
#define ISP_UART_IRQHandler				(UART1_IRQHandler)



/* Define maximum packet size */
#define MAX_PKT_SIZE            64

/*-------------------------------------------------------------*/

extern uint8_t  uart_rcvbuf[];
extern uint8_t volatile bUartDataReady;
extern uint8_t volatile bufhead;

/*-------------------------------------------------------------*/
void ISP_UART_Init(void);
void ISP_UART_IRQHandler(void);
void PutString(void);

#endif  /* __UART_TRANS_H__ */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
