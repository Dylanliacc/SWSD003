#ifndef _ATC_H_
#define _ATC_H_

#include <stdbool.h>
#include <stdint.h>
#include "smtc_hal_mcu_uart.h"

typedef void (*ATC_EventCallback)(char* param1, char* param2);


typedef struct {
    char* Event;                     
    ATC_EventCallback EventCallback;  
} ATC_EventTypeDef;

typedef struct {
    smtc_hal_mcu_uart_inst_t hUart;
    char Name[8];
    ATC_EventTypeDef* psEvents;
    uint32_t Events;
    uint16_t Size;
    uint16_t RxIndex;
    uint8_t* pRxBuff;
    uint8_t* pReadBuff;
} ATC_HandleTypeDef;

bool ATC_Init(ATC_HandleTypeDef* hAtc, smtc_hal_mcu_uart_inst_t hUart, uint16_t BufferSize, const char* pName);
bool ATC_SetEvents(ATC_HandleTypeDef* hAtc, ATC_EventTypeDef* events);
void ATC_Loop(ATC_HandleTypeDef* hAtc);
void ATC_IdleLineCallback(ATC_HandleTypeDef* hAtc, uint16_t Len);


//macros redefine
extern int ATC_M_TX_OUTPUT_POWER_DBM;

extern uint32_t ATC_M_RF_FREQ_IN_HZ;

extern int ATC_M_TXRX_SWITCH;

extern int ATC_M_LORA_SF;

extern int ATC_M_LORA_BW;

extern int ATC_M_LORA_CR;

extern int ATC_M_CW_SWITCH;

extern int ATC_M_NB_FRAME;

extern int ATC_M_LORA_RX_BOOST;

extern int ATC_M_LORA_SLEEP;

extern int ATC_M_PA_PA_HP_SEL;

extern int ATC_M_PA_PA_SEL;

extern int ATC_M_PA_PA_RGE_SUPPLY;

extern int ATC_M_PA_PA_DUTY_CYCLE;

extern int ATC_M_PA_PA_REAL_POWER;
#endif
