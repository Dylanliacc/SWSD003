#ifndef _ATC_H_
#define _ATC_H_

#include <stdbool.h>
#include <stdint.h>
#include "smtc_hal_mcu_uart.h"

typedef struct {
    char* Event;
    void (*EventCallback)(const char*);
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

#endif
