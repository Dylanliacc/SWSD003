#include "atc.h"
#include <string.h>
#include <stdlib.h>
#include "smtc_hal_dbg_trace.h"

bool ATC_Init(ATC_HandleTypeDef* hAtc, smtc_hal_mcu_uart_inst_t hUart, uint16_t BufferSize, const char* pName) {
    if (hAtc == NULL || hUart == NULL) {
        HAL_DBG_TRACE_ERROR("ATC_Init failed: invalid parameters.\n");
        return false;
    }

    memset(hAtc, 0, sizeof(ATC_HandleTypeDef));
    hAtc->hUart = hUart;
    strncpy(hAtc->Name, pName, sizeof(hAtc->Name) - 1);

    hAtc->pRxBuff = malloc(BufferSize);
    hAtc->pReadBuff = malloc(BufferSize);

    if (hAtc->pRxBuff == NULL || hAtc->pReadBuff == NULL) {
        HAL_DBG_TRACE_ERROR("ATC_Init failed: memory allocation failed.\n");
        free(hAtc->pRxBuff);
        free(hAtc->pReadBuff);
        return false;
    }

    memset(hAtc->pRxBuff, 0, BufferSize);
    memset(hAtc->pReadBuff, 0, BufferSize);
    hAtc->Size = BufferSize;

    hAtc->Events = 0;
    hAtc->psEvents = NULL;

    HAL_DBG_TRACE_INFO("ATC initialized on %s\n", pName);
    return true;
}


bool ATC_SetEvents(ATC_HandleTypeDef* hAtc, ATC_EventTypeDef* events) {
    if (hAtc == NULL || events == NULL) {
        return false;
    }

    uint32_t count = 0;
    while (events[count].Event != NULL && events[count].EventCallback != NULL) {
        count++;
    }

    hAtc->psEvents = events;
    hAtc->Events = count;

    HAL_DBG_TRACE_INFO("ATC events configured. Total: %d\n", count);
    return true;
}

void ATC_Loop(ATC_HandleTypeDef* hAtc) {
    if (hAtc->RxIndex > 0) {
        for (uint32_t i = 0; i < hAtc->Events; i++) {
      
            HAL_DBG_TRACE_INFO("Checking for event: %s\n", hAtc->psEvents[i].Event);
            HAL_DBG_TRACE_INFO("Received data: %s\n", hAtc->pReadBuff);


            char* found = strstr((char*)hAtc->pReadBuff, hAtc->psEvents[i].Event);
            if (found != NULL && hAtc->psEvents[i].EventCallback != NULL) {
                HAL_DBG_TRACE_INFO("Event '%s' matched. Triggering callback.\n", hAtc->psEvents[i].Event);
                hAtc->psEvents[i].EventCallback(found);
            }
        }
        hAtc->RxIndex = 0;
        memset(hAtc->pReadBuff, 0, hAtc->Size);
    }
}



void ATC_IdleLineCallback(ATC_HandleTypeDef* hAtc, uint16_t Len) {
    HAL_DBG_TRACE_INFO("ATC_IdleLineCallback called with Len: %d, RxIndex: %d\n", Len, hAtc->RxIndex);

    if (Len > hAtc->Size - hAtc->RxIndex) {
        HAL_DBG_TRACE_WARNING("Len (%d) exceeds available buffer space (%d). Truncating to fit.\n", Len, hAtc->Size - hAtc->RxIndex);
        Len = hAtc->Size - hAtc->RxIndex;
    }


    HAL_DBG_TRACE_INFO("Copying data to pReadBuff at index %d. Data: %.*s\n", hAtc->RxIndex, Len, hAtc->pRxBuff);


    memcpy(&hAtc->pReadBuff[hAtc->RxIndex], hAtc->pRxBuff, Len);
    hAtc->RxIndex += Len;

    HAL_DBG_TRACE_INFO("Updated RxIndex: %d. Current buffer content: %s\n", hAtc->RxIndex, hAtc->pReadBuff);
}


