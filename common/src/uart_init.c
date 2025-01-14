/*!
 * @file      uart_init.c
 *
 * @brief     UART init helper functions implementation with AT command integration
 *
 * The Clear BSD License Copyright Semtech Corporation 2023.
 *
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stddef.h>
#include <string.h>
#include "uart_init.h"
#include "stm32l4xx.h"
#include "smtc_hal_mcu_uart_stm32l4.h"
#include "smtc_hal_dbg_trace.h"
#include "smtc_hal_mcu.h"
#include "stm32l4xx_ll_utils.h"
#include "atc.h"  // Include the AT command handler header



uint8_t rx_buffer[256];  
uint16_t rx_index = 0;
bool at_command_received = false;
uint16_t rx_length = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static smtc_hal_mcu_uart_inst_t inst_uart = NULL;
static ATC_HandleTypeDef atc_handle;  // Handle for AT command module
void atc_per_event_callback(const char* event_data);
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief UART RX interrupt callback
 *
 * @param data The received byte
 */
static void uart_rx_callback(uint8_t data);

/**
 * @brief Base function to initialize UART peripheral
 *
 * @param callback_rx The callback called on RX byte reception. Can be NULL
 */
static void uart_init_base(void (*callback_rx)(uint8_t data));

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void uart_init(void)
{
    // Initialize UART with RX callback
    uart_init_base(uart_rx_callback);

    // Initialize AT command handler
    if (!ATC_Init(&atc_handle, inst_uart, 125, "UART2")) {
        HAL_DBG_TRACE_ERROR("Failed to initialize AT command handler!\n");
        return;
    }

    // Configure AT command events
    static ATC_EventTypeDef atc_events[] = {
        {"AT+PER", atc_per_event_callback},  // Example AT command
        {NULL, NULL},
    };

    if (!ATC_SetEvents(&atc_handle, atc_events)) {
        HAL_DBG_TRACE_ERROR("Failed to set AT events!\n");
        return;
    }

    HAL_DBG_TRACE_INFO("UART and AT command handler initialized.\n");
		rx_index = 0;
}

void vprint(const char* fmt, va_list argp)
{
    char string[255];
    if (0 < vsprintf(string, fmt, argp)) {  // Build the formatted string
        smtc_hal_mcu_uart_send(inst_uart, (uint8_t*)string, strlen(string));
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void uart_init_base(void (*callback_rx)(uint8_t data)) {
    const struct smtc_hal_mcu_uart_cfg_s cfg_uart = {
        .usart = USART2,
    };

    const smtc_hal_mcu_uart_cfg_app_t uart_cfg_app = {
        .baudrate    = 921600,
        .callback_rx = callback_rx,
    };

    int init_result = smtc_hal_mcu_uart_init(( const smtc_hal_mcu_uart_cfg_t )&cfg_uart, &uart_cfg_app, &inst_uart);
    
    if (init_result != 0) {
        HAL_DBG_TRACE_ERROR("UART initialization failed with error code %d.\n", init_result);
        return;
    }

    HAL_DBG_TRACE_INFO("UART initialized on USART2 with baudrate 921600.\n");
}

void ATC_IdleLine(ATC_HandleTypeDef* hAtc, uint16_t Len) ;
static void uart_rx_callback(uint8_t data)
{
    if (rx_index < sizeof(rx_buffer) - 1) {
        rx_buffer[rx_index++] = data;
    }

    // 判断接收到的命令是否结束（例如以换行符 '\n' 或 '\r' 结尾）
    if ((data == '\n' || data == '\r')&& rx_index > 1) {
        if (rx_index > 0 && (rx_buffer[rx_index - 1] == '\n' || rx_buffer[rx_index - 1] == '\r')) {
            rx_buffer[rx_index - 1] = '\0';  // 确保字符串以 NULL 结束
        }
        HAL_DBG_TRACE_INFO("Received command: %s\n", rx_buffer);

        // 将接收到的数据传递到 ATC 系统
        rx_length = rx_index-1;  // 更新接收到的长度
        memcpy(atc_handle.pRxBuff,rx_buffer, rx_length);  // 复制数据到 ATC 缓冲区

				
        at_command_received = true;  // 设置标志位，表示命令已接收
        rx_index = 0;  // 重置接收索引
    }
}

void ATC_IdleLine(ATC_HandleTypeDef* hAtc, uint16_t Len) {
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

void main_loop(void)
{
    while (1) {
        if (at_command_received) {
            at_command_received = false;  // 清除标志位
            if (rx_length > 0) {  // 确保只有在有数据时才处理
                ATC_IdleLine(&atc_handle,rx_length);
                ATC_Loop(&atc_handle);
            }
            rx_length = 0;  // 重置数据长度
        }
				LL_mDelay( 20 );
        // 其他主循环代码
    }
}



void atc_per_event_callback(const char* event_data)
{
    HAL_DBG_TRACE_INFO("AT+PER received: %s\n", event_data);

    // Add PER measurement reset logic here
    HAL_DBG_TRACE_INFO("Resetting PER measurement...\n");
}
