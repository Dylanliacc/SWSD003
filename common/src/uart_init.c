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

int AT_start_flag = 0;  // 启动标志位

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static smtc_hal_mcu_uart_inst_t inst_uart = NULL;
static ATC_HandleTypeDef atc_handle;  // Handle for AT command module

void atc_per_event_callback(const char* event_data); //测试指令
void AT_Param_Callback(char* param1, char* param2); //超参数设置指令

void AT_Power_Callback(char* param1, char* param2);  // 功率设置指令

void AT_Freq_Callback(char* param1, char* param2);  // 频率设置指令

void AT_Help_Callback(char* param1, char* param2);  // 帮助指令

void AT_START_event_callback(char* param1, char* param2);  // 启动指令
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
/***************************************************************************/
    // Configure AT command events
    static ATC_EventTypeDef atc_events[] = {
    {"AT+FREQ", AT_Freq_Callback},   // 频率设置指令
    {"AT+POWER", AT_Power_Callback}, // 功率设置指令
    {"AT+PARAM", AT_Param_Callback}, // 超参数设置指令
    {"AT+HELP",AT_Help_Callback},    // 帮助指令
    {"AT+PER", atc_per_event_callback},  // 测试指令
    {"AT+START", AT_START_event_callback},  // 启动指令
    {NULL, NULL}  // 事件结束标志
};
		
/***************************************************************************/		
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

int main_loop(void)
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
        if(AT_start_flag == 1){
            return 1;
        }
    }
}



void atc_per_event_callback(const char* event_data)
{
    HAL_DBG_TRACE_INFO("AT+PER received: %s\n", event_data);
		
    // Add PER measurement reset logic here
    HAL_DBG_TRACE_INFO("Resetting PER measurement...\n");
}


void AT_Freq_Callback(char* param1, char* param2) {
    if (param1 != NULL) {
        int frequency = atoi(param1);  // 将频率字符串转换为整数
        HAL_DBG_TRACE_INFO("Frequency set to: %d Hz\n", frequency);
        // 在这里进行频率设置的具体操作
    } else {
        HAL_DBG_TRACE_INFO("Invalid frequency parameter.\n");
    }
}


// 处理功率的回调函数
void AT_Power_Callback(char* param1, char* param2) {
    if (param1 != NULL) {
        int power = atoi(param1);  // 将功率字符串转换为整数
        HAL_DBG_TRACE_INFO("Power set to: %d dBm\n", power);
        // 在这里进行功率设置的具体操作
    } else {
        HAL_DBG_TRACE_INFO("Invalid power parameter.\n");
    }
}


// 处理超参数的回调函数
void AT_Param_Callback(char* param1, char* param2) {
    if (param1 != NULL) {
        int param = atoi(param1);  // 将超参数字符串转换为整数
        HAL_DBG_TRACE_INFO("Parameter set to: %d\n", param);
        // 在这里进行超参数设置的具体操作
    } else {
        HAL_DBG_TRACE_INFO("Invalid parameter.\n");
    }
}

void AT_Help_Callback(char* param1, char* param2) {
    HAL_DBG_TRACE_INFO("AT+HELP received.\n");
    // 在这里添加帮助信息 :写一下下面的帮助信息
    HAL_DBG_TRACE_INFO("AT+FREQ=<frequency> : Set the frequency in Hz\n");
    HAL_DBG_TRACE_INFO("AT+POWER=<power> : Set the power in dBm\n");
    HAL_DBG_TRACE_INFO("AT+PARAM=<param> : Set a parameter(Not work but availilable to call)\n");
    HAL_DBG_TRACE_INFO("AT+PER : Perform PER measurement(Not work but availilable to call)\n");
}

void AT_START_event_callback(char* param1, char* param2){
    HAL_DBG_TRACE_INFO("AT+START received.\n");
    // 在这里添加启动操作
    HAL_DBG_TRACE_INFO("Start the operation...\n");
    AT_start_flag = 1;
}