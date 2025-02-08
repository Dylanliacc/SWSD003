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
#include "lr11xx_radio_types.h"
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

void AT_SF_Callback(char* param1, char* param2);  // SF设置指令

void AT_BW_Callback(char* param1, char* param2);  // BW设置指令

void AT_CR_Callback(char* param1, char* param2);  // CR设置指令

void AT_NB_FRAME_event_callback(char* param1, char* param2);  // NB_FRAME设置指令

void AT_Help_Callback(char* param1, char* param2);  // 帮助指令

void AT_CW_event_callback(char* param1, char* param2); 

void AT_RX_BOOST_event_callback(char* param1, char* param2); 

void AT_TRSW_event_callback(char* param1, char* param2);  

void AT_SLEEP_event_callback(char* param1, char* param2);  //Deepsleep

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
	{"AT+TRSW", AT_TRSW_event_callback},  // 启动指令
    {"AT+SF", AT_SF_Callback},  // SF设置指令
    {"AT+BW", AT_BW_Callback},  // BW设置指令
    {"AT+CR", AT_CR_Callback},  // CR设置指令
		{"AT+CWSW", AT_CW_event_callback},  // 
		{"AT+NBFRAME", AT_NB_FRAME_event_callback},
		{"AT+RXBOOST", AT_RX_BOOST_event_callback},
    {"AT+HELP",AT_Help_Callback},    // 帮助指令
    {"AT+PER", atc_per_event_callback},  // 测试指令
		
		{"AT+SLEEP", AT_SLEEP_event_callback},  // 启动指令
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
        //HAL_DBG_TRACE_INFO("Received command: %s\n", rx_buffer);

        // 将接收到的数据传递到 ATC 系统
        rx_length = rx_index-1;  // 更新接收到的长度
        memcpy(atc_handle.pRxBuff,rx_buffer, rx_length);  // 复制数据到 ATC 缓冲区

				
        at_command_received = true;  // 设置标志位，表示命令已接收
        rx_index = 0;  // 重置接收索引
    }
}

void ATC_IdleLine(ATC_HandleTypeDef* hAtc, uint16_t Len) {
    //HAL_DBG_TRACE_INFO("ATC_IdleLineCallback called with Len: %d, RxIndex: %d\n", Len, hAtc->RxIndex);

    if (Len > hAtc->Size - hAtc->RxIndex) {
        HAL_DBG_TRACE_WARNING("Len (%d) exceeds available buffer space (%d). Truncating to fit.\n", Len, hAtc->Size - hAtc->RxIndex);
        Len = hAtc->Size - hAtc->RxIndex;
    }


    //HAL_DBG_TRACE_INFO("Copying data to pReadBuff at index %d. Data: %.*s\n", hAtc->RxIndex, Len, hAtc->pRxBuff);


    memcpy(&hAtc->pReadBuff[hAtc->RxIndex], hAtc->pRxBuff, Len);
    hAtc->RxIndex += Len;

    //HAL_DBG_TRACE_INFO("Updated RxIndex: %d. Current buffer content: %s\n", hAtc->RxIndex, hAtc->pReadBuff);
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
        char* endptr;
        uint32_t frequency = strtoul(param1, &endptr, 10);  // 使用 strtoul 转换字符串为 uint32_t
        if (*endptr != '\0') {
            HAL_DBG_TRACE_INFO("Invalid frequency parameter: non-numeric character found.\n");
        } else {
            HAL_DBG_TRACE_INFO("Frequency set to: %u Hz\n", frequency);
            ATC_M_RF_FREQ_IN_HZ = frequency;
        }
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
				ATC_M_TX_OUTPUT_POWER_DBM =power;
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


//SF

void AT_SF_Callback(char* param1, char* param2) {
    if (param1 != NULL) {
        int param = atoi(param1);  // 将超参数字符串转换为整数
        HAL_DBG_TRACE_INFO("Parameter set to: %d\n", param);
        
        // 在这里进行超参数设置的具体操作
        switch (param) {
            case 5:
                ATC_M_LORA_SF = LR11XX_RADIO_LORA_SF5;
                break;
            case 6:
                ATC_M_LORA_SF = LR11XX_RADIO_LORA_SF6;
                break;
            case 7:
                ATC_M_LORA_SF = LR11XX_RADIO_LORA_SF7;
                break;
            case 8:
                ATC_M_LORA_SF = LR11XX_RADIO_LORA_SF8;
                break;
            case 9:
                ATC_M_LORA_SF = LR11XX_RADIO_LORA_SF9;
                break;
            case 10:
                ATC_M_LORA_SF = LR11XX_RADIO_LORA_SF10;
                break;
            case 11:
                ATC_M_LORA_SF = LR11XX_RADIO_LORA_SF11;
                break;
            case 12:
                ATC_M_LORA_SF = LR11XX_RADIO_LORA_SF12;
                break;
            default:
                HAL_DBG_TRACE_INFO("Invalid parameter value.\n");
                return;
        }
    } else {
        HAL_DBG_TRACE_INFO("Invalid parameter.\n");
    }
}

//BW

void AT_BW_Callback(char* param1, char* param2) {
    if (param1 != NULL) {
        int param = atoi(param1);  // 将超参数字符串转换为整数
        HAL_DBG_TRACE_INFO("Band width set to: %d\n", param);
        
        // 在这里进行超参数设置的具体操作
        switch (param) {
            case 10:
                ATC_M_LORA_BW = LR11XX_RADIO_LORA_BW_10;
                break;
            case 15:
                ATC_M_LORA_BW = LR11XX_RADIO_LORA_BW_15;
                break;
            case 20:
                ATC_M_LORA_BW = LR11XX_RADIO_LORA_BW_20;
                break;
            case 31:
                ATC_M_LORA_BW = LR11XX_RADIO_LORA_BW_31;
                break;
            case 41:
                ATC_M_LORA_BW = LR11XX_RADIO_LORA_BW_41;
                break;
            case 62:
                ATC_M_LORA_BW = LR11XX_RADIO_LORA_BW_62;
                break;
            case 125:
                ATC_M_LORA_BW = LR11XX_RADIO_LORA_BW_125;
                break;
            case 250:
                ATC_M_LORA_BW = LR11XX_RADIO_LORA_BW_250;
                break;
            case 500:
                ATC_M_LORA_BW = LR11XX_RADIO_LORA_BW_500;
                break;
            case 200:
                ATC_M_LORA_BW = LR11XX_RADIO_LORA_BW_200;
                break;
            case 400:
                ATC_M_LORA_BW = LR11XX_RADIO_LORA_BW_400;
                break;
            case 800:
                ATC_M_LORA_BW = LR11XX_RADIO_LORA_BW_800;
                break;
            default:
                HAL_DBG_TRACE_INFO("Invalid parameter value.\n");
                return;
        }
    } else {
        HAL_DBG_TRACE_INFO("Invalid parameter.\n");
    }
}

//CR

void AT_CR_Callback(char* param1, char* param2) {
    if (param1 != NULL) {
        int param = atoi(param1);  // 将超参数字符串转换为整数
        HAL_DBG_TRACE_INFO("Parameter set to: %d\n", param);
        // 在这里进行超参数设置的具体操作
        switch (param) {
            case 0:
                ATC_M_LORA_CR = LR11XX_RADIO_LORA_NO_CR;
                break;
            case 1:
                ATC_M_LORA_CR = LR11XX_RADIO_LORA_CR_4_5;
                break;
            case 2:
                ATC_M_LORA_CR = LR11XX_RADIO_LORA_CR_4_6;
                break;
            case 3:
                ATC_M_LORA_CR = LR11XX_RADIO_LORA_CR_4_7;
                break;
            case 4:
                ATC_M_LORA_CR = LR11XX_RADIO_LORA_CR_4_8;
                break;
            case 5:
                ATC_M_LORA_CR = LR11XX_RADIO_LORA_CR_LI_4_5;
                break;
            case 6:
                ATC_M_LORA_CR = LR11XX_RADIO_LORA_CR_LI_4_6;
                break;
            case 7:
                ATC_M_LORA_CR = LR11XX_RADIO_LORA_CR_LI_4_8;
                break;
            default:
                HAL_DBG_TRACE_INFO("Invalid parameter value.\n");
                return;
        }
    } else {
        HAL_DBG_TRACE_INFO("Invalid parameter.\n");
    }
}



void AT_Help_Callback(char* param1, char* param2) {
    HAL_DBG_TRACE_INFO("AT+HELP received.\n");
    // 在这里添加帮助信息
    HAL_DBG_TRACE_INFO("AT+FREQ=<frequency> : Set the frequency in Hz (e.g., AT+FREQ=868000000)\n");
    HAL_DBG_TRACE_INFO("AT+POWER=<power> : Set the power in dBm (e.g., AT+POWER=14)\n");
    HAL_DBG_TRACE_INFO("AT+PARAM=<param> : Set a parameter (Not work but available to call)\n");
    HAL_DBG_TRACE_INFO("AT+PER : Perform PER measurement (Not work but available to call)\n");
    HAL_DBG_TRACE_INFO("AT+START : Start the operation\n");
    HAL_DBG_TRACE_INFO("AT+SF=<SF> : Set the Spreading Factor (e.g., AT+SF=7 for SF7)\n");
    HAL_DBG_TRACE_INFO("AT+BW=<BW> : Set the Bandwidth (e.g., AT+BW=125 for 125 kHz)\n");
    HAL_DBG_TRACE_INFO("AT+CR=<CR> : Set the Coding Rate (e.g., AT+CR=1 for 4/5 coding rate)\n");
    HAL_DBG_TRACE_INFO("AT+TRSW=<param> : Set the TX/RX switch parameter\n");
    HAL_DBG_TRACE_INFO("AT+CWSW=<param> : Set the CW switch parameter\n");
    HAL_DBG_TRACE_INFO("Available Spreading Factors (SF):\n");
    HAL_DBG_TRACE_INFO("  5: LR11XX_RADIO_LORA_SF5\n");
    HAL_DBG_TRACE_INFO("  6: LR11XX_RADIO_LORA_SF6\n");
    HAL_DBG_TRACE_INFO("  7: LR11XX_RADIO_LORA_SF7\n");
    HAL_DBG_TRACE_INFO("  8: LR11XX_RADIO_LORA_SF8\n");
    HAL_DBG_TRACE_INFO("  9: LR11XX_RADIO_LORA_SF9\n");
    HAL_DBG_TRACE_INFO(" 10: LR11XX_RADIO_LORA_SF10\n");
    HAL_DBG_TRACE_INFO(" 11: LR11XX_RADIO_LORA_SF11\n");
    HAL_DBG_TRACE_INFO(" 12: LR11XX_RADIO_LORA_SF12\n");
    HAL_DBG_TRACE_INFO("Available Bandwidths (BW):\n");
    HAL_DBG_TRACE_INFO("   10.42 kHz\n");
    HAL_DBG_TRACE_INFO("   15.63 kHz\n");
    HAL_DBG_TRACE_INFO("   20.83 kHz\n");
    HAL_DBG_TRACE_INFO("   31.25 kHz\n");
    HAL_DBG_TRACE_INFO("  41.67 kHz\n");
    HAL_DBG_TRACE_INFO("   62.50 kHz\n");
    HAL_DBG_TRACE_INFO("   125.00 kHz\n");
    HAL_DBG_TRACE_INFO("   250.00 kHz\n");
    HAL_DBG_TRACE_INFO("   500.00 kHz\n");
    HAL_DBG_TRACE_INFO("  203.00 kHz (2G4 and compatible with LR112x chips only)\n");
    HAL_DBG_TRACE_INFO("  406.00 kHz (2G4 and compatible with LR112x chips only)\n");
    HAL_DBG_TRACE_INFO("  812.00 kHz (2G4 and compatible with LR112x chips only)\n");
    HAL_DBG_TRACE_INFO("Available Coding Rates (CR):\n");
    HAL_DBG_TRACE_INFO("  0: No coding rate\n");
    HAL_DBG_TRACE_INFO("  1: 4/5\n");
    HAL_DBG_TRACE_INFO("  2: 4/6\n");
    HAL_DBG_TRACE_INFO("  3: 4/7\n");
    HAL_DBG_TRACE_INFO("  4: 4/8\n");
    HAL_DBG_TRACE_INFO("  5: LI 4/5\n");
    HAL_DBG_TRACE_INFO("  6: LI 4/6\n");
    HAL_DBG_TRACE_INFO("  7: LI 4/8\n");
}

void AT_START_event_callback(char* param1, char* param2){
    HAL_DBG_TRACE_INFO("AT+START received.\n");
    // 在这里添加启动操作
    HAL_DBG_TRACE_INFO("Start the operation...\n");
    AT_start_flag = 1;
}

void AT_TRSW_event_callback(char* param1, char* param2){
    if (param1 != NULL) {
        int param = atoi(param1);  
        HAL_DBG_TRACE_INFO("Parameter set to: %d\n", param);
        // 在这里进行参数设置的具体操作
				ATC_M_TXRX_SWITCH =param;
    } else {
        HAL_DBG_TRACE_INFO("Invalid parameter.\n");
    }
}

void AT_NB_FRAME_event_callback(char* param1, char* param2){
    if (param1 != NULL) {
        int param = atoi(param1);  
        HAL_DBG_TRACE_INFO("NB_FRAME set to: %d\n", param);
        // 在这里进行参数设置的具体操作
				ATC_M_NB_FRAME =param;
    } else {
        HAL_DBG_TRACE_INFO("Invalid parameter.\n");
    }
}

void AT_CW_event_callback(char* param1, char* param2){
    if (param1 != NULL) {
        int param = atoi(param1);  
        HAL_DBG_TRACE_INFO("CW Switch set to: %d\n", param);
        // 在这里进行参数设置的具体操作
				ATC_M_CW_SWITCH =param;
    } else {
        HAL_DBG_TRACE_INFO("Invalid parameter.\n");
    }
}

void AT_RX_BOOST_event_callback(char* param1, char* param2){
    if (param1 != NULL) {
        int param = atoi(param1);  
        HAL_DBG_TRACE_INFO("RX_BOOST Switch set to: %d\n", param);
        // 在这里进行参数设置的具体操作
				ATC_M_LORA_RX_BOOST =param;
    } else {
        HAL_DBG_TRACE_INFO("Invalid parameter.\n");
    }
}

void AT_SLEEP_event_callback(char* param1, char* param2){
    if (param1 != NULL) {
        int param = atoi(param1);  
        HAL_DBG_TRACE_INFO("SLEEP Time set to: %d\n", param);
        // 在这里进行参数设置的具体操作
				ATC_M_LORA_SLEEP=param;
    } else {
        HAL_DBG_TRACE_INFO("Invalid parameter.\n");
    }
}