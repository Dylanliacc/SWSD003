/*!
 * @file      main_per.c
 *
 * @brief     Packet Error Rate (PER) example for LR11xx chip
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdio.h>
#include <string.h>

#include "apps_common.h"
#include "apps_utilities.h"
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
#include "lr11xx_system.h"
#include "main_per.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_dbg_trace.h"
#include "uart_init.h"
#include "atc.h"
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief LR11xx interrupt mask used by the application
 */
#define IRQ_MASK                                                                                               \
    ( LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_TIMEOUT |                      \
      LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED | LR11XX_SYSTEM_IRQ_HEADER_ERROR | LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR | \
      LR11XX_SYSTEM_IRQ_CRC_ERROR )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#if( RECEIVER == 1 )
const char* mode = "Receiver";
#else
const char* mode = "Transmitter";
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lr11xx_hal_context_t* context;

static uint8_t buffer[PAYLOAD_LENGTH];

static uint16_t nb_ok            = 0;
static uint16_t nb_rx_timeout    = 0;
static uint16_t nb_rx_error      = 0;
static uint16_t nb_fsk_len_error = 0;

static uint8_t rolling_counter = 0;

static uint16_t per_index      = 0;
static bool     first_pkt_flag = false;

static uint8_t  per_msg[PAYLOAD_LENGTH];
static uint32_t rx_timeout = RX_TIMEOUT_VALUE;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Handle reception failure for PER example
 *
 * @param [in] failure_counter pointer to the counter for each type of reception failure
 */
static void per_reception_failure_handling( uint16_t* failure_counter );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */



int main( void )
{
    smtc_hal_mcu_init( );
    apps_common_shield_init( );
    uart_init();
    HAL_DBG_TRACE_INFO( "===== LR11xx PER example - %s =====\n\n", mode );
		
    apps_common_print_sdk_driver_version( );
    // 等待并处理AT指令，直到满足某个条件
    bool at_command_ready = false;
    while (!at_command_ready) {
        at_command_ready = main_loop();  // 处理AT指令
    }
		lr11xx_system_sleep_cfg_t sleep_cfg = {true, true};

    context = apps_common_lr11xx_get_context( );
		
		if(ATC_M_LORA_SLEEP ==1 ){
			lr11xx_system_set_sleep((void* ) context ,sleep_cfg,
                                         10000);
		}
		//lr11xx_radio_cfg_rx_boosted( context, true);
    apps_common_lr11xx_system_init( ( void* ) context );
    apps_common_lr11xx_fetch_and_print_version( ( void* ) context );
    apps_common_lr11xx_radio_init( ( void* ) context );

		if(ATC_M_CW_SWITCH==1){
			    apps_common_lr11xx_handle_pre_tx( );
					ASSERT_LR11XX_RC( lr11xx_radio_set_tx_cw( context ) );
					while( 1 ){}
		}else{
    ASSERT_LR11XX_RC( lr11xx_system_set_dio_irq_params( context, IRQ_MASK, 0 ) );
    ASSERT_LR11XX_RC( lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK ) );

    for( int i = 1; i < PAYLOAD_LENGTH; i++ )
    {
        buffer[i] = i;
    }
    // Adjust reception timeout taking into account time on air
    rx_timeout += get_time_on_air_in_ms( );

	if(ATC_M_TXRX_SWITCH ==1){
    apps_common_lr11xx_handle_pre_rx( );
    ASSERT_LR11XX_RC( lr11xx_radio_set_rx( context, rx_timeout ) );
    memcpy( per_msg, &buffer[1], PAYLOAD_LENGTH - 1 );
	}else{
    buffer[0] = 0;
    ASSERT_LR11XX_RC( lr11xx_regmem_write_buffer8( context, buffer, PAYLOAD_LENGTH ) );
    apps_common_lr11xx_handle_pre_tx( );
    ASSERT_LR11XX_RC( lr11xx_radio_set_tx( context, 0 ) );
	}

		
    while( per_index < ATC_M_NB_FRAME )
    {
        apps_common_lr11xx_irq_process( context, IRQ_MASK );
    }
		
    if( per_index > ATC_M_NB_FRAME )  // The last validated packet should not be counted in this case
    {
        nb_ok--;
    }
    /* Display PER*/
    HAL_DBG_TRACE_PRINTF( "PER = %d \n", 100 - ( ( nb_ok * 100 ) / ATC_M_NB_FRAME ) );

    HAL_DBG_TRACE_PRINTF( "Final PER index: %d \n", per_index );
    HAL_DBG_TRACE_PRINTF( "Valid reception amount: %d \n", nb_ok );
    HAL_DBG_TRACE_PRINTF( "Timeout reception amount: %d \n", nb_rx_timeout );
    HAL_DBG_TRACE_PRINTF( "CRC Error reception amount: %d \n", nb_rx_error );
    if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_GFSK )
    {
        HAL_DBG_TRACE_PRINTF( "FSK Length Error reception amount: %d \n", nb_fsk_len_error );
    }
	}
}

void on_tx_done( void )
{
    apps_common_lr11xx_handle_post_tx( );

    LL_mDelay( TX_TO_TX_DELAY_IN_MS );

    buffer[0]++;
    HAL_DBG_TRACE_INFO( "Counter value: %d\n", buffer[0] );
    ASSERT_LR11XX_RC( lr11xx_regmem_write_buffer8( context, buffer, 20 ) );

    apps_common_lr11xx_handle_pre_tx( );
    ASSERT_LR11XX_RC( lr11xx_radio_set_tx( context, 0 ) );
}

// void on_rx_done( void )
// {
//     uint8_t size;

//     apps_common_lr11xx_handle_post_rx( );

//     apps_common_lr11xx_receive( context, buffer, PAYLOAD_LENGTH, &size );

//     // if( memcmp( &buffer[1], per_msg, PAYLOAD_LENGTH - 1 ) == 0 )
//     // {
//         // Let's start counting after the first received packet
//         if( first_pkt_flag == true )
//         {
//             uint8_t rolling_counter_gap = ( uint8_t ) ( buffer[0] - rolling_counter );
//             nb_ok++;
//             per_index += rolling_counter_gap;
//             if( rolling_counter_gap > 1 )
//             {
//                 HAL_DBG_TRACE_WARNING( "%d packet(s) missed\n", ( rolling_counter_gap - 1 ) );
//             }
//             rolling_counter = buffer[0];
//         }
//         else
//         {
//             first_pkt_flag  = true;
//             rolling_counter = buffer[0];
//         }
//         HAL_DBG_TRACE_INFO( "Counter value: %d, PER index: %d\n", buffer[0], per_index );
//     // }else{
//     //     HAL_DBG_TRACE_INFO( "break at 01\n");
//     // }
//     if( per_index < NB_FRAME )  // Re-start Rx only if the expected number of frames is not reached
//     {
//         apps_common_lr11xx_handle_pre_rx( );
//         ASSERT_LR11XX_RC( lr11xx_radio_set_rx( context, rx_timeout ) );
//     }
//     else{
//         HAL_DBG_TRACE_INFO( "break at 02\n");
//     }
// }


//void on_rx_done( void )
//{
//    uint8_t size;

//    // Post reception handling
//    apps_common_lr11xx_handle_post_rx();

//    // Receive the buffer content
//    apps_common_lr11xx_receive(context, buffer, PAYLOAD_LENGTH, &size);

//    // Log received buffer content and size
//    HAL_DBG_TRACE_INFO("Received buffer content: ");
//    for (int i = 0; i < size; i++) {
//        HAL_DBG_TRACE_PRINTF("%02X ", buffer[i]);
//    }
//    HAL_DBG_TRACE_INFO("\n");
//    HAL_DBG_TRACE_PRINTF("Received size: %d\n", size);

//    // Check if the first packet flag is set
//    if (first_pkt_flag == true) {
//        uint8_t rolling_counter_gap = (uint8_t)(buffer[0] - rolling_counter);
//        if (rolling_counter_gap > 0) {
//            nb_ok++;
//            per_index += rolling_counter_gap;
//            HAL_DBG_TRACE_PRINTF("Rolling counter gap: %d\n", rolling_counter_gap);

//            if (rolling_counter_gap > 1) {
//                HAL_DBG_TRACE_WARNING("%d packet(s) missed\n", (rolling_counter_gap - 1));
//            }

//            rolling_counter = buffer[0];
//        } else {
//            HAL_DBG_TRACE_WARNING("Unexpected rolling counter gap: %d\n", rolling_counter_gap);
//        }
//    } else {
//        // If this is the first received packet
//        first_pkt_flag = true;
//        rolling_counter = buffer[0];
//        HAL_DBG_TRACE_INFO("First packet received. Rolling counter initialized to: %d\n", rolling_counter);
//    }

//    HAL_DBG_TRACE_INFO("Counter value: %d, PER index: %d\n", buffer[0], per_index);

//    // Check if the PER test is complete
//    if (per_index < NB_FRAME) {
//        // Restart reception for the next packet
//        apps_common_lr11xx_handle_pre_rx();
//        ASSERT_LR11XX_RC(lr11xx_radio_set_rx(context, rx_timeout));
//    } else {
//        HAL_DBG_TRACE_INFO("PER test complete. Stopping RX.\n");
//    }
//}

void on_rx_done( void )
{
    uint8_t size;

    // Post reception handling
    apps_common_lr11xx_handle_post_rx();

    // Receive the buffer content
    apps_common_lr11xx_receive(context, buffer, PAYLOAD_LENGTH, &size);

    // Log received buffer content and size
    HAL_DBG_TRACE_INFO("Received buffer content: Jumped");
    //for (int i = 0; i < size; i++) {
      //  HAL_DBG_TRACE_PRINTF("%02X ", buffer[i]);
    //}
    HAL_DBG_TRACE_INFO("\n");
    HAL_DBG_TRACE_PRINTF("Received size: %d\n", size);

    // Check received data
    if (size == PAYLOAD_LENGTH) {
        // Increment the valid packet counter
        nb_ok++;
        HAL_DBG_TRACE_INFO("Valid packet received. Total valid: %d\n", nb_ok);
    } else {
        HAL_DBG_TRACE_WARNING("Invalid packet size. Expected: %d, Received: %d\n", PAYLOAD_LENGTH, size);
    }

    // Increment the PER index
    per_index++;

    // Log PER calculation
    HAL_DBG_TRACE_PRINTF("PER index: %d\n", per_index);

    // Check if the PER test is complete
    if (per_index >= ATC_M_NB_FRAME) {
        HAL_DBG_TRACE_INFO("PER test complete.\n");
        // Calculate PER
        uint16_t per = 100 - ((nb_ok * 100) / ATC_M_NB_FRAME);
        HAL_DBG_TRACE_PRINTF("Final PER: %d%%\n", per);
    } else {
        // Restart reception for the next packet
        apps_common_lr11xx_handle_pre_rx();
        ASSERT_LR11XX_RC(lr11xx_radio_set_rx(context, rx_timeout));
    }
}

void on_rx_timeout( void )
{
    per_reception_failure_handling( &nb_rx_timeout );
}

void on_rx_crc_error( void )
{
    per_reception_failure_handling( &nb_rx_error );
}

void on_fsk_len_error( void )
{
    per_reception_failure_handling( &nb_fsk_len_error );
}

static void per_reception_failure_handling( uint16_t* failure_counter )
{
    apps_common_lr11xx_handle_post_rx( );

    // Let's start counting after the first received packet
    if( first_pkt_flag == true )
    {
        ( *failure_counter )++;
    }

    apps_common_lr11xx_handle_pre_rx( );
    ASSERT_LR11XX_RC( lr11xx_radio_set_rx( context, rx_timeout ) );
}