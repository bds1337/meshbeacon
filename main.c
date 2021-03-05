/* Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

/*
 * для прошивки донгла используй BOARD_PCA10059 вместо BOARD_PCA1005
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h" //added
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_ble_scan.h" // added 
#include "app_timer.h"
#include "ble_nus_c.h"
#include "app_util_platform.h"

#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "mesh_main.h"
#include "mesh_app_utils.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define APP_BLE_CONN_CFG_TAG        1                                   /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO       3                                   /**< BLE observer priority of the application. There is no need to modify this value. */

#define SCAN_INTERVAL                   0x00A0                              /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                     0x0050                              /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_DURATION                   0x0000                              /**< Timout when scanning. 0x0000 disables timeout. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */


BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< Database discovery module instance. */
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                       /**< Scanning module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);
APP_TIMER_DEF(m_sst_id); // таймер между командами измерения давления и пульса

extern bool m_device_provisioned;

//Состояния таймера отпраки данных по мешу
#define SST_DISCONNECTED                0
#define SST_PULSE_MEASURE_START         1 
#define SST_PULSE_MEASURE_STOP          2
#define SST_PRESSURE_MEASURE_START      3
#define SST_PRESSURE_MEASURE_STOP       4
#define SST_IDLE                        5
//Время ожидания таймера (для проведения измерений и отпраки команд старт/стоп)
#define SST_MEASURE_TIME                60000
#define SST_SENDCMD_TIME                2000 
#define SST_IDLE_TIME                   10000
// Команды для измерения пульса и давления 
#define WR4119_CMD_LENGHT               0x0007 //12
static const uint8_t wr4119_cmd_pulse_start[7]    = { 0xAB, 0x00, 0x04, 0xFF, 0x31, 0x09, 0x01 }; 
static const uint8_t wr4119_cmd_pulse_stop[7]     = { 0xAB, 0x00, 0x04, 0xFF, 0x31, 0x09, 0x00 };
static const uint8_t wr4119_cmd_pressure_start[7] = { 0xAB, 0x00, 0x04, 0xFF, 0x31, 0x21, 0x01 };
static const uint8_t wr4119_cmd_pressure_stop[7]  = { 0xAB, 0x00, 0x04, 0xFF, 0x31, 0x21, 0x00 };
static const uint8_t wr4119_cmd_recv_type[5]      = { 0xAB, 0x00, 0x05, 0xFF, 0x31 };

typedef struct
{
    struct
    {
        uint8_t pressure_up;
        uint8_t pressure_down;
        bool is_ready;
    } pressure;
    struct
    {
        uint8_t pulse;
        bool is_ready;
    } pulse;
} smartband_data_t;

static smartband_data_t smartband_data;

static uint16_t m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static uint8_t sst_context  = SST_DISCONNECTED;
static bool is_connected = false;

/**< Адрес для подключения к устройсту-браслету */
static ble_gap_addr_t m_target_periph_addr =
{
    .addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
//    .addr      = {0x50, 0x87, 0xF8, 0x8F, 0xCC, 0xEF} // reversed from nrf connect
    .addr      = {0xFA, 0x04, 0xBF, 0x50, 0x7A, 0xE2}
};

/**< Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t m_scan_param =
{
    .active        = 0x01,
#if (NRF_SD_BLE_API_VERSION > 7)
    .interval_us   = NRF_BLE_SCAN_SCAN_INTERVAL * UNIT_0_625_MS,
    .window_us     = NRF_BLE_SCAN_SCAN_WINDOW * UNIT_0_625_MS,
#else
    .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,
    .window        = NRF_BLE_SCAN_SCAN_WINDOW,
#endif
        .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL, //BLE_GAP_SCAN_FP_WHITELIST BLE_GAP_SCAN_FP_ACCEPT_ALL
        .timeout       = 0, //SCAN_DURATION_WHITELIST
        .scan_phys     = BLE_GAP_PHY_1MBPS,
};

static bool m_memory_access_in_progress;
static bsp_indication_t led_state;



/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for handling the Nordic UART Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nus_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    if (!m_device_provisioned)
    {
        NRF_LOG_INFO("Device not provisioned.");
        return;
    }

    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Scan start");
}

/**@brief Команда отправки сообщения через сервич Nordic UART (NUS)
 *
 * @param[in]   p_data	    Данные для отправки.
 * @param[in]   data_len    Длина данных.
 */
static void ble_nus_send(const uint8_t * p_data, uint16_t data_len)
{
    ret_code_t ret_val;

    do
    {
        ret_val = ble_nus_c_string_send(&m_ble_nus_c, p_data, data_len);
        if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY) && (ret_val != NRF_ERROR_INVALID_STATE))
        {
            NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", ret_val);
            APP_ERROR_CHECK(ret_val);
        }
    } while (ret_val == NRF_ERROR_BUSY);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;
    bool healthdata;
    uint8_t recived_type;

    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete.");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Nordic UART Service.");
            
            sst_context = SST_PULSE_MEASURE_START;
            err_code = app_timer_start(m_sst_id, APP_TIMER_TICKS(SST_SENDCMD_TIME), NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
        {
            healthdata = true;
            
            //NRF_LOG_RAW_HEXDUMP_INFO(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);

            if ( p_ble_nus_evt->data_len < 6 )
                return;            

            for ( int i = 0; i < 5; i++ )
            {
                if ( wr4119_cmd_recv_type[i] != p_ble_nus_evt->p_data[i] )
                {
                    healthdata = false;
                    break;
                }
            }
            if ( healthdata == true )
            {
                recived_type = p_ble_nus_evt->p_data[5];
                if ( recived_type == 0x09 )
                {
                    smartband_data.pulse.pulse = p_ble_nus_evt->p_data[6];
                    smartband_data.pulse.is_ready = true;
                    NRF_LOG_DEBUG("GOT pulse: %x", p_ble_nus_evt->p_data[6]);
                }
                if ( recived_type == 0x21 )
                {
                    smartband_data.pressure.pressure_up = p_ble_nus_evt->p_data[6];
                    smartband_data.pressure.pressure_down = p_ble_nus_evt->p_data[7];
                    smartband_data.pressure.is_ready = true;
                    NRF_LOG_DEBUG("GOT pressure: %x", p_ble_nus_evt->p_data[6]);
                    NRF_LOG_DEBUG("GOT pressure: %x", p_ble_nus_evt->p_data[7]);
                }
            }
         }  break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            scan_start();
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;

    init.evt_handler   = ble_nus_c_evt_handler;
    init.error_handler = nus_error_handler;
    init.p_gatt_queue  = &m_ble_gatt_queue;

    err_code = ble_nus_c_init(&m_ble_nus_c, &init);
    APP_ERROR_CHECK(err_code);
   
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connection established: %02x", p_gap_evt->conn_handle);
            NRF_LOG_INFO("Connection established: %02x%02x%02x%02x%02x%02x",
                p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[0], 
                p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[1], 
                p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[2], 
                p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[3], 
                p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[4], 
                p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[5]
            );

            err_code = ble_nus_c_handles_assign(&m_ble_nus_c,
                                                p_gap_evt->conn_handle,
                                                NULL);
            APP_ERROR_CHECK(err_code);


            err_code = ble_db_discovery_start(&m_db_disc,
                                              p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);

            is_connected = true;
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

        } break;

        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("LBS central link 0x%x disconnected (reason: 0x%x)",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            is_connected = false;
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GAP_EVT_ADV_REPORT:
        {
            NRF_LOG_RAW_HEXDUMP_INFO (m_scan.scan_buffer.p_data, m_scan.scan_buffer.len);

        } break;
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling Scaning events.
 *
 * @param[in]   p_scan_evt   Scanning event.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
        } break;

         case NRF_BLE_SCAN_EVT_CONNECTED:
         {
              ble_gap_evt_connected_t const * p_connected =
                               p_scan_evt->params.connected.p_connected;
             // Scan is automatically stopped by the connection.
             NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                      p_connected->peer_addr.addr[0],
                      p_connected->peer_addr.addr[1],
                      p_connected->peer_addr.addr[2],
                      p_connected->peer_addr.addr[3],
                      p_connected->peer_addr.addr[4],
                      p_connected->peer_addr.addr[5]
                      );
         } break;
       
        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
            NRF_LOG_INFO("Scan timed out.");
            scan_start();
        } break; 
        default:
          break;
    }
}

/**@brief Timeout handler
 */
static void sst_handler(void * p_context)
{
    rtls_set_params_t set_params;
    
    ret_code_t err_code;
    NRF_LOG_INFO("SST: %d", sst_context);
    if (!is_connected)
    {
        NRF_LOG_INFO("device disconnected");
        sst_context = SST_DISCONNECTED;
        return;
    }
    switch(sst_context)
    {
        case SST_PULSE_MEASURE_START:
        {
            sst_context = SST_PULSE_MEASURE_STOP;
            ble_nus_send(wr4119_cmd_pulse_start, WR4119_CMD_LENGHT);
            err_code = app_timer_start(m_sst_id, APP_TIMER_TICKS(SST_MEASURE_TIME), NULL); 
            APP_ERROR_CHECK(err_code);
        } break;
        case SST_PULSE_MEASURE_STOP:
        {
            sst_context = SST_PRESSURE_MEASURE_START;
            ble_nus_send(wr4119_cmd_pulse_stop, WR4119_CMD_LENGHT);
            err_code = app_timer_start(m_sst_id, APP_TIMER_TICKS(SST_SENDCMD_TIME), NULL); 
            APP_ERROR_CHECK(err_code);
        } break;
        case SST_PRESSURE_MEASURE_START:
        {
            if ( smartband_data.pulse.is_ready )
            {
                // данные пульса уже получены, отправляю в меш
                set_params.pulse = smartband_data.pulse.pulse;
                set_params.type = RTLS_PULSE_TYPE;
                smartband_data.pulse.is_ready = false;
                NRF_LOG_INFO("SEND pulse      %02x", set_params.pulse);
                mesh_main_send_message(&set_params);
            }
            sst_context = SST_PRESSURE_MEASURE_STOP;
            ble_nus_send(wr4119_cmd_pressure_start, WR4119_CMD_LENGHT);
            err_code = app_timer_start(m_sst_id, APP_TIMER_TICKS(SST_MEASURE_TIME), NULL);
            APP_ERROR_CHECK(err_code);
        } break;
        case SST_PRESSURE_MEASURE_STOP:
        {
            sst_context = SST_IDLE;
            ble_nus_send(wr4119_cmd_pressure_stop, WR4119_CMD_LENGHT);
            err_code = app_timer_start(m_sst_id, APP_TIMER_TICKS(SST_SENDCMD_TIME), NULL);
            APP_ERROR_CHECK(err_code);
        } break;
        case SST_IDLE:
        {
            if ( smartband_data.pressure.is_ready )
            {
                // данные получены, отправляю в меш
                set_params.pressure.pressure_down = smartband_data.pressure.pressure_down;
                set_params.pressure.pressure_up = smartband_data.pressure.pressure_up;
                set_params.type = RTLS_PRESSURE_TYPE;
                smartband_data.pressure.is_ready = false;
                NRF_LOG_INFO("SEND pressure d %02x", set_params.pressure.pressure_down);
                NRF_LOG_INFO("SEND pressure u %02x", set_params.pressure.pressure_up);
                mesh_main_send_message(&set_params);
            }
            sst_context = SST_PULSE_MEASURE_START;
            err_code = app_timer_start(m_sst_id, APP_TIMER_TICKS(SST_IDLE_TIME), NULL); 
            APP_ERROR_CHECK(err_code);
        } break;
        default: 
            break;
    }
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Ble NUS max data length set //to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    //err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code); 
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}

/**@brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_sst_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                sst_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Инициализация сканирования устройств. 
 * is_conn - true = подкл. к устройству
 */
static void scan_init(bool is_conn)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = is_conn;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;
    init_scan.p_scan_param     = &m_scan_param;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);
   
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_ADDR_FILTER, m_target_periph_addr.addr);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_ble_scan_filters_enable(&m_scan,
                                           NRF_BLE_SCAN_ADDR_FILTER,
                                           false);
    APP_ERROR_CHECK(err_code);
    
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    nrf_pwr_mgmt_run();
}

void bsp_event_handler(bsp_event_t event)
{
    switch(event)
    {
        case BSP_EVENT_KEY_0:
        case BSP_EVENT_KEY_1:
        case BSP_EVENT_KEY_2:
        case BSP_EVENT_KEY_3:
            mesh_main_button_event_handler(event-BSP_EVENT_KEY_0);
            break;
        default:
            break;
    }
}

static void buttons_leds_init()
{
    bsp_event_t startup_event;
    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);
    
    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    led_state = BSP_INDICATE_IDLE;
}


/**@brief Application main function.
 */
int main(void)
{
    // Initialize.
    log_init();
    timers_init();
    buttons_leds_init();
    db_discovery_init();
    power_management_init();
    ble_stack_init();
    gatt_init();
    services_init();
    scan_init(true); // true - ччтобы подключаться
    
    conn_params_init();
    mesh_main_initialize();
    
    // Start execution.
    NRF_LOG_INFO("[main] initialization completed");
    NRF_LOG_FLUSH();

    scan_start();
    mesh_main_start();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
