/* Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
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

#include <stdint.h>
#include <string.h>

/* HAL */
#include "boards.h"
#include "app_timer.h"

/* Core */
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "nrf_sdh_soc.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "rtls_client.h"
#include "rtls_rssi_client.h"

/* Logging and RTT */
#include "nrf_log.h"

/* Example specific includes */
#include "app_config.h"
//#include "nrf_mesh_config_examples.h"
#include "nrf_mesh_config_app.h"
#include "example_common.h"
//#include "app_db.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/
#define APP_UNACK_MSG_REPEAT_COUNT   (2)

#define MESH_SOC_OBSERVER_PRIO  0

/* Controls if the model instance should force all mesh messages to be segmented messages. */
#define APP_FORCE_SEGMENTATION          (false)
/* Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_MIC_SIZE                    (NRF_MESH_TRANSMIC_SIZE_SMALL)
/* Delay value used by the OnOff client for sending OnOff Set messages. */
#define APP_ONOFF_DELAY_MS              (50)
/* Transition time value used by the OnOff client for sending OnOff Set messages. */
#define APP_ONOFF_TRANSITION_TIME_MS    (100)


/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
static void app_rtls_rssi_client_status_cb(const rtls_rssi_client_t * p_self, 
                                                        const access_message_rx_meta_t * p_meta);
static void app_rtls_rssi_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status);

static void app_rtls_client_status_cb(const rtls_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const rtls_status_params_t * p_in);

static void app_rtls_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status);

/*****************************************************************************
 * Static variables
 *****************************************************************************/
static rtls_rssi_client_t m_rssi_clients[1];
const rtls_rssi_client_callbacks_t rssi_client_cbs =
{
    .rtls_status_cb = app_rtls_rssi_client_status_cb,
    .ack_transaction_status_cb = app_rtls_rssi_client_transaction_status_cb
};

static rtls_client_t          m_clients[CLIENT_MODEL_INSTANCE_COUNT]; //CLIENT_MODEL_INSTANCE_COUNT
bool                   m_device_provisioned;

const rtls_client_callbacks_t client_cbs =
{
    .rtls_status_cb = app_rtls_client_status_cb,
    .ack_transaction_status_cb = app_rtls_client_transaction_status_cb
};


static void mesh_soc_evt_handler(uint32_t evt_id, void * p_context)
{
    nrf_mesh_on_sd_evt(evt_id);
}

NRF_SDH_SOC_OBSERVER(m_mesh_soc_observer, MESH_SOC_OBSERVER_PRIO, mesh_soc_evt_handler, NULL);

static void unicast_address_print(void)
{
    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);
}

static void provisioning_complete_cb(void)
{
    NRF_LOG_INFO("Successfully provisioned\n");

    unicast_address_print();
}

/* Client model interface: Process the received status message in this callback */
static void app_rtls_rssi_client_status_cb(const rtls_rssi_client_t * p_self, 
                                                        const access_message_rx_meta_t * p_meta)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Data was delivered from 0x%x to 0x%x.\n", p_meta->src.value, p_meta->dst.value);
}

/* Acknowledged transaction status callback, if acknowledged transfer fails, application can
* determine suitable course of action (e.g. re-initiate previous transaction) by using this
* callback.
*/
static void app_rtls_rssi_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status)
{
    switch(status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer success.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            //hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer timeout.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer cancelled.\n");
            break;

        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }
}

static void app_rtls_client_status_cb(const rtls_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const rtls_status_params_t * p_in)
{
    if (p_in->type == RTLS_PULSE_TYPE)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transfering data: 0x%x, to: 0x%x complete.\n", p_in->pulse, p_meta->src.value);
    }
    else if (p_in->type == RTLS_PRESSURE_TYPE)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transfering data: 0x%x|0x%x to: 0x%x complete.\n", p_in->pressure.pressure_up, 
                                            p_in->pressure.pressure_down, p_meta->src.value);
    }
    else if (p_in->type == RTLS_RSSI_TYPE)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transfering data: rssi = 0x%x from 0x%x, addr = 0x%x complete.\n", 
                                            p_in->rssi.rssi, p_meta->src.value,
                                            p_in->rssi.tag_id);
    }
}

/* Acknowledged transaction status callback, if acknowledged transfer fails, application can
* determine suitable course of action (e.g. re-initiate previous transaction) by using this
* callback.
*/
static void app_rtls_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status)
{
    switch(status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer success.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            //hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer timeout.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer cancelled.\n");
            break;

        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }
}

static void node_reset(void)
{
    NRF_LOG_INFO("----- Node reset  -----\n");
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

/* Функция отправки параметров о состоянии здороья по меш-сети */
void mesh_main_send_message(const rtls_set_params_t * msg_params)
{
    uint32_t status = NRF_SUCCESS;
    static uint8_t tid = 0;

    (void)access_model_reliable_cancel(m_clients[0].model_handle);
    status = rtls_client_set_unack(&m_clients[0], msg_params, NULL, 2);

    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_NO_MEM:
        case NRF_ERROR_BUSY:
        case NRF_ERROR_INVALID_STATE:
            NRF_LOG_INFO("Client cannot send\n");
            break;

        case NRF_ERROR_INVALID_PARAM:
            /* Publication not enabled for this client. One (or more) of the following is wrong:
             * - An application key is missing, or there is no application key bound to the model
             * - The client does not have its publication state set
             *
             * It is the provisioner that adds an application key, binds it to the model and sets
             * the model's publication state.
             */
            NRF_LOG_INFO("Publication not configured for client \n");
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}

void mesh_main_button_event_handler(uint32_t button_number)
{
    /* Increase button number because the buttons on the board is marked with 1 to 4 */
    button_number++;
    NRF_LOG_INFO("Button %u pressed\n", button_number);

    //db_t device;
    uint32_t status = NRF_SUCCESS;
    rtls_set_params_t set_params;

    switch(button_number)
    {
        case 1:
            set_params.type = RTLS_PULSE_TYPE;
            set_params.pulse = 0xFB;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transfering data: 0x%x start.\n", set_params.pulse);
            break;
        case 2:
            set_params.type = RTLS_PRESSURE_TYPE;
            set_params.pressure.pressure_up = 0xB2;
            set_params.pressure.pressure_down = 0x2B;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transfering data: 0x%x|0x%x start.\n", set_params.pressure.pressure_up, 
                                            set_params.pressure.pressure_down);
            break;
        case 3:
            set_params.type = RTLS_SPO2_TYPE;
            set_params.spo2 = 0xB2;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transfering data: 0x%x|0x%x start.\n", set_params.pressure.pressure_up, 
                                            set_params.pressure.pressure_down);
            break;
        case 4:
            set_params.type = RTLS_RSSI_TYPE;
            set_params.rssi.rssi = 0xAA;
            set_params.rssi.tag_id = 0xBDBD;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transfering data: rssi = 0x%x addr = 0x%x start.\n", set_params.rssi.rssi,
                                            set_params.rssi.tag_id);
            break;
    }

    NRF_LOG_INFO("pulse      %02x", set_params.pulse);
    NRF_LOG_INFO("pressure d %02x", set_params.pressure.pressure_down);
    NRF_LOG_INFO("pressure u %02x", set_params.pressure.pressure_up);
    NRF_LOG_INFO("spo2       %02x", set_params.spo2);
    switch (button_number)
    {
        case 1:
        case 2:
        case 3:
            break;
        case 4:
            mesh_main_send_message(&set_params);
            break;
        default:
            break;
    }

    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_NO_MEM:
        case NRF_ERROR_BUSY:
        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Client %u cannot send\n", button_number);
            // TODO: светодиод для донгла
            //hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            break;

        case NRF_ERROR_INVALID_PARAM:
            /* Publication not enabled for this client. One (or more) of the following is wrong:
             * - An application key is missing, or there is no application key bound to the model
             * - The client does not have its publication state set
             *
             * It is the provisioner that adds an application key, binds it to the model and sets
             * the model's publication state.
             */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication not configured for client %u\n", button_number);
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}

static void models_init_cb(void)
{
    NRF_LOG_INFO("Initializing and adding models\n");

    for (uint32_t i = 0; i < 1; ++i)
    {
        m_clients[i].settings.p_callbacks = &client_cbs;
        m_clients[i].settings.timeout = 0;
        m_clients[i].settings.force_segmented = APP_FORCE_SEGMENTATION;
        m_clients[i].settings.transmic_size = APP_MIC_SIZE;
        ERROR_CHECK(rtls_client_init(&m_clients[i], i));

        m_rssi_clients[i].settings.p_callbacks = &rssi_client_cbs;
        m_rssi_clients[i].settings.timeout = 0;
        m_rssi_clients[i].settings.force_segmented = APP_FORCE_SEGMENTATION;
        m_rssi_clients[i].settings.transmic_size = APP_MIC_SIZE;
        ERROR_CHECK(rtls_rssi_client_init(&m_rssi_clients[i], i));
    }
}

void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };

    uint32_t status = mesh_stack_init(&init_params, &m_device_provisioned);
    switch (status)
    {
        case NRF_ERROR_INVALID_DATA:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Data in the persistent memory was corrupted. Device starts as unprovisioned.\n");
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Reset device before starting of the provisioning process.\n");
            break;
        case NRF_SUCCESS:
            break;
        default:
            ERROR_CHECK(status);
    }
}

void mesh_main_initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);

    mesh_init();
}

void mesh_main_start(void)
{
    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = NULL,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = NULL,
            .p_device_uri = EX_URI_RTLS_DONGLE
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    else
    {
        unicast_address_print();
    }

    const uint8_t *p_uuid = nrf_mesh_configure_device_uuid_get();
    NRF_LOG_INFO("Device UUID");
    NRF_LOG_RAW_HEXDUMP_INFO(p_uuid, NRF_MESH_UUID_SIZE);

    ERROR_CHECK(mesh_stack_start());
}
