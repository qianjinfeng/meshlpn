/* Copyright (c) 2010 - 2019, Nordic Semiconductor ASA
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
 */

#include <stdint.h>
#include <string.h>

/* HAL */
#include "boards.h"
#include "simple_hal.h"
#include "app_timer.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "proxy.h"

/* LPN */
#include "mesh_lpn.h"
#include "mesh_friendship_types.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "generic_onoff_client.h"
#include "generic_battery_server.h"
#include "ali_but_server.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "nrf_mesh_config_examples.h"
#include "example_common.h"
#include "ble_softdevice_support.h"
#include "ble_dfu_support.h"

/* nRF5 SDK */
#include "nrf_soc.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_twi_mngr.h"
#include "nrf_drv_saadc.h"

#include "sht3x.h"

/** The maximum duration to scan for incoming Friend Offers. */
#define FRIEND_REQUEST_TIMEOUT_MS (MESH_LPN_FRIEND_REQUEST_TIMEOUT_MAX_MS)
/** The upper limit for two subsequent Friend Polls. */
#define POLL_TIMEOUT_MS (SEC_TO_MS(10))
/** The time between LPN sending a request and listening for a response. */
#define RECEIVE_DELAY_MS (100)

#define APP_STATE_OFF                   0
#define APP_STATE_ON                    1

/** The time before state ON is switched to OFF */
#define APP_STATE_ON_TIMEOUT_MS         (SEC_TO_MS(5))

#define APP_UNACK_MSG_REPEAT_COUNT      2
#define STATIC_AUTH_DATA                {0x6E, 0x6F, 0x72, 0x64, 0x69, 0x63, 0x5F, 0x65, \
                                         0x78, 0x61, 0x6D, 0x70, 0x6C, 0x65, 0x5F, 0x31}

/* aligenie Static OOB */
#define STATIC_AUTH_DATA_ALI            {0x19, 0x3c, 0x86, 0xaf, 0x32, 0x89, 0x83, 0x6c, \
                                         0xf9, 0x40, 0x31, 0x7e, 0x58, 0x29, 0x7f, 0x93} 

#define UUID_ALI                        {0x01, 0xA8, 0x71, 0x00, 0x00, 0x2b, 0x2d, 0xf8, \
                                         0xa7, 0x63, 0x93, 0xa4, 0x3e, 0x02, 0x00, 0x00}

#define TWI_INSTANCE_ID             0
#define MAX_PENDING_TRANSACTIONS    5
NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                                     /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                                       /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  270                                     /**< Typical forward voltage drop of the diode . */
#define ADC_RES_10BIT                   1024                                    /**< Maximum digital value for 10-bit ADC conversion. */
/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 *
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

static ali_but_server_t m_alibutton;
static generic_onoff_client_t m_client;
static generic_battery_server_t m_battery_server;
static nrf_saadc_value_t adc_buf[2];
static uint8_t           m_percentage_batt_lvl;
static bool                   m_device_provisioned;
static bool                   m_device_state = false;

/** The timer emulates an occupancy sensor by turning lights off after a certain interval,
 * when no activity is detected. The timer starts after an On State message is sent
 * and sends an Off State message after the timeout @ref APP_STATE_ON_TIMEOUT_MS. */
APP_TIMER_DEF(m_state_on_timer);
APP_TIMER_DEF(m_battery_timer_id);

/* Forward declaration */
static void app_gen_onoff_client_publish_interval_cb(access_model_handle_t handle, void * p_self);
static void app_generic_onoff_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_onoff_status_params_t * p_in);

static void app_generic_battery_state_get_cb(const generic_battery_server_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               generic_battery_status_params_t * p_out);
static uint16_t app_ali_but_get_cb(const ali_but_server_t * p_self);
                             
static void app_mesh_core_event_cb (const nrf_mesh_evt_t * p_evt);
static void send_app_state(bool is_state_on);

static nrf_mesh_evt_handler_t m_mesh_core_event_handler = { .evt_cb = app_mesh_core_event_cb };

static const generic_onoff_client_callbacks_t client_cbs =
{
    .onoff_status_cb = app_generic_onoff_client_status_cb,
    .ack_transaction_status_cb = NULL,
    .periodic_publish_cb = app_gen_onoff_client_publish_interval_cb
};

static void device_identification_start_cb(uint8_t attention_duration_s)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "device_identification_start_cb\n");
#if SIMPLE_HAL_LEDS_ENABLED
    hal_led_mask_set(LEDS_MASK, false);
    hal_led_blink_ms(LEDS_MASK,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_s));
#endif
}

static void provisioning_aborted_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "provision aborted\n");
#if SIMPLE_HAL_LEDS_ENABLED
    hal_led_blink_stop();
#endif
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

    /* Restores the application parameters after switching from the Provisioning
     * service to the Proxy  */
    gap_params_init();
    conn_params_init();

#if BLE_DFU_SUPPORT_ENABLED
    ble_dfu_support_service_init();
#endif

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);


#if SIMPLE_HAL_LEDS_ENABLED
    hal_led_blink_stop();
    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
#endif
}

/* This callback is called periodically if model is configured for periodic publishing */
static void app_gen_onoff_client_publish_interval_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Publish desired message here.\n");
}

/* Generic OnOff client model interface: Process the received status message in this callback */
static void app_generic_onoff_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_onoff_status_params_t * p_in)
{
    if (p_in->remaining_time_ms > 0)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "OnOff server: 0x%04x, Present OnOff: %d, Target OnOff: %d, Remaining Time: %d ms\n",
              p_meta->src.value, p_in->present_on_off, p_in->target_on_off, p_in->remaining_time_ms);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "OnOff server: 0x%04x, Present OnOff: %d\n",
              p_meta->src.value, p_in->present_on_off);
        m_device_state = p_in->present_on_off;
    }
}
/* This callback is called */
static void app_generic_battery_state_get_cb(const generic_battery_server_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               generic_battery_status_params_t * p_out)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "battery get message here.\n");
    p_out->battery_level = m_percentage_batt_lvl;
    p_out->flags = 0;
}

static uint16_t app_ali_but_get_cb(const ali_but_server_t * p_self)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "ali button get message here.\n");
    return 0;
}

/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SAADC event entered");
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result;
        uint16_t          batt_lvl_in_milli_volts;
        uint32_t          err_code;

        adc_result = p_event->data.done.p_buffer[0];

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
        APP_ERROR_CHECK(err_code);

        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;
        m_percentage_batt_lvl = battery_level_in_percent(batt_lvl_in_milli_volts);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SAADC event: battery level %d\n", m_percentage_batt_lvl);
        generic_battery_status_params_t status = {
            .battery_level = m_percentage_batt_lvl,
            .flags = 0
        };
        generic_battery_server_status_publish(&m_battery_server, &status);
    }
}

static void state_on_timer_handler(void *p_unused)
{
    UNUSED_VARIABLE(p_unused);

    /* Send state off */
    send_app_state(APP_STATE_OFF);
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");

#if SIMPLE_HAL_LEDS_ENABLED
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
#endif

    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
    else if (p_evt->type == CONFIG_SERVER_EVT_MODEL_APP_BIND)
    {
      __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Model app bind  -----\n");
      __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- 0x%04x\n", p_evt->params.model_app_bind.appkey_handle);


    }
    else if (p_evt->type == CONFIG_SERVER_EVT_APPKEY_ADD)
    {
      __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- app key add  -----\n");
      dsm_handle_t app_key_handle = p_evt->params.appkey_add.appkey_handle;

      ERROR_CHECK(access_model_application_bind(m_alibutton.model_handle, app_key_handle));
      ERROR_CHECK(access_model_publish_application_set(m_alibutton.model_handle, app_key_handle));

      /* Add the address to the DSM as a subscription address: */
      dsm_handle_t pub_address_handle;
      ERROR_CHECK(dsm_address_subscription_add(0xC001, &pub_address_handle));
      ERROR_CHECK(access_model_subscription_add(m_alibutton.model_handle, pub_address_handle));
    }
    else if (p_evt->type == CONFIG_SERVER_EVT_MODEL_PUBLICATION_SET)
    {
      __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- publication set  -----\n");
    }
}

static void send_app_state(bool is_state_on)
{
    uint32_t status = NRF_SUCCESS;
    generic_onoff_set_params_t set_params;
    model_transition_t transition_params;
    static uint8_t tid = 0;

    m_device_state = is_state_on;

    set_params.on_off = is_state_on;
    set_params.tid = tid++;
    transition_params.delay_ms = APP_CONFIG_ONOFF_DELAY_MS;
    transition_params.transition_time_ms = APP_CONFIG_ONOFF_TRANSITION_TIME_MS;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: ONOFF SET %d\n", set_params.on_off);

    /* Demonstrate un-acknowledged transaction, using the client model instance */
    /* In this examples, users will not be blocked if the model is busy */
    status = generic_onoff_client_set_unack(&m_client,
                                            &set_params,
                                            &transition_params,
                                            APP_UNACK_MSG_REPEAT_COUNT);

    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_NO_MEM:
        case NRF_ERROR_BUSY:
        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Cannot send the message\n");
#if SIMPLE_HAL_LEDS_ENABLED
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
#endif
            break;

        case NRF_ERROR_INVALID_PARAM:
            /* Publication not enabled for this client. One (or more) of the following is wrong:
             * - An application key is missing, or there is no application key bound to the model
             * - The client does not have its publication state set
             *
             * It is the provisioner that adds an application key, binds it to the model and sets
             * the model's publication state.
             */
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Publication not configured\n");
#if SIMPLE_HAL_LEDS_ENABLED
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_ERROR);
#endif
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}

static void initiate_friendship()
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initiating the friendship establishment procedure.\n");

    mesh_lpn_friend_request_t freq;
    freq.friend_criteria.friend_queue_size_min_log = MESH_FRIENDSHIP_MIN_FRIEND_QUEUE_SIZE_16;
    freq.friend_criteria.receive_window_factor = MESH_FRIENDSHIP_RECEIVE_WINDOW_FACTOR_1_0;
    freq.friend_criteria.rssi_factor = MESH_FRIENDSHIP_RSSI_FACTOR_2_0;
    freq.poll_timeout_ms = POLL_TIMEOUT_MS;
    freq.receive_delay_ms = RECEIVE_DELAY_MS;

    uint32_t status = mesh_lpn_friend_request(freq, FRIEND_REQUEST_TIMEOUT_MS);
    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Already in an active friendship\n");
#if SIMPLE_HAL_LEDS_ENABLED
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_ERROR);
#endif
            break;

        case NRF_ERROR_INVALID_PARAM:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Friend request parameters outside of valid ranges.\n");
#if SIMPLE_HAL_LEDS_ENABLED
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_ERROR);
#endif
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}

static void terminate_friendship()
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Terminating the active friendship\n");

    uint32_t status = mesh_lpn_friendship_terminate();
    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Not in an active friendship\n");
#if SIMPLE_HAL_LEDS_ENABLED
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_ERROR);
#endif
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}
#if BUTTON_BOARD
static void button_event_handler(uint32_t button_number)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    if (!mesh_stack_is_device_provisioned())
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "The device is not provisioned.\n");
        return;
    }

//    ERROR_CHECK(app_timer_stop(m_state_on_timer));

    switch(button_number)
    {
        case 0:
            if (!mesh_lpn_is_in_friendship())
            {
                initiate_friendship();
            }
            else
            {
              if (!m_device_state)
              {
                send_app_state(APP_STATE_ON);
              }
              else 
              {
                send_app_state(APP_STATE_OFF);
              }
            }

//            ERROR_CHECK(app_timer_start(m_state_on_timer,
//                                        HAL_MS_TO_RTC_TICKS(APP_STATE_ON_TIMEOUT_MS),
//                                        NULL));
            break;
          default:
            break;
    }
}
#else
/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    if (!mesh_stack_is_device_provisioned())
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "The device is not provisioned.\n");
        return;
    }

    switch (event)
    {
        case BSP_EVENT_KEY_0:
            if (!mesh_lpn_is_in_friendship())
            {
                initiate_friendship();
            }
            else
            {
              if (!m_device_state)
              {
                send_app_state(APP_STATE_ON);
              }
              else 
              {
                send_app_state(APP_STATE_OFF);
              }
            }
            
            break;

        case BSP_EVENT_RESET:
            if (!mesh_lpn_is_in_friendship())
            {
                initiate_friendship();
            }
            else /* In a friendship */
            {
                terminate_friendship();
            }
            break;

        default:
            break;
    }
}
#endif

#if RTT_INPUT_ENABLED
static void rtt_input_handler(int key)
{
    if (key >= '0' && key <= '3')
    {
        uint32_t button_number = key - '0';
        button_event_handler(button_number);
    }
}
#else
static void buttons_leds_init()
{
    uint32_t err_code;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_event_to_button_action_assign(0,
                                                 BSP_BUTTON_ACTION_LONG_PUSH,
                                                 BSP_EVENT_RESET);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_event_to_button_action_assign(0,
                                                 BSP_BUTTON_ACTION_PUSH,
                                                 BSP_EVENT_KEY_0);
    APP_ERROR_CHECK(err_code);

}
#endif
static void app_mesh_core_event_cb(const nrf_mesh_evt_t * p_evt)
{
    /* USER_NOTE: User can insert mesh core event proceesing here */
    switch (p_evt->type)
    {
        case NRF_MESH_EVT_LPN_FRIEND_OFFER:
        {
            const nrf_mesh_evt_lpn_friend_offer_t *p_offer = &p_evt->params.friend_offer;

            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                  "Received friend offer from 0x%04X\n",
                  p_offer->src);

            uint32_t status = mesh_lpn_friend_accept(p_offer);
            switch (status)
            {
                case NRF_SUCCESS:
                    break;

                case NRF_ERROR_INVALID_STATE:
                case NRF_ERROR_INVALID_PARAM:
                case NRF_ERROR_NULL:
                    __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
                          "Cannot accept friendship: %d\n",
                          status);
#if SIMPLE_HAL_LEDS_ENABLED
                    hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS,
                                     LED_BLINK_CNT_ERROR);
#endif
                    break;

                default:
                    ERROR_CHECK(status);
                    break;
            }

            break;
        }

        case NRF_MESH_EVT_LPN_FRIEND_POLL_COMPLETE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Friend poll procedure complete\n");
            break;

        case NRF_MESH_EVT_LPN_FRIEND_REQUEST_TIMEOUT:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Friend Request timed out\n");
#if SIMPLE_HAL_LEDS_ENABLED
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_ERROR);
#endif
            break;

        case NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED:
        {
            const nrf_mesh_evt_friendship_established_t *p_est =
                    &p_evt->params.friendship_established;
            (void) p_est;

            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                  "Friendship established with: 0x%04X\n",
                  p_est->friend_src);

#if SIMPLE_HAL_LEDS_ENABLED
            hal_led_pin_set(BSP_LED_0, true);
#endif
            break;
        }

        case NRF_MESH_EVT_FRIENDSHIP_TERMINATED:
        {
            const nrf_mesh_evt_friendship_terminated_t *p_term = &p_evt->params.friendship_terminated;
            UNUSED_VARIABLE(p_term);

            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                  "Friendship with 0x%04X terminated. Reason: %d\n",
                  p_term->friend_src, p_term->reason);

#if SIMPLE_HAL_LEDS_ENABLED
            hal_led_pin_set(BSP_LED_0, false);
#endif

//            ERROR_CHECK(app_timer_stop(m_state_on_timer));
            break;
        }

        default:
            break;
    }
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");

    m_client.settings.p_callbacks = &client_cbs;
    m_client.settings.timeout = 0;
    m_client.settings.force_segmented = APP_CONFIG_FORCE_SEGMENTATION;
    m_client.settings.transmic_size = APP_CONFIG_MIC_SIZE;

    ERROR_CHECK(generic_onoff_client_init(&m_client, 1));

    m_battery_server.settings.p_get_cb = app_generic_battery_state_get_cb;
    m_battery_server.settings.force_segmented = APP_CONFIG_FORCE_SEGMENTATION;
    m_battery_server.settings.transmic_size = APP_CONFIG_MIC_SIZE;

    ERROR_CHECK(generic_battery_server_init(&m_battery_server, 1));

    m_alibutton.get_cb = app_ali_but_get_cb;
    ERROR_CHECK(ali_but_server_init(&m_alibutton, 1));
}

// TWI (with transaction manager) initialization.
static void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
       .clear_bus_init     = false
    };

    err_code = nrf_twi_mngr_init(&m_nrf_twi_mngr, &config);
    ERROR_CHECK(err_code);
}

/**@brief Function for configuring ADC to do battery level conversion.
 */
static void adc_configure(void)
{
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
    err_code = nrf_drv_saadc_channel_init(0, &config);
    ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], 1);
    ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[1], 1);
    ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_sample();
    ERROR_CHECK(err_code);
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *          This function will start the ADC.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code;
    err_code = nrf_drv_saadc_sample();
    ERROR_CHECK(err_code);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Create battery timer.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    ERROR_CHECK(err_code);
}

static void mesh_init(void)
{
    static const uint8_t static_uuid[NRF_MESH_KEY_SIZE] = UUID_ALI;
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = static_uuid,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };

    uint32_t status = mesh_stack_init(&init_params, &m_device_provisioned);
    switch (status)
    {
        case NRF_ERROR_INVALID_DATA:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Data in the persistent memory was corrupted. Device starts as unprovisioned.\n");
            break;
        case NRF_SUCCESS:
            break;
        default:
            ERROR_CHECK(status);
    }

    /* Register event handler to receive LPN and friendship events. */
    nrf_mesh_evt_handler_add(&m_mesh_core_event_handler);
}

#if BLE_DFU_SUPPORT_ENABLED
#endif
/** Initializes Power Management. Required for BLE DFU. */
static void power_management_init(void)
{
    uint32_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh LPN Demo -----\n");

    ERROR_CHECK(app_timer_init());

#if SIMPLE_HAL_LEDS_ENABLED
    hal_leds_init();
#else
    buttons_leds_init();
#endif

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif

#if BLE_DFU_SUPPORT_ENABLED
    ble_dfu_support_init();
#endif

    power_management_init();
    timers_init();
    adc_configure();
    twi_config();
    // Initialize sensors.
//    APP_ERROR_CHECK(nrf_twi_mngr_perform(&m_nrf_twi_mngr, NULL, sht3x_init_transfers,
//        SHT3X_INIT_TRANSFER_COUNT, NULL));
//    uint8_t address;
//    uint8_t sample_data;
//    ret_code_t err_code;
//    for (address = 1; address <= 127; address++)
//    {
//        err_code = nrf_drv_twi_rx(&m_nrf_twi_mngr.twi, address, &sample_data, sizeof(sample_data));
//        if (err_code == NRF_SUCCESS)
//        {
//            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "TWI device detected at address 0x%x.", address);
//        }
//    }

    ble_stack_init();
    gap_params_init();
    conn_params_init();

#if BLE_DFU_SUPPORT_ENABLED
    ble_dfu_support_service_init();
#endif

    mesh_init();
    ERROR_CHECK(sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE));

    mesh_lpn_init();
}

static void start(void)
{
#if RTT_INPUT_ENABLED
    rtt_input_enable(rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);
#endif

//    ERROR_CHECK(app_timer_create(&m_state_on_timer, APP_TIMER_MODE_SINGLE_SHOT,
//                                 state_on_timer_handler));

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA_ALI;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = provisioning_aborted_cb,
            .p_device_uri = EX_URI_LPN
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

#if SIMPLE_HAL_LEDS_ENABLED
    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
#endif
}

int main(void)
{
    initialize();
    start();

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
