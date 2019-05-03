/*
	Copyright 2017 - 2019 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "nrf_delay.h"

#include "pca10028.h"

#include "nrf51_bitfields.h"
#include "nrf_uart.h"
#include "nrf51.h"

#include "nrf_esb.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"

#include "packet.h"
#include "crc.h"
#include "buffer.h"
#include "datatypes.h"

#ifndef FW_16K
#include "esb_timeslot.h"
#endif

#if 0
#ifdef APP_ERROR_CHECK
#undef APP_ERROR_CHECK
#endif
void my_printf(const char* format, ...);
#define APP_ERROR_CHECK(ERR_CODE)                       \
do                                                      \
{                                                       \
	if (ERR_CODE != NRF_SUCCESS)                  		\
		{                                               \
			my_printf("Error %d: %s L %d\r\n", ERR_CODE, __FILE__, __LINE__); \
        }                                               \
} while (0)
#endif

#define PRINT_EN						0

#ifndef MODULE_TRAMPA
#define MODULE_TRAMPA					0
#endif

#ifndef MODULE_BUILTIN
#define MODULE_BUILTIN					0
#endif

#ifndef MODULE_WT
#define MODULE_WT						0
#endif

#if MODULE_TRAMPA
#define UART_RX							2
#define UART_TX							1
#define UART_TX_DISABLED				25
#define EN_DEFAULT						1
#define LED_PIN							3
#elif MODULE_BUILTIN
#define UART_RX							1
#define UART_TX							2
#define UART_TX_DISABLED				25
#define EN_DEFAULT						1
#define LED_PIN							3
#elif MODULE_WT
#define UART_RX							1
#define UART_TX							2
#define UART_TX_DISABLED				25
#define EN_DEFAULT						1
#define LED_PIN							3
#else
#define UART_RX							11
#define UART_TX							9
#define UART_TX_DISABLED				25
#define EN_DEFAULT						1
#define LED_PIN							3
#endif

// https://devzone.nordicsemi.com/question/59389/solved-help-with-wgt51822-s2-module/
#undef NRF_CLOCK_LFCLKSRC
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_SYNTH,           \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

// Defines
#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#if MODULE_BUILTIN
#define DEVICE_NAME                     "VESC BUILTIN BLE"
#else
#define DEVICE_NAME                     "VESC BLE UART"
#endif

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               6											/**< Minimum acceptable connection interval (7.5 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               25											/**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#ifndef FW_16K
#define UART_TX_BUF_SIZE                1024
#define UART_RX_BUF_SIZE                8192
#else
#define UART_TX_BUF_SIZE                256
#define UART_RX_BUF_SIZE                512
#endif

#define PACKET_VESC						0
#define PACKET_BLE						1

APP_TIMER_DEF(m_packet_timer);
#ifndef FW_16K
APP_TIMER_DEF(m_nrf_timer);
#endif

// Private variables
static ble_nus_t                        m_nus;
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;
static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};
static bool								m_is_enabled = EN_DEFAULT;
static bool								m_uart_error = false;
static int								m_other_comm_disable_time = 0;

static app_uart_comm_params_t m_uart_comm_params = {
		UART_RX,
		EN_DEFAULT ? UART_TX : UART_TX_DISABLED,
		0,
		0,
		APP_UART_FLOW_CONTROL_DISABLED,
		false,
		UART_BAUDRATE_BAUDRATE_Baud115200
};

void my_printf(const char* format, ...) {
#if PRINT_EN
	va_list arg;
	va_start (arg, format);
	int len;
	static char print_buffer[255];

	len = vsnprintf(print_buffer, 255, format, arg);
	va_end (arg);

	for (int i = 0;i < len;i++) {
		app_uart_put(print_buffer[i]);
	}
#else
	(void)format;
#endif
}

/*
 * NOTE:
 * APP_ERROR_HANDLER resets the CPU, which results in an immediate disconnect.
 * APP_ERROR_CHECK calls APP_ERROR_HANDLER on errors.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
	app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

static void gap_params_init(void) {
	uint32_t                err_code;
	ble_gap_conn_params_t   gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	err_code = sd_ble_gap_device_name_set(&sec_mode,
			(const uint8_t *) DEVICE_NAME,
			strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.slave_latency     = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);
}

static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length) {
	(void)p_nus;

	for (uint32_t i = 0; i < length; i++) {
		packet_process_byte(p_data[i], PACKET_BLE);
	}
}

static void services_init(void) {
	uint32_t       err_code;
	ble_nus_init_t nus_init;

	memset(&nus_init, 0, sizeof(nus_init));

	nus_init.data_handler = nus_data_handler;

	err_code = ble_nus_init(&m_nus, &nus_init);
	APP_ERROR_CHECK(err_code);
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt) {
	uint32_t err_code;

	if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
		err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
		APP_ERROR_CHECK(err_code);
	}
}

static void conn_params_error_handler(uint32_t nrf_error) {
	APP_ERROR_HANDLER(nrf_error);
}

static void conn_params_init(void) {
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

static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
	switch (ble_adv_evt) {
	case BLE_ADV_EVT_FAST:

		break;
	case BLE_ADV_EVT_IDLE: {
//		sleep_mode_enter();
		// start advertising again
		ret_code_t err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
		APP_ERROR_CHECK(err_code);
	} break;
	default:
		break;
	}
}

static void on_ble_evt(ble_evt_t * p_ble_evt) {
	uint32_t err_code;

	switch (p_ble_evt->header.evt_id) {
	case BLE_GAP_EVT_CONNECTED:
		m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		my_printf("Connected\r\n");
		nrf_gpio_pin_set(LED_PIN);
//		nrf_gpio_cfg_output(UART_TX);
//		nrf_gpio_cfg_input(UART_RX, NRF_GPIO_PIN_NOPULL);
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		m_conn_handle = BLE_CONN_HANDLE_INVALID;
		my_printf("Disconnected\r\n");
		nrf_gpio_pin_clear(LED_PIN);
//		nrf_gpio_cfg_default(UART_RX);
//		nrf_gpio_cfg_default(UART_TX);
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

	case BLE_EVT_USER_MEM_REQUEST:
		err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST: {
		ble_gatts_evt_rw_authorize_request_t  req;
		ble_gatts_rw_authorize_reply_params_t auth_reply;

		req = p_ble_evt->evt.gatts_evt.params.authorize_request;

		if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID) {
			if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
					(req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
					(req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL)) {
				if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE) {
					auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
				} else {
					auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
				}
				auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
				err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
						&auth_reply);
				APP_ERROR_CHECK(err_code);
			}
		}
	} break;

#if (NRF_SD_BLE_API_VERSION == 3)
	case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
		err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
				NRF_BLE_MAX_MTU_SIZE);
		APP_ERROR_CHECK(err_code);
		break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

	default:
		break;
	}
}

static void ble_evt_dispatch(ble_evt_t * p_ble_evt) {
	ble_conn_params_on_ble_evt(p_ble_evt);
	ble_nus_on_ble_evt(&m_nus, p_ble_evt);
	on_ble_evt(p_ble_evt);
	ble_advertising_on_ble_evt(p_ble_evt);
}

static void ble_stack_init(void) {
	uint32_t err_code;

	nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

	// Initialize SoftDevice.
	SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

	ble_enable_params_t ble_enable_params;
	err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
			PERIPHERAL_LINK_COUNT,
			&ble_enable_params);
	APP_ERROR_CHECK(err_code);

	//Check the ram settings against the used number of links
	CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

	// Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
	ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
	err_code = softdevice_enable(&ble_enable_params);
	APP_ERROR_CHECK(err_code);

	// Subscribe for BLE events.
	err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
	APP_ERROR_CHECK(err_code);
}

void uart_event_handle(app_uart_evt_t * p_event) {
	switch (p_event->evt_type) {
	case APP_UART_DATA_READY: {
		// Data is processed in main
	} break;

	case APP_UART_COMMUNICATION_ERROR: {
//		APP_ERROR_HANDLER(p_event->data.error_communication);
		m_uart_error = true;
	} break;

	case APP_UART_FIFO_ERROR:
//		APP_ERROR_HANDLER(p_event->data.error_code);
//		uart_error = true;
		app_uart_flush();
		packet_reset(PACKET_VESC);
		break;

	default:
		break;
	}
}

static void uart_init(void) {
	uint32_t err_code;

	APP_UART_FIFO_INIT(&m_uart_comm_params,
			UART_RX_BUF_SIZE,
			UART_TX_BUF_SIZE,
			uart_event_handle,
			APP_IRQ_PRIORITY_MID,
			err_code);

	APP_ERROR_CHECK(err_code);
}

static void advertising_init(void) {
	uint32_t               err_code;
	ble_advdata_t          advdata;
	ble_advdata_t          scanrsp;
	ble_adv_modes_config_t options;

	// Build advertising data struct to pass into @ref ble_advertising_init.
	memset(&advdata, 0, sizeof(advdata));
	advdata.name_type          = BLE_ADVDATA_FULL_NAME;
	advdata.include_appearance = false;
	advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

	memset(&scanrsp, 0, sizeof(scanrsp));
	scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

	memset(&options, 0, sizeof(options));
	options.ble_adv_fast_enabled  = true;
	options.ble_adv_fast_interval = APP_ADV_INTERVAL;
	options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

	err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
	APP_ERROR_CHECK(err_code);
}

static void power_manage(void) {
	uint32_t err_code = sd_app_evt_wait();
	APP_ERROR_CHECK(err_code);
}

static void set_enabled(bool en) {
	m_is_enabled = en;

	if (m_is_enabled) {
		app_uart_close();
		m_uart_comm_params.tx_pin_no = UART_TX;
		uart_init();
		nrf_gpio_cfg_default(UART_TX_DISABLED);
	} else {
		app_uart_close();
		m_uart_comm_params.tx_pin_no = UART_TX_DISABLED;
		uart_init();
		nrf_gpio_cfg_default(UART_TX);
	}
}

static void ble_send_buffer(unsigned char *data, unsigned int len) {
	if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
		uint32_t err_code = NRF_SUCCESS;
		int ind = 0;

		while (len > BLE_NUS_MAX_DATA_LEN) {
			do{
				err_code = ble_nus_string_send(&m_nus, data + ind, BLE_NUS_MAX_DATA_LEN);
			} while(m_conn_handle != BLE_CONN_HANDLE_INVALID &&
					err_code == BLE_ERROR_NO_TX_PACKETS);

			len -= BLE_NUS_MAX_DATA_LEN;
			ind += BLE_NUS_MAX_DATA_LEN;
		}

		if (len > 0) {
			do{
				err_code = ble_nus_string_send(&m_nus, data + ind, len);
			} while(m_conn_handle != BLE_CONN_HANDLE_INVALID &&
					err_code == BLE_ERROR_NO_TX_PACKETS);
		}

		APP_ERROR_CHECK(err_code);
	}
}

static void uart_send_buffer(unsigned char *data, unsigned int len) {
	for (int i = 0;i < len;i++) {
		app_uart_put(data[i]);
//		while (app_uart_put(data[i]) == NRF_ERROR_NO_MEM) {
//			nrf_delay_us(100);
//		}
	}
}

#ifndef FW_16K
void rfhelp_send_data_crc(uint8_t *data, unsigned int len) {
	uint8_t buffer[len + 2];
	unsigned short crc = crc16((unsigned char*)data, len);
	memcpy(buffer, data, len);
	buffer[len] = (char)(crc >> 8);
	buffer[len + 1] = (char)(crc & 0xFF);
	esb_timeslot_set_next_packet(buffer, len + 2);
}
#endif

static void process_packet_ble(unsigned char *data, unsigned int len) {
	if (data[0] == COMM_ERASE_NEW_APP ||
			data[0] == COMM_WRITE_NEW_APP_DATA ||
			data[0] == COMM_ERASE_NEW_APP_ALL_CAN ||
			data[0] == COMM_WRITE_NEW_APP_DATA_ALL_CAN) {
		m_other_comm_disable_time = 5000;
	}

	CRITICAL_REGION_ENTER();
	packet_send_packet(data, len, PACKET_VESC);
	CRITICAL_REGION_EXIT();
}

static void process_packet_vesc(unsigned char *data, unsigned int len) {
	if (data[0] == COMM_EXT_NRF_ESB_SET_CH_ADDR) {
#ifndef FW_16K
		esb_timeslot_set_ch_addr(data[1], data[2], data[3], data[4]);
#endif
	} else if (data[0] == COMM_EXT_NRF_ESB_SEND_DATA) {
#ifndef FW_16K
		rfhelp_send_data_crc(data + 1, len - 1);
#endif
	} else if (data[0] == COMM_EXT_NRF_SET_ENABLED) {
		set_enabled(data[1]);
	} else {
		if (m_is_enabled) {
			packet_send_packet(data, len, PACKET_BLE);
		}
	}
}

#ifndef FW_16K
static void esb_timeslot_data_handler(void *p_data, uint16_t length) {
	if (m_other_comm_disable_time == 0) {
		uint8_t buffer[length + 1];
		buffer[0] = COMM_EXT_NRF_ESB_RX_DATA;
		memcpy(buffer + 1, p_data, length);
		CRITICAL_REGION_ENTER();
		packet_send_packet(buffer, length + 1, PACKET_VESC);
		CRITICAL_REGION_EXIT();
	}
}
#endif

static void packet_timer_handler(void *p_context) {
	(void)p_context;
	packet_timerfunc();

	CRITICAL_REGION_ENTER();
	if (m_other_comm_disable_time > 0) {
		m_other_comm_disable_time--;
	}
	CRITICAL_REGION_EXIT();
}

#ifndef FW_16K
static void nrf_timer_handler(void *p_context) {
	(void)p_context;

	if (m_other_comm_disable_time == 0) {
		uint8_t buffer[1];
		buffer[0] = COMM_EXT_NRF_PRESENT;
		CRITICAL_REGION_ENTER();
		packet_send_packet(buffer, 1, PACKET_VESC);
		CRITICAL_REGION_EXIT();
	}
}
#endif

int main(void) {
	// The EYSGJNZXX and EYSGJNZWY modules use a 32 MHz crystals
#if MODULE_TRAMPA || MODULE_BUILTIN
	NRF_CLOCK->XTALFREQ = 0xFFFFFF00;

	// Start the external high frequency crystal
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;

	// Wait for the external oscillator to start up
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}

	nrf_gpio_cfg_output(LED_PIN);
	nrf_gpio_pin_clear(LED_PIN);
#endif

	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	uart_init();

	packet_init(uart_send_buffer, process_packet_vesc, PACKET_VESC);
	packet_init(ble_send_buffer, process_packet_ble, PACKET_BLE);

	ble_stack_init();
	gap_params_init();
	services_init();
	advertising_init();
	conn_params_init();

	app_timer_create(&m_packet_timer, APP_TIMER_MODE_REPEATED, packet_timer_handler);
	app_timer_start(m_packet_timer, APP_TIMER_TICKS(1, APP_TIMER_PRESCALER), NULL);

#ifndef FW_16K
	app_timer_create(&m_nrf_timer, APP_TIMER_MODE_REPEATED, nrf_timer_handler);
	app_timer_start(m_nrf_timer, APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER), NULL);

	esb_timeslot_init(esb_timeslot_data_handler);
//	esb_timeslot_set_ch_addr(67, 0xC6, 0xC5, 0x0);
	esb_timeslot_sd_start();
#endif

	ble_advertising_start(BLE_ADV_MODE_FAST);

	for (;;) {
		// Restore uart on errors
		if (m_uart_error) {
			app_uart_close();

			uint32_t error = NRF_UART0->ERRORSRC;
			NRF_UART0->ERRORSRC = error;

			for (int i = 0;i < 6;i++) {
				nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_RXDRDY);
				error = NRF_UART0->RXD;
				nrf_delay_us(10);
			}

			uart_init();
			packet_reset(PACKET_VESC);

			nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_RXDRDY);
			nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_RXTO);
			nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_TXDRDY);
			nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_ERROR);

			m_uart_error = false;
		}

		// Poll UART
		uint8_t byte;
		while (app_uart_get(&byte) == NRF_SUCCESS) {
			packet_process_byte(byte, PACKET_VESC);
		}

		power_manage();
	}
}
