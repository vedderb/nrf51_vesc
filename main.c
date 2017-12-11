/*
	Copyright 2017 Benjamin Vedder	benjamin@vedder.se

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
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_delay.h"

#include "pca10028.h"

#include "nrf51_bitfields.h"
#include "nrf_uart.h"
#include "nrf51.h"

#include "packet.h"

// Use WT module (rx = p0.01 ; tx = p0.02 ; no 32k crystal)
#ifndef MODULE_WT
#define MODULE_WT						0
#endif

// https://devzone.nordicsemi.com/question/59389/solved-help-with-wgt51822-s2-module/
#if MODULE_WT
#undef NRF_CLOCK_LFCLKSRC
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_SYNTH,           \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}
#endif

// Defines
#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "VESC BLE UART"                             /**< Name of device. Will be included in the advertising data. */
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

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256
#define UART_RX_BUF_SIZE                256

#define PACKET_VESC						0
#define PACKET_BLE						1
#define PRINT_EN						0

// Private variables
static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

static void my_printf(const char* format, ...) {
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
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
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

static void sleep_mode_enter(void) {
	uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
	APP_ERROR_CHECK(err_code);

	// Prepare wakeup buttons.
	err_code = bsp_btn_ble_sleep_mode_prepare();
	APP_ERROR_CHECK(err_code);

	// Go to system-off mode (this function will not return; wakeup will cause a reset).
	err_code = sd_power_system_off();
	APP_ERROR_CHECK(err_code);
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
	uint32_t err_code;

	switch (ble_adv_evt) {
	case BLE_ADV_EVT_FAST:
		err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
		APP_ERROR_CHECK(err_code);
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
		err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
		APP_ERROR_CHECK(err_code);
		m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		my_printf("Connected\r\n");
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		err_code = bsp_indication_set(BSP_INDICATE_IDLE);
		APP_ERROR_CHECK(err_code);
		m_conn_handle = BLE_CONN_HANDLE_INVALID;
		my_printf("Disconnected\r\n");
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
	bsp_btn_ble_on_ble_evt(p_ble_evt);
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

void bsp_event_handler(bsp_event_t event) {
	uint32_t err_code;
	switch (event) {
	case BSP_EVENT_SLEEP:
		sleep_mode_enter();
		break;

	case BSP_EVENT_DISCONNECT:
		err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		if (err_code != NRF_ERROR_INVALID_STATE) {
			APP_ERROR_CHECK(err_code);
		}
		break;

	case BSP_EVENT_WHITELIST_OFF:
		if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
			err_code = ble_advertising_restart_without_whitelist();
			if (err_code != NRF_ERROR_INVALID_STATE) {
				APP_ERROR_CHECK(err_code);
			}
		}
		break;

	default:
		break;
	}
}

static bool uart_error = false;

void uart_event_handle(app_uart_evt_t * p_event) {
	switch (p_event->evt_type) {
	case APP_UART_DATA_READY: {
		// Data is processed in main
	} break;

	case APP_UART_COMMUNICATION_ERROR: {
//		APP_ERROR_HANDLER(p_event->data.error_communication);
		uart_error = true;
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

#if MODULE_WT
	const app_uart_comm_params_t comm_params = {
			1,
			2,
			0,
			0,
			APP_UART_FLOW_CONTROL_DISABLED,
			false,
			UART_BAUDRATE_BAUDRATE_Baud115200
	};
#else
	const app_uart_comm_params_t comm_params = {
			11,
			9,
			0,
			0,
			APP_UART_FLOW_CONTROL_DISABLED,
			false,
			UART_BAUDRATE_BAUDRATE_Baud115200
	};
#endif

	APP_UART_FIFO_INIT(&comm_params,
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
		while (app_uart_put(data[i]) == NRF_ERROR_NO_MEM) {
			nrf_delay_us(100);
		}
	}
}

static void process_packet_ble(unsigned char *data, unsigned int len) {
	packet_send_packet(data, len, PACKET_VESC);
}

static void process_packet_vesc(unsigned char *data, unsigned int len) {
	packet_send_packet(data, len, PACKET_BLE);
}

void TIMER2_IRQHandler(void) {
	if ((NRF_TIMER2->EVENTS_COMPARE[0] != 0)
			&& ((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE0_Msk) != 0)) {
		NRF_TIMER2->EVENTS_COMPARE[0] = 0;
		packet_timerfunc();
	}
}

static void timer_init(void) {
	NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER2->TASKS_CLEAR = 1; // clear the task first to be usable for later
	NRF_TIMER2->PRESCALER = 4;
	NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
	NRF_TIMER2->CC[0] = 2000;

	// Clear the timer when COMPARE0 event is triggered
	NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;

	// Interrupt
	NRF_TIMER2->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
	NVIC_EnableIRQ(TIMER2_IRQn);

	// Start
	NRF_TIMER2->TASKS_START = 1;
}

//static void update_connection(void) {
//	ble_gap_conn_params_t param;
//	param.conn_sup_timeout = 10000;
//	param.min_conn_interval = 6;
//	param.max_conn_interval = 10;
//	param.slave_latency = 0;
//
//	sd_ble_gap_conn_param_update(m_conn_handle, &param);
//}

int main(void) {
	uint32_t err_code;

	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	uart_init();

	packet_init(uart_send_buffer, process_packet_vesc, PACKET_VESC);
	packet_init(ble_send_buffer, process_packet_ble, PACKET_BLE);

	ble_stack_init();
	gap_params_init();
	services_init();
	advertising_init();
	conn_params_init();

	timer_init();

	err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
	APP_ERROR_CHECK(err_code);

	my_printf("UART TEST\r\n");

	// Enter main loop.
	for (;;) {
		// Restore uart on errors
		if (uart_error) {
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

			uart_error = false;
		}

		// Poll UART
		uint8_t byte;
		while (app_uart_get(&byte) == NRF_SUCCESS) {
			packet_process_byte(byte, PACKET_VESC);
		}

		power_manage();
	}
}
