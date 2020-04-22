#include "esb_timeslot.h"

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "softdevice_handler.h"
#include "boards.h"
#include "sdk_common.h"
#include "app_util_platform.h"

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

#define TIMESLOT_BEGIN_IRQn         LPCOMP_IRQn             /**< Re-used LPCOMP interrupt for processing the beginning of timeslot. */
#define TIMESLOT_BEGIN_IRQHandler   LPCOMP_IRQHandler       /**< The IRQ handler of LPCOMP interrupt */
#define TIMESLOT_BEGIN_IRQPriority  1                       /**< Interrupt priority of @ref TIMESLOT_BEGIN_IRQn. */

#define TIMESLOT_END_IRQn           QDEC_IRQn               /**< Re-used QDEC interrupt for processing the end of timeslot. */
#define TIMESLOT_END_IRQHandler     QDEC_IRQHandler         /**< The IRQ handler of QDEC interrupt */
#define TIMESLOT_END_IRQPriority    1                       /**< Interrupt priority of @ref TIMESLOT_END_IRQn. */

#define UESB_RX_HANDLE_IRQn         WDT_IRQn                /**< Re-used WDT interrupt for processing the RX data from UESB. */
#define UESB_RX_HANDLE_IRQHandler   WDT_IRQHandler          /**< The IRQ handler of WDT interrupt */
#define UESB_RX_HANDLE_IRQPriority  3                       /**< Interrupt priority of @ref UESB_RX_HANDLE_IRQn. */

#define TS_LEN_US                   (2500UL)                /**< Length of timeslot to be requested. */
#define TX_LEN_EXTENSION_US         (2500UL)                /**< Length of timeslot to be extended. */
#define TS_SAFETY_MARGIN_US         (400UL)                 /**< The timeslot activity should be finished with this much to spare. */
#define TS_EXTEND_MARGIN_US         (800UL)                /**< Margin reserved for extension processing. */

static volatile enum {
	STATE_IDLE, /**< Default state. */
	STATE_RX, /**< Waiting for packets. */
	STATE_TX /**< Trying to transmit packet. */
} m_state = STATE_IDLE;

/** Constants for timeslot API */
static nrf_radio_request_t m_timeslot_request; /**< Persistent request structure for softdevice. */
static nrf_esb_config_t nrf_esb_config; /**< Configuration structure for nrf_esb initialization. */
static ut_data_handler_t m_evt_handler = 0; /**< Event handler which passes received data to application. */

static nrf_radio_signal_callback_return_param_t signal_callback_return_param;
static uint32_t m_total_timeslot_length = 0;
void RADIO_IRQHandler(void);

static uint8_t m_base_addr_0[4] = { 0x25, 0, 0, 0 };
static uint8_t m_addr_prefix[8] = { 0x16, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };
static uint8_t m_channel = 23;
static nrf_esb_payload_t m_next_payload;
static volatile bool m_next_packet_set;

/**@brief Request next timeslot event in earliest configuration.
 * @note  Will call softdevice API.
 */
uint32_t request_next_event_earliest(void) {
	m_timeslot_request.request_type = NRF_RADIO_REQ_TYPE_EARLIEST;
	m_timeslot_request.params.earliest.hfclk = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
	m_timeslot_request.params.earliest.priority = NRF_RADIO_PRIORITY_NORMAL;
	m_timeslot_request.params.earliest.length_us = TS_LEN_US;
	m_timeslot_request.params.earliest.timeout_us = 1000000;
	return sd_radio_request(&m_timeslot_request);
}

/**@brief Configure next timeslot event in earliest configuration.
 */
void configure_next_event_earliest(void) {
	m_timeslot_request.request_type = NRF_RADIO_REQ_TYPE_EARLIEST;
	m_timeslot_request.params.earliest.hfclk = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
	m_timeslot_request.params.earliest.priority = NRF_RADIO_PRIORITY_NORMAL;
	m_timeslot_request.params.earliest.length_us = TS_LEN_US;
	m_timeslot_request.params.earliest.timeout_us = 1000000;
}

/**@brief Timeslot signal handler.
 */
void nrf_evt_signal_handler(uint32_t evt_id) {
	uint32_t err_code;

	switch (evt_id) {
	case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
		// No implementation needed
		break;

	case NRF_EVT_RADIO_SESSION_IDLE:
		err_code = sd_radio_session_close();
		APP_ERROR_CHECK(err_code);
		break;

	case NRF_EVT_RADIO_SESSION_CLOSED:
		// No implementation needed, session ended
		break;

	case NRF_EVT_RADIO_BLOCKED:
		// Fall through

	case NRF_EVT_RADIO_CANCELED:
		err_code = request_next_event_earliest();
		APP_ERROR_CHECK(err_code);
		break;

	default:
		break;
	}
}

/**@brief Timeslot event handler.
 */
nrf_radio_signal_callback_return_param_t * radio_callback(uint8_t signal_type) {
	switch (signal_type) {
	case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
		/* Start of the timeslot - set up timer interrupt */
		signal_callback_return_param.params.request.p_next = NULL;
		signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;

		NRF_TIMER0->TASKS_STOP = 1;
		NRF_TIMER0->TASKS_CLEAR = 1;
		NRF_TIMER0->MODE = (TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos);
		NRF_TIMER0->EVENTS_COMPARE[0] = 0;
		NRF_TIMER0->EVENTS_COMPARE[1] = 0;
		NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk | TIMER_INTENSET_COMPARE1_Msk;
		NRF_TIMER0->CC[0] = TS_LEN_US - TS_SAFETY_MARGIN_US;
		NRF_TIMER0->CC[1] = (TS_LEN_US - TS_EXTEND_MARGIN_US);
		NRF_TIMER0->BITMODE = (TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos);
		NRF_TIMER0->TASKS_START = 1;
		NRF_RADIO->POWER = (RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos);

		/* Call TIMESLOT_BEGIN_IRQHandler later. */
		NVIC_EnableIRQ(TIMER0_IRQn);
		NVIC_SetPendingIRQ(TIMESLOT_BEGIN_IRQn);
		break;

	case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
		signal_callback_return_param.params.request.p_next = NULL;
		signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
		RADIO_IRQHandler();
		break;

	case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
		if (NRF_TIMER0->EVENTS_COMPARE[0]
				&& (NRF_TIMER0->INTENSET
						& (TIMER_INTENSET_COMPARE0_Enabled
								<< TIMER_INTENCLR_COMPARE0_Pos))) {
			NRF_TIMER0->TASKS_STOP = 1;
			NRF_TIMER0->EVENTS_COMPARE[0] = 0;

			/* This is the "timeslot is about to end" timeout. */
			if (!nrf_esb_is_idle()) {
				NRF_RADIO->INTENCLR = 0xFFFFFFFF;
				NRF_RADIO->TASKS_DISABLE = 1;
			}

			/* Schedule next timeslot. */
			configure_next_event_earliest();
			signal_callback_return_param.params.request.p_next = &m_timeslot_request;
			signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
		}

		if (NRF_TIMER0->EVENTS_COMPARE[1] && (NRF_TIMER0->INTENSET
						& (TIMER_INTENSET_COMPARE1_Enabled
								<< TIMER_INTENCLR_COMPARE1_Pos))) {
			NRF_TIMER0->EVENTS_COMPARE[1] = 0;

			/* This is the "Time to extend timeslot" timeout. */
			if (m_total_timeslot_length < (128000000UL - 1UL - TX_LEN_EXTENSION_US)) {
				/* Request timeslot extension if total length does not exceed 128 seconds. */
				signal_callback_return_param.params.extend.length_us = TX_LEN_EXTENSION_US;
				signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND;
			} else {
				/* Return with no action request. */
				signal_callback_return_param.params.request.p_next = NULL;
				signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
			}
		}
		break;

	case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
		NRF_TIMER0->TASKS_STOP = 1;
		NRF_TIMER0->EVENTS_COMPARE[0] = 0;
		NRF_TIMER0->EVENTS_COMPARE[1] = 0;
		NRF_TIMER0->CC[0] += (TX_LEN_EXTENSION_US - 25);
		NRF_TIMER0->CC[1] += (TX_LEN_EXTENSION_US - 25);
		NRF_TIMER0->TASKS_START = 1;

		m_total_timeslot_length += TX_LEN_EXTENSION_US;
		NVIC_SetPendingIRQ(TIMESLOT_BEGIN_IRQn);
		signal_callback_return_param.params.request.p_next = NULL;
		signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
		break;

	case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED: {
		/* Tried scheduling a new timeslot, but failed. */

		/* Disabling UESB is done in a lower interrupt priority. */
		/* Call TIMESLOT_END_IRQHandler later. */
		NVIC_SetPendingIRQ(TIMESLOT_END_IRQn);
		signal_callback_return_param.params.request.p_next = NULL;
		signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
	} break;

	default:
		/* No implementation needed. */
		break;
	}

	return (&signal_callback_return_param);
}

uint32_t esb_timeslot_sd_start(void) {
	uint32_t err_code;

	err_code = sd_radio_session_open(radio_callback);
	if (err_code != NRF_SUCCESS) {
		return err_code;
	}

	err_code = request_next_event_earliest();
	if (err_code != NRF_SUCCESS) {
		(void) sd_radio_session_close();
		return err_code;
	}

	return NRF_SUCCESS;
}

uint32_t esb_timeslot_sd_stop(void) {
	return sd_radio_session_close();
}

/**@brief IRQHandler used for execution context management. 
 *       Any available handler can be used as we're not using the associated hardware.
 *       This handler is used to stop and disable UESB.
 */
void TIMESLOT_END_IRQHandler(void) {
	uint32_t err_code;

	/* Timeslot is about to end: stop UESB. */
	if (m_state == STATE_RX) {
		err_code = nrf_esb_stop_rx();
	}

	err_code = nrf_esb_flush_tx();
	APP_ERROR_CHECK(err_code);

	err_code = nrf_esb_flush_rx();
	APP_ERROR_CHECK(err_code);

	err_code = nrf_esb_disable();
	APP_ERROR_CHECK(err_code);

	m_total_timeslot_length = 0;
	m_state = STATE_IDLE;
}

/**@brief IRQHandler used for execution context management. 
 *       Any available handler can be used as we're not using the associated hardware.
 *       This handler is used to initiate UESB RX/TX.
 */
void TIMESLOT_BEGIN_IRQHandler(void) {
	if (m_state == STATE_IDLE) {
		nrf_esb_init(&nrf_esb_config);
		nrf_esb_set_address_length(3);
		nrf_esb_set_rf_channel(m_channel);
		nrf_esb_set_base_address_0(m_base_addr_0);
		nrf_esb_set_base_address_1(m_base_addr_0);
		nrf_esb_set_prefixes(m_addr_prefix, 1);

		if (!m_next_packet_set) {
			nrf_esb_start_rx();
			m_state = STATE_RX;
		} else {
			m_state = STATE_TX;
		}
	}

	CRITICAL_REGION_ENTER();
	if (m_next_packet_set) {
		m_next_packet_set = false;

		if (m_state == STATE_RX) {
			nrf_esb_stop_rx();
			m_state = STATE_TX;
		}

		nrf_esb_write_payload(&m_next_payload);
	} else {
		if (m_state == STATE_TX) {
			nrf_esb_start_rx();
			m_state = STATE_RX;
		}
	}
	CRITICAL_REGION_EXIT();
}

void esb_timeslot_set_next_packet(uint8_t *data, unsigned int len) {
	if (len >= 32 || m_next_packet_set) {
		return;
	}

	memcpy(m_next_payload.data, data, len);
	m_next_payload.pipe = 0;
	m_next_payload.noack = false;
	m_next_payload.length = len;
	m_next_packet_set = true;
}

void esb_timeslot_set_ch_addr(uint8_t ch, uint8_t b0, uint8_t b1, uint8_t b2) {
	m_channel = ch;
	m_addr_prefix[0] = b0;
	m_base_addr_0[0] = b1;
	m_base_addr_0[1] = b2;
}

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event) {
	if (p_event->evt_id == NRF_ESB_EVENT_TX_FAILED) {
		nrf_esb_flush_tx();
	}

	if (p_event->evt_id == NRF_ESB_EVENT_TX_SUCCESS) {

	}

	if (p_event->evt_id & NRF_ESB_EVENT_RX_RECEIVED) {
		/* Data reception is handled in a lower priority interrupt. */
		/* Call UESB_RX_HANDLE_IRQHandler later. */
		NVIC_SetPendingIRQ(UESB_RX_HANDLE_IRQn);
	}
}

uint32_t esb_timeslot_init(ut_data_handler_t evt_handler) {
	nrf_esb_config_t tmp_config = NRF_ESB_DEFAULT_CONFIG;

	m_evt_handler = evt_handler;

	memcpy(&nrf_esb_config, &tmp_config, sizeof(nrf_esb_config_t));
	nrf_esb_config.protocol = NRF_ESB_PROTOCOL_ESB_DPL;
	nrf_esb_config.retransmit_delay = 1000;
	nrf_esb_config.retransmit_count = 1;
	nrf_esb_config.tx_mode = NRF_ESB_TXMODE_AUTO;
	nrf_esb_config.bitrate = NRF_ESB_BITRATE_1MBPS;
	nrf_esb_config.event_handler = nrf_esb_event_handler;
	nrf_esb_config.mode = NRF_ESB_MODE_PTX;
	nrf_esb_config.selective_auto_ack = false;
	nrf_esb_config.crc = NRF_ESB_CRC_8BIT;
	nrf_esb_config.tx_output_power = NRF_ESB_TX_POWER_4DBM;

	// Using three available interrupt handlers for interrupt level management
	// These can be any available IRQ as we're not using any of the hardware,
	// simply triggering them through software
	NVIC_ClearPendingIRQ(TIMESLOT_END_IRQn);
	NVIC_SetPriority(TIMESLOT_END_IRQn, 1);
	NVIC_EnableIRQ(TIMESLOT_END_IRQn);

	NVIC_ClearPendingIRQ(TIMESLOT_BEGIN_IRQn);
	NVIC_SetPriority(TIMESLOT_BEGIN_IRQn, 1);
	NVIC_EnableIRQ(TIMESLOT_BEGIN_IRQn);

	NVIC_ClearPendingIRQ(UESB_RX_HANDLE_IRQn);
	NVIC_SetPriority(UESB_RX_HANDLE_IRQn, 1);
	NVIC_EnableIRQ(UESB_RX_HANDLE_IRQn);

	return NRF_SUCCESS;
}

void UESB_RX_HANDLE_IRQHandler(void) {
	nrf_esb_payload_t rx_payload;
	nrf_esb_read_rx_payload(&rx_payload);
	m_evt_handler(rx_payload.data, rx_payload.length);
}
