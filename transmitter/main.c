#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "nrf_802154.h"
#include "boards.h"
#include "nrf_delay.h"
#include "app_uart.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#define PACKET_HEADER_LENGTH 7
#define MAX_MESSAGE_SIZE 120
#define CHANNEL          11

static volatile bool m_tx_in_progress;
static volatile bool m_tx_done;
static uint8_t message[MAX_MESSAGE_SIZE];
static uint8_t usb_message[MAX_MESSAGE_SIZE];
static uint8_t usb_message_index = PACKET_HEADER_LENGTH;

/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif


static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1


/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);

#define READ_SIZE 1

static char m_rx_buffer[READ_SIZE];
/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {

            /*Setup first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                   m_rx_buffer,
                                                   READ_SIZE);
            UNUSED_VARIABLE(ret);
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;
            do
            {
                /*Get amount of data transfered*/
                size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);

                /* Fetch data until internal buffer is empty */
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                            m_rx_buffer,
                                            READ_SIZE);
                usb_message[usb_message_index] = *m_rx_buffer;
                usb_message_index++;
                if(*m_rx_buffer == '\n' || *m_rx_buffer == '\r')
                {
                usb_message[usb_message_index] = '\0';
                    do
                    {
                
                        if (!m_tx_in_progress)
                        {
                            m_tx_in_progress = nrf_802154_transmit(usb_message, (PACKET_HEADER_LENGTH + usb_message_index), true);
                        }
                        if (m_tx_done)
                        {
                            m_tx_in_progress = false;
                            m_tx_done        = false;
                        }
                    } while (!m_tx_in_progress);
                    usb_message_index = PACKET_HEADER_LENGTH;
                }
            } while (ret == NRF_SUCCESS);
            
            break;
        }
        default:
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            break;
        case APP_USBD_EVT_DRV_RESUME:
            break;
        case APP_USBD_EVT_STARTED:
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            break;
        case APP_USBD_EVT_POWER_DETECTED:

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            app_usbd_start();
            break;
        default:
            break;
    }
}


#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

/**@brief   Function for handling app_uart events.**/
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[MAX_MESSAGE_SIZE];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r'))
            {
                if (index > 1)
                {
                    uint8_t length = index < (MAX_MESSAGE_SIZE-PACKET_HEADER_LENGTH) ? index : (MAX_MESSAGE_SIZE-PACKET_HEADER_LENGTH);
                    memcpy((message+PACKET_HEADER_LENGTH), data_array, length);
                    if(PACKET_HEADER_LENGTH + index < MAX_MESSAGE_SIZE)
                    {
                        message[PACKET_HEADER_LENGTH+length] = '\0';
                    }
                    else
                    {
                        message[MAX_MESSAGE_SIZE-2] = '\n';
                        message[MAX_MESSAGE_SIZE-1] = '\0';
                    }
                    do
                    {
                        
                        if (!m_tx_in_progress)
                        {
                            m_tx_in_progress = nrf_802154_transmit(message, (PACKET_HEADER_LENGTH + length), true);
                        }
                        if (m_tx_done)
                        {
                            m_tx_in_progress = false;
                            m_tx_done        = false;
                        }
                    } while (!m_tx_in_progress);
                }
                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}


/**@snippet [UART Initialization] */

void usbd_init(void)
{
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };
    
    
    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
    
    nrf_drv_clock_lfclk_request(NULL);

    while(!nrf_drv_clock_lfclk_is_running())
    {
        /* Just waiting */
    }

    app_usbd_serial_num_generate();

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

    if (USBD_POWER_DETECTION)
    {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        app_usbd_enable();
        app_usbd_start();
    }
}

int main(int argc, char *argv[])
{
    (void) argc;
    (void) argv;

    usbd_init();
    uart_init(); 

    for (uint32_t i = 0; i < PACKET_HEADER_LENGTH; i++)
    {
        message[i] = i;
        usb_message[i] = i;
    }

    message[0] = 0x41;                // Set MAC header: short addresses, no ACK
    usb_message[0] = 0x41;
    message[1] = 0x98;                // Set MAC header
    usb_message[1] = 0x98;

    m_tx_in_progress = false;
    m_tx_done        = false;

    nrf_802154_init();
    nrf_802154_channel_set(CHANNEL);
    nrf_802154_receive();

    while (1)
    {
         while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }
        // Wait for UART
    }

    return 0;
}

void nrf_802154_transmitted(const uint8_t * p_frame, uint8_t * p_ack, uint8_t length, int8_t power, uint8_t lqi)
{
    (void) p_frame;
    (void) length;
    (void) power;
    (void) lqi;

    m_tx_done = true;

    if (p_ack != NULL)
    {
        nrf_802154_buffer_free(p_ack);
    }
}
