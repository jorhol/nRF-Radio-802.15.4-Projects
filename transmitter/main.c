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

#define PACKET_HEADER_LENGTH 7
#define MAX_MESSAGE_SIZE 120
#define CHANNEL          11

static volatile bool m_tx_in_progress;
static volatile bool m_tx_done;
static uint8_t message[MAX_MESSAGE_SIZE];

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

int main(int argc, char *argv[])
{
    (void) argc;
    (void) argv;

    uart_init(); 

    for (uint32_t i = 0; i < PACKET_HEADER_LENGTH; i++)
    {
        message[i] = i;
    }

    message[0] = 0x41;                // Set MAC header: short addresses, no ACK
    message[1] = 0x98;                // Set MAC header

    m_tx_in_progress = false;
    m_tx_done        = false;

    nrf_802154_init();
    nrf_802154_channel_set(CHANNEL);
    nrf_802154_receive();

    while (1)
    {
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
