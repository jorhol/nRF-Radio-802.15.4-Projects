#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_802154.h"
#include "boards.h"
#include "app_uart.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define PACKET_HEADER_LENGTH 7
#define MAX_MESSAGE_SIZE 127
#define CHANNEL          11

static uint8_t m_message[MAX_MESSAGE_SIZE];

static volatile uint32_t rx_counter;

#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */


void uart_event_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

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

    uint8_t extended_address[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
    uint8_t short_address[]    = {0x05, 0x06};
    uint8_t pan_id[]           = {0x03, 0x04};

    nrf_802154_init();

    nrf_802154_short_address_set(short_address);
    nrf_802154_extended_address_set(extended_address);
    nrf_802154_pan_id_set(pan_id);

    nrf_802154_channel_set(CHANNEL);
    nrf_802154_receive();

    while (1)
    {
        // Intentionally empty
    }

    return 0;
}

void nrf_802154_received_timestamp(uint8_t * p_data, uint8_t length, int8_t power, uint8_t lqi, uint32_t time)
{
    (void) power;
    (void) lqi;

    if (length > MAX_MESSAGE_SIZE)
    {
        goto exit;
    }
    {
        memcpy(m_message, p_data, length);
        uint32_t err_code;
        uint8_t string_length = length - PACKET_HEADER_LENGTH - 2;
        uint8_t received_string[string_length];
        memcpy(received_string, p_data+PACKET_HEADER_LENGTH, string_length);
        printf("t: %i, RSSI: %d, %s", time, power, received_string);
    }
    rx_counter++;

exit:
    nrf_802154_buffer_free(p_data);

    return;
}
