#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_802154.h"

#define MAX_MESSAGE_SIZE 127
#define CHANNEL          11

static uint8_t m_message[MAX_MESSAGE_SIZE];

static volatile uint32_t rx_counter;

int main(int argc, char *argv[])
{
    (void) argc;
    (void) argv;

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

void nrf_802154_received(uint8_t * p_data, uint8_t length, int8_t power, uint8_t lqi)
{
    (void) power;
    (void) lqi;

    if (length > MAX_MESSAGE_SIZE)
    {
        goto exit;
    }

    memcpy(m_message, p_data, length);
    rx_counter++;

exit:
    nrf_802154_buffer_free(p_data);

    return;
}
