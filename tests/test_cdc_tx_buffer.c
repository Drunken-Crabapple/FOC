#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../BSP/retarget/cdc_tx_buffer.h"

static uint8_t tx_storage[4][64];
static uint16_t tx_lengths[4];
static int tx_call_count;
static int tx_busy_mode;
static int tx_in_flight;

static uint8_t fake_tx(uint8_t *buf, uint16_t len) {
    if (tx_busy_mode || tx_in_flight) {
        return 1U;
    }

    assert(tx_call_count < 4);
    memcpy(tx_storage[tx_call_count], buf, len);
    tx_lengths[tx_call_count] = len;
    tx_call_count++;
    tx_in_flight = 1;
    return 0U;
}

static void reset_state(void) {
    memset(tx_storage, 0, sizeof(tx_storage));
    memset(tx_lengths, 0, sizeof(tx_lengths));
    tx_call_count = 0;
    tx_busy_mode = 0;
    tx_in_flight = 0;
}

static void test_flushes_buffer_on_complete(void) {
    CdcTxBuffer buffer;
    char first[] = "AB";
    char second[] = "CD";

    reset_state();
    CDC_TxBuffer_Init(&buffer, fake_tx);

    assert(CDC_TxBuffer_Write(&buffer, first, (uint16_t)strlen(first)) == 1);
    assert(tx_call_count == 1);
    assert(tx_lengths[0] == 2);
    assert(memcmp(tx_storage[0], "AB", 2) == 0);

    assert(CDC_TxBuffer_Write(&buffer, second, (uint16_t)strlen(second)) == 1);
    assert(tx_call_count == 1);

    tx_in_flight = 0;
    CDC_TxBuffer_TransmitComplete(&buffer);
    assert(tx_call_count == 2);
    assert(tx_lengths[1] == 2);
    assert(memcmp(tx_storage[1], "CD", 2) == 0);
}

static void test_retries_when_endpoint_was_busy(void) {
    CdcTxBuffer buffer;
    char payload[] = "HELLO";

    reset_state();
    CDC_TxBuffer_Init(&buffer, fake_tx);

    tx_busy_mode = 1;
    assert(CDC_TxBuffer_Write(&buffer, payload, (uint16_t)strlen(payload)) == 1);
    assert(tx_call_count == 0);

    tx_busy_mode = 0;
    CDC_TxBuffer_TransmitComplete(&buffer);
    assert(tx_call_count == 1);
    assert(tx_lengths[0] == 5);
    assert(memcmp(tx_storage[0], "HELLO", 5) == 0);
}

int main(void) {
    test_flushes_buffer_on_complete();
    test_retries_when_endpoint_was_busy();
    puts("test_cdc_tx_buffer: PASS");
    return 0;
}
