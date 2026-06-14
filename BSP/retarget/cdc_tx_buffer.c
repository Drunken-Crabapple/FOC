#include "cdc_tx_buffer.h"

#include <string.h>
#include "stm32g4xx_hal.h"

static void CDC_TxBuffer_StartTransmit(CdcTxBuffer *buffer) {
    char *swap_buffer;

    if (buffer->buffer_index == 0U || buffer->tx_func == 0 || buffer->transmitting != 0U) {
        return;
    }

    if (buffer->tx_func((uint8_t *)buffer->vice_buffer, buffer->buffer_index) == 0U) {
        swap_buffer = buffer->main_buffer;
        buffer->main_buffer = buffer->vice_buffer;
        buffer->vice_buffer = swap_buffer;
        buffer->buffer_index = 0U;
        buffer->transmitting = 1U;
    }
}

void CDC_TxBuffer_Init(CdcTxBuffer *buffer, CdcTransmitFunc tx_func) {
    memset(buffer, 0, sizeof(*buffer));
    buffer->main_buffer = buffer->buffer1;
    buffer->vice_buffer = buffer->buffer2;
    buffer->tx_func = tx_func;
}

uint8_t CDC_TxBuffer_Write(CdcTxBuffer *buffer, const char *data, uint16_t len) {
    uint8_t result;

    HAL_NVIC_DisableIRQ(USB_LP_IRQn);

    if (len > (uint16_t)(CDC_TX_BUFFER_CAPACITY - buffer->buffer_index)) {
        HAL_NVIC_EnableIRQ(USB_LP_IRQn);
        return 0U;
    }

    memcpy(buffer->vice_buffer + buffer->buffer_index, data, len);
    buffer->buffer_index = (uint16_t)(buffer->buffer_index + len);

    CDC_TxBuffer_StartTransmit(buffer);

    result = 1U;
    HAL_NVIC_EnableIRQ(USB_LP_IRQn);
    return result;
}

void CDC_TxBuffer_TransmitComplete(CdcTxBuffer *buffer) {
    buffer->transmitting = 0U;
    CDC_TxBuffer_StartTransmit(buffer);
}
