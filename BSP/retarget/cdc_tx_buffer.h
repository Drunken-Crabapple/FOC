#ifndef CDC_TX_BUFFER_H
#define CDC_TX_BUFFER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CDC_TX_BUFFER_CAPACITY 512U

typedef uint8_t (*CdcTransmitFunc)(uint8_t *buf, uint16_t len);

typedef struct {
    char buffer1[CDC_TX_BUFFER_CAPACITY];
    char buffer2[CDC_TX_BUFFER_CAPACITY];
    char *main_buffer;
    char *vice_buffer;
    uint16_t buffer_index;
    uint8_t transmitting;
    CdcTransmitFunc tx_func;
} CdcTxBuffer;

void CDC_TxBuffer_Init(CdcTxBuffer *buffer, CdcTransmitFunc tx_func);
uint8_t CDC_TxBuffer_Write(CdcTxBuffer *buffer, const char *data, uint16_t len);
void CDC_TxBuffer_TransmitComplete(CdcTxBuffer *buffer);

#ifdef __cplusplus
}
#endif

#endif
