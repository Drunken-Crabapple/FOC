#include "task_public.h"
#include "fdcan.h"
#include "usart.h"
#include "QD4310.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <math.h>
#include <string.h>
#include <stdint.h>

typedef uint8_t PlugType;

enum {
    PLUG_CAN = 0x00,
    PLUG_UART = 0x01,
    PLUG_PWM = 0x02,
};

typedef uint8_t CmdType;

enum {
    CMD_NOP = 0x00,
    CMD_ENABLE = 0x01,
    CMD_DISABLE = 0x02,
    CMD_CURRENT_CTRL = 0x03,
    CMD_SPEED_CTRL = 0x04,
    CMD_ANGLE_CTRL = 0x05,
    CMD_LOW_SPEED_CTRL = 0x06,
    CMD_STEP_ANGLE_CTRL = 0x07,
};

typedef union {
    struct __attribute__((packed)) {
        CmdType cmd_type;
        int16_t data;
    } fields;
    uint8_t raw[3];
} RxData;

typedef struct {
    RxData cmd;
    PlugType plug;
} RxCommand;

_Static_assert(sizeof(RxData) == 3U, "RxData must match original C++ 3-byte command payload");
_Static_assert(sizeof(RxCommand) == 4U, "RxCommand must match original C++ command + plug layout");

uint8_t UART_RxBuffer[10];
static QueueHandle_t xQueue1;

static void FDCAN_Filter_INIT(FDCAN_HandleTypeDef *hfdcan);
static void CAN_Transmit(uint8_t length, uint8_t *pdata);
static uint8_t CRC8(const uint8_t *data, uint32_t len, uint8_t polynomial, uint8_t init,
                    uint8_t xor_out, bool input_invert, bool output_invert);

void StartCommunicateTask(void *argument) {
    (void)argument;
    xQueue1 = xQueueCreate(5, sizeof(RxCommand));
    while (!qd4310.foc.initialized) {
        delay(10);
    }
    FDCAN_Filter_INIT(&hfdcan1);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART_RxBuffer, sizeof(UART_RxBuffer));
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);

    RxCommand rx_command;
    while (true) {
        xQueueReceive(xQueue1, &rx_command, portMAX_DELAY);
        switch (rx_command.cmd.fields.cmd_type) {
            case CMD_NOP:
                break;
            case CMD_ENABLE:
                QD4310_Start(&qd4310);
                break;
            case CMD_DISABLE:
                QD4310_Stop(&qd4310);
                break;
            case CMD_CURRENT_CTRL:
                QD4310_Ctrl(&qd4310, FOC_CTRL_CURRENT, rx_command.cmd.fields.data * 10.0f / INT16_MAX);
                break;
            case CMD_SPEED_CTRL:
                QD4310_Ctrl(&qd4310, FOC_CTRL_SPEED, rx_command.cmd.fields.data * 1000.0f / INT16_MAX);
                break;
            case CMD_ANGLE_CTRL:
                QD4310_Ctrl(&qd4310, FOC_CTRL_ANGLE, rx_command.cmd.fields.data * 2.0f * FOC_PI / UINT16_MAX);
                break;
            case CMD_LOW_SPEED_CTRL:
                QD4310_Ctrl(&qd4310, FOC_CTRL_LOW_SPEED, rx_command.cmd.fields.data * 1000.0f / INT16_MAX);
                break;
            case CMD_STEP_ANGLE_CTRL:
                QD4310_Ctrl(&qd4310, FOC_CTRL_STEP_ANGLE, rx_command.cmd.fields.data * 2.0f * FOC_PI / INT16_MAX);
                break;
            default:
                break;
        }

        if (rx_command.cmd.fields.cmd_type <= CMD_STEP_ANGLE_CTRL) {
            union {
                struct __attribute__((packed)) {
                    uint8_t id;
                    uint8_t motor_state;
                    uint8_t error_code;
                    int16_t current;
                    int16_t speed;
                    int16_t angle;
                    uint8_t crc8;
                } data;
                uint8_t raw[10];
            } tx_data = {0};

            tx_data.data.id = qd4310.id;
            tx_data.data.motor_state = qd4310.foc.started ? 0x01U : 0x00U;
            tx_data.data.error_code = 0x00U;
            tx_data.data.current = (int16_t)(qd4310.foc.iq / 10.0f * INT16_MAX);
            tx_data.data.speed = (int16_t)(qd4310.foc.speed / 1000.0f * INT16_MAX);
            tx_data.data.angle = (int16_t)(QD4310_GetAngle(&qd4310) / (2.0f * FOC_PI) * UINT16_MAX);
            tx_data.data.crc8 = CRC8(tx_data.raw, sizeof(tx_data.raw) - 1U, 0x07, 0x00, 0x00, false, false);
            if (rx_command.plug == PLUG_CAN) {
                CAN_Transmit(sizeof(tx_data.raw) - 2U, tx_data.raw + 1U);
            } else if (rx_command.plug == PLUG_UART) {
                HAL_UART_Transmit_DMA(&huart3, tx_data.raw, sizeof(tx_data.raw));
            }
        }
    }
}

static void FDCAN_Filter_INIT(FDCAN_HandleTypeDef *hfdcan) {
    FDCAN_FilterTypeDef filter;
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x400;
    filter.FilterID2 = 0x40F;
    HAL_FDCAN_ConfigFilter(hfdcan, &filter);
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_Start(hfdcan);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    (void)RxFifo0ITs;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (hfdcan == &hfdcan1) {
        static FDCAN_RxHeaderTypeDef rx_header;
        static RxCommand rx_command = {.cmd = {.raw = {0}}, .plug = PLUG_CAN};
        if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0)) {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_command.cmd.raw);
            if (rx_header.Identifier == 0x400U + qd4310.id && rx_header.DataLength == 3U) {
                xQueueSendToBackFromISR(xQueue1, &rx_command, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (huart->Instance == huart3.Instance) {
        static RxCommand rx_command = {.cmd = {.raw = {0}}, .plug = PLUG_UART};
        if (UART_RxBuffer[0] == qd4310.id && Size == 5U &&
            CRC8(UART_RxBuffer, 4U, 0x07, 0x00, 0x00, false, false) == UART_RxBuffer[4]) {
            memcpy(rx_command.cmd.raw, UART_RxBuffer + 1U, sizeof(rx_command.cmd.raw));
            xQueueSendToBackFromISR(xQueue1, &rx_command, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        memset(UART_RxBuffer, 0, sizeof(UART_RxBuffer));
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART_RxBuffer, sizeof(UART_RxBuffer));
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {
        __HAL_UNLOCK(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART_RxBuffer, sizeof(UART_RxBuffer));
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
}

static void CAN_Transmit(uint8_t length, uint8_t *pdata) {
    static FDCAN_TxHeaderTypeDef tx_header = {
        0x500, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8, FDCAN_ESI_ACTIVE,
        FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN, FDCAN_NO_TX_EVENTS, 0
    };
    tx_header.Identifier = 0x500U + qd4310.id;
    tx_header.DataLength = length;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, pdata);
}

static uint8_t ReverseBits(uint8_t data) {
    data = ((data & 0x55U) << 1) | ((data & 0xAAU) >> 1);
    data = ((data & 0x33U) << 2) | ((data & 0xCCU) >> 2);
    data = ((data & 0x0FU) << 4) | ((data & 0xF0U) >> 4);
    return data;
}

static uint8_t CRC8(const uint8_t *data, uint32_t len, uint8_t polynomial, uint8_t init,
                    uint8_t xor_out, bool input_invert, bool output_invert) {
    uint8_t crc = init;
    while (len--) {
        crc ^= input_invert ? ReverseBits(*(data++)) : *(data++);
        for (uint8_t i = 0; i < 8U; ++i) {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ polynomial) : (uint8_t)(crc << 1);
        }
    }
    return output_invert ? ReverseBits(crc ^ xor_out) : (uint8_t)(crc ^ xor_out);
}
