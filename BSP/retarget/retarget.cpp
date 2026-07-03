#include <_ansi.h>
#include <_syslist.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/times.h>
#include "retarget.h"
#include <stdint.h>

#include "usbd_cdc_if.h"
#include "CharCircularQueue.h"
#include "CDC_Tx_DualBuffer.h"

#if !defined(OS_USE_SEMIHOSTING)

#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

void RetargetInit() {
#if USE_TinyPrintf == 0 && STDIO_SUPPORT == 1
    /* Disable I/O buffering for STDOUT stream, so that
     * chars are sent out as soon as they are printed. */
    setvbuf(stdout, NULL, _IONBF, 0);
#endif
}

namespace {
constexpr uint16_t RX_QUEUE_SIZE = 128U;
uint8_t rx_queue[RX_QUEUE_SIZE]{};
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;

bool rx_is_empty() {
    return rx_head == rx_tail;
}

bool rx_is_full() {
    return static_cast<uint16_t>((rx_head + 1U) % RX_QUEUE_SIZE) == rx_tail;
}

void rx_enqueue(uint8_t data) {
    if (rx_is_full()) return;
    rx_queue[rx_head] = data;
    rx_head = static_cast<uint16_t>((rx_head + 1U) % RX_QUEUE_SIZE);
}

bool rx_dequeue(char &data) {
    if (rx_is_empty()) return false;
    data = static_cast<char>(rx_queue[rx_tail]);
    rx_tail = static_cast<uint16_t>((rx_tail + 1U) % RX_QUEUE_SIZE);
    return true;
}
}

TxDualBuffer<char, 512> tx_buffer(&CDC_Transmit_FS);

void CDC_Receive_FS_Callback(uint8_t *Buf, uint32_t *Len) {
    auto length = *Len;
    while (length--) rx_enqueue(*(Buf++));
}

void CDC_TransmitCplt_FS_Callback() {
    tx_buffer.transmitComplete();
}

signed short shellRead(char *data, unsigned short len) {
    signed short i = 0;
    for (i = 0; i < len; ++i, ++data) {
        if (!rx_dequeue(*data)) break;
    }
    delay(1); // 延时1ms,因为shellTask是死循环一点delay都没有,为了让IDLE线程能够运行以释放内存等
    return i;
}

signed short shellWrite(char *data, unsigned short len) {
    return tx_buffer.inBuffer(data, len) ? 0 : -1;
}

#if USE_TinyPrintf == 1
// provide your own _putchar() low level function as console/serial output
void _putchar(char character) {
    shellWrite(&character, 1);
}
#endif

#if STDIO_SUPPORT == 1
int _write(int fd, char *ptr, int len) {
    if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
        return shellWrite(ptr, len) == 0 ? len : EIO;
    }
    errno = EBADF;
    return -1;
}

int _read(int fd, char *ptr, int len) {
    if (fd == STDIN_FILENO) {
        return shellRead(ptr, len);
    }
    errno = EBADF;
    return -1;
}

int _close(int fd) {
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
        return 0;

    errno = EBADF;
    return -1;
}

int _fstat(int fd, struct stat *st) {
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) {
        st->st_mode = S_IFCHR;
        return 0;
    }

    errno = EBADF;
    return -1;
}

int _isatty(int fd) {
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
        return 1;

    errno = EBADF;
    return 0;
}

int _lseek(int fd, int ptr, int dir) {
    (void)fd;
    (void)ptr;
    (void)dir;

    errno = EBADF;
    return -1;
}
#endif

#endif //#if !defined(OS_USE_SEMIHOSTING)
