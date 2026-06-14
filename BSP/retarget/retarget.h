#ifndef _RETARGET_H__
#define _RETARGET_H__

#ifdef __cplusplus
extern "C" {
#endif

#define USE_TinyPrintf 1

#define STDIO_SUPPORT 1

#include "stm32g4xx_hal.h"
#if defined(__ARMCC_VERSION)
#define S_IFCHR 0020000
struct stat {
    int st_mode;
};
#else
#include <sys/stat.h>
#endif

#if USE_TinyPrintf == 1

#include "printf.h"

#endif

#ifndef EIO
#define EIO 5
#endif

#ifndef EBADF
#define EBADF 9
#endif

#if STDIO_SUPPORT == 1

#include <stdio.h>

#endif

#if STDIO_SUPPORT == 1

int _isatty(int fd);

int _write(int fd, char *ptr, int len);

int _close(int fd);

int _lseek(int fd, int ptr, int dir);

int _read(int fd, char *ptr, int len);

int _fstat(int fd, struct stat *st);

#endif

void RetargetInit(void);
void CDC_Receive_FS_Callback(uint8_t *Buf, uint32_t *Len);
void CDC_TransmitCplt_FS_Callback(void);
signed short shellRead(char *data, unsigned short len);
signed short shellWrite(char *data, unsigned short len);

#ifdef __cplusplus
};
#endif

#endif //#ifndef _RETARGET_H__
