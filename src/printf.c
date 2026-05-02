#include "debug_log.h"

#if DEBUG_LOG_ENABLED

#include "main.h"

#include <errno.h>
#include <unistd.h>

#define LOG_UART huart2
#define LOG_UART_TIMEOUT_MS 100U

extern UART_HandleTypeDef LOG_UART;

int _write(int file, char *ptr, int len)
{
    if (file == STDOUT_FILENO || file == STDERR_FILENO) {
        if (len <= 0) {
            return 0;
        }

        if (HAL_UART_Transmit(&LOG_UART,
                              (uint8_t *)ptr,
                              (uint16_t)len,
                              LOG_UART_TIMEOUT_MS) == HAL_OK) {
            return len;
        }

        errno = EIO;
        return -1;
    }

    errno = EBADF;
    return -1;
}

#endif
