/* 串口（CV610 侧 POSIX termios）：打开、原始模式、非阻塞读、写。 */
#ifndef EDGE_UART_H
#define EDGE_UART_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    int fd;
} uart_t;

/* 打开串口：8N1、无流控、原始模式、非阻塞读。失败返回 -1（fd 保持 -1）。 */
int uart_open(uart_t *u, const char *device, int baud);

/* 非阻塞读取所有可用字节到 data，返回实际读取数；无数据返回 0；错误返回 -1。 */
int uart_read(uart_t *u, uint8_t *data, size_t max);

/* 非阻塞写＋poll，整条写入最多等待20ms；返回0仅表示内核接受，-1含部分写失败。 */
int uart_write_all(uart_t *u, const uint8_t *data, size_t n);
/* 一次write，不等待、不重试：0=完整写入，1=本次未写任何字节，-1=错误，-2=部分写入。 */
int uart_try_write_all(uart_t *u, const uint8_t *data, size_t n);

void uart_close(uart_t *u);

#endif /* EDGE_UART_H */
