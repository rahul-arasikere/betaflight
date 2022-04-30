#include <platform.h>

#include "common/time.h"
#include "common/printf.h"
#include "common/printf_serial.h"

#include "drivers/rx/xbee.h"
#include "drivers/serial_uart.h"

#include "io/serial.h"

static serialPort_t *xbee_port = NULL;
static uartPort_t *xbee_uart = NULL;

void xbee_init()
{
    xbee_port = openSerialPort(XBEE_SERIAL_PORT, FUNCTION_RX_SERIAL, NULL, NULL, XBEE_BAUD_RATE, MODE_RXTX, SERIAL_STOPBITS_1);
}