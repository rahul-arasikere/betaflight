#include <platform.h>

#include "common/time.h"
#include "common/printf.h"
#include "common/printf_serial.h"

#include "drivers/serial.h"

#include "io/serial.h"
#include "io/xbee.h"

#ifdef XBEE_SUPPORT
static serialPort_t *xbeePort = NULL;

void xbeeInit()
{
    xbeePort = openSerialPort(XBEE_SERIAL_PORT, FUNCTION_RX_SERIAL, NULL, NULL, BAUD_115200, MODE_RXTX, SERIAL_STOPBITS_1);
    setPrintfSerialPort(xbeePort);
}
#endif