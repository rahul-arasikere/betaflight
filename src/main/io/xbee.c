#include <platform.h>

#include "common/printf.h"
#include "common/printf_serial.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "io/serial.h"
#include "io/xbee.h"

#ifdef XBEE_SUPPORT
static serialPort_t *xbeePort = NULL;

void xbeeInit()
{
    xbeePort = openSerialPort(XBEE_SERIAL_PORT, FUNCTION_RX_SERIAL, NULL, NULL, BAUD_115200, MODE_RXTX, SERIAL_STOPBITS_1);
    if (millis() < 10000)
    {
        /* delay */
        delay(10000 - millis());
    }
    setPrintfSerialPort(xbeePort);
    tfp_printf(" b\r");
    delay(300);
    while(serialRxBytesWaiting(xbeePort)) {
        serialRead(xbeePort);
    }
}
#endif