#include <platform.h>
#include <stdio.h>

#include "common/printf.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "io/serial.h"
#include "io/xbee.h"

#ifdef XBEE_SUPPORT
static serialPort_t *xbeePort = NULL;
static char xbeeStringBuff[256];

void xprintf(const char *str, ...)
{
    size_t len = snprintf(xbeeStringBuff, 256, str);
    serialWriteBuf(xbeePort, (const uint8_t *)xbeeStringBuff, len);
}

uint8_t xbeeGetByte()
{
    return serialRead(xbeePort);
}

int xbeeGetBytesWaiting()
{
    return serialRxBytesWaiting(xbeePort);
}

void xbeeInit()
{
    xbeePort = openSerialPort(XBEE_SERIAL_PORT, FUNCTION_RX_SERIAL, NULL, NULL, BAUD_115200, MODE_RXTX, SERIAL_STOPBITS_1);
    if (millis() < 10000)
    {
        /* delay */
        delay(10000 - millis());
    }
    xprintf(" b\r");
    delay(300);
    while (serialRxBytesWaiting(xbeePort))
    {
        serialRead(xbeePort);
    }

    xprintf("xbee init reached!\n");
}
#endif