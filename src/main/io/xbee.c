#include <platform.h>
#include <stdio.h>
#include <stdarg.h>

#include "common/printf.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "io/serial.h"
#include "io/xbee.h"

#ifdef XBEE_SUPPORT

#define XBEE_STRING_BUFF_LEN 212

static serialPort_t *xbeePort = NULL;
static char xbeeStringBuff[XBEE_STRING_BUFF_LEN];

void xprintf(const char *str, ...)
{
    va_list arg;
    va_start(arg, str);
    size_t len = vsnprintf(xbeeStringBuff, XBEE_STRING_BUFF_LEN, str, arg);
    va_end(arg);
    serialWriteBuf(xbeePort, (const uint8_t *)xbeeStringBuff, (int)len);
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
    xbeePort = openSerialPort(XBEE_SERIAL_PORT, FUNCTION_NONE, NULL, NULL, 115200, MODE_RXTX, SERIAL_NOT_INVERTED);
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
    delay(300);
}
#endif