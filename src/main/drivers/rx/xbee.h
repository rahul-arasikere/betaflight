#pragma once

#ifndef XBEE_BAUD_RATE
#error "XBEE_BAUD_RATE needs to be defined with XBEE_ENABLE"
#endif

#ifndef XBEE_SERIAL_PORT
#error "XBEE_SERIAL_PORT needs to be defined with XBEE_ENABLE"
#endif

void xbee_init();