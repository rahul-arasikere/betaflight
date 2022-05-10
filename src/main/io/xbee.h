#pragma once

#ifdef XBEE_SUPPORT

#ifndef XBEE_SERIAL_PORT
#error "XBEE_SERIAL_PORT needs to be defined with XBEE_ENABLE"
#endif

void xbeeInit();
void xprintf(const char *str, ...);
uint8_t xbeeGetByte();
int xbeeGetBytesWaiting();

#endif