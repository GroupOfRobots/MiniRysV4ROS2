/**
 * Author:
 * MJBogusz <mjbogusz.email.address@domain.name.com>
 *
 * Based on platform.c
 * by:
 * Anh Luong <luong@eng.utah.edu>
 * Peter Hillyard <peterhillyard@gmail.com>
 * (source: https://github.com/SPAN-UofU/dw1000_bbb)
 */

#ifndef _DWM_PLATFORM_H
#define _DWM_PLATFORM_H

#include "deca_api/deca_device_api.h"
#include <stdint.h>

extern int spiFileDescriptor;

extern uint32_t spiMode;
extern uint8_t spiBits;
extern uint32_t spiSpeed;
extern uint16_t spiDelay;

void deca_sleep(unsigned int time_ms);

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer);
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer);

decaIrqStatus_t decamutexon(void);
void decamutexoff(decaIrqStatus_t s);

void dwt_readtx_sys_count(uint8 * timestamp);
void dwt_readrx_sys_count(uint8 * timestamp);

#endif
