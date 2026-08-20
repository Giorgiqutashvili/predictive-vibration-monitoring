/*
 * iis3dwb_platform.h
 *
 *  Created on: Aug 20, 2026
 *      Author: lukam
 */

#ifndef IIS3DWB_PLATFORM_H_
#define IIS3DWB_PLATFORM_H_

#include <stdint.h>
#include "main.h"
#include "iis3dwb_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

extern stmdev_ctx_t dev_ctx;

/* Platform-specific SPI read/write function declarations */
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

/* Sensor initialization function declaration */
void IIS3DWB_Init_Platform(void);

#ifdef __cplusplus
}
#endif

#endif /* IIS3DWB_PLATFORM_H_ */
