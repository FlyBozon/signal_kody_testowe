#ifndef __SPI_H__
#define __SPI_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

extern SPI_HandleTypeDef hspi2;

void MX_SPI2_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */
