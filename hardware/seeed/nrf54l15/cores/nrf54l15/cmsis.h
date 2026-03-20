#ifndef CMSIS_H
#define CMSIS_H

#include <nrf54l15.h>

#if !defined(NRF_I2S) && defined(NRF_I2S20)
#define NRF_I2S NRF_I2S20
#endif

#if !defined(NRF_QDEC) && defined(NRF_QDEC20)
#define NRF_QDEC NRF_QDEC20
#endif

#endif
