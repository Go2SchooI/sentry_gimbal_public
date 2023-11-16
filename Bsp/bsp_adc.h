#ifndef _BSP_ADC_H
#define _BSP_ADC_H
#include "stdint.h"

void init_vrefint_reciprocal(void);
float get_temprate(void);
float get_battery_voltage(void);
uint8_t get_hardware_version(void);
#endif
