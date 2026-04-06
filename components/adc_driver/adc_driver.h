#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#ifdef __cplusplus
extern "C" {
#endif

void adc_init(void);
void adc_read(void);
float adc_read_current(void);
float adc_read_voltage(void);

#ifdef __cplusplus
}
#endif
