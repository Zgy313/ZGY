#ifndef DAJINFA_H
#define DAJINFA_H

#include "../../../libraries/infineon_libraries/iLLD/TC26B/Tricore/Cpu/Std/Platform_Types.h"
#include "../../../libraries/zf_device/zf_device_mt9v03x.h"


extern uint8 original_image[MT9V03X_H][MT9V03X_W];
extern uint8 bin_Multi_threshold[MT9V03X_H][MT9V03X_W];
extern uint8 Difference_image[MT9V03X_H][MT9V03X_W];


void medianFilter2D(void);
void Multi_threshold(void);
void turn_to_bin(void);
void Difference_ratio_and(void);
#endif

