#ifndef __IO_DRIVER_H
#define __IO_DRIVER_H

#include "ameba_soc.h"

#ifdef __cplusplus
extern "C" {
#endif

void test_1();
void test_2();
void test_3();

#define encoder_neck_power_on()  test_1();
#define encoder_neck_power_off() test_2();
#define encoder_neck_read()      test_3();



#ifdef __cplusplus
}
#endif

#endif  // __IO_DRIVER_H