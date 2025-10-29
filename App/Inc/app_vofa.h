
#ifndef VOFA_H
#define VOFA_H
#include <stdint.h>
#include "app_serial.h"
#include "string.h"

typedef union
{
    float float_t;
    uint8_t uint8_t[4];
} send_float;
void vofa_justfloat_output(float *data, uint8_t num ,Serialctrl *SerialCtrl);

#endif // !1#define 

