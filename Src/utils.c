//
// Created by morrissettal2 on 11/28/19.
//

#include "utils.h"

#include <math.h>

uint16_t sbusToPwm(uint16_t val) {
    return (uint16_t )roundf((float)val*(75.0f/1639) + 294.129f);
}