// Simple magnetometer utilities
#ifndef MAG_H
#define MAG_H

#include <math.h>

static inline void normalizeMag(float &mx, float &my, float &mz) {
    float n = sqrtf(mx*mx + my*my + mz*mz);
    if (n <= 1e-9f) return;
    mx /= n; my /= n; mz /= n;
}

#endif // MAG_H
