/*
 GNSS Process Engine
 Copyright(R) 2021, Easy Navigation Technology Inc.
 Author: Dr. Yudan Yi
 Date: 09/20/2021
*/
#ifndef _G_MODEL_H_
#define _G_MODEL_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "GNSSCore_Api.h"


#ifndef PI
#define	PI 3.14159265358979
#endif

#ifndef R2D
#define	R2D (180/PI)
#endif

#ifndef D2R
#define	D2R (PI/180)
#endif

GNSSCORE_API void xyz2blh_(const double* xyz, double* blh);
GNSSCORE_API void blh2xyz_(const double* blh, double* xyz);

#ifdef __cplusplus
}
#endif

#endif