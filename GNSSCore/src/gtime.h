/*
 GNSS Process Engine
 Copyright(R) 2021, Easy Navigation Technology Inc.
 Author: Dr. Yudan Yi
 Date: 09/20/2021
*/
#ifndef _G_TIME_H_
#define _G_TIME_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "GNSSCore_Api.h"

/* data struct used in the engine */

GNSSCORE_API double ConvertToTimeGPS(int year, int mon, int day, int hour, int min, double sec, int* wn);
GNSSCORE_API double ConvertFromTimeGPS(int wn, double ws, int* year, int* mon, int* day, int* hour, int* min);

#ifdef __cplusplus
}
#endif

#endif