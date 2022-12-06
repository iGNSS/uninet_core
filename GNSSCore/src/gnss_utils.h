/*
 GNSS Process Engine
 Copyright(R) 2021, Easy Navigation Technology Inc.
 Author: Dr. Yudan Yi
 Date: 02/26/2022
 provide the conversion between rtklib-style data struct with gnss core engine 
*/
#ifndef _GNSS_UTILS_H_
#define _GNSS_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "GNSSCore_Api.h"

#include <stdint.h>

/* from rtklib code/data */
#include "gnss.h"
/* used in the engine */
#include "gnss_obs.h"

GNSSCORE_API char sys2char(int sys);
GNSSCORE_API int  addobs(obs_t* obs, obsd_t *obsd);
GNSSCORE_API int  addepoch(epoch_t* epoch, obsd_t* obsd);
GNSSCORE_API int  obs2epoch(obs_t *obs, epoch_t *epoch);
GNSSCORE_API int  obsnav2epoch(obs_t* obs, nav_t *nav, epoch_t* epoch);
GNSSCORE_API int  epoch2obs(epoch_t* epoch, obs_t* obs);

#ifdef __cplusplus
}
#endif

#endif