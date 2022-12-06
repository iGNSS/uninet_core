//------------------------------------------------------------------------------
#ifndef _VRS_H_
#define _VRS_H_

#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------------
#include "GNSSCore_Api.h"
//------------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>

/* interface to the raw data buffer (rtcm3 MSM) */

/* set the rtcm data buffer to the engine 
*  The observation data must have 
* 1005/1006 with reciver ID => the coordinate will treat as known coordinate, please make sure to send the exact coordinate instead of approximate coordinate
* 1007/1008/1033 => antenna info and receiver ID => it will be used to correct the GNSS measurements, if one station do not have correpsonding antenna info, then ADVNULLANTENNA will be used
* 1230 => GLO L1 and L2 code-phase biases for each station 
* 1019 => GPS Ephemeris 
* 1020 => GLO Ephemeris 
* 1042 => BDS Ephemeris 
* 1044 => QZS Ephemeris 
* 1045 => GAL Ephemeris (F/NAV)
* 1046 => GAL Ephemeris (I/NAV)
* 1074/1075/1076/1077 GPS MSM observation
* 1084/1085/1086/1087 GLO MSM observation
* 1094/1095/1096/1097 GAL MSM observation
* 1104/1105/1106/1107 SBS MSM observation (Support-TBD)
* 1114/1115/1116/1117 QZS MSM observation
* 1124/1125/1126/1127 BDS MSM observation
*/
/* with base station coordinate */
GNSSCORE_API int set_rtcm_data_buff(int staid, uint8_t* buffer, int nbyte, double *xyz);

/* add vrs rove data */
GNSSCORE_API int add_vrs_rover_data(int vrsid, double* xyz);
/* get the rtcm buffer for the rover, it will include
* 1005 => vrs location
* 1074 => GPS MSM observation
* 1084 => GLO MSM observation
* 1094 => GAL MSM observation
* 1114 => QZS MSM observation
* 1124 => BDS MSM observation
* 1230 => GLO L1 and L2 code-phase biases
* return the nbyte of the total buffer
* note that, the buffer should be big enough to hold all message
*/
GNSSCORE_API int get_vrs_rove_buff(int vrsid, uint8_t*buffer);

/* delete rove station using ID */
GNSSCORE_API void del_vrs_rove_data(int vrsid);

/* delete receiver station using ID */
GNSSCORE_API void del_vrs_base_data(int staid);

/* system reset */
GNSSCORE_API void system_reset();

/* system exit for house-keeping (free memory, close files, etc.) */
GNSSCORE_API void system_exit();

/* set the approximate time for post-processing */
GNSSCORE_API void set_appr_time(int year, int mon, int day, int hour);

/* raw & log data options */
GNSSCORE_API void set_raw_data_option(int opt);
GNSSCORE_API void set_log_data_option(int opt);

/* system status output */
GNSSCORE_API void system_status_output(FILE* fout);

//------------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif
