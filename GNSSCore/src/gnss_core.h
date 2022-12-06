/*
 GNSS Process Engine
 Copyright(R) 2021, Easy Navigation Technology Inc.
 Author: Dr. Yudan Yi
 Date: 09/20/2021
*/
#ifndef _GNSS_CORE_H_
#define _GNSS_CORE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "gnss_obs.h"
#include "GNSSCore_Api.h"

/* global constants */
#ifndef PI
#define	PI 3.14159265358979
#endif

#ifndef R2D
#define	R2D (180/PI)
#endif

#ifndef D2R
#define	D2R (PI/180)
#endif

#ifndef HALF_PI
#define HALF_PI (PI/2)
#endif

#ifndef TWO_PI
#define TWO_PI (PI+PI)
#endif

#ifndef ae_WGS84
#define ae_WGS84 6378137.0
#endif

#ifndef finv_WGS84
#define finv_WGS84 298.257223563
#endif

#ifndef MAX_SYS
#define MAX_SYS 6 /* GPS, GLO, GAL, BDS, QZS, SBS */
#endif

/* global functions */
GNSSCORE_API double baseline_distance(double* xyz1, double* xyz2);
GNSSCORE_API int is_time_same(double ws1, double ws2);
GNSSCORE_API int is_time_less(double ws1, double ws2);
GNSSCORE_API int    week_number(double time);
GNSSCORE_API double week_second(double time);
GNSSCORE_API double median_data(double *data, int n);

#ifndef MAX_EPOCH
#define MAX_EPOCH 3
#endif

#ifndef MAX_BASE
#define MAX_BASE 20
#endif

#ifndef MAX_ROVE
#define MAX_ROVE 200
#endif

#define OMGE        7.2921151467E-5     /* earth angular velocity (IS-GPS) (rad/s) */

#define RE_WGS84    6378137.0           /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563) /* earth flattening (WGS84) */
#define CLIGHT      299792458.0         /* speed of light (m/s) */

/* generate VRS measurement by offset */
GNSSCORE_API int make_vrs_measurement(sat_obs_t* src_obs, sat_vec_t* src_vec, double* src_xyz, int n, double* new_xyz, sat_obs_t* new_obs, sat_vec_t* new_vec);

/* struct for base station */
typedef struct
{
	int ID; /* station ID, will not change */
	double xyz[3]; /* assigned coordinate */
	double ws;
	epoch_t epochs[MAX_EPOCH];
	unsigned long numofepoch;
	int status;
}base_t;

/* data for rover */
typedef struct
{
	int ID;
	int baseID;
	double cur_xyz[3]; /* current coordinate to determine which base station to be used */
	double vrs_xyz[3]; /* coordinate used to compute the observation */
	double base_xyz[3];
	epoch_t epochs[MAX_EPOCH];
	int status;
}rove_t;

/* data for the network */
typedef struct
{
	int nb; /* number of base stations */
	int nr; /* number of rove stations */
	base_t bases[MAX_BASE]; /* all base station decoded */
	rove_t roves[MAX_ROVE]; /* all rove stations */
	double ws[MAX_BASE]; /* current epoch */
	double time;
	unsigned long numofepoch; /* total number of epochs */
	int status;
}network_t;

/* input */
/* add satellite eph to network, first cast sat_eph_t or glo_eph_t to char *, then pass the pointer */
GNSSCORE_API int  add_obs_to_network(network_t* network, int staid, epoch_t *epoch);
GNSSCORE_API int  add_vrs_to_network(network_t* network, int vrsid, double* xyz);
GNSSCORE_API int  add_bas_to_network(network_t* network, int staid, double* xyz);
/* delete base and rove stations from database */
GNSSCORE_API void del_bas_from_network(network_t *network, int staid);
GNSSCORE_API void del_vrs_from_network(network_t* network, int vrsid);
/* output */
GNSSCORE_API int  get_vrs_from_network(network_t* network, int vrsid, epoch_t* epoch);
/* data process and init */
GNSSCORE_API void network_processor(network_t* network);
GNSSCORE_API void network_init(network_t* network);

#ifdef __cplusplus
}
#endif


#endif