#ifndef _RTCM_BUFF_H
#define _RTCM_BUFF_H

#include "GNSSCore_Api.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "gnss_eph.h"

#define RTCM3PREAMB 0xD3        /* rtcm ver.3 frame preamble */

GNSSCORE_API unsigned int crc24q    (unsigned char* buff, int len);
GNSSCORE_API         void setbitu   (unsigned char* buff, int pos, int len, unsigned int data);
GNSSCORE_API unsigned int getbitu   (unsigned char* buff, int pos, int len);
GNSSCORE_API          int getbits   (unsigned char *buff, int pos, int len);
GNSSCORE_API       double getbits_38(unsigned char* buff, int pos);

//void setbitu  (unsigned char *buff, int pos, int len, unsigned int data);
void setbits  (unsigned char *buff, int pos, int len, int data);
void setbitg  (unsigned char *buff, int pos, int len, int value);
void set38bits(unsigned char *buff, int pos, double value);

int decode_type1005_(unsigned char* buff, int len, int* staid, int* itrf, double* pos);
int decode_type1006_(unsigned char* buff, int len, int* staid, int* itrf, double* pos, double* anth);
int decode_type1007_(unsigned char* buff, int len, int* staid, char* des, int* setup);
int decode_type1008_(unsigned char* buff, int len, int* staid, char* des, int* setup, char* sno);

int decode_type1019_(unsigned char* buff, int len, sat_eph_t* eph); /* GPS */
int decode_type1020_(unsigned char* buff, int len, glo_eph_t* eph); /* GLO */
int decode_type1041_(unsigned char* buff, int len, sat_eph_t* eph); /* IRNSS */
int decode_type1042_(unsigned char* buff, int len, sat_eph_t* eph); /* BDS */
int decode_type1044_(unsigned char* buff, int len, sat_eph_t* eph); /* QZSS */
int decode_type1045_(unsigned char* buff, int len, sat_eph_t* eph); /* GAL */
int decode_type1046_(unsigned char* buff, int len, sat_eph_t* eph); /* GAL */

int decode_rtcm_data(unsigned char* buff, int nbyte, int* type, int* crc, int* staid, double* tow, int* sync, int* prn, int* frq, int* week);
int decode_rtcm_type(unsigned char* buff, int nbyte);
int change_rtcm_id(unsigned char* buff, int nbyte, int staid);

int encode_type1005(unsigned char* buff, int staid, double* pos);

/* user defined rtcm messages */
/* 4094 => rove/vrs information */
/* 4095 => current system time */
/* user defined message to store the received time tage */
int encode_type4095(unsigned char* buff, int staid, unsigned int week, double tow);
int decode_type4095(unsigned char* buff, int nbyte, int *staid, unsigned int *week, double *tow);
/* user define message to store the rover information */
int encode_type4094(unsigned char* buff, int vrsid, double* rov_xyz, double* vrs_xyz);
int decode_type4094(unsigned char* buff, int nbyte, int *vrsid, double* rov_xyz, double* vrs_xyz);

int check_rtcm_type(unsigned char* buff, int nbyte, int* len, int* crc, int* staid);

int is_rtcm_eph_type_(int type);
int is_rtcm_ssr_type_(int type);
int is_rtcm_obs_type_(int type); /* observation */
int is_rtcm_sta_type_(int type); /* station related */
int is_rtcm_msm4_(int type);
int is_rtcm_msm5_(int type);
int is_rtcm_msm6_(int type);
int is_rtcm_msm7_(int type);


#ifdef __cplusplus
}
#endif


#endif
