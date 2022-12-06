#ifndef _RTCM_H
#define _RTCM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gnss_obs.h"
#include "gnss_eph.h"

typedef struct {                    /* multi-signal-message header type */
    unsigned char iod;              /* issue of data station */
    unsigned char time_s;           /* cumulative session transmitting time */
    unsigned char clk_str;          /* clock steering indicator */
    unsigned char clk_ext;          /* external clock indicator */
    unsigned char smooth;           /* divergence free smoothing indicator */
    unsigned char tint_s;           /* soothing interval */
    unsigned char nsat,nsig;        /* number of satellites/signals */
    unsigned char sats[64];         /* satellites */
    unsigned char sigs[32];         /* signals */
    unsigned char cellmask[64];     /* cell mask */
} msm_h_t;

typedef struct 
{
    double r[64],rr[64],pr[64],cp[64],rrf[64],cnr[64];
    int lock[64];
    int ex[64],half[64];
}msm_d_t;

int decode_msm4_(unsigned char *buff, int len, unsigned char sys, int* staid, double* ws, sat_obs_t *obs, int *nobs, msm_h_t *msmh, msm_d_t *msmd);
int decode_msm5_(unsigned char *buff, int len, unsigned char sys, int* staid, double* ws, sat_obs_t *obs, int* nobs, msm_h_t *msmh, msm_d_t *msmd);
int decode_msm6_(unsigned char *buff, int len, unsigned char sys, int* staid, double* ws, sat_obs_t *obs, int* nobs, msm_h_t *msmh, msm_d_t *msmd);
int decode_msm7_(unsigned char *buff, int len, unsigned char sys, int* staid, double* ws, sat_obs_t *obs, int* nobs, msm_h_t *msmh, msm_d_t *msmd);
int decode_msmx_(unsigned char* buff, int len, int* staid, double* ws, sat_obs_t* obs, int* nobs, msm_h_t* msmh, msm_d_t* msmd);

double satwavelen_code(unsigned char sys, unsigned char prn, int code);

int decode_rtcm_obs(unsigned char* buff, int nbyte, double* ws, sat_obs_t* obs, int* nobs, int* type, int* staid, int* crc, msm_h_t* msmh, msm_d_t* msmd);

#ifdef __cplusplus
}
#endif


#endif
