/*
 GNSS Process Engine
 Copyright(R) 2021, Easy Navigation Technology Inc.
 Author: Dr. Yudan Yi
 Date: 09/20/2021
*/
#ifndef _GNSS_OBS_H_
#define _GNSS_OBS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* data struct used in the engine */

/*
*                       0     1     2     3     4
*           --------------------------------------
*            GPS       L1    L2    L5     -     -
*            GLONASS   G1    G2    G3     -     -  (G1=G1,G1a,G2=G2,G2a)
*            Galileo   E1    E5b   E5a   E6   E5ab
*            QZSS      L1    L2    L5    L6     -
*            SBAS      L1     -    L5     -     -
*            BDS       B1    B2    B2a   B3   B2ab (B1=B1I,B1C,B2=B2I,B2b)
*            NavIC     L5     S     -     -     -
*/

#ifndef MAX_FRQ 
#define MAX_FRQ 5
#endif

#ifndef MAX_SAT 
#define MAX_SAT 80
#endif

typedef struct
{
    uint8_t sat; /* satellite/receiver number */
    uint8_t sys; /* satellite sys ID G,R,E,C,J,S */
    uint8_t prn; /* satellite prn number/slot number */
    uint16_t SNR[MAX_FRQ]; /* signal strength (0.001 dBHz) */
    uint8_t  LLI[MAX_FRQ]; /* loss of lock indicator */
    uint8_t code[MAX_FRQ]; /* code indicator (CODE_???) */
    double L[MAX_FRQ]; /* observation data carrier-phase (cycle) */
    double P[MAX_FRQ]; /* observation data pseudorange (m) */
    float  D[MAX_FRQ]; /* observation data doppler frequency (Hz) */
    double offset[MAX_FRQ]; /* constant carrier phase offset in cycle */
    double wave[MAX_FRQ]; /* wavelength of the current phase */
}sat_obs_t;

typedef struct {
    int	   sat;        /*prn*/
    double rs[6];
    double dts[2];
    double var;
    int svh;
    double azel[2];    /*azimuth,elevation*/
    double e[3];       /*partial deviation*/
    double tgd;        /* tgd*/
    double r;          /* vector */
    double tro;        /* tropospheric */
}sat_vec_t;

typedef struct
{
    int n;
    int wk;
    double ws;
    double pos[3];
    sat_obs_t obs[MAX_SAT];
    sat_vec_t vec[MAX_SAT];
}epoch_t;

#ifdef __cplusplus
}
#endif

#endif