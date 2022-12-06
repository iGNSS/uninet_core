/*
 GNSS Process Engine
 Copyright(R) 2021, Easy Navigation Technology Inc.
 Author: Dr. Yudan Yi
 Date: 09/20/2021
*/
#ifndef _G_OBS_H_
#define _G_OBS_H_

#ifdef __cplusplus
extern "C" {
#endif

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
typedef struct
{
	unsigned char sys; /* satellite sys ID G,R,E,C,J,S */
	unsigned char prn; /* satellite prn number/slot number */
	unsigned char code; /* measurement code */
	double P; /* code measurement in meter */
	double L; /* phase measurement in cycle */
	double D; /* doppler measurement in HZ */
	double offset; /* constant carrier phase offset in cycle */
	unsigned char S; /* signal SNR/CN0 */
	unsigned char LLI; /* lost-lock indicator */
	unsigned int lock;
}sat_obs_t;

#ifdef __cplusplus
}
#endif

#endif