/*
 GNSS Process Engine
 Copyright(R) 2021, Easy Navigation Technology Inc.
 Author: Dr. Yudan Yi
 Date: 09/20/2021
*/
#ifndef _GNSS_EPH_H_
#define _GNSS_EPH_H_

/* data struct used in the engine */

typedef struct {        /* GPS/QZS/GAL broadcast ephemeris type */
	unsigned char sys;
	unsigned char prn;
	int iode, iodc;      /* IODE,IODC */
	int sva;            /* SV accuracy (URA index) */
	int svh;            /* SV health (0:ok) */
	int week;           /* GPS/QZS: gps week, GAL: galileo week */
	int code;           /* GPS/QZS: code on L2 */
						/* GAL: data source defined as rinex 3.03 */
						/* BDS: data source (0:unknown,1:B1I,2:B1Q,3:B2I,4:B2Q,5:B3I,6:B3Q) */
	int flag;           /* GPS/QZS: L2 P data flag */
						/* BDS: nav type (0:unknown,1:IGSO/MEO,2:GEO) */
	double toes;        /* Toe (s) in week */
	double tocs;        /* Toc (s) in week */
	double ttr;			/* Trans (s) in week */
	double A, e, i0, OMG0, omg, M0, deln, OMGd, idot;
	double crc, crs, cuc, cus, cic, cis;
	double fit;         /* fit interval (h) */
	double f0, f1, f2;    /* SV clock parameters (af0,af1,af2) */
	double tgd[6];      /* group delay parameters */
						/* GPS/QZS:tgd[0]=TGD */
						/* GAL:tgd[0]=BGD_E1E5a,tgd[1]=BGD_E1E5b */
						/* CMP:tgd[0]=TGD_B1I ,tgd[1]=TGD_B2I/B2b,tgd[2]=TGD_B1Cp */
						/*     tgd[3]=TGD_B2ap,tgd[4]=ISC_B1Cd   ,tgd[5]=ISC_B2ad */
	double Adot, ndot;   /* Adot,ndot for CNAV */
} sat_eph_t;

typedef struct {        /* GLONASS broadcast ephemeris type */
	unsigned char sys;
	unsigned char prn;

	int iode;           /* IODE (0-6 bit of tb field) */
	int frq;            /* satellite frequency number */
	int svh, sva, age;    /* satellite health, accuracy, age of operation */
	//int tk_h, tk_m, tk_s, tb, NT;
	double toes;        /* epoch of epherides (gpst) => seconds in day */
	double tofs;        /* message frame time (gpst) => seconds in day */
	double pos[3];      /* satellite position (ecef) (m) */
	double vel[3];      /* satellite velocity (ecef) (m/s) */
	double acc[3];      /* satellite acceleration (ecef) (m/s^2) */
	double taun, gamn;   /* SV clock bias (s)/relative freq bias */
	double dtaun;       /* delay between L1 and L2 (s) */
} glo_eph_t;

#endif