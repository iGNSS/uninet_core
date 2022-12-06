#ifndef _EPHEMERIS_H_
#define _EPHEMERIS_H_

#include "gnss.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef WIN_DLL
#define EXPORT __declspec(dllexport) /* for Windows DLL */
#else
#define EXPORT
#endif

/* type definitions ----------------------------------------------------------*/

#define MAXSBSAGEF  30.0                /* max age of SBAS fast correction (s) */
#define MAXSBSAGEL  1800.0              /* max age of SBAS long term corr (s) */
#define MAXSBSURA   8                   /* max URA of SBAS satellite */
#define MAXBAND     10                  /* max SBAS band of IGP */
#define MAXNIGP     201                 /* max number of IGP in SBAS band */
#define MAXNGEO     4                   /* max number of GEO satellites */

#define MAXDTOE     7200.0              /* max time difference to GPS Toe (s) */
#define MAXDTOE_QZS 7200.0              /* max time difference to QZSS Toe (s) */
#define MAXDTOE_GAL 14400.0             /* max time difference to Galileo Toe (s) */
#define MAXDTOE_CMP 21600.0             /* max time difference to BeiDou Toe (s) */
#define MAXDTOE_GLO 1800.0              /* max time difference to GLONASS Toe (s) */
#define MAXDTOE_IRN 7200.0              /* max time difference to IRNSS Toe (s) */
#define MAXDTOE_SBS 360.0               /* max time difference to SBAS Toe (s) */
#define MAXDTOE_S   86400.0             /* max time difference to ephem toe (s) for other */

#define EPHOPT_BRDC 0                   /* ephemeris option: broadcast ephemeris */
#define EPHOPT_PREC 1                   /* ephemeris option: precise ephemeris */
#define EPHOPT_SBAS 2                   /* ephemeris option: broadcast + SBAS */
#define EPHOPT_SSRAPC 3                 /* ephemeris option: broadcast + SSR_APC */
#define EPHOPT_SSRCOM 4                 /* ephemeris option: broadcast + SSR_COM */


/* ephemeris and clock functions ---------------------------------------------*/
EXPORT double eph2clk (gtime_t time, const eph_t  *eph);
EXPORT double geph2clk(gtime_t time, const geph_t *geph);
EXPORT void eph2pos (gtime_t time, const eph_t  *eph,  double *rs, double *dts, double *var);
EXPORT void geph2pos(gtime_t time, const geph_t *geph, double *rs, double *dts, double *var);
EXPORT int  satpos(gtime_t time, gtime_t teph, int sat, int ephopt, const nav_t *nav, double *rs, double *dts, double *var, int *svh);
EXPORT void satposs(gtime_t time, const obsd_t *obs, int n, const nav_t *nav, int sateph, double *rs, double *dts, double *var, int *svh);
EXPORT void setseleph(int sys, int sel);
EXPORT int  getseleph(int sys);

#ifdef __cplusplus
}
#endif
#endif /* _EPHEMERIS_H_ */
