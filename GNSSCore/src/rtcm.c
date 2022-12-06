#include <string.h>
#include <stdio.h>

#include <math.h>
#include <stdint.h>

#include "rtcm.h"
#include "rtcm_buff.h"

#ifndef PI
#define	PI 3.14159265358979
#endif

#ifndef CLIGHT
#define CLIGHT      299792458.0         /* speed of light (m/s) */
#endif

#define PRUNIT_GPS  299792.458  /* rtcm ver.3 unit of gps pseudorange (m) */
#define PRUNIT_GLO  599584.916  /* rtcm ver.3 unit of glonass pseudorange (m) */
#define RANGE_MS    (CLIGHT*0.001)      /* range in 1 ms */

#define SC2RAD      3.1415926535898     /* semi-circle to radian (IS-GPS) */

#define P2_4        0.0625                /* 2^-4 */
#define P2_5        0.03125               /* 2^-5 */
#define P2_6        0.015625              /* 2^-6 */
#define P2_10       0.0009765625          /* 2^-10 */
#define P2_11       4.882812500000000E-04 /* 2^-11 */
#define P2_15       3.051757812500000E-05 /* 2^-15 */
#define P2_17       7.629394531250000E-06 /* 2^-17 */
#define P2_19       1.907348632812500E-06 /* 2^-19 */
#define P2_20       9.536743164062500E-07 /* 2^-20 */
#define P2_21       4.768371582031250E-07 /* 2^-21 */
#define P2_23       1.192092895507810E-07 /* 2^-23 */
#define P2_24       5.960464477539063E-08 /* 2^-24 */
#define P2_27       7.450580596923828E-09 /* 2^-27 */
#define P2_29       1.862645149230957E-09 /* 2^-29 */
#define P2_30       9.313225746154785E-10 /* 2^-30 */
#define P2_31       4.656612873077393E-10 /* 2^-31 */
#define P2_32       2.328306436538696E-10 /* 2^-32 */
#define P2_33       1.164153218269348E-10 /* 2^-33 */
#define P2_34       5.820766091346740E-11 /* 2^-34 */
#define P2_35       2.910383045673370E-11 /* 2^-35 */
#define P2_38       3.637978807091710E-12 /* 2^-38 */
#define P2_39       1.818989403545856E-12 /* 2^-39 */
#define P2_40       9.094947017729280E-13 /* 2^-40 */
#define P2_43       1.136868377216160E-13 /* 2^-43 */
#define P2_46       1.421085471520200E-14 /* 2^-46 */
#define P2_48       3.552713678800501E-15 /* 2^-48 */
#define P2_50       8.881784197001252E-16 /* 2^-50 */
#define P2_55       2.775557561562891E-17 /* 2^-55 */
#define P2_59       1.734723475976810E-18 /* 2^-59 */
#define P2_66       1.355252715606880E-20 /* 2^-66 */

/* type definition -----------------------------------------------------------*/

/* define signal based on RINEX 3.0X */
/* Q signal = I-0.25 cycle */
#define FREQ1       1.57542E9           /* L1/E1 (GPS:L1C/L1S/L1I/L1X/L1P/L1W/L1N,GAL:L1B/L1C/L1X,BDS:B1D/L1P/L1X)  frequency (Hz) */
#define FREQ2       1.22760E9           /* L2 (GPS:L2C/L2D/L2S/L2L/L2X/L2P/L2W/L2N) frequency (Hz) */
#define FREQ5       1.17645E9           /* L5/E5a (GPS:L5I/L5Q/L5X,GAL-E5a:L5I/L5Q/L5X,BDS-B2a:L5D/L5P/L5X) frequency (Hz) */
#define FREQ6       1.27875E9           /* E6/LEX (L6B/L6C/L6X) frequency (Hz) */
#define FREQ7       1.20714E9           /* E5b (L7I/L7Q/L7X) frequency (Hz) */
#define FREQ8       1.191795E9          /* E5a+b (L8I/L8Q/L8X), B2a+B2b (L8D/L8P/L8X)  frequency (Hz) */
#define FREQ9       2.492028E9          /* S IRNSS (L9A/L9B/L9C/L9X)     frequency (Hz) */
#define FREQ1_GLO   1.60200E9           /* GLONASS G1 base frequency (Hz) */
#define DFRQ1_GLO   0.56250E6           /* GLONASS G1 bias frequency (Hz/n) */
#define FREQ2_GLO   1.24600E9           /* GLONASS G2 base frequency (Hz) */
#define DFRQ2_GLO   0.43750E6           /* GLONASS G2 bias frequency (Hz/n) */
#define FREQ3_GLO   1.202025E9          /* GLONASS G3 frequency (Hz) */
#define FREQ1_CMP   1.561098E9          /* BeiDou B1-2 (L2I/L2Q/L2X) frequency (Hz) */
#define FREQ2_CMP   1.20714E9           /* BeiDou B2b (BDS-2 L7I/L7Q/L7X & BDS-3 L7D/L7P/L7Z) frequency (Hz) */
#define FREQ3_CMP   1.26852E9           /* BeiDou B3 (L6I/L6Q/L6X/L6A) frequency (Hz) */

static int default_glo_frq_table[30] = { 1, -4, 05, 06, 01, -4, 05, 06, -2, -7, 00, -1, -2, -7, 00, -1, 04, -3, 03, 02, 04, -3, 03, 02, 0, -5, -99, -99, -99, -99 };
extern void set_glo_frq_(unsigned char prn, int frq)
{
    int max_prn = sizeof(default_glo_frq_table) / sizeof(int);
    if (prn <= max_prn)
    {
        //default_glo_frq_table[prn - 1] = frq;
    }
    return;
}

extern int get_glo_frq_(unsigned char prn)
{
    int max_prn = sizeof(default_glo_frq_table) / sizeof(int);
    int frq = -99;
    if (prn <= max_prn)
    {
        frq = default_glo_frq_table[prn - 1];
    }
    return frq;
}

    /* satellite carrier wave length -----------------------------------------------
    * get satellite carrier wave lengths
    * args   : int    sat       I   satellite number
    *          int    frq       I   frequency index (0:L1,1:L2,2:L5/3,...)
    *          nav_t  *nav      I   navigation messages
    * return : carrier wave length (m) (0.0: error)
    *-----------------------------------------------------------------------------*/
extern double satwavelen_code(unsigned char sys, unsigned char prn, int code)
{
    const double freq_glo[] = { FREQ1_GLO,FREQ2_GLO };
    const double dfrq_glo[] = { DFRQ1_GLO,DFRQ2_GLO };
    int frqNum = -99, frq = 0;

    if (sys == 'R') {
#if 0
        const char* msm_sig_glo[32] = {
            /* GLONASS: ref [13] table 3.5-93, ref [14][15] table 3.5-97 */
            ""  ,"1C","1P",""  ,""  ,""  ,""  ,"2C","2P",""  ,"3I","3Q",
            "3X",""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
            ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
};
#endif
        frqNum = get_glo_frq_(prn);
        if (frqNum == -99) return 0.0;
        else if (code == 2 || code == 3) return CLIGHT / (freq_glo[0] + dfrq_glo[0] * frqNum);
        else if (code == 8 || code == 9) return CLIGHT / (freq_glo[1] + dfrq_glo[1] * frqNum);
        else return 0.0;
        }
    else if (sys == 'C') {
#if 0
        const char* msm_sig_cmp[32] = {
            /* BeiDou: ref [15] table 3.5-106 */
            ""  ,"1I","1Q","1X",""  ,""  ,""  ,"6I","6Q","6X",""  ,""  ,
            ""  ,"7I","7Q","7X",""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
            ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
        };
#endif
        if (code == 2 || code == 3 || code == 4) return CLIGHT / FREQ1_CMP; /* B1I BDS2/3 2/1 */
        else if (code == 8 || code == 9 || code == 10) return CLIGHT / FREQ3_CMP; /* B3 BDS2/3  6 */
        else if (code == 14 || code == 15 || code == 16) return CLIGHT / FREQ2_CMP; /* B2I BDS-2 7 */
        else if (code == 22 || code == 23 || code == 24) return CLIGHT / FREQ5; /* B2a (BDS3) 5D/P/X 5 */
        else return 0.0;
        }
    else if (sys == 'E') {
#if 0
        const char* msm_sig_gal[32] = {
            /* Galileo: ref [15] table 3.5-100 */
            ""  ,"1C","1A","1B","1X","1Z",""  ,"6C","6A","6B","6X","6Z",
            ""  ,"7I","7Q","7X",""  ,"8I","8Q","8X",""  ,"5I","5Q","5X",
            ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
        };
#endif
        if (code == 2 || code == 3 || code == 4 || code == 5 || code == 6) return CLIGHT / FREQ1; /* E1 1 */
        else if (code == 22 || code == 23 || code == 24) return CLIGHT / FREQ5; /* E5a 5 */
        else if (code == 14 || code == 15 || code == 16) return CLIGHT / FREQ7; /* E5b 7*/
        else return 0.0;
    }
    else if (sys == 'J') {
#if 0
        const char* msm_sig_qzs[32] = {
            /* QZSS: ref [15] table 3.5-103 */
            ""  ,"1C",""  ,""  ,""  ,""  ,""  ,""  ,"6S","6L","6X",""  ,
            ""  ,""  ,"2S","2L","2X",""  ,""  ,""  ,""  ,"5I","5Q","5X",
            ""  ,""  ,""  ,""  ,""  ,"1S","1L","1X"
    };
#endif
        if (code == 2 || code == 30 || code == 31 || code == 32) return CLIGHT / FREQ1; /* L1 */
        else if (code == 15 || code == 16 || code == 17) return CLIGHT / FREQ2; /* L2 */
        else if (code == 22 || code == 23 || code == 24) return CLIGHT / FREQ5; /* L5 */
        else return 0.0;
    }
    else if (sys == 'S') {
#if 0
        const char* msm_sig_sbs[32] = {
            /* SBAS: ref [13] table 3.5-T+005 */
            ""  ,"1C",""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
            ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,"5I","5Q","5X",
            ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
        };
#endif
        if (code == 2) return CLIGHT / FREQ1; /* L1 */
        else if (code == 22 || code == 23 || code == 24) return CLIGHT / FREQ5; /* L5 */
        else return 0.0;
    }
    else if (sys == 'G') {
#if 0
        /* msm signal id table -------------------------------------------------------*/
        const char* msm_sig_gps[32] = {
            /* GPS: ref [13] table 3.5-87, ref [14][15] table 3.5-91 */
            ""  ,"1C","1P","1W","1Y","1M",""  ,"2C","2P","2W","2Y","2M", /*  1-12 */
            ""  ,""  ,"2S","2L","2X",""  ,""  ,""  ,""  ,"5I","5Q","5X", /* 13-24 */
            ""  ,""  ,""  ,""  ,""  ,"1S","1L","1X"                      /* 25-32 */
        };
#endif
        if (code == 2 || code == 3 || code == 4 || code == 5 || code == 6 || code == 30 || code == 31 || code == 32) return CLIGHT / FREQ1; /* L1 */
        else if (code == 8 || code == 9 || code == 10 || code == 11 || code == 12 || code == 15 || code == 16 || code == 17) return CLIGHT / FREQ2; /* L2 */
        else if (code == 22 || code == 23 || code == 24) return CLIGHT / FREQ5; /* L5 */
        else return 0.0;
    }
    return 0.0;
    }


static int save_msm_obs(unsigned char *buff, int len, unsigned char sys, sat_obs_t* obs, msm_h_t *msmh, const double *r,
                         const double *pr, const double *cp, const double *rr,
                         const double *rrf, const double *cnr, const int *lock,
                         const int *ex, const int *half)
{
    double wl;
    int i,j,k,type=getbitu(buff,24,12),prn,fn,nobs=0;
    sat_obs_t *obsd=0;
    
    for (i=j=0;i<msmh->nsat;i++) {
        
        prn=msmh->sats[i];
        
        for (k=0;k<msmh->nsig;k++) {
            if (!msmh->cellmask[k+i*msmh->nsig]) continue;
            
            /* glonass wave length by extended info */
            if (sys=='R'&&ex&&ex[i]<=13) {
                fn=ex[i]-7;
                //set_glo_frq_(prn,fn);
            }

            /* satellite carrier wave length */
            wl=satwavelen_code(sys,prn,msmh->sigs[k]);

            obsd=obs+nobs;
            memset(obsd, 0, sizeof(sat_obs_t));
            obsd->sys = sys;
            obsd->prn = prn;

            /* pseudorange (m) */
            if (r[i]!=0.0&&pr[j]>-1E12) {
                obsd->P =r[i]+pr[j];
            }
            /* carrier-phase (cycle) */
            if (r[i]!=0.0&&cp[j]>-1E12&&wl>0.0) {
                obsd->L=(r[i]+cp[j])/wl;
            }
            /* doppler (hz) */
            if (rr&&rrf&&rrf[j]>-1E12&&wl>0.0) {
                obsd->D=(float)(-(rr[i]+rrf[j])/wl);
            }
            obsd->lock=lock[j];
            /*obsd->half=half[j];*/
            obsd->S =(unsigned char)(cnr[j]);
            obsd->code=msmh->sigs[k];
            //trace_(2, "rtcm3 %d prn=%c%3d, code=%3i,%14.4f,%14.4f,%10.4f,%6i,%3i\n", type, sys, prn
            //    , obsd->code, obsd->P, obsd->L, obsd->D, obsd->lock, /*obsd->half*/half[j]);
            ++nobs;
            j++;
        }
    }
    return nobs;
}
/* decode type msm message header --------------------------------------------*/
static int decode_msm_head(unsigned char *buff, int len, unsigned char sys, int *staid, double *ws, int *sync, int *iod, msm_h_t *msmh, int *hsize)
{
    double tod;
    int i=24,j,dow,mask,type,ncell=0;
    
    type=getbitu(buff,i,12); i+=12;
    
    memset(msmh,0,sizeof(msm_h_t));/**msmh=h0;*/
    
    if (i+157<=len*8) {
        *staid     =getbitu(buff,i,12);       i+=12;
        
        if (sys=='R') {
            dow   =getbitu(buff,i, 3);       i+= 3;
            tod   =getbitu(buff,i,27)*0.001; i+=27;
            *ws   =dow*24*3600.0+tod-3*3600.0+18;
        }
        else if (sys=='C') {
            *ws   =getbitu(buff,i,30)*0.001; i+=30;
            *ws  +=14.0; /* BDT -> GPST */
        }
        else {
            *ws   =getbitu(buff,i,30)*0.001; i+=30;
        }
        *sync     =getbitu(buff,i, 1);       i+= 1;
        *iod      =getbitu(buff,i, 3);       i+= 3;
        msmh->time_s =getbitu(buff,i, 7);       i+= 7;
        msmh->clk_str=getbitu(buff,i, 2);       i+= 2;
        msmh->clk_ext=getbitu(buff,i, 2);       i+= 2;
        msmh->smooth =getbitu(buff,i, 1);       i+= 1;
        msmh->tint_s =getbitu(buff,i, 3);       i+= 3;
        for (j=1;j<=64;j++) {
            mask=getbitu(buff,i,1); i+=1;
            if (mask) msmh->sats[msmh->nsat++]=j;
        }
        for (j=1;j<=32;j++) {
            mask=getbitu(buff,i,1); i+=1;
            if (mask) msmh->sigs[msmh->nsig++]=j;
        }
    }
    else {
        return -1;
    }
    /* test station id */
    
    if (msmh->nsat*msmh->nsig>64) {
        //trace_(2,"rtcm3 %d number of sats and sigs error: nsat=%d nsig=%d\n",
        //      type,msmh->nsat,msmh->nsig);
        return -1;
    }
    if (i+msmh->nsat*msmh->nsig>len*8) {
        //trace_(2,"rtcm3 %d length error: len=%d nsat=%d nsig=%d\n",type,
        //      len,msmh->nsat,msmh->nsig);
        return -1;
    }
    for (j=0;j<msmh->nsat*msmh->nsig;j++) {
        msmh->cellmask[j]=getbitu(buff,i,1); i+=1;
        if (msmh->cellmask[j]) ncell++;
    }
    *hsize=i;
    
    //trace_(4,"decode_head_msm: tow=%10.3f sys=%c staid=%d nsat=%d nsig=%d sync=%d iod=%d ncell=%d\n",
    //      *ws,sys,staid,msmh->nsat,msmh->nsig,*sync,*iod,ncell);
    
    return ncell;
}
/* decode msm 4: full pseudorange and phaserange plus cnr --------------------*/
extern int decode_msm4_(unsigned char *buff, int len, unsigned char sys, int *staid, double *ws, sat_obs_t *obs, int *nobs, msm_h_t *msmh, msm_d_t *msmd)
{
    /*double r[64],pr[64],cp[64],cnr[64],tow=0.0;*/
    int i,j,type=getbitu(buff,24,12),sync,iod,ncell,rng,rng_m,prv,cpv/*,lock[64],half[64]*/;
    memset(msmh,0,sizeof(msm_h_t));
    memset(msmd,0,sizeof(msm_d_t));     
    /* decode msm header */
    if ((ncell=decode_msm_head(buff,len,sys,staid,ws,&sync,&iod,msmh,&i))<0) return -1;
    
    if (i+msmh->nsat*18+ncell*48>len*8) {
        return -1;
    }
    for (j=0;j<msmh->nsat;j++) msmd->r[j]=0.0;
    for (j=0;j<ncell;j++) msmd->pr[j]=msmd->cp[j]=-1E16;
    
    /* decode satellite data */
    for (j=0;j<msmh->nsat;j++) { /* range */
        rng  =getbitu(buff,i, 8); i+= 8;
        if (rng!=255) msmd->r[j]=rng*RANGE_MS;
    }
    for (j=0;j<msmh->nsat;j++) {
        rng_m=getbitu(buff,i,10); i+=10;
        if (msmd->r[j]!=0.0) msmd->r[j]+=rng_m*P2_10*RANGE_MS;
    }
    /* decode signal data */
    for (j=0;j<ncell;j++) { /* pseudorange */
        prv=getbits(buff,i,15); i+=15;
        if (prv!=-16384) msmd->pr[j]=prv*P2_24*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* phaserange */
        cpv=getbits(buff,i,22); i+=22;
        if (cpv!=-2097152) msmd->cp[j]=cpv*P2_29*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* lock time */
        msmd->lock[j]=getbitu(buff,i,4); i+=4;
    }
    for (j=0;j<ncell;j++) { /* half-cycle ambiguity */
        msmd->half[j]=getbitu(buff,i,1); i+=1;
    }
    for (j=0;j<ncell;j++) { /* cnr */
        msmd->cnr[j]=getbitu(buff,i,6)*1.0; i+=6;
    }
    /* save obs data in msm message */
    *nobs=save_msm_obs(buff,len,sys,obs,msmh,msmd->r,msmd->pr,msmd->cp,NULL,NULL,msmd->cnr,msmd->lock,NULL,msmd->half);
    
    return sync?0:1;
}
/* decode msm 5: full pseudorange, phaserange, phaserangerate and cnr --------*/
extern int decode_msm5_(unsigned char *buff, int len, unsigned char sys, int *staid, double* ws, sat_obs_t *obs, int *nobs, msm_h_t *msmh, msm_d_t *msmd)
{
    /*double r[64],rr[64],pr[64],cp[64],rrf[64],cnr[64],tow=0.0;*/
    int i,j,type=getbitu(buff,24,12),sync,iod,ncell,rng,rng_m,rate,prv,cpv,rrv/*,lock[64]*/;
    /*int ex[64],half[64];*/
    memset(msmh,0,sizeof(msm_h_t));
    memset(msmd,0,sizeof(msm_d_t));    
    /* decode msm header */
    if ((ncell=decode_msm_head(buff,len,sys,staid,ws,&sync,&iod,msmh,&i))<0) return -1;
    
    if (i+msmh->nsat*36+ncell*63>len*8) {
        return -1;
    }
    for (j=0;j<msmh->nsat;j++) {
        msmd->r[j]=msmd->rr[j]=0.0; msmd->ex[j]=15;
    }
    for (j=0;j<ncell;j++) msmd->pr[j]=msmd->cp[j]=msmd->rrf[j]=-1E16;
    
    /* decode satellite data */
    for (j=0;j<msmh->nsat;j++) { /* range */
        rng  =getbitu(buff,i, 8); i+= 8;
        if (rng!=255) msmd->r[j]=rng*RANGE_MS;
    }
    for (j=0;j<msmh->nsat;j++) { /* extended info */
        msmd->ex[j]=getbitu(buff,i, 4); i+= 4;
    }
    for (j=0;j<msmh->nsat;j++) {
        rng_m=getbitu(buff,i,10); i+=10;
        if (msmd->r[j]!=0.0) msmd->r[j]+=rng_m*P2_10*RANGE_MS;
    }
    for (j=0;j<msmh->nsat;j++) { /* phaserangerate */
        rate =getbits(buff,i,14); i+=14;
        if (rate!=-8192) msmd->rr[j]=rate*1.0;
    }
    /* decode signal data */
    for (j=0;j<ncell;j++) { /* pseudorange */
        prv=getbits(buff,i,15); i+=15;
        if (prv!=-16384) msmd->pr[j]=prv*P2_24*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* phaserange */
        cpv=getbits(buff,i,22); i+=22;
        if (cpv!=-2097152) msmd->cp[j]=cpv*P2_29*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* lock time */
        msmd->lock[j]=getbitu(buff,i,4); i+=4;
    }
    for (j=0;j<ncell;j++) { /* half-cycle ambiguity */
        msmd->half[j]=getbitu(buff,i,1); i+=1;
    }
    for (j=0;j<ncell;j++) { /* cnr */
        msmd->cnr[j]=getbitu(buff,i,6)*1.0; i+=6;
    }
    for (j=0;j<ncell;j++) { /* phaserangerate */
        rrv=getbits(buff,i,15); i+=15;
        if (rrv!=-16384) msmd->rrf[j]=rrv*0.0001;
    }
    /* save obs data in msm message */
    *nobs=save_msm_obs(buff,len,sys,obs,msmh,msmd->r,msmd->pr,msmd->cp,msmd->rr,msmd->rrf,msmd->cnr,msmd->lock,msmd->ex,msmd->half);
    
    //obs->obsflag=!sync;
    return sync?0:1;
}
/* decode msm 6: full pseudorange and phaserange plus cnr (high-res) ---------*/
extern int decode_msm6_(unsigned char *buff, int len, unsigned char sys, int *staid, double* ws, sat_obs_t *obs, int *nobs, msm_h_t *msmh, msm_d_t *msmd)
{
    /*double r[64],pr[64],cp[64],cnr[64],tow=0.0;*/
    int i,j,type=getbitu(buff,24,12),sync,iod,ncell,rng,rng_m,prv,cpv/*,lock[64],half[64]*/;
    memset(msmh,0,sizeof(msm_h_t));
    memset(msmd,0,sizeof(msm_d_t));
    
    /* decode msm header */
    if ((ncell=decode_msm_head(buff,len,sys,staid,ws,&sync,&iod,msmh,&i))<0) return -1;
    
    if (i+msmh->nsat*18+ncell*65>len*8) {
        return -1;
    }
    for (j=0;j<msmh->nsat;j++) msmd->r[j]=0.0;
    for (j=0;j<ncell;j++) msmd->pr[j]=msmd->cp[j]=-1E16;
    
    /* decode satellite data */
    for (j=0;j<msmh->nsat;j++) { /* range */
        rng  =getbitu(buff,i, 8); i+= 8;
        if (rng!=255) msmd->r[j]=rng*RANGE_MS;
    }
    for (j=0;j<msmh->nsat;j++) {
        rng_m=getbitu(buff,i,10); i+=10;
        if (msmd->r[j]!=0.0) msmd->r[j]+=rng_m*P2_10*RANGE_MS;
    }
    /* decode signal data */
    for (j=0;j<ncell;j++) { /* pseudorange */
        prv=getbits(buff,i,20); i+=20;
        if (prv!=-524288) msmd->pr[j]=prv*P2_29*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* phaserange */
        cpv=getbits(buff,i,24); i+=24;
        if (cpv!=-8388608) msmd->cp[j]=cpv*P2_31*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* lock time */
        msmd->lock[j]=getbitu(buff,i,10); i+=10;
    }
    for (j=0;j<ncell;j++) { /* half-cycle ambiguity */
        msmd->half[j]=getbitu(buff,i,1); i+=1;
    }
    for (j=0;j<ncell;j++) { /* cnr */
        msmd->cnr[j]=getbitu(buff,i,10)*0.0625; i+=10;
    }
    /* save obs data in msm message */
    *nobs=save_msm_obs(buff,len,sys,obs,msmh,msmd->r,msmd->pr,msmd->cp,NULL,NULL,msmd->cnr,msmd->lock,NULL,msmd->half);
    
    //obs->obsflag=!sync;
    return sync?0:1;
}
/* decode msm 7: full pseudorange, phaserange, phaserangerate and cnr (h-res) */
extern int decode_msm7_(unsigned char *buff, int len, unsigned char sys, int *staid, double* ws, sat_obs_t *obs, int *nobs, msm_h_t *msmh, msm_d_t *msmd)
{
      
    /*double r[64],rr[64],pr[64],cp[64],rrf[64],cnr[64],tow=0.0;*/
    int i,j,type=getbitu(buff,24,12),sync,iod,ncell,rng,rng_m,rate,prv,cpv,rrv/*,lock[64]*/;
    /*int ex[64],half[64]*/;
    memset(msmh,0,sizeof(msm_h_t));
    memset(msmd,0,sizeof(msm_d_t));
        
    /* decode msm header */
    if ((ncell=decode_msm_head(buff,len,sys,staid,ws,&sync,&iod,msmh,&i))<0) return -1;
    
    if (i+msmh->nsat*36+ncell*80>len*8) {
        return -1;
    }
    for (j=0;j<msmh->nsat;j++) {
        msmd->r[j]=msmd->rr[j]=0.0; msmd->ex[j]=15;
    }
    for (j=0;j<ncell;j++) msmd->pr[j]=msmd->cp[j]=msmd->rrf[j]=-1E16;
    
    /* decode satellite data */
    for (j=0;j<msmh->nsat;j++) { /* range */
        rng  =getbitu(buff,i, 8); i+= 8;
        if (rng!=255) msmd->r[j]=rng*RANGE_MS;
    }
    for (j=0;j<msmh->nsat;j++) { /* extended info */
        msmd->ex[j]=getbitu(buff,i, 4); i+= 4;
    }
    for (j=0;j<msmh->nsat;j++) {
        rng_m=getbitu(buff,i,10); i+=10;
        if (msmd->r[j]!=0.0) msmd->r[j]+=rng_m*P2_10*RANGE_MS;
    }
    for (j=0;j<msmh->nsat;j++) { /* phaserangerate */
        rate =getbits(buff,i,14); i+=14;
        if (rate!=-8192) msmd->rr[j]=rate*1.0;
    }
    /* decode signal data */
    for (j=0;j<ncell;j++) { /* pseudorange */
        prv=getbits(buff,i,20); i+=20;
        if (prv!=-524288) msmd->pr[j]=prv*P2_29*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* phaserange */
        cpv=getbits(buff,i,24); i+=24;
        if (cpv!=-8388608) msmd->cp[j]=cpv*P2_31*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* lock time */
        msmd->lock[j]=getbitu(buff,i,10); i+=10;
    }
    for (j=0;j<ncell;j++) { /* half-cycle amiguity */
        msmd->half[j]=getbitu(buff,i,1); i+=1;
    }
    for (j=0;j<ncell;j++) { /* cnr */
        msmd->cnr[j]=getbitu(buff,i,10)*0.0625; i+=10;
    }
    for (j=0;j<ncell;j++) { /* phaserangerate */
        rrv=getbits(buff,i,15); i+=15;
        if (rrv!=-16384) msmd->rrf[j]=rrv*0.0001;
    }
    /* save obs data in msm message */
    *nobs=save_msm_obs(buff,len,sys,obs,msmh,msmd->r,msmd->pr,msmd->cp,msmd->rr,msmd->rrf,msmd->cnr,msmd->lock,msmd->ex,msmd->half);
    
    //obs->obsflag=!sync;
    return sync?0:1;
}


extern int decode_msmx_(unsigned char* buff, int len, int *staid, double* ws, sat_obs_t* obs, int* nobs, msm_h_t* msmh, msm_d_t* msmd)
{
    int type=getbitu(buff,24,12);
    int ret=0;
    switch (type) {
        case 1074: ret=decode_msm4_(buff,len,'G',staid,ws,obs,nobs,msmh,msmd); break;
        case 1075: ret=decode_msm5_(buff,len,'G',staid,ws,obs,nobs,msmh,msmd); break;
        case 1076: ret=decode_msm6_(buff,len,'G',staid,ws,obs,nobs,msmh,msmd); break;
        case 1077: ret=decode_msm7_(buff,len,'G',staid,ws,obs,nobs,msmh,msmd); break;
        case 1084: ret=decode_msm4_(buff,len,'R',staid,ws,obs,nobs,msmh,msmd); break;
        case 1085: ret=decode_msm5_(buff,len,'R',staid,ws,obs,nobs,msmh,msmd); break;
        case 1086: ret=decode_msm6_(buff,len,'R',staid,ws,obs,nobs,msmh,msmd); break;
        case 1087: ret=decode_msm7_(buff,len,'R',staid,ws,obs,nobs,msmh,msmd); break;
        case 1094: ret=decode_msm4_(buff,len,'E',staid,ws,obs,nobs,msmh,msmd); break;
        case 1095: ret=decode_msm5_(buff,len,'E',staid,ws,obs,nobs,msmh,msmd); break;
        case 1096: ret=decode_msm6_(buff,len,'E',staid,ws,obs,nobs,msmh,msmd); break;
        case 1097: ret=decode_msm7_(buff,len,'E',staid,ws,obs,nobs,msmh,msmd); break;
        case 1104: ret=decode_msm4_(buff,len,'S',staid,ws,obs,nobs,msmh,msmd); break;
        case 1105: ret=decode_msm5_(buff,len,'S',staid,ws,obs,nobs,msmh,msmd); break;
        case 1106: ret=decode_msm6_(buff,len,'S',staid,ws,obs,nobs,msmh,msmd); break;
        case 1107: ret=decode_msm7_(buff,len,'S',staid,ws,obs,nobs,msmh,msmd); break;
        case 1114: ret=decode_msm4_(buff,len,'J',staid,ws,obs,nobs,msmh,msmd); break;
        case 1115: ret=decode_msm5_(buff,len,'J',staid,ws,obs,nobs,msmh,msmd); break;
        case 1116: ret=decode_msm6_(buff,len,'J',staid,ws,obs,nobs,msmh,msmd); break;
        case 1117: ret=decode_msm7_(buff,len,'J',staid,ws,obs,nobs,msmh,msmd); break;
        case 1124: ret=decode_msm4_(buff,len,'C',staid,ws,obs,nobs,msmh,msmd); break;
        case 1125: ret=decode_msm5_(buff,len,'C',staid,ws,obs,nobs,msmh,msmd); break;
        case 1126: ret=decode_msm6_(buff,len,'C',staid,ws,obs,nobs,msmh,msmd); break;
        case 1127: ret=decode_msm7_(buff,len,'C',staid,ws,obs,nobs,msmh,msmd); break;
    }
	return ret;
}


extern int decode_rtcm_obs(unsigned char* buff, int nbyte, double *ws, sat_obs_t* obs, int *nobs, int* type, int* staid, int* crc, msm_h_t* msmh, msm_d_t* msmd)
{
    int i = 0, ret = 0, len = 0;
    if (nbyte < 3) return 0;
    len = getbitu(buff, 14, 10) + 3; /* length without parity */
    if (nbyte < len + 3) return ret;
    i = 24;
    *type = getbitu(buff, i, 12); i += 12;

    /* check parity */
    if (crc24q(buff, len) != getbitu(buff, len * 8, 24)) {
        *crc = 1;
        return ret;
    }
    *staid= getbitu(buff, i, 12);

    memset(obs, 0, sizeof(sat_obs_t)*64);

    return decode_msmx_(buff, len, staid, ws, obs, nobs, msmh, msmd);
}
