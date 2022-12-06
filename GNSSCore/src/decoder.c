/*------------------------------------------------------------------------------
* rtcm.c : rtcm functions
*
*          Copyright (C) 2009-2020 by T.TAKASU, All rights reserved.
*
* references :
*     [1]  RTCM Recommended Standards for Differential GNSS (Global Navigation
*          Satellite Systems) Service version 2.3, August 20, 2001
*     [7]  RTCM Standard 10403.1 - Amendment 5, Differential GNSS (Global
*          Navigation Satellite Systems) Services - version 3, July 1, 2011
*     [10] RTCM Paper 059-2011-SC104-635 (draft Galileo and QZSS ssr messages)
*     [15] RTCM Standard 10403.2, Differential GNSS (Global Navigation Satellite
*          Systems) Services - version 3, with amendment 1/2, November 7, 2013
*     [16] Proposal of new RTCM SSR Messages (ssr_1_gal_qzss_sbas_dbs_v05)
*          2014/04/17
*     [17] RTCM Standard 10403.3, Differential GNSS (Global Navigation Satellite
*          Systems) Services - version 3, with amendment 1, April 28, 2020
*     [18] IGS State Space Representation (SSR) Format version 1.00, October 5,
*          2020
*
* version : $Revision:$ $Date:$
* history : 2009/04/10 1.0  new
*           2009/06/29 1.1  support type 1009-1012 to get synchronous-gnss-flag
*           2009/12/04 1.2  support type 1010,1012,1020
*           2010/07/15 1.3  support type 1057-1068 for ssr corrections
*                           support type 1007,1008,1033 for antenna info
*           2010/09/08 1.4  fix problem of ephemeris and ssr sequence upset
*                           (2.4.0_p8)
*           2012/05/11 1.5  comply with RTCM 3 final SSR format (RTCM 3
*                           Amendment 5) (ref [7]) (2.4.1_p6)
*           2012/05/14 1.6  separate rtcm2.c, rtcm3.c
*                           add options to select used codes for msm
*           2013/04/27 1.7  comply with rtcm 3.2 with amendment 1/2 (ref[15])
*           2013/12/06 1.8  support SBAS/BeiDou SSR messages (ref[16])
*           2018/01/29 1.9  support RTCM 3.3 (ref[17])
*                           crc24q() -> rtk_crc24q()
*           2018/10/10 1.10 fix bug on initializing rtcm struct
*                           add rtcm option -GALINAV, -GALFNAV
*           2018/11/05 1.11 add notes for api gen_rtcm3()
*           2020/11/30 1.12 modify API gen_rtcm3()
*                           support NavIC/IRNSS MSM and ephemeris (ref [17])
*                           allocate double size of ephemeris buffer to support
*                            multiple ephemeris sets in init_rtcm()
*                           delete references [2]-[6],[8],[9],[11]-[14]
*                           update reference [17]
*                           use integer types in stdint.h
*-----------------------------------------------------------------------------*/
#include "gnss.h"

/* function prototypes -------------------------------------------------------*/
extern int decode_rtcm2(rtcm_t *rtcm, nav_t *nav);
extern int decode_rtcm3(rtcm_t *rtcm, nav_t* nav);
extern int encode_rtcm3(rtcm_t *rtcm, nav_t *nav, int type, int subtype, int sync);

/* constants -----------------------------------------------------------------*/


/* from rtcm3.c */
#define PRUNIT_GPS  299792.458  /* rtcm ver.3 unit of gps pseudorange (m) */
#define PRUNIT_GLO  599584.916  /* rtcm ver.3 unit of glonass pseudorange (m) */
#define RANGE_MS    (CLIGHT*0.001)      /* range in 1 ms */

#define P2_10       0.0009765625          /* 2^-10 */
#define P2_28       3.725290298461914E-09 /* 2^-28 */
#define P2_34       5.820766091346740E-11 /* 2^-34 */
#define P2_41       4.547473508864641E-13 /* 2^-41 */
#define P2_46       1.421085471520200E-14 /* 2^-46 */
#define P2_59       1.734723475976810E-18 /* 2^-59 */
#define P2_66       1.355252715606880E-20 /* 2^-66 */

/* from rtcm3e.c */
#define PRUNIT_GPS  299792.458          /* rtcm 3 unit of gps pseudorange (m) */
#define PRUNIT_GLO  599584.916          /* rtcm 3 unit of glo pseudorange (m) */
#define RANGE_MS    (CLIGHT*0.001)      /* range in 1 ms */
#define P2_10       0.0009765625          /* 2^-10 */
#define P2_28       3.725290298461914E-09 /* 2^-28 */
#define P2_34       5.820766091346740E-11 /* 2^-34 */
#define P2_41       4.547473508864641E-13 /* 2^-41 */
#define P2_46       1.421085471520200E-14 /* 2^-46 */
#define P2_59       1.734723475976810E-18 /* 2^-59 */
#define P2_66       1.355252715606880E-20 /* 2^-66 */

#define ROUND(x)    ((int)floor((x)+0.5))
#define ROUND_U(x)  ((uint32_t)floor((x)+0.5))
#define MIN(x,y)    ((x)<(y)?(x):(y))

/* from rtcm2.c */

/* adjust hourly rollover of rtcm 2 time -------------------------------------*/
static void adjhour(rtcm_t *rtcm, double zcnt)
{
    double tow,hour,sec;
    int week;
    
    /* if no time, get cpu time */
    if (rtcm->time.time==0) rtcm->time=utc2gpst(timeget());
    tow=time2gpst(rtcm->time,&week);
    hour=floor(tow/3600.0);
    sec=tow-hour*3600.0;
    if      (zcnt<sec-1800.0) zcnt+=3600.0;
    else if (zcnt>sec+1800.0) zcnt-=3600.0;
    rtcm->time=gpst2time(week,hour*3600+zcnt);
}
/* get observation data index ------------------------------------------------*/
static int obsindex(obs_t *obs, gtime_t time, int sat)
{
    int i,j;
    
    for (i=0;i<obs->n;i++) {
        if (obs->data[i].sat==sat) return i; /* field already exists */
    }
    if (i>=MAXOBS) return -1; /* overflow */
    
    /* add new field */
    obs->data[i].time=time;
    obs->data[i].sat=sat;
    for (j=0;j<NFREQ+NEXOBS;j++) {
        obs->data[i].L[j]=obs->data[i].P[j]=0.0;
        obs->data[i].D[j]=0.0;
        obs->data[i].SNR[j]=obs->data[i].LLI[j]=obs->data[i].code[j]=0;
    }
    obs->n++;
    return i;
}
/* decode type 1/9: differential gps correction/partial correction set -------*/
static int decode_type1(rtcm_t *rtcm)
{
    int i=48,fact,udre,prn,sat,iod;
    double prc,rrc;
    
    trace(4,"decode_type1: len=%d\n",rtcm->len);
    
    while (i+40<=rtcm->len*8) {
        fact=getbitu(rtcm->buff,i, 1); i+= 1;
        udre=getbitu(rtcm->buff,i, 2); i+= 2;
        prn =getbitu(rtcm->buff,i, 5); i+= 5;
        prc =getbits(rtcm->buff,i,16); i+=16;
        rrc =getbits(rtcm->buff,i, 8); i+= 8;
        iod =getbits(rtcm->buff,i, 8); i+= 8;
        if (prn==0) prn=32;
        if (prc==0x80000000||rrc==0xFFFF8000) {
            trace(2,"rtcm2 1 prc/rrc indicates satellite problem: prn=%d\n",prn);
            continue;
        }
    }
    return 7;
}
/* decode type 3: reference station parameter --------------------------------*/
static int decode_type3(rtcm_t *rtcm)
{
    int i=48;
    
    trace(4,"decode_type3: len=%d\n",rtcm->len);
    
    if (i+96<=rtcm->len*8) {
        rtcm->sta.pos[0]=getbits(rtcm->buff,i,32)*0.01; i+=32;
        rtcm->sta.pos[1]=getbits(rtcm->buff,i,32)*0.01; i+=32;
        rtcm->sta.pos[2]=getbits(rtcm->buff,i,32)*0.01;
    }
    else {
        trace(2,"rtcm2 3 length error: len=%d\n",rtcm->len);
        return -1;
    }
    return 5;
}
/* decode type 14: gps time of week ------------------------------------------*/
static int decode_type14(rtcm_t *rtcm)
{
    double zcnt;
    int i=48,week,hour,leaps;
    
    trace(4,"decode_type14: len=%d\n",rtcm->len);
    
    zcnt=getbitu(rtcm->buff,24,13);
    if (i+24<=rtcm->len*8) {
        week =getbitu(rtcm->buff,i,10); i+=10;
        hour =getbitu(rtcm->buff,i, 8); i+= 8;
        leaps=getbitu(rtcm->buff,i, 6);
    }
    else {
        trace(2,"rtcm2 14 length error: len=%d\n",rtcm->len);
        return -1;
    }
    week=adjgpsweek(week);
    rtcm->time=gpst2time(week,hour*3600.0+zcnt*0.6);
    //rtcm->nav.utc_gps[4]=leaps;
    return 6;
}
/* decode type 16: gps special message ---------------------------------------*/
static int decode_type16(rtcm_t *rtcm)
{
    int i=48,n=0;
    
    trace(4,"decode_type16: len=%d\n",rtcm->len);
    
    while (i+8<=rtcm->len*8&&n<90) {
        rtcm->msg[n++]=getbitu(rtcm->buff,i,8); i+=8;
    }
    rtcm->msg[n]='\0';
    
    trace(3,"rtcm2 16 message: %s\n",rtcm->msg);
    return 9;
}
/* decode type 17: gps ephemerides -------------------------------------------*/
static int decode_type17(rtcm_t *rtcm, eph_t *eph)
{
    double toc,sqrtA;
    int i=48,week,prn,sat;
    
    trace(4,"decode_type17: len=%d\n",rtcm->len);
    
    if (i+480<=rtcm->len*8) {
        week       =getbitu(rtcm->buff,i,10);              i+=10;
        eph->idot  =getbits(rtcm->buff,i,14)*P2_43*SC2RAD; i+=14;
        eph->iode  =getbitu(rtcm->buff,i, 8);              i+= 8;
        toc        =getbitu(rtcm->buff,i,16)*16.0;         i+=16;
        eph->f1    =getbits(rtcm->buff,i,16)*P2_43;        i+=16;
        eph->f2    =getbits(rtcm->buff,i, 8)*P2_55;        i+= 8;
        eph->crs   =getbits(rtcm->buff,i,16)*P2_5;         i+=16;
        eph->deln  =getbits(rtcm->buff,i,16)*P2_43*SC2RAD; i+=16;
        eph->cuc   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph->e     =getbitu(rtcm->buff,i,32)*P2_33;        i+=32;
        eph->cus   =getbits(rtcm->buff,i,16);              i+=16;
        sqrtA      =getbitu(rtcm->buff,i,32)*P2_19;        i+=32;
        eph->toes  =getbitu(rtcm->buff,i,16);              i+=16;
        eph->OMG0  =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cic   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph->i0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cis   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph->omg   =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->crc   =getbits(rtcm->buff,i,16)*P2_5;         i+=16;
        eph->OMGd  =getbits(rtcm->buff,i,24)*P2_43*SC2RAD; i+=24;
        eph->M0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->iodc  =getbitu(rtcm->buff,i,10);              i+=10;
        eph->f0    =getbits(rtcm->buff,i,22)*P2_31;        i+=22;
        prn        =getbitu(rtcm->buff,i, 5);              i+= 5+3;
        eph->tgd[0]=getbits(rtcm->buff,i, 8)*P2_31;        i+= 8;
        eph->code  =getbitu(rtcm->buff,i, 2);              i+= 2;
        eph->sva   =getbitu(rtcm->buff,i, 4);              i+= 4;
        eph->svh   =getbitu(rtcm->buff,i, 6);              i+= 6;
        eph->flag  =getbitu(rtcm->buff,i, 1);
    }
    else {
        trace(2,"rtcm2 17 length error: len=%d\n",rtcm->len);
        return -1;
    }
    if (prn==0) prn=32;
    sat=satno(SYS_GPS,prn);
    eph->sat=sat;
    eph->week=adjgpsweek(week);
    eph->toe=gpst2time(eph->week,eph->toes);
    eph->toc=gpst2time(eph->week,toc);
    eph->ttr=rtcm->time;
    eph->A=sqrtA*sqrtA;
    return 2;
}
/* decode type 18: rtk uncorrected carrier-phase -----------------------------*/
static int decode_type18(rtcm_t *rtcm)
{
    gtime_t time;
    double usec,cp,tt;
    int i=48,index,freq,sync=1,code,sys,prn,sat,loss;
    
    trace(4,"decode_type18: len=%d\n",rtcm->len);
    
    if (i+24<=rtcm->len*8) {
        freq=getbitu(rtcm->buff,i, 2); i+= 2+2;
        usec=getbitu(rtcm->buff,i,20); i+=20;
    }
    else {
        trace(2,"rtcm2 18 length error: len=%d\n",rtcm->len);
        return -1;
    }
    if (freq&0x1) {
        trace(2,"rtcm2 18 not supported frequency: freq=%d\n",freq);
        return -1;
    }
    freq>>=1;
    
    while (i+48<=rtcm->len*8&&rtcm->obs.n<MAXOBS) {
        sync=getbitu(rtcm->buff,i, 1); i+= 1;
        code=getbitu(rtcm->buff,i, 1); i+= 1;
        sys =getbitu(rtcm->buff,i, 1); i+= 1;
        prn =getbitu(rtcm->buff,i, 5); i+= 5+3;
        loss=getbitu(rtcm->buff,i, 5); i+= 5;
        cp  =getbits(rtcm->buff,i,32); i+=32;
        if (prn==0) prn=32;
        if (!(sat=satno(sys?SYS_GLO:SYS_GPS,prn))) {
            trace(2,"rtcm2 18 satellite number error: sys=%d prn=%d\n",sys,prn);
            continue;
        }
        time=timeadd(rtcm->time,usec*1E-6);
        if (sys) time=utc2gpst(time); /* convert glonass time to gpst */
        
        tt=timediff(rtcm->obs.data[0].time,time);
        if (rtcm->obsflag||fabs(tt)>1E-9) {
            rtcm->obs.n=rtcm->obsflag=0;
        }
        if ((index=obsindex(&rtcm->obs,time,sat))>=0) {
            rtcm->obs.data[index].L[freq]=-cp/256.0;
            rtcm->obs.data[index].LLI[freq]=rtcm->loss[sat-1][freq]!=loss;
            rtcm->obs.data[index].code[freq]=
                !freq?(code?CODE_L1P:CODE_L1C):(code?CODE_L2P:CODE_L2C);
            rtcm->loss[sat-1][freq]=loss;
        }
    }
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode type 19: rtk uncorrected pseudorange -------------------------------*/
static int decode_type19(rtcm_t *rtcm)
{
    gtime_t time;
    double usec,pr,tt;
    int i=48,index,freq,sync=1,code,sys,prn,sat;
    
    trace(4,"decode_type19: len=%d\n",rtcm->len);
    
    if (i+24<=rtcm->len*8) {
        freq=getbitu(rtcm->buff,i, 2); i+= 2+2;
        usec=getbitu(rtcm->buff,i,20); i+=20;
    }
    else {
        trace(2,"rtcm2 19 length error: len=%d\n",rtcm->len);
        return -1;
    }
    if (freq&0x1) {
        trace(2,"rtcm2 19 not supported frequency: freq=%d\n",freq);
        return -1;
    }
    freq>>=1;
    
    while (i+48<=rtcm->len*8&&rtcm->obs.n<MAXOBS) {
        sync=getbitu(rtcm->buff,i, 1); i+= 1;
        code=getbitu(rtcm->buff,i, 1); i+= 1;
        sys =getbitu(rtcm->buff,i, 1); i+= 1;
        prn =getbitu(rtcm->buff,i, 5); i+= 5+8;
        pr  =getbitu(rtcm->buff,i,32); i+=32;
        if (prn==0) prn=32;
        if (!(sat=satno(sys?SYS_GLO:SYS_GPS,prn))) {
            trace(2,"rtcm2 19 satellite number error: sys=%d prn=%d\n",sys,prn);
            continue;
        }
        time=timeadd(rtcm->time,usec*1E-6);
        if (sys) time=utc2gpst(time); /* convert glonass time to gpst */
        
        tt=timediff(rtcm->obs.data[0].time,time);
        if (rtcm->obsflag||fabs(tt)>1E-9) {
            rtcm->obs.n=rtcm->obsflag=0;
        }
        if ((index=obsindex(&rtcm->obs,time,sat))>=0) {
            rtcm->obs.data[index].P[freq]=pr*0.02;
            rtcm->obs.data[index].code[freq]=
                !freq?(code?CODE_L1P:CODE_L1C):(code?CODE_L2P:CODE_L2C);
        }
    }
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode type 22: extended reference station parameter ----------------------*/
static int decode_type22(rtcm_t *rtcm)
{
    double del[2][3]={{0}},hgt=0.0;
    int i=48,j,noh;
    
    trace(4,"decode_type22: len=%d\n",rtcm->len);
    
    if (i+24<=rtcm->len*8) {
        del[0][0]=getbits(rtcm->buff,i,8)/25600.0; i+=8;
        del[0][1]=getbits(rtcm->buff,i,8)/25600.0; i+=8;
        del[0][2]=getbits(rtcm->buff,i,8)/25600.0; i+=8;
    }
    else {
        trace(2,"rtcm2 22 length error: len=%d\n",rtcm->len);
        return -1;
    }
    if (i+24<=rtcm->len*8) {
        i+=5; noh=getbits(rtcm->buff,i,1); i+=1;
        hgt=noh?0.0:getbitu(rtcm->buff,i,18)/25600.0;
        i+=18;
    }
    if (i+24<=rtcm->len*8) {
        del[1][0]=getbits(rtcm->buff,i,8)/1600.0; i+=8;
        del[1][1]=getbits(rtcm->buff,i,8)/1600.0; i+=8;
        del[1][2]=getbits(rtcm->buff,i,8)/1600.0;
    }
    rtcm->sta.deltype=1; /* xyz */
    for (j=0;j<3;j++) rtcm->sta.del[j]=del[0][j];
    rtcm->sta.hgt=hgt;
    return 5;
}

/* decode type 23: antenna type definition record ----------------------------*/
static int decode_type23(rtcm_t *rtcm)
{
    return 0;
}
/* decode type 24: antenna reference point (arp) -----------------------------*/
static int decode_type24(rtcm_t *rtcm)
{
    return 0;
}
/* decode type 31: differential glonass correction ---------------------------*/
static int decode_type31(rtcm_t *rtcm)
{
    return 0;
}
/* decode type 32: differential glonass reference station parameters ---------*/
static int decode_type32(rtcm_t *rtcm)
{
    return 0;
}
/* decode type 34: glonass partial differential correction set ---------------*/
static int decode_type34(rtcm_t *rtcm)
{
    return 0;
}
/* decode type 36: glonass special message -----------------------------------*/
static int decode_type36(rtcm_t *rtcm)
{
    return 0;
}
/* decode type 37: gnss system time offset -----------------------------------*/
static int decode_type37(rtcm_t *rtcm)
{
    return 0;
}
/* decode type 59: proprietary message ---------------------------------------*/
static int decode_type59(rtcm_t *rtcm)
{
    return 0;
}

/* decode rtcm ver.2 message -------------------------------------------------*/
extern int decode_rtcm2(rtcm_t *rtcm, nav_t *nav)
{
	eph_t eph={0};
    double zcnt;
    int staid,seqno,stah,ret=0,type=getbitu(rtcm->buff,8,6);
    
    trace(3,"decode_rtcm2: type=%2d len=%3d\n",type,rtcm->len);
    
    if ((zcnt=getbitu(rtcm->buff,24,13)*0.6)>=3600.0) {
        trace(2,"rtcm2 modified z-count error: zcnt=%.1f\n",zcnt);
        return -1;
    }
    adjhour(rtcm,zcnt);
    staid=getbitu(rtcm->buff,14,10);
    seqno=getbitu(rtcm->buff,37, 3);
    stah =getbitu(rtcm->buff,45, 3);
    if (seqno-rtcm->seqno!=1&&seqno-rtcm->seqno!=-7) {
        trace(2,"rtcm2 message outage: seqno=%d->%d\n",rtcm->seqno,seqno);
    }
    rtcm->seqno=seqno;
    rtcm->stah =stah;
    
    if (rtcm->outtype) {
        sprintf(rtcm->msgtype,"RTCM %2d (%4d) zcnt=%7.1f staid=%3d seqno=%d",
                type,rtcm->len,zcnt,staid,seqno);
    }
    if (type==3||type==22||type==23||type==24) {
        if (rtcm->staid!=0&&staid!=rtcm->staid) {
           trace(2,"rtcm2 station id changed: %d->%d\n",rtcm->staid,staid);
        }
        rtcm->staid=staid;
    }
    if (rtcm->staid!=0&&staid!=rtcm->staid) {
        trace(2,"rtcm2 station id invalid: %d %d\n",staid,rtcm->staid);
        return -1;
    }
    switch (type) {
        case  1: ret=decode_type1 (rtcm); break;
        case  3: ret=decode_type3 (rtcm); break;
        case  9: ret=decode_type1 (rtcm); break;
        case 14: ret=decode_type14(rtcm); break;
        case 16: ret=decode_type16(rtcm); break;
        case 17: ret=decode_type17(rtcm,&eph); break;
        case 18: ret=decode_type18(rtcm); break;
        case 19: ret=decode_type19(rtcm); break;
        case 22: ret=decode_type22(rtcm); break;
        case 23: ret=decode_type23(rtcm); break; /* not supported */
        case 24: ret=decode_type24(rtcm); break; /* not supported */
        case 31: ret=decode_type31(rtcm); break; /* not supported */
        case 32: ret=decode_type32(rtcm); break; /* not supported */
        case 34: ret=decode_type34(rtcm); break; /* not supported */
        case 36: ret=decode_type36(rtcm); break; /* not supported */
        case 37: ret=decode_type37(rtcm); break; /* not supported */
        case 59: ret=decode_type59(rtcm); break; /* not supported */
    }
    if (ret>=0) {
        if (1<=type&&type<=99) rtcm->nmsg2[type]++; else rtcm->nmsg2[0]++;
    }
    return ret;
}

/* from rtcm3.c */
/* type definition -----------------------------------------------------------*/

typedef struct {              /* multi-signal-message header type */
    uint8_t iod;              /* issue of data station */
    uint8_t time_s;           /* cumulative session transmitting time */
    uint8_t clk_str;          /* clock steering indicator */
    uint8_t clk_ext;          /* external clock indicator */
    uint8_t smooth;           /* divergence free smoothing indicator */
    uint8_t tint_s;           /* soothing interval */
    uint8_t nsat,nsig;        /* number of satellites/signals */
    uint8_t sats[64];         /* satellites */
    uint8_t sigs[32];         /* signals */
    uint8_t cellmask[64];     /* cell mask */
} msm_h_t;

/* MSM signal ID table -------------------------------------------------------*/
const char *msm_sig_gps[32]={
    /* GPS: ref [17] table 3.5-91 */
    ""  ,"1C","1P","1W",""  ,""  ,""  ,"2C","2P","2W",""  ,""  , /*  1-12 */
    ""  ,""  ,"2S","2L","2X",""  ,""  ,""  ,""  ,"5I","5Q","5X", /* 13-24 */
    ""  ,""  ,""  ,""  ,""  ,"1S","1L","1X"                      /* 25-32 */
};
const char *msm_sig_glo[32]={
    /* GLONASS: ref [17] table 3.5-96 */
    ""  ,"1C","1P",""  ,""  ,""  ,""  ,"2C","2P",""  ,""  ,""  ,
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
};
const char *msm_sig_gal[32]={
    /* Galileo: ref [17] table 3.5-99 */
    ""  ,"1C","1A","1B","1X","1Z",""  ,"6C","6A","6B","6X","6Z",
    ""  ,"7I","7Q","7X",""  ,"8I","8Q","8X",""  ,"5I","5Q","5X",
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
};
const char *msm_sig_qzs[32]={
    /* QZSS: ref [17] table 3.5-105 */
    ""  ,"1C",""  ,""  ,""  ,""  ,""  ,""  ,"6S","6L","6X",""  ,
    ""  ,""  ,"2S","2L","2X",""  ,""  ,""  ,""  ,"5I","5Q","5X",
    ""  ,""  ,""  ,""  ,""  ,"1S","1L","1X"
};
const char *msm_sig_sbs[32]={
    /* SBAS: ref [17] table 3.5-102 */
    ""  ,"1C",""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,"5I","5Q","5X",
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
};
const char *msm_sig_cmp[32]={
    /* BeiDou: ref [17] table 3.5-108 */
    ""  ,"2I","2Q","2X",""  ,""  ,""  ,"6I","6Q","6X",""  ,""  ,
    ""  ,"7I","7Q","7X",""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
};
const char *msm_sig_irn[32]={
    /* NavIC/IRNSS: ref [17] table 3.5-108.3 */
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,"5A",""  ,""  ,
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
};
/* SSR signal and tracking mode IDs ------------------------------------------*/
const uint8_t ssr_sig_gps[32]={
    CODE_L1C,CODE_L1P,CODE_L1W,CODE_L1S,CODE_L1L,CODE_L2C,CODE_L2D,CODE_L2S,
    CODE_L2L,CODE_L2X,CODE_L2P,CODE_L2W,       0,       0,CODE_L5I,CODE_L5Q
};
const uint8_t ssr_sig_glo[32]={
    CODE_L1C,CODE_L1P,CODE_L2C,CODE_L2P,CODE_L4A,CODE_L4B,CODE_L6A,CODE_L6B,
    CODE_L3I,CODE_L3Q
};
const uint8_t ssr_sig_gal[32]={
    CODE_L1A,CODE_L1B,CODE_L1C,       0,       0,CODE_L5I,CODE_L5Q,       0,
    CODE_L7I,CODE_L7Q,       0,CODE_L8I,CODE_L8Q,       0,CODE_L6A,CODE_L6B,
    CODE_L6C
};
const uint8_t ssr_sig_qzs[32]={
    CODE_L1C,CODE_L1S,CODE_L1L,CODE_L2S,CODE_L2L,       0,CODE_L5I,CODE_L5Q,
           0,CODE_L6S,CODE_L6L,       0,       0,       0,       0,       0,
           0,CODE_L6E
};
const uint8_t ssr_sig_cmp[32]={
    CODE_L2I,CODE_L2Q,       0,CODE_L6I,CODE_L6Q,       0,CODE_L7I,CODE_L7Q,
           0,CODE_L1D,CODE_L1P,       0,CODE_L5D,CODE_L5P,       0,CODE_L1A,
           0,       0,CODE_L6A
};
const uint8_t ssr_sig_sbs[32]={
    CODE_L1C,CODE_L5I,CODE_L5Q
};
/* SSR update intervals ------------------------------------------------------*/
static const double ssrudint[16]={
    1,2,5,10,15,30,60,120,240,300,600,900,1800,3600,7200,10800
};
/* get sign-magnitude bits ---------------------------------------------------*/
static double getbitg(const uint8_t *buff, int pos, int len)
{
    double value=getbitu(buff,pos+1,len-1);
    return getbitu(buff,pos,1)?-value:value;
}
/* adjust weekly rollover of GPS time ----------------------------------------*/
static void adjweek(rtcm_t *rtcm, double tow)
{
    double tow_p;
    int week;
    
    /* if no time, get cpu time */
    if (rtcm->time.time==0) rtcm->time=utc2gpst(timeget());
    tow_p=time2gpst(rtcm->time,&week);
    if      (tow<tow_p-302400.0) tow+=604800.0;
    else if (tow>tow_p+302400.0) tow-=604800.0;
    rtcm->time=gpst2time(week,tow);
}
/* adjust weekly rollover of BDS time ----------------------------------------*/
static int adjbdtweek(int week)
{
    int w;
    (void)time2bdt(gpst2bdt(utc2gpst(timeget())),&w);
    if (w<1) w=1; /* use 2006/1/1 if time is earlier than 2006/1/1 */
    return week+(w-week+512)/1024*1024;
}
/* adjust daily rollover of GLONASS time -------------------------------------*/
static void adjday_glot(rtcm_t *rtcm, double tod)
{
    gtime_t time;
    double tow,tod_p;
    int week;
    
    if (rtcm->time.time==0) rtcm->time=utc2gpst(timeget());
    time=timeadd(gpst2utc(rtcm->time),10800.0); /* glonass time */
    tow=time2gpst(time,&week);
    tod_p=fmod(tow,86400.0); tow-=tod_p;
    if      (tod<tod_p-43200.0) tod+=86400.0;
    else if (tod>tod_p+43200.0) tod-=86400.0;
    time=gpst2time(week,tow+tod);
    rtcm->time=utc2gpst(timeadd(time,-10800.0));
}
/* adjust carrier-phase rollover ---------------------------------------------*/
static double adjcp(rtcm_t *rtcm, int sat, int idx, double cp)
{
    if (rtcm->cp[sat-1][idx]==0.0) ;
    else if (cp<rtcm->cp[sat-1][idx]-750.0) cp+=1500.0;
    else if (cp>rtcm->cp[sat-1][idx]+750.0) cp-=1500.0;
    rtcm->cp[sat-1][idx]=cp;
    return cp;
}
/* loss-of-lock indicator ----------------------------------------------------*/
static int lossoflock(rtcm_t *rtcm, int sat, int idx, int lock)
{
    int lli=(!lock&&!rtcm->lock[sat-1][idx])||lock<rtcm->lock[sat-1][idx];
    rtcm->lock[sat-1][idx]=(uint16_t)lock;
    return lli;
}
/* S/N ratio -----------------------------------------------------------------*/
static uint16_t snratio(double snr)
{
    return (uint16_t)(snr<=0.0||100.0<=snr?0.0:snr/SNR_UNIT+0.5);
}
#if 0
/* get observation data index ------------------------------------------------*/
static int obsindex(obs_t *obs, gtime_t time, int sat)
{
    int i,j;
    
    for (i=0;i<obs->n;i++) {
        if (obs->data[i].sat==sat) return i; /* field already exists */
    }
    if (i>=MAXOBS) return -1; /* overflow */
    
    /* add new field */
    obs->data[i].time=time;
    obs->data[i].sat=sat;
    for (j=0;j<NFREQ+NEXOBS;j++) {
        obs->data[i].L[j]=obs->data[i].P[j]=0.0;
        obs->data[i].D[j]=0.0;
        obs->data[i].SNR[j]=obs->data[i].LLI[j]=obs->data[i].code[j]=0;
    }
    obs->n++;
    return i;
}
#endif
/* test station ID consistency -----------------------------------------------*/
static int test_staid(rtcm_t *rtcm, int staid)
{
    char *p;
    int type,id;
    
    /* test station id option */
    if ((p=strstr(rtcm->opt,"-STA="))&&sscanf(p,"-STA=%d",&id)==1) {
        if (staid!=id) return 0;
    }
    /* save station id */
    if (rtcm->staid==0||rtcm->obsflag) {
        rtcm->staid=staid;
    }
    else if (staid!=rtcm->staid) {
        type=getbitu(rtcm->buff,24,12);
        trace(2,"rtcm3 %d staid invalid id=%d %d\n",type,staid,rtcm->staid);
        
        /* reset station id if station id error */
        rtcm->staid=0;
        return 0;
    }
    return 1;
}
/* decode type 1001-1004 message header --------------------------------------*/
static int decode_head1001(rtcm_t *rtcm, int *sync)
{
    double tow;
    char *msg,tstr[64];
    int i=24,staid,nsat,type;
    
    type=getbitu(rtcm->buff,i,12); i+=12;
    
    if (i+52<=rtcm->len*8) {
        staid=getbitu(rtcm->buff,i,12);       i+=12;
        tow  =getbitu(rtcm->buff,i,30)*0.001; i+=30;
        *sync=getbitu(rtcm->buff,i, 1);       i+= 1;
        nsat =getbitu(rtcm->buff,i, 5);
    }
    else {
        trace(2,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
    /* test station ID */
    if (!test_staid(rtcm,staid)) return -1;
    
    adjweek(rtcm,tow);
    
    time2str(rtcm->time,tstr,2);
    trace(4,"decode_head1001: time=%s nsat=%d sync=%d\n",tstr,nsat,*sync);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," staid=%4d %s nsat=%2d sync=%d",staid,tstr,nsat,*sync);
    }
    return nsat;
}
/* decode type 1001: L1-only GPS RTK observation -----------------------------*/
static int decode_type1001(rtcm_t *rtcm)
{
    int sync;
    if (decode_head1001(rtcm,&sync)<0) return -1;
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode type 1002: extended L1-only GPS RTK observables --------------------*/
static int decode_type1002(rtcm_t *rtcm)
{
    double pr1,cnr1,tt,cp1,freq=FREQ1;
    int i=24+64,j,index,nsat,sync,prn,code,sat,ppr1,lock1,amb,sys;
    
    if ((nsat=decode_head1001(rtcm,&sync))<0) return -1;
    
    for (j=0;j<nsat&&rtcm->obs.n<MAXOBS&&i+74<=rtcm->len*8;j++) {
        prn  =getbitu(rtcm->buff,i, 6); i+= 6;
        code =getbitu(rtcm->buff,i, 1); i+= 1;
        pr1  =getbitu(rtcm->buff,i,24); i+=24;
        ppr1 =getbits(rtcm->buff,i,20); i+=20;
        lock1=getbitu(rtcm->buff,i, 7); i+= 7;
        amb  =getbitu(rtcm->buff,i, 8); i+= 8;
        cnr1 =getbitu(rtcm->buff,i, 8); i+= 8;
        if (prn<40) {
            sys=SYS_GPS;
        }
        else {
            sys=SYS_SBS; prn+=80;
        }
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 1002 satellite number error: prn=%d\n",prn);
            continue;
        }
        tt=timediff(rtcm->obs.data[0].time,rtcm->time);
        if (rtcm->obsflag||fabs(tt)>1E-9) {
            rtcm->obs.n=rtcm->obsflag=0;
        }
        if ((index=obsindex(&rtcm->obs,rtcm->time,sat))<0) continue;
        pr1=pr1*0.02+amb*PRUNIT_GPS;
        rtcm->obs.data[index].P[0]=pr1;
        
        if (ppr1!=(int)0xFFF80000) {
            cp1=adjcp(rtcm,sat,0,ppr1*0.0005*freq/CLIGHT);
            rtcm->obs.data[index].L[0]=pr1*freq/CLIGHT+cp1;
        }
        rtcm->obs.data[index].LLI[0]=lossoflock(rtcm,sat,0,lock1);
        rtcm->obs.data[index].SNR[0]=snratio(cnr1*0.25);
        rtcm->obs.data[index].code[0]=code?CODE_L1P:CODE_L1C;
    }
    return sync?0:1;
}
/* decode type 1003: L1&L2 gps rtk observables -------------------------------*/
static int decode_type1003(rtcm_t *rtcm)
{
    int sync;
    if (decode_head1001(rtcm,&sync)<0) return -1;
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode type 1004: extended L1&L2 GPS RTK observables ----------------------*/
static int decode_type1004(rtcm_t *rtcm)
{
    const int L2codes[]={CODE_L2X,CODE_L2P,CODE_L2D,CODE_L2W};
    double pr1,cnr1,cnr2,tt,cp1,cp2,freq[2]={FREQ1,FREQ2};
    int i=24+64,j,index,nsat,sync,prn,sat,code1,code2,pr21,ppr1,ppr2;
    int lock1,lock2,amb,sys;
    
    if ((nsat=decode_head1001(rtcm,&sync))<0) return -1;
    
    for (j=0;j<nsat&&rtcm->obs.n<MAXOBS&&i+125<=rtcm->len*8;j++) {
        prn  =getbitu(rtcm->buff,i, 6); i+= 6;
        code1=getbitu(rtcm->buff,i, 1); i+= 1;
        pr1  =getbitu(rtcm->buff,i,24); i+=24;
        ppr1 =getbits(rtcm->buff,i,20); i+=20;
        lock1=getbitu(rtcm->buff,i, 7); i+= 7;
        amb  =getbitu(rtcm->buff,i, 8); i+= 8;
        cnr1 =getbitu(rtcm->buff,i, 8); i+= 8;
        code2=getbitu(rtcm->buff,i, 2); i+= 2;
        pr21 =getbits(rtcm->buff,i,14); i+=14;
        ppr2 =getbits(rtcm->buff,i,20); i+=20;
        lock2=getbitu(rtcm->buff,i, 7); i+= 7;
        cnr2 =getbitu(rtcm->buff,i, 8); i+= 8;
        if (prn<40) {
            sys=SYS_GPS;
        }
        else {
            sys=SYS_SBS; prn+=80;
        }
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 1004 satellite number error: sys=%d prn=%d\n",sys,prn);
            continue;
        }
        tt=timediff(rtcm->obs.data[0].time,rtcm->time);
        if (rtcm->obsflag||fabs(tt)>1E-9) {
            rtcm->obs.n=rtcm->obsflag=0;
        }
        if ((index=obsindex(&rtcm->obs,rtcm->time,sat))<0) continue;
        pr1=pr1*0.02+amb*PRUNIT_GPS;
        rtcm->obs.data[index].P[0]=pr1;
        
        if (ppr1!=(int)0xFFF80000) {
            cp1=adjcp(rtcm,sat,0,ppr1*0.0005*freq[0]/CLIGHT);
            rtcm->obs.data[index].L[0]=pr1*freq[0]/CLIGHT+cp1;
        }
        rtcm->obs.data[index].LLI[0]=lossoflock(rtcm,sat,0,lock1);
        rtcm->obs.data[index].SNR[0]=snratio(cnr1*0.25);
        rtcm->obs.data[index].code[0]=code1?CODE_L1P:CODE_L1C;
        
        if (pr21!=(int)0xFFFFE000) {
            rtcm->obs.data[index].P[1]=pr1+pr21*0.02;
        }
        if (ppr2!=(int)0xFFF80000) {
            cp2=adjcp(rtcm,sat,1,ppr2*0.0005*freq[1]/CLIGHT);
            rtcm->obs.data[index].L[1]=pr1*freq[1]/CLIGHT+cp2;
        }
        rtcm->obs.data[index].LLI[1]=lossoflock(rtcm,sat,1,lock2);
        rtcm->obs.data[index].SNR[1]=snratio(cnr2*0.25);
        rtcm->obs.data[index].code[1]=L2codes[code2];
    }
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* get signed 38bit field ----------------------------------------------------*/
static double getbits_38(const uint8_t *buff, int pos)
{
    return (double)getbits(buff,pos,32)*64.0+getbitu(buff,pos+32,6);
}
/* decode type 1005: stationary RTK reference station ARP --------------------*/
static int decode_type1005(rtcm_t *rtcm)
{
    double rr[3],re[3],pos[3];
    char *msg;
    int i=24+12,j,staid,itrf;
    
    if (i+140==rtcm->len*8) {
        staid=getbitu(rtcm->buff,i,12); i+=12;
        itrf =getbitu(rtcm->buff,i, 6); i+= 6+4;
        rr[0]=getbits_38(rtcm->buff,i); i+=38+2;
        rr[1]=getbits_38(rtcm->buff,i); i+=38+2;
        rr[2]=getbits_38(rtcm->buff,i);
    }
    else {
        trace(2,"rtcm3 1005 length error: len=%d\n",rtcm->len);
        return -1;
    }
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        for (j=0;j<3;j++) re[j]=rr[j]*0.0001;
        ecef2pos(re,pos);
        sprintf(msg," staid=%4d pos=%.8f %.8f %.3f",staid,pos[0]*R2D,pos[1]*R2D,
                pos[2]);
    }
    /* test station id */
    if (!test_staid(rtcm,staid)) return -1;
    
    sprintf(rtcm->sta.name,"%04d",staid);
    rtcm->sta.deltype=0; /* xyz */
    for (j=0;j<3;j++) {
        rtcm->sta.pos[j]=rr[j]*0.0001;
        rtcm->sta.del[j]=0.0;
    }
    rtcm->sta.hgt=0.0;
    rtcm->sta.itrf=itrf;
    return 5;
}
/* decode type 1006: stationary RTK reference station ARP with height --------*/
static int decode_type1006(rtcm_t *rtcm)
{
    double rr[3],re[3],pos[3],anth;
    char *msg;
    int i=24+12,j,staid,itrf;
    
    if (i+156<=rtcm->len*8) {
        staid=getbitu(rtcm->buff,i,12); i+=12;
        itrf =getbitu(rtcm->buff,i, 6); i+= 6+4;
        rr[0]=getbits_38(rtcm->buff,i); i+=38+2;
        rr[1]=getbits_38(rtcm->buff,i); i+=38+2;
        rr[2]=getbits_38(rtcm->buff,i); i+=38;
        anth =getbitu(rtcm->buff,i,16);
    }
    else {
        trace(2,"rtcm3 1006 length error: len=%d\n",rtcm->len);
        return -1;
    }
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        for (j=0;j<3;j++) re[j]=rr[j]*0.0001;
        ecef2pos(re,pos);
        sprintf(msg," staid=%4d pos=%.8f %.8f %.3f anth=%.3f",staid,pos[0]*R2D,
                pos[1]*R2D,pos[2],anth*0.0001);
    }
    /* test station id */
    if (!test_staid(rtcm,staid)) return -1;
    
    sprintf(rtcm->sta.name,"%04d",staid);
    rtcm->sta.deltype=1; /* xyz */
    for (j=0;j<3;j++) {
        rtcm->sta.pos[j]=rr[j]*0.0001;
        rtcm->sta.del[j]=0.0;
    }
    rtcm->sta.hgt=anth*0.0001;
    rtcm->sta.itrf=itrf;
    return 5;
}
/* decode type 1007: antenna descriptor --------------------------------------*/
static int decode_type1007(rtcm_t *rtcm)
{
    char des[32]="";
    char *msg;
    int i=24+12,j,staid,n,setup;
    
    n=getbitu(rtcm->buff,i+12,8);
    
    if (i+28+8*n<=rtcm->len*8) {
        staid=getbitu(rtcm->buff,i,12); i+=12+8;
        for (j=0;j<n&&j<31;j++) {
            des[j]=(char)getbitu(rtcm->buff,i,8); i+=8;
        }
        setup=getbitu(rtcm->buff,i, 8);
    }
    else {
        trace(2,"rtcm3 1007 length error: len=%d\n",rtcm->len);
        return -1;
    }
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," staid=%4d",staid);
    }
    /* test station ID */
    if (!test_staid(rtcm,staid)) return -1;
    
    sprintf(rtcm->sta.name,"%04d",staid);
    strncpy(rtcm->sta.antdes,des,n); rtcm->sta.antdes[n]='\0';
    rtcm->sta.antsetup=setup;
    rtcm->sta.antsno[0]='\0';
    return 5;
}
/* decode type 1008: antenna descriptor & serial number ----------------------*/
static int decode_type1008(rtcm_t *rtcm)
{
    char des[32]="",sno[32]="";
    char *msg;
    int i=24+12,j,staid,n,m,setup;
    
    n=getbitu(rtcm->buff,i+12,8);
    m=getbitu(rtcm->buff,i+28+8*n,8);
    
    if (i+36+8*(n+m)<=rtcm->len*8) {
        staid=getbitu(rtcm->buff,i,12); i+=12+8;
        for (j=0;j<n&&j<31;j++) {
            des[j]=(char)getbitu(rtcm->buff,i,8); i+=8;
        }
        setup=getbitu(rtcm->buff,i, 8); i+=8+8;
        for (j=0;j<m&&j<31;j++) {
            sno[j]=(char)getbitu(rtcm->buff,i,8); i+=8;
        }
    }
    else {
        trace(2,"rtcm3 1008 length error: len=%d\n",rtcm->len);
        return -1;
    }
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," staid=%4d",staid);
    }
    /* test station ID */
    if (!test_staid(rtcm,staid)) return -1;
    
    sprintf(rtcm->sta.name,"%04d",staid);
    strncpy(rtcm->sta.antdes,des,n); rtcm->sta.antdes[n]='\0';
    rtcm->sta.antsetup=setup;
    strncpy(rtcm->sta.antsno,sno,m); rtcm->sta.antsno[m]='\0';
    return 5;
}
/* decode type 1009-1012 message header --------------------------------------*/
static int decode_head1009(rtcm_t *rtcm, int *sync)
{
    double tod;
    char *msg,tstr[64];
    int i=24,staid,nsat,type;
    
    type=getbitu(rtcm->buff,i,12); i+=12;
    
    if (i+49<=rtcm->len*8) {
        staid=getbitu(rtcm->buff,i,12);       i+=12;
        tod  =getbitu(rtcm->buff,i,27)*0.001; i+=27; /* sec in a day */
        *sync=getbitu(rtcm->buff,i, 1);       i+= 1;
        nsat =getbitu(rtcm->buff,i, 5);
    }
    else {
        trace(2,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
    /* test station ID */
    if (!test_staid(rtcm,staid)) return -1;
    
    adjday_glot(rtcm,tod);
    
    time2str(rtcm->time,tstr,2);
    trace(4,"decode_head1009: time=%s nsat=%d sync=%d\n",tstr,nsat,*sync);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," staid=%4d %s nsat=%2d sync=%d",staid,tstr,nsat,*sync);
    }
    return nsat;
}
/* decode type 1009: L1-only glonass rtk observables -------------------------*/
static int decode_type1009(rtcm_t *rtcm)
{
    int sync;
    if (decode_head1009(rtcm,&sync)<0) return -1;
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode type 1010: extended L1-only glonass rtk observables ----------------*/
static int decode_type1010(rtcm_t *rtcm, nav_t *nav)
{
    double pr1,cnr1,tt,cp1,freq1;
    int i=24+61,j,index,nsat,sync,prn,sat,code,fcn,ppr1,lock1,amb,sys=SYS_GLO;
    
    if ((nsat=decode_head1009(rtcm,&sync))<0) return -1;
    
    for (j=0;j<nsat&&rtcm->obs.n<MAXOBS&&i+79<=rtcm->len*8;j++) {
        prn  =getbitu(rtcm->buff,i, 6); i+= 6;
        code =getbitu(rtcm->buff,i, 1); i+= 1;
        fcn  =getbitu(rtcm->buff,i, 5); i+= 5; /* fcn+7 */
        pr1  =getbitu(rtcm->buff,i,25); i+=25;
        ppr1 =getbits(rtcm->buff,i,20); i+=20;
        lock1=getbitu(rtcm->buff,i, 7); i+= 7;
        amb  =getbitu(rtcm->buff,i, 7); i+= 7;
        cnr1 =getbitu(rtcm->buff,i, 8); i+= 8;
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 1010 satellite number error: prn=%d\n",prn);
            continue;
        }
        if (!nav->glo_fcn[prn-1]) {
            nav->glo_fcn[prn-1]=fcn-7+8; /* fcn+8 */
        }
        tt=timediff(rtcm->obs.data[0].time,rtcm->time);
        if (rtcm->obsflag||fabs(tt)>1E-9) {
            rtcm->obs.n=rtcm->obsflag=0;
        }
        if ((index=obsindex(&rtcm->obs,rtcm->time,sat))<0) continue;
        pr1=pr1*0.02+amb*PRUNIT_GLO;
        rtcm->obs.data[index].P[0]=pr1;
        
        if (ppr1!=(int)0xFFF80000) {
            freq1=code2freq(SYS_GLO,CODE_L1C,fcn-7);
            cp1=adjcp(rtcm,sat,0,ppr1*0.0005*freq1/CLIGHT);
            rtcm->obs.data[index].L[0]=pr1*freq1/CLIGHT+cp1;
        }
        rtcm->obs.data[index].LLI[0]=lossoflock(rtcm,sat,0,lock1);
        rtcm->obs.data[index].SNR[0]=snratio(cnr1*0.25);
        rtcm->obs.data[index].code[0]=code?CODE_L1P:CODE_L1C;
    }
    return sync?0:1;
}
/* decode type 1011: L1&L2 GLONASS RTK observables ---------------------------*/
static int decode_type1011(rtcm_t *rtcm)
{
    int sync;
    if (decode_head1009(rtcm,&sync)<0) return -1;
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode type 1012: extended L1&L2 GLONASS RTK observables ------------------*/
static int decode_type1012(rtcm_t *rtcm, nav_t *nav)
{
    double pr1,cnr1,cnr2,tt,cp1,cp2,freq1,freq2;
    int i=24+61,j,index,nsat,sync,prn,sat,fcn,code1,code2,pr21,ppr1,ppr2;
    int lock1,lock2,amb,sys=SYS_GLO;
    
    if ((nsat=decode_head1009(rtcm,&sync))<0) return -1;
    
    for (j=0;j<nsat&&rtcm->obs.n<MAXOBS&&i+130<=rtcm->len*8;j++) {
        prn  =getbitu(rtcm->buff,i, 6); i+= 6;
        code1=getbitu(rtcm->buff,i, 1); i+= 1;
        fcn  =getbitu(rtcm->buff,i, 5); i+= 5; /* fcn+7 */
        pr1  =getbitu(rtcm->buff,i,25); i+=25;
        ppr1 =getbits(rtcm->buff,i,20); i+=20;
        lock1=getbitu(rtcm->buff,i, 7); i+= 7;
        amb  =getbitu(rtcm->buff,i, 7); i+= 7;
        cnr1 =getbitu(rtcm->buff,i, 8); i+= 8;
        code2=getbitu(rtcm->buff,i, 2); i+= 2;
        pr21 =getbits(rtcm->buff,i,14); i+=14;
        ppr2 =getbits(rtcm->buff,i,20); i+=20;
        lock2=getbitu(rtcm->buff,i, 7); i+= 7;
        cnr2 =getbitu(rtcm->buff,i, 8); i+= 8;
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 1012 satellite number error: sys=%d prn=%d\n",sys,prn);
            continue;
        }
        if (!nav->glo_fcn[prn-1]) {
            nav->glo_fcn[prn-1]=fcn-7+8; /* fcn+8 */
        }
        tt=timediff(rtcm->obs.data[0].time,rtcm->time);
        if (rtcm->obsflag||fabs(tt)>1E-9) {
            rtcm->obs.n=rtcm->obsflag=0;
        }
        if ((index=obsindex(&rtcm->obs,rtcm->time,sat))<0) continue;
        pr1=pr1*0.02+amb*PRUNIT_GLO;
        rtcm->obs.data[index].P[0]=pr1;
        
        if (ppr1!=(int)0xFFF80000) {
            freq1=code2freq(SYS_GLO,CODE_L1C,fcn-7);
            cp1=adjcp(rtcm,sat,0,ppr1*0.0005*freq1/CLIGHT);
            rtcm->obs.data[index].L[0]=pr1*freq1/CLIGHT+cp1;
        }
        rtcm->obs.data[index].LLI[0]=lossoflock(rtcm,sat,0,lock1);
        rtcm->obs.data[index].SNR[0]=snratio(cnr1*0.25);
        rtcm->obs.data[index].code[0]=code1?CODE_L1P:CODE_L1C;
        
        if (pr21!=(int)0xFFFFE000) {
            rtcm->obs.data[index].P[1]=pr1+pr21*0.02;
        }
        if (ppr2!=(int)0xFFF80000) {
            freq2=code2freq(SYS_GLO,CODE_L2C,fcn-7);
            cp2=adjcp(rtcm,sat,1,ppr2*0.0005*freq2/CLIGHT);
            rtcm->obs.data[index].L[1]=pr1*freq2/CLIGHT+cp2;
        }
        rtcm->obs.data[index].LLI[1]=lossoflock(rtcm,sat,1,lock2);
        rtcm->obs.data[index].SNR[1]=snratio(cnr2*0.25);
        rtcm->obs.data[index].code[1]=code2?CODE_L2P:CODE_L2C;
    }
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode type 1013: system parameters ---------------------------------------*/
static int decode_type1013(rtcm_t *rtcm)
{
    return 0;
}
/* decode type 1019: GPS ephemerides -----------------------------------------*/
static int decode_type1019(rtcm_t *rtcm, eph_t *eph)
{
    double toc,sqrtA,tt;
    char *msg;
    int i=24+12,prn,sat,week,sys=SYS_GPS;
    
    if (i+476<=rtcm->len*8) {
        prn        =getbitu(rtcm->buff,i, 6);              i+= 6;
        week       =getbitu(rtcm->buff,i,10);              i+=10;
        eph->sva   =getbitu(rtcm->buff,i, 4);              i+= 4;
        eph->code  =getbitu(rtcm->buff,i, 2);              i+= 2;
        eph->idot  =getbits(rtcm->buff,i,14)*P2_43*SC2RAD; i+=14;
        eph->iode  =getbitu(rtcm->buff,i, 8);              i+= 8;
        toc        =getbitu(rtcm->buff,i,16)*16.0;         i+=16;
        eph->f2    =getbits(rtcm->buff,i, 8)*P2_55;        i+= 8;
        eph->f1    =getbits(rtcm->buff,i,16)*P2_43;        i+=16;
        eph->f0    =getbits(rtcm->buff,i,22)*P2_31;        i+=22;
        eph->iodc  =getbitu(rtcm->buff,i,10);              i+=10;
        eph->crs   =getbits(rtcm->buff,i,16)*P2_5;         i+=16;
        eph->deln  =getbits(rtcm->buff,i,16)*P2_43*SC2RAD; i+=16;
        eph->M0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cuc   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph->e     =getbitu(rtcm->buff,i,32)*P2_33;        i+=32;
        eph->cus   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        sqrtA      =getbitu(rtcm->buff,i,32)*P2_19;        i+=32;
        eph->toes  =getbitu(rtcm->buff,i,16)*16.0;         i+=16;
        eph->cic   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph->OMG0  =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cis   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph->i0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->crc   =getbits(rtcm->buff,i,16)*P2_5;         i+=16;
        eph->omg   =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->OMGd  =getbits(rtcm->buff,i,24)*P2_43*SC2RAD; i+=24;
        eph->tgd[0]=getbits(rtcm->buff,i, 8)*P2_31;        i+= 8;
        eph->svh   =getbitu(rtcm->buff,i, 6);              i+= 6;
        eph->flag  =getbitu(rtcm->buff,i, 1);              i+= 1;
        eph->fit   =getbitu(rtcm->buff,i, 1)?0.0:4.0; /* 0:4hr,1:>4hr */
    }
    else {
        trace(2,"rtcm3 1019 length error: len=%d\n",rtcm->len);
        return -1;
    }
    if (prn>=40) {
        sys=SYS_SBS; prn+=80;
    }
    trace(4,"decode_type1019: prn=%d iode=%d toe=%.0f\n",prn,eph->iode,eph->toes);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," prn=%2d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
                prn,eph->iode,eph->iodc,week,eph->toes,toc,eph->svh);
    }
    if (!(sat=satno(sys,prn))) {
        trace(2,"rtcm3 1019 satellite number error: prn=%d\n",prn);
        return -1;
    }
    eph->sat=sat;
    eph->week=adjgpsweek(week);
    if (rtcm->time.time==0) rtcm->time=utc2gpst(timeget());
    tt=timediff(gpst2time(eph->week,eph->toes),rtcm->time);
    if      (tt<-302400.0) eph->week++;
    else if (tt>=302400.0) eph->week--;
    eph->toe=gpst2time(eph->week,eph->toes);
    eph->toc=gpst2time(eph->week,toc);
    eph->ttr=rtcm->time;
    eph->A=sqrtA*sqrtA;
    return 2;
}
/* decode type 1020: GLONASS ephemerides -------------------------------------*/
static int decode_type1020(rtcm_t *rtcm, geph_t *geph)
{
    double tk_h,tk_m,tk_s,toe,tow,tod,tof;
    char *msg;
    int i=24+12,prn,sat,week,tb,bn,sys=SYS_GLO;
    
    if (i+348<=rtcm->len*8) {
        prn         =getbitu(rtcm->buff,i, 6);           i+= 6;
        geph->frq   =getbitu(rtcm->buff,i, 5)-7;         i+= 5+2+2;
        tk_h        =getbitu(rtcm->buff,i, 5);           i+= 5;
        tk_m        =getbitu(rtcm->buff,i, 6);           i+= 6;
        tk_s        =getbitu(rtcm->buff,i, 1)*30.0;      i+= 1;
        bn          =getbitu(rtcm->buff,i, 1);           i+= 1+1;
        tb          =getbitu(rtcm->buff,i, 7);           i+= 7;
        geph->vel[0]=getbitg(rtcm->buff,i,24)*P2_20*1E3; i+=24;
        geph->pos[0]=getbitg(rtcm->buff,i,27)*P2_11*1E3; i+=27;
        geph->acc[0]=getbitg(rtcm->buff,i, 5)*P2_30*1E3; i+= 5;
        geph->vel[1]=getbitg(rtcm->buff,i,24)*P2_20*1E3; i+=24;
        geph->pos[1]=getbitg(rtcm->buff,i,27)*P2_11*1E3; i+=27;
        geph->acc[1]=getbitg(rtcm->buff,i, 5)*P2_30*1E3; i+= 5;
        geph->vel[2]=getbitg(rtcm->buff,i,24)*P2_20*1E3; i+=24;
        geph->pos[2]=getbitg(rtcm->buff,i,27)*P2_11*1E3; i+=27;
        geph->acc[2]=getbitg(rtcm->buff,i, 5)*P2_30*1E3; i+= 5+1;
        geph->gamn  =getbitg(rtcm->buff,i,11)*P2_40;     i+=11+3;
        geph->taun  =getbitg(rtcm->buff,i,22)*P2_30;     i+=22;
        geph->dtaun =getbitg(rtcm->buff,i, 5)*P2_30;     i+=5;
        geph->age   =getbitu(rtcm->buff,i, 5);
    }
    else {
        trace(2,"rtcm3 1020 length error: len=%d\n",rtcm->len);
        return -1;
    }
    if (!(sat=satno(sys,prn))) {
        trace(2,"rtcm3 1020 satellite number error: prn=%d\n",prn);
        return -1;
    }
    trace(4,"decode_type1020: prn=%d tk=%02.0f:%02.0f:%02.0f\n",prn,tk_h,tk_m,tk_s);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," prn=%2d tk=%02.0f:%02.0f:%02.0f frq=%2d bn=%d tb=%d",
                prn,tk_h,tk_m,tk_s,geph->frq,bn,tb);
    }
    geph->sat=sat;
    geph->svh=bn;
    geph->iode=tb&0x7F;
    if (rtcm->time.time==0) rtcm->time=utc2gpst(timeget());
    tow=time2gpst(gpst2utc(rtcm->time),&week);
    tod=fmod(tow,86400.0); tow-=tod;
    tof=tk_h*3600.0+tk_m*60.0+tk_s-10800.0; /* lt->utc */
    if      (tof<tod-43200.0) tof+=86400.0;
    else if (tof>tod+43200.0) tof-=86400.0;
    geph->tof=utc2gpst(gpst2time(week,tow+tof));
    toe=tb*900.0-10800.0; /* lt->utc */
    if      (toe<tod-43200.0) toe+=86400.0;
    else if (toe>tod+43200.0) toe-=86400.0;
    geph->toe=utc2gpst(gpst2time(week,tow+toe)); /* utc->gpst */
    return 2;
}
/* decode type 1021: helmert/abridged molodenski -----------------------------*/
static int decode_type1021(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1021: not supported message\n");
    return 0;
}
/* decode type 1022: Moledenski-Badekas transfromation -----------------------*/
static int decode_type1022(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1022: not supported message\n");
    return 0;
}
/* decode type 1023: residual, ellipsoidal grid representation ---------------*/
static int decode_type1023(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1023: not supported message\n");
    return 0;
}
/* decode type 1024: residual, plane grid representation ---------------------*/
static int decode_type1024(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1024: not supported message\n");
    return 0;
}
/* decode type 1025: projection (types except LCC2SP,OM) ---------------------*/
static int decode_type1025(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1025: not supported message\n");
    return 0;
}
/* decode type 1026: projection (LCC2SP - lambert conic conformal (2sp)) -----*/
static int decode_type1026(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1026: not supported message\n");
    return 0;
}
/* decode type 1027: projection (type OM - oblique mercator) -----------------*/
static int decode_type1027(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1027: not supported message\n");
    return 0;
}
/* decode type 1029: UNICODE text string -------------------------------------*/
static int decode_type1029(rtcm_t *rtcm)
{
    char *msg;
    int i=24+12,j,staid,mjd,tod,nchar,cunit;
    
    if (i+60<=rtcm->len*8) {
        staid=getbitu(rtcm->buff,i,12); i+=12;
        mjd  =getbitu(rtcm->buff,i,16); i+=16;
        tod  =getbitu(rtcm->buff,i,17); i+=17;
        nchar=getbitu(rtcm->buff,i, 7); i+= 7;
        cunit=getbitu(rtcm->buff,i, 8); i+= 8;
    }
    else {
        trace(2,"rtcm3 1029 length error: len=%d\n",rtcm->len);
        return -1;
    }
    if (i+nchar*8>rtcm->len*8) {
        trace(2,"rtcm3 1029 length error: len=%d nchar=%d\n",rtcm->len,nchar);
        return -1;
    } 
    for (j=0;j<nchar&&j<126;j++) {
        rtcm->msg[j]=getbitu(rtcm->buff,i,8); i+=8;
    }
    rtcm->msg[j]='\0';
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," staid=%4d text=%s",staid,rtcm->msg);
    }
    return 0;
}
/* decode type 1030: network RTK residual ------------------------------------*/
static int decode_type1030(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1030: not supported message\n");
    return 0;
}
/* decode type 1031: GLONASS network RTK residual ----------------------------*/
static int decode_type1031(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1031: not supported message\n");
    return 0;
}
/* decode type 1032: physical reference station position information ---------*/
static int decode_type1032(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1032: not supported message\n");
    return 0;
}
/* decode type 1033: receiver and antenna descriptor -------------------------*/
static int decode_type1033(rtcm_t *rtcm)
{
    char des[32]="",sno[32]="",rec[32]="",ver[32]="",rsn[32]="";
    char *msg;
    int i=24+12,j,staid,n,m,n1,n2,n3,setup;
    
    n =getbitu(rtcm->buff,i+12,8);
    m =getbitu(rtcm->buff,i+28+8*n,8);
    n1=getbitu(rtcm->buff,i+36+8*(n+m),8);
    n2=getbitu(rtcm->buff,i+44+8*(n+m+n1),8);
    n3=getbitu(rtcm->buff,i+52+8*(n+m+n1+n2),8);
    
    if (i+60+8*(n+m+n1+n2+n3)<=rtcm->len*8) {
        staid=getbitu(rtcm->buff,i,12); i+=12+8;
        for (j=0;j<n&&j<31;j++) {
            des[j]=(char)getbitu(rtcm->buff,i,8); i+=8;
        }
        setup=getbitu(rtcm->buff,i, 8); i+=8+8;
        for (j=0;j<m&&j<31;j++) {
            sno[j]=(char)getbitu(rtcm->buff,i,8); i+=8;
        }
        i+=8;
        for (j=0;j<n1&&j<31;j++) {
            rec[j]=(char)getbitu(rtcm->buff,i,8); i+=8;
        }
        i+=8;
        for (j=0;j<n2&&j<31;j++) {
            ver[j]=(char)getbitu(rtcm->buff,i,8); i+=8;
        }
        i+=8;
        for (j=0;j<n3&&j<31;j++) {
            rsn[j]=(char)getbitu(rtcm->buff,i,8); i+=8;
        }
    }
    else {
        trace(2,"rtcm3 1033 length error: len=%d\n",rtcm->len);
        return -1;
    }
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," staid=%4d",staid);
    }
    /* test station id */
    if (!test_staid(rtcm,staid)) return -1;
    
    sprintf(rtcm->sta.name,"%04d",staid);
    strncpy(rtcm->sta.antdes, des,n ); rtcm->sta.antdes [n] ='\0';
    rtcm->sta.antsetup=setup;
    strncpy(rtcm->sta.antsno, sno,m ); rtcm->sta.antsno [m] ='\0';
    strncpy(rtcm->sta.rectype,rec,n1); rtcm->sta.rectype[n1]='\0';
    strncpy(rtcm->sta.recver, ver,n2); rtcm->sta.recver [n2]='\0';
    strncpy(rtcm->sta.recsno, rsn,n3); rtcm->sta.recsno [n3]='\0';
    
    trace(3,"rtcm3 1033: ant=%s:%s rec=%s:%s:%s\n",des,sno,rec,ver,rsn);
    return 5;
}
/* decode type 1034: GPS network FKP gradient --------------------------------*/
static int decode_type1034(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1034: not supported message\n");
    return 0;
}
/* decode type 1035: GLONASS network FKP gradient ----------------------------*/
static int decode_type1035(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1035: not supported message\n");
    return 0;
}
/* decode type 1037: GLONASS network RTK ionospheric correction difference ---*/
static int decode_type1037(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1037: not supported message\n");
    return 0;
}
/* decode type 1038: GLONASS network RTK geometic correction difference ------*/
static int decode_type1038(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1038: not supported message\n");
    return 0;
}
/* decode type 1039: GLONASS network RTK combined correction difference ------*/
static int decode_type1039(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1039: not supported message\n");
    return 0;
}
/* decode type 1041: NavIC/IRNSS ephemerides ---------------------------------*/
static int decode_type1041(rtcm_t *rtcm, eph_t *eph)
{
    double toc,sqrtA,tt;
    char *msg;
    int i=24+12,prn,sat,week,sys=SYS_IRN;
    
    if (i+482-12<=rtcm->len*8) {
        prn        =getbitu(rtcm->buff,i, 6);              i+= 6;
        week       =getbitu(rtcm->buff,i,10);              i+=10;
        eph->f0    =getbits(rtcm->buff,i,22)*P2_31;        i+=22;
        eph->f1    =getbits(rtcm->buff,i,16)*P2_43;        i+=16;
        eph->f2    =getbits(rtcm->buff,i, 8)*P2_55;        i+= 8;
        eph->sva   =getbitu(rtcm->buff,i, 4);              i+= 4;
        toc        =getbitu(rtcm->buff,i,16)*16.0;         i+=16;
        eph->tgd[0]=getbits(rtcm->buff,i, 8)*P2_31;        i+= 8;
        eph->deln  =getbits(rtcm->buff,i,22)*P2_41*SC2RAD; i+=22;
        eph->iode  =getbitu(rtcm->buff,i, 8);              i+= 8+10; /* IODEC */
        eph->svh   =getbitu(rtcm->buff,i, 2);              i+= 2; /* L5+Sflag */
        eph->cuc   =getbits(rtcm->buff,i,15)*P2_28;        i+=15;
        eph->cus   =getbits(rtcm->buff,i,15)*P2_28;        i+=15;
        eph->cic   =getbits(rtcm->buff,i,15)*P2_28;        i+=15;
        eph->cis   =getbits(rtcm->buff,i,15)*P2_28;        i+=15;
        eph->crc   =getbits(rtcm->buff,i,15)*0.0625;       i+=15;
        eph->crs   =getbits(rtcm->buff,i,15)*0.0625;       i+=15;
        eph->idot  =getbits(rtcm->buff,i,14)*P2_43*SC2RAD; i+=14;
        eph->M0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->toes  =getbitu(rtcm->buff,i,16)*16.0;         i+=16;
        eph->e     =getbitu(rtcm->buff,i,32)*P2_33;        i+=32;
        sqrtA      =getbitu(rtcm->buff,i,32)*P2_19;        i+=32;
        eph->OMG0  =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->omg   =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->OMGd  =getbits(rtcm->buff,i,22)*P2_41*SC2RAD; i+=22;
        eph->i0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD;
    }
    else {
        trace(2,"rtcm3 1041 length error: len=%d\n",rtcm->len);
        return -1;
    }
    trace(4,"decode_type1041: prn=%d iode=%d toe=%.0f\n",prn,eph->iode,eph->toes);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," prn=%2d iode=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
                prn,eph->iode,week,eph->toes,toc,eph->svh);
    }
    if (!(sat=satno(sys,prn))) {
        trace(2,"rtcm3 1041 satellite number error: prn=%d\n",prn);
        return -1;
    }
    eph->sat=sat;
    eph->week=adjgpsweek(week);
    if (rtcm->time.time==0) rtcm->time=utc2gpst(timeget());
    tt=timediff(gpst2time(eph->week,eph->toes),rtcm->time);
    if      (tt<-302400.0) eph->week++;
    else if (tt>=302400.0) eph->week--;
    eph->toe=gpst2time(eph->week,eph->toes);
    eph->toc=gpst2time(eph->week,toc);
    eph->ttr=rtcm->time;
    eph->A=sqrtA*sqrtA;
    eph->iodc=eph->iode;
    return 2;
}
/* decode type 1044: QZSS ephemerides ----------------------------------------*/
static int decode_type1044(rtcm_t *rtcm, eph_t *eph)
{
    double toc,sqrtA,tt;
    char *msg;
    int i=24+12,prn,sat,week,sys=SYS_QZS;
    
    if (i+473<=rtcm->len*8) {
        prn        =getbitu(rtcm->buff,i, 4)+192;          i+= 4;
        toc        =getbitu(rtcm->buff,i,16)*16.0;         i+=16;
        eph->f2    =getbits(rtcm->buff,i, 8)*P2_55;        i+= 8;
        eph->f1    =getbits(rtcm->buff,i,16)*P2_43;        i+=16;
        eph->f0    =getbits(rtcm->buff,i,22)*P2_31;        i+=22;
        eph->iode  =getbitu(rtcm->buff,i, 8);              i+= 8;
        eph->crs   =getbits(rtcm->buff,i,16)*P2_5;         i+=16;
        eph->deln  =getbits(rtcm->buff,i,16)*P2_43*SC2RAD; i+=16;
        eph->M0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cuc   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph->e     =getbitu(rtcm->buff,i,32)*P2_33;        i+=32;
        eph->cus   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        sqrtA      =getbitu(rtcm->buff,i,32)*P2_19;        i+=32;
        eph->toes  =getbitu(rtcm->buff,i,16)*16.0;         i+=16;
        eph->cic   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph->OMG0  =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cis   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph->i0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->crc   =getbits(rtcm->buff,i,16)*P2_5;         i+=16;
        eph->omg   =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->OMGd  =getbits(rtcm->buff,i,24)*P2_43*SC2RAD; i+=24;
        eph->idot  =getbits(rtcm->buff,i,14)*P2_43*SC2RAD; i+=14;
        eph->code  =getbitu(rtcm->buff,i, 2);              i+= 2;
        week       =getbitu(rtcm->buff,i,10);              i+=10;
        eph->sva   =getbitu(rtcm->buff,i, 4);              i+= 4;
        eph->svh   =getbitu(rtcm->buff,i, 6);              i+= 6;
        eph->tgd[0]=getbits(rtcm->buff,i, 8)*P2_31;        i+= 8;
        eph->iodc  =getbitu(rtcm->buff,i,10);              i+=10;
        eph->fit   =getbitu(rtcm->buff,i, 1)?0.0:2.0; /* 0:2hr,1:>2hr */
    }
    else {
        trace(2,"rtcm3 1044 length error: len=%d\n",rtcm->len);
        return -1;
    }
    trace(4,"decode_type1044: prn=%d iode=%d toe=%.0f\n",prn,eph->iode,eph->toes);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," prn=%3d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
                prn,eph->iode,eph->iodc,week,eph->toes,toc,eph->svh);
    }
    if (!(sat=satno(sys,prn))) {
        trace(2,"rtcm3 1044 satellite number error: prn=%d\n",prn);
        return -1;
    }
    eph->sat=sat;
    eph->week=adjgpsweek(week);
    if (rtcm->time.time==0) rtcm->time=utc2gpst(timeget());
    tt=timediff(gpst2time(eph->week,eph->toes),rtcm->time);
    if      (tt<-302400.0) eph->week++;
    else if (tt>=302400.0) eph->week--;
    eph->toe=gpst2time(eph->week,eph->toes);
    eph->toc=gpst2time(eph->week,toc);
    eph->ttr=rtcm->time;
    eph->A=sqrtA*sqrtA;
    eph->flag=1; /* fixed to 1 */
    return 2;
}
/* decode type 1045: Galileo F/NAV satellite ephemerides ---------------------*/
static int decode_type1045(rtcm_t *rtcm, eph_t *eph)
{
    double toc,sqrtA,tt;
    char *msg;
    int i=24+12,prn,sat,week,e5a_hs,e5a_dvs,rsv,sys=SYS_GAL;
    
    if (strstr(rtcm->opt,"-GALINAV")) return 0;

    if (i+484<=rtcm->len*8) {
        prn        =getbitu(rtcm->buff,i, 6);              i+= 6;
        week       =getbitu(rtcm->buff,i,12);              i+=12; /* gst-week */
        eph->iode  =getbitu(rtcm->buff,i,10);              i+=10;
        eph->sva   =getbitu(rtcm->buff,i, 8);              i+= 8;
        eph->idot  =getbits(rtcm->buff,i,14)*P2_43*SC2RAD; i+=14;
        toc        =getbitu(rtcm->buff,i,14)*60.0;         i+=14;
        eph->f2    =getbits(rtcm->buff,i, 6)*P2_59;        i+= 6;
        eph->f1    =getbits(rtcm->buff,i,21)*P2_46;        i+=21;
        eph->f0    =getbits(rtcm->buff,i,31)*P2_34;        i+=31;
        eph->crs   =getbits(rtcm->buff,i,16)*P2_5;         i+=16;
        eph->deln  =getbits(rtcm->buff,i,16)*P2_43*SC2RAD; i+=16;
        eph->M0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cuc   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph->e     =getbitu(rtcm->buff,i,32)*P2_33;        i+=32;
        eph->cus   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        sqrtA      =getbitu(rtcm->buff,i,32)*P2_19;        i+=32;
        eph->toes  =getbitu(rtcm->buff,i,14)*60.0;         i+=14;
        eph->cic   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph->OMG0  =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cis   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph->i0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->crc   =getbits(rtcm->buff,i,16)*P2_5;         i+=16;
        eph->omg   =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->OMGd  =getbits(rtcm->buff,i,24)*P2_43*SC2RAD; i+=24;
        eph->tgd[0]=getbits(rtcm->buff,i,10)*P2_32;        i+=10; /* E5a/E1 */
        e5a_hs     =getbitu(rtcm->buff,i, 2);              i+= 2; /* OSHS */
        e5a_dvs    =getbitu(rtcm->buff,i, 1);              i+= 1; /* OSDVS */
        rsv        =getbitu(rtcm->buff,i, 7);
    }
    else {
        trace(2,"rtcm3 1045 length error: len=%d\n",rtcm->len);
        return -1;
    }
    trace(4,"decode_type1045: prn=%d iode=%d toe=%.0f\n",prn,eph->iode,eph->toes);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," prn=%2d iode=%3d week=%d toe=%6.0f toc=%6.0f hs=%d dvs=%d",
                prn,eph->iode,week,eph->toes,toc,e5a_hs,e5a_dvs);
    }
    if (!(sat=satno(sys,prn))) {
        trace(2,"rtcm3 1045 satellite number error: prn=%d\n",prn);
        return -1;
    }
    if (strstr(rtcm->opt,"-GALINAV")) {
        return 0;
    }
    eph->sat=sat;
    eph->week=week+1024; /* gal-week = gst-week + 1024 */
    if (rtcm->time.time==0) rtcm->time=utc2gpst(timeget());
    tt=timediff(gpst2time(eph->week,eph->toes),rtcm->time);
    if      (tt<-302400.0) eph->week++;
    else if (tt>=302400.0) eph->week--;
    eph->toe=gpst2time(eph->week,eph->toes);
    eph->toc=gpst2time(eph->week,toc);
    eph->ttr=rtcm->time;
    eph->A=sqrtA*sqrtA;
    eph->svh=(e5a_hs<<4)+(e5a_dvs<<3);
    eph->code=(1<<1)+(1<<8); /* data source = F/NAV+E5a */
    eph->iodc=eph->iode;
    return 2;
}
/* decode type 1046: Galileo I/NAV satellite ephemerides ---------------------*/
static int decode_type1046(rtcm_t *rtcm, eph_t *eph)
{
    double toc,sqrtA,tt;
    char *msg;
    int i=24+12,prn,sat,week,e5b_hs,e5b_dvs,e1_hs,e1_dvs,sys=SYS_GAL;
    
    if (strstr(rtcm->opt,"-GALFNAV")) return 0;

    if (i+492<=rtcm->len*8) {
        prn        =getbitu(rtcm->buff,i, 6);              i+= 6;
        week       =getbitu(rtcm->buff,i,12);              i+=12;
        eph->iode  =getbitu(rtcm->buff,i,10);              i+=10;
        eph->sva   =getbitu(rtcm->buff,i, 8);              i+= 8;
        eph->idot  =getbits(rtcm->buff,i,14)*P2_43*SC2RAD; i+=14;
        toc        =getbitu(rtcm->buff,i,14)*60.0;         i+=14;
        eph->f2    =getbits(rtcm->buff,i, 6)*P2_59;        i+= 6;
        eph->f1    =getbits(rtcm->buff,i,21)*P2_46;        i+=21;
        eph->f0    =getbits(rtcm->buff,i,31)*P2_34;        i+=31;
        eph->crs   =getbits(rtcm->buff,i,16)*P2_5;         i+=16;
        eph->deln  =getbits(rtcm->buff,i,16)*P2_43*SC2RAD; i+=16;
        eph->M0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cuc   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph->e     =getbitu(rtcm->buff,i,32)*P2_33;        i+=32;
        eph->cus   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        sqrtA      =getbitu(rtcm->buff,i,32)*P2_19;        i+=32;
        eph->toes  =getbitu(rtcm->buff,i,14)*60.0;         i+=14;
        eph->cic   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph->OMG0  =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cis   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph->i0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->crc   =getbits(rtcm->buff,i,16)*P2_5;         i+=16;
        eph->omg   =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->OMGd  =getbits(rtcm->buff,i,24)*P2_43*SC2RAD; i+=24;
        eph->tgd[0]=getbits(rtcm->buff,i,10)*P2_32;        i+=10; /* E5a/E1 */
        eph->tgd[1]=getbits(rtcm->buff,i,10)*P2_32;        i+=10; /* E5b/E1 */
        e5b_hs     =getbitu(rtcm->buff,i, 2);              i+= 2; /* E5b OSHS */
        e5b_dvs    =getbitu(rtcm->buff,i, 1);              i+= 1; /* E5b OSDVS */
        e1_hs      =getbitu(rtcm->buff,i, 2);              i+= 2; /* E1 OSHS */
        e1_dvs     =getbitu(rtcm->buff,i, 1);              i+= 1; /* E1 OSDVS */
    }
    else {
        trace(2,"rtcm3 1046 length error: len=%d\n",rtcm->len);
        return -1;
    }
    trace(4,"decode_type1046: prn=%d iode=%d toe=%.0f\n",prn,eph->iode,eph->toes);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," prn=%2d iode=%3d week=%d toe=%6.0f toc=%6.0f hs=%d %d dvs=%d %d",
                prn,eph->iode,week,eph->toes,toc,e5b_hs,e1_hs,e5b_dvs,e1_dvs);
    }
    if (!(sat=satno(sys,prn))) {
        trace(2,"rtcm3 1046 satellite number error: prn=%d\n",prn);
        return -1;
    }
    if (strstr(rtcm->opt,"-GALFNAV")) {
        return 0;
    }
    eph->sat=sat;
    eph->week=week+1024; /* gal-week = gst-week + 1024 */
    if (rtcm->time.time==0) rtcm->time=utc2gpst(timeget());
    tt=timediff(gpst2time(eph->week,eph->toes),rtcm->time);
    if      (tt<-302400.0) eph->week++;
    else if (tt>=302400.0) eph->week--;
    eph->toe=gpst2time(eph->week,eph->toes);
    eph->toc=gpst2time(eph->week,toc);
    eph->ttr=rtcm->time;
    eph->A=sqrtA*sqrtA;
    eph->svh=(e5b_hs<<7)+(e5b_dvs<<6)+(e1_hs<<1)+(e1_dvs<<0);
    eph->code=(1<<0)+(1<<2)+(1<<9); /* data source = I/NAV+E1+E5b */
    eph->iodc=eph->iode;
    return 2;
}
/* decode type 1042/63: Beidou ephemerides -----------------------------------*/
static int decode_type1042(rtcm_t *rtcm, eph_t *eph)
{
    double toc,sqrtA,tt;
    char *msg;
    int i=24+12,prn,sat,week,sys=SYS_CMP;
    
    if (i+499<=rtcm->len*8) {
        prn        =getbitu(rtcm->buff,i, 6);              i+= 6;
        week       =getbitu(rtcm->buff,i,13);              i+=13;
        eph->sva   =getbitu(rtcm->buff,i, 4);              i+= 4;
        eph->idot  =getbits(rtcm->buff,i,14)*P2_43*SC2RAD; i+=14;
        eph->iode  =getbitu(rtcm->buff,i, 5);              i+= 5; /* AODE */
        toc        =getbitu(rtcm->buff,i,17)*8.0;          i+=17;
        eph->f2    =getbits(rtcm->buff,i,11)*P2_66;        i+=11;
        eph->f1    =getbits(rtcm->buff,i,22)*P2_50;        i+=22;
        eph->f0    =getbits(rtcm->buff,i,24)*P2_33;        i+=24;
        eph->iodc  =getbitu(rtcm->buff,i, 5);              i+= 5; /* AODC */
        eph->crs   =getbits(rtcm->buff,i,18)*P2_6;         i+=18;
        eph->deln  =getbits(rtcm->buff,i,16)*P2_43*SC2RAD; i+=16;
        eph->M0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cuc   =getbits(rtcm->buff,i,18)*P2_31;        i+=18;
        eph->e     =getbitu(rtcm->buff,i,32)*P2_33;        i+=32;
        eph->cus   =getbits(rtcm->buff,i,18)*P2_31;        i+=18;
        sqrtA      =getbitu(rtcm->buff,i,32)*P2_19;        i+=32;
        eph->toes  =getbitu(rtcm->buff,i,17)*8.0;          i+=17;
        eph->cic   =getbits(rtcm->buff,i,18)*P2_31;        i+=18;
        eph->OMG0  =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cis   =getbits(rtcm->buff,i,18)*P2_31;        i+=18;
        eph->i0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->crc   =getbits(rtcm->buff,i,18)*P2_6;         i+=18;
        eph->omg   =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->OMGd  =getbits(rtcm->buff,i,24)*P2_43*SC2RAD; i+=24;
        eph->tgd[0]=getbits(rtcm->buff,i,10)*1E-10;        i+=10;
        eph->tgd[1]=getbits(rtcm->buff,i,10)*1E-10;        i+=10;
        eph->svh   =getbitu(rtcm->buff,i, 1);              i+= 1;
    }
    else {
        trace(2,"rtcm3 1042 length error: len=%d\n",rtcm->len);
        return -1;
    }
    trace(4,"decode_type1042: prn=%d iode=%d toe=%.0f\n",prn,eph->iode,eph->toes);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," prn=%2d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
                prn,eph->iode,eph->iodc,week,eph->toes,toc,eph->svh);
    }
    if (!(sat=satno(sys,prn))) {
        trace(2,"rtcm3 1042 satellite number error: prn=%d\n",prn);
        return -1;
    }
    eph->sat=sat;
    eph->week=adjbdtweek(week);
    if (rtcm->time.time==0) rtcm->time=utc2gpst(timeget());
    tt=timediff(bdt2gpst(bdt2time(eph->week,eph->toes)),rtcm->time);
    if      (tt<-302400.0) eph->week++;
    else if (tt>=302400.0) eph->week--;
    eph->toe=bdt2gpst(bdt2time(eph->week,eph->toes)); /* bdt -> gpst */
    eph->toc=bdt2gpst(bdt2time(eph->week,toc));      /* bdt -> gpst */
    eph->ttr=rtcm->time;
    eph->A=sqrtA*sqrtA;
    return 2;
}
/* decode SSR message epoch time ---------------------------------------------*/
static int decode_ssr_epoch(rtcm_t *rtcm, int sys, int subtype)
{
    double tod,tow;
    int i=24+12;
    
    if (subtype==0) { /* RTCM SSR */
        
        if (sys==SYS_GLO) {
            tod=getbitu(rtcm->buff,i,17); i+=17;
            adjday_glot(rtcm,tod);
        }
        else {
            tow=getbitu(rtcm->buff,i,20); i+=20;
            adjweek(rtcm,tow);
        }
    }
    else { /* IGS SSR */
        i+=3+8;
        tow=getbitu(rtcm->buff,i,20); i+=20;
        adjweek(rtcm,tow);
    }
    return i;
}
/* decode SSR 1,4 message header ---------------------------------------------*/
static int decode_ssr1_head(rtcm_t *rtcm, int sys, int subtype, int *sync,
                            int *iod, double *udint, int *refd, int *hsize)
{
    char *msg,tstr[64];
    int i=24+12,nsat,udi,provid=0,solid=0,ns;
    
    if (subtype==0) { /* RTCM SSR */
        ns=(sys==SYS_QZS)?4:6;
        if (i+((sys==SYS_GLO)?53:50+ns)>rtcm->len*8) return -1;
    }
    else { /* IGS SSR */
        ns=6;
        if (i+3+8+50+ns>rtcm->len*8) return -1;
    }
    i=decode_ssr_epoch(rtcm,sys,subtype);
    udi   =getbitu(rtcm->buff,i, 4); i+= 4;
    *sync =getbitu(rtcm->buff,i, 1); i+= 1;
    if (subtype==0) { /* RTCM SSR */
        *refd=getbitu(rtcm->buff,i,1); i+=1; /* satellite ref datum */
    }
    *iod  =getbitu(rtcm->buff,i, 4); i+= 4; /* IOD SSR */
    provid=getbitu(rtcm->buff,i,16); i+=16; /* provider ID */
    solid =getbitu(rtcm->buff,i, 4); i+= 4; /* solution ID */
    if (subtype>0) { /* IGS SSR */
        *refd=getbitu(rtcm->buff,i,1); i+=1; /* global/regional CRS indicator */
    }
    nsat  =getbitu(rtcm->buff,i,ns); i+=ns;
    *udint=ssrudint[udi];
    
    time2str(rtcm->time,tstr,2);
    trace(4,"decode_ssr1_head: time=%s sys=%d subtype=%d nsat=%d sync=%d iod=%d"
         " provid=%d solid=%d\n",tstr,sys,subtype,nsat,*sync,*iod,provid,solid);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," %s nsat=%2d iod=%2d udi=%2d sync=%d",tstr,nsat,*iod,udi,
                *sync);
    }
    *hsize=i;
    return nsat;
}
/* decode SSR 2,3,5,6 message header -----------------------------------------*/
static int decode_ssr2_head(rtcm_t *rtcm, int sys, int subtype, int *sync,
                            int *iod, double *udint, int *hsize)
{
    char *msg,tstr[64];
    int i=24+12,nsat,udi,provid=0,solid=0,ns;
    
    if (subtype==0) { /* RTCM SSR */
        ns=(sys==SYS_QZS)?4:6;
        if (i+((sys==SYS_GLO)?52:49+ns)>rtcm->len*8) return -1;
    }
    else {
        ns=6;
        if (i+3+8+49+ns>rtcm->len*8) return -1;
    }
    i=decode_ssr_epoch(rtcm,sys,subtype);
    udi   =getbitu(rtcm->buff,i, 4); i+= 4;
    *sync =getbitu(rtcm->buff,i, 1); i+= 1;
    *iod  =getbitu(rtcm->buff,i, 4); i+= 4;
    provid=getbitu(rtcm->buff,i,16); i+=16; /* provider ID */
    solid =getbitu(rtcm->buff,i, 4); i+= 4; /* solution ID */
    nsat  =getbitu(rtcm->buff,i,ns); i+=ns;
    *udint=ssrudint[udi];
    
    time2str(rtcm->time,tstr,2);
    trace(4,"decode_ssr2_head: time=%s sys=%d subtype=%d nsat=%d sync=%d iod=%d"
         " provid=%d solid=%d\n",tstr,sys,subtype,nsat,*sync,*iod,provid,solid);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," %s nsat=%2d iod=%2d udi=%2d sync=%d",tstr,nsat,*iod,udi,
                *sync);
    }
    *hsize=i;
    return nsat;
}
/* decode SSR 1: orbit corrections -------------------------------------------*/
static int decode_ssr1(rtcm_t *rtcm, nav_t *nav, int sys, int subtype)
{
    double udint,deph[3],ddeph[3];
    int i,j,k,type,sync,iod,nsat,prn,sat,iode,iodcrc=0,refd=0,np,ni,nj,offp;
    
    type=getbitu(rtcm->buff,24,12);
    
    if ((nsat=decode_ssr1_head(rtcm,sys,subtype,&sync,&iod,&udint,&refd,&i))<0) {
        trace(2,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
    switch (sys) {
        case SYS_GPS: np=6; ni= 8; nj= 0; offp=  0; break;
        case SYS_GLO: np=5; ni= 8; nj= 0; offp=  0; break;
        case SYS_GAL: np=6; ni=10; nj= 0; offp=  0; break;
        case SYS_QZS: np=4; ni= 8; nj= 0; offp=192; break;
        case SYS_CMP: np=6; ni=10; nj=24; offp=  1; break;
        case SYS_SBS: np=6; ni= 9; nj=24; offp=120; break;
        default: return sync?0:10;
    }
    if (subtype>0) { /* IGS SSR */
        np=6; ni=8; nj=0;
        if      (sys==SYS_CMP) offp=0;
        else if (sys==SYS_SBS) offp=119;
    }
    for (j=0;j<nsat&&i+121+np+ni+nj<=rtcm->len*8;j++) {
        prn     =getbitu(rtcm->buff,i,np)+offp; i+=np;
        iode    =getbitu(rtcm->buff,i,ni);      i+=ni;
        iodcrc  =getbitu(rtcm->buff,i,nj);      i+=nj;
        deph [0]=getbits(rtcm->buff,i,22)*1E-4; i+=22;
        deph [1]=getbits(rtcm->buff,i,20)*4E-4; i+=20;
        deph [2]=getbits(rtcm->buff,i,20)*4E-4; i+=20;
        ddeph[0]=getbits(rtcm->buff,i,21)*1E-6; i+=21;
        ddeph[1]=getbits(rtcm->buff,i,19)*4E-6; i+=19;
        ddeph[2]=getbits(rtcm->buff,i,19)*4E-6; i+=19;
        
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 %d satellite number error: prn=%d\n",type,prn);
            continue;
        }
        nav->ssr[sat-1].t0 [0]=rtcm->time;
        nav->ssr[sat-1].udi[0]=udint;
        nav->ssr[sat-1].iod[0]=iod;
        nav->ssr[sat-1].iode=iode;     /* SBAS/BDS: toe/t0 modulo */
        nav->ssr[sat-1].iodcrc=iodcrc; /* SBAS/BDS: IOD CRC */
        nav->ssr[sat-1].refd=refd;
        
        for (k=0;k<3;k++) {
            nav->ssr[sat-1].deph [k]=deph [k];
            nav->ssr[sat-1].ddeph[k]=ddeph[k];
        }
        nav->ssr[sat-1].update=1;
    }
    return sync?0:10;
}
/* decode SSR 2: clock corrections -------------------------------------------*/
static int decode_ssr2(rtcm_t *rtcm, nav_t *nav, int sys, int subtype)
{
    double udint,dclk[3];
    int i,j,k,type,sync,iod,nsat,prn,sat,np,offp;
    
    type=getbitu(rtcm->buff,24,12);
    
    if ((nsat=decode_ssr2_head(rtcm,sys,subtype,&sync,&iod,&udint,&i))<0) {
        trace(2,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
    switch (sys) {
        case SYS_GPS: np=6; offp=  0; break;
        case SYS_GLO: np=5; offp=  0; break;
        case SYS_GAL: np=6; offp=  0; break;
        case SYS_QZS: np=4; offp=192; break;
        case SYS_CMP: np=6; offp=  1; break;
        case SYS_SBS: np=6; offp=120; break;
        default: return sync?0:10;
    }
    if (subtype>0) { /* IGS SSR */
        np=6;
        if      (sys==SYS_CMP) offp=0;
        else if (sys==SYS_SBS) offp=119;
    }
    for (j=0;j<nsat&&i+70+np<=rtcm->len*8;j++) {
        prn    =getbitu(rtcm->buff,i,np)+offp; i+=np;
        dclk[0]=getbits(rtcm->buff,i,22)*1E-4; i+=22;
        dclk[1]=getbits(rtcm->buff,i,21)*1E-6; i+=21;
        dclk[2]=getbits(rtcm->buff,i,27)*2E-8; i+=27;
        
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 %d satellite number error: prn=%d\n",type,prn);
            continue;
        }
        nav->ssr[sat-1].t0 [1]=rtcm->time;
        nav->ssr[sat-1].udi[1]=udint;
        nav->ssr[sat-1].iod[1]=iod;
        
        for (k=0;k<3;k++) {
            nav->ssr[sat-1].dclk[k]=dclk[k];
        }
        nav->ssr[sat-1].update=1;
    }
    return sync?0:10;
}
/* decode SSR 3: satellite code biases ---------------------------------------*/
static int decode_ssr3(rtcm_t *rtcm, nav_t* nav, int sys, int subtype)
{
    const uint8_t *sigs;
    double udint,bias,cbias[MAXCODE];
    int i,j,k,type,mode,sync,iod,nsat,prn,sat,nbias,np,offp;
    
    type=getbitu(rtcm->buff,24,12);
    
    if ((nsat=decode_ssr2_head(rtcm,sys,subtype,&sync,&iod,&udint,&i))<0) {
        trace(2,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
    switch (sys) {
        case SYS_GPS: np=6; offp=  0; sigs=ssr_sig_gps; break;
        case SYS_GLO: np=5; offp=  0; sigs=ssr_sig_glo; break;
        case SYS_GAL: np=6; offp=  0; sigs=ssr_sig_gal; break;
        case SYS_QZS: np=4; offp=192; sigs=ssr_sig_qzs; break;
        case SYS_CMP: np=6; offp=  1; sigs=ssr_sig_cmp; break;
        case SYS_SBS: np=6; offp=120; sigs=ssr_sig_sbs; break;
        default: return sync?0:10;
    }
    if (subtype>0) { /* IGS SSR */
        np=6;
        if      (sys==SYS_CMP) offp=0;
        else if (sys==SYS_SBS) offp=119;
    }
    for (j=0;j<nsat&&i+5+np<=rtcm->len*8;j++) {
        prn  =getbitu(rtcm->buff,i,np)+offp; i+=np;
        nbias=getbitu(rtcm->buff,i, 5);      i+= 5;
        
        for (k=0;k<MAXCODE;k++) cbias[k]=0.0;
        for (k=0;k<nbias&&i+19<=rtcm->len*8;k++) {
            mode=getbitu(rtcm->buff,i, 5);      i+= 5;
            bias=getbits(rtcm->buff,i,14)*0.01; i+=14;
            if (sigs[mode]) {
                cbias[sigs[mode]-1]=(float)bias;
            }
            else {
                trace(2,"rtcm3 %d not supported mode: mode=%d\n",type,mode);
            }
        }
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 %d satellite number error: prn=%d\n",type,prn);
            continue;
        }
        nav->ssr[sat-1].t0 [4]=rtcm->time;
        nav->ssr[sat-1].udi[4]=udint;
        nav->ssr[sat-1].iod[4]=iod;
        
        for (k=0;k<MAXCODE;k++) {
            nav->ssr[sat-1].cbias[k]=(float)cbias[k];
        }
        nav->ssr[sat-1].update=1;
    }
    return sync?0:10;
}
/* decode SSR 4: combined orbit and clock corrections ------------------------*/
static int decode_ssr4(rtcm_t *rtcm, nav_t* nav, int sys, int subtype)
{
    double udint,deph[3],ddeph[3],dclk[3];
    int i,j,k,type,nsat,sync,iod,prn,sat,iode,iodcrc=0,refd=0,np,ni,nj,offp;
    
    type=getbitu(rtcm->buff,24,12);
    
    if ((nsat=decode_ssr1_head(rtcm,sys,subtype,&sync,&iod,&udint,&refd,&i))<0) {
        trace(2,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
    switch (sys) {
        case SYS_GPS: np=6; ni= 8; nj= 0; offp=  0; break;
        case SYS_GLO: np=5; ni= 8; nj= 0; offp=  0; break;
        case SYS_GAL: np=6; ni=10; nj= 0; offp=  0; break;
        case SYS_QZS: np=4; ni= 8; nj= 0; offp=192; break;
        case SYS_CMP: np=6; ni=10; nj=24; offp=  1; break;
        case SYS_SBS: np=6; ni= 9; nj=24; offp=120; break;
        default: return sync?0:10;
    }
    if (subtype>0) { /* IGS SSR */
        np=6; ni=8; nj=0;
        if      (sys==SYS_CMP) offp=0;
        else if (sys==SYS_SBS) offp=119;
    }
    for (j=0;j<nsat&&i+191+np+ni+nj<=rtcm->len*8;j++) {
        prn     =getbitu(rtcm->buff,i,np)+offp; i+=np;
        iode    =getbitu(rtcm->buff,i,ni);      i+=ni;
        iodcrc  =getbitu(rtcm->buff,i,nj);      i+=nj;
        deph [0]=getbits(rtcm->buff,i,22)*1E-4; i+=22;
        deph [1]=getbits(rtcm->buff,i,20)*4E-4; i+=20;
        deph [2]=getbits(rtcm->buff,i,20)*4E-4; i+=20;
        ddeph[0]=getbits(rtcm->buff,i,21)*1E-6; i+=21;
        ddeph[1]=getbits(rtcm->buff,i,19)*4E-6; i+=19;
        ddeph[2]=getbits(rtcm->buff,i,19)*4E-6; i+=19;
        
        dclk [0]=getbits(rtcm->buff,i,22)*1E-4; i+=22;
        dclk [1]=getbits(rtcm->buff,i,21)*1E-6; i+=21;
        dclk [2]=getbits(rtcm->buff,i,27)*2E-8; i+=27;
        
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 %d satellite number error: prn=%d\n",type,prn);
            continue;
        }
        nav->ssr[sat-1].t0 [0]= nav->ssr[sat-1].t0 [1]=rtcm->time;
        nav->ssr[sat-1].udi[0]= nav->ssr[sat-1].udi[1]=udint;
        nav->ssr[sat-1].iod[0]= nav->ssr[sat-1].iod[1]=iod;
        nav->ssr[sat-1].iode=iode;
        nav->ssr[sat-1].iodcrc=iodcrc;
        nav->ssr[sat-1].refd=refd;
        
        for (k=0;k<3;k++) {
            nav->ssr[sat-1].deph [k]=deph [k];
            nav->ssr[sat-1].ddeph[k]=ddeph[k];
            nav->ssr[sat-1].dclk [k]=dclk [k];
        }
        nav->ssr[sat-1].update=1;
    }
    return sync?0:10;
}
/* decode SSR 5: URA ---------------------------------------------------------*/
static int decode_ssr5(rtcm_t *rtcm, nav_t* nav, int sys, int subtype)
{
    double udint;
    int i,j,type,nsat,sync,iod,prn,sat,ura,np,offp;
    
    type=getbitu(rtcm->buff,24,12);
    
    if ((nsat=decode_ssr2_head(rtcm,sys,subtype,&sync,&iod,&udint,&i))<0) {
        trace(2,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
    switch (sys) {
        case SYS_GPS: np=6; offp=  0; break;
        case SYS_GLO: np=5; offp=  0; break;
        case SYS_GAL: np=6; offp=  0; break;
        case SYS_QZS: np=4; offp=192; break;
        case SYS_CMP: np=6; offp=  1; break;
        case SYS_SBS: np=6; offp=120; break;
        default: return sync?0:10;
    }
    if (subtype>0) { /* IGS SSR */
        np=6;
        if      (sys==SYS_CMP) offp=0;
        else if (sys==SYS_SBS) offp=119;
    }
    for (j=0;j<nsat&&i+6+np<=rtcm->len*8;j++) {
        prn=getbitu(rtcm->buff,i,np)+offp; i+=np;
        ura=getbitu(rtcm->buff,i, 6);      i+= 6;
        
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 %d satellite number error: prn=%d\n",type,prn);
            continue;
        }
        nav->ssr[sat-1].t0 [3]=rtcm->time;
        nav->ssr[sat-1].udi[3]=udint;
        nav->ssr[sat-1].iod[3]=iod;
        nav->ssr[sat-1].ura=ura;
        nav->ssr[sat-1].update=1;
    }
    return sync?0:10;
}
/* decode SSR 6: high rate clock correction ----------------------------------*/
static int decode_ssr6(rtcm_t *rtcm, nav_t* nav, int sys, int subtype)
{
    double udint,hrclk;
    int i,j,type,nsat,sync,iod,prn,sat,np,offp;
    
    type=getbitu(rtcm->buff,24,12);
    
    if ((nsat=decode_ssr2_head(rtcm,sys,subtype,&sync,&iod,&udint,&i))<0) {
        trace(2,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
    switch (sys) {
        case SYS_GPS: np=6; offp=  0; break;
        case SYS_GLO: np=5; offp=  0; break;
        case SYS_GAL: np=6; offp=  0; break;
        case SYS_QZS: np=4; offp=192; break;
        case SYS_CMP: np=6; offp=  1; break;
        case SYS_SBS: np=6; offp=120; break;
        default: return sync?0:10;
    }
    if (subtype>0) { /* IGS SSR */
        np=6;
        if      (sys==SYS_CMP) offp=0;
        else if (sys==SYS_SBS) offp=119;
    }
    for (j=0;j<nsat&&i+22+np<=rtcm->len*8;j++) {
        prn  =getbitu(rtcm->buff,i,np)+offp; i+=np;
        hrclk=getbits(rtcm->buff,i,22)*1E-4; i+=22;
        
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 %d satellite number error: prn=%d\n",type,prn);
            continue;
        }
        nav->ssr[sat-1].t0 [2]=rtcm->time;
        nav->ssr[sat-1].udi[2]=udint;
        nav->ssr[sat-1].iod[2]=iod;
        nav->ssr[sat-1].hrclk=hrclk;
        nav->ssr[sat-1].update=1;
    }
    return sync?0:10;
}
/* decode SSR 7 message header -----------------------------------------------*/
static int decode_ssr7_head(rtcm_t *rtcm, nav_t* nav, int sys, int subtype, int *sync,
                            int *iod, double *udint, int *dispe, int *mw,
                            int *hsize)
{
    char *msg,tstr[64];
    int i=24+12,nsat,udi,provid=0,solid=0,ns;
    
    if (subtype==0) { /* RTCM SSR */
        ns=(sys==SYS_QZS)?4:6;
        if (i+((sys==SYS_GLO)?54:51+ns)>rtcm->len*8) return -1;
    }
    else { /* IGS SSR */
        ns=6;
        if (i+3+8+51+ns>rtcm->len*8) return -1;
    }
    i=decode_ssr_epoch(rtcm,sys,subtype);
    udi   =getbitu(rtcm->buff,i, 4); i+= 4;
    *sync =getbitu(rtcm->buff,i, 1); i+= 1;
    *iod  =getbitu(rtcm->buff,i, 4); i+= 4;
    provid=getbitu(rtcm->buff,i,16); i+=16; /* provider ID */
    solid =getbitu(rtcm->buff,i, 4); i+= 4; /* solution ID */
    *dispe=getbitu(rtcm->buff,i, 1); i+= 1; /* dispersive bias consistency ind */
    *mw   =getbitu(rtcm->buff,i, 1); i+= 1; /* MW consistency indicator */
    nsat  =getbitu(rtcm->buff,i,ns); i+=ns;
    *udint=ssrudint[udi];
    
    time2str(rtcm->time,tstr,2);
    trace(4,"decode_ssr7_head: time=%s sys=%d subtype=%d nsat=%d sync=%d iod=%d"
          " provid=%d solid=%d\n",tstr,sys,subtype,nsat,*sync,*iod,provid,solid);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," %s nsat=%2d iod=%2d udi=%2d sync=%d",tstr,nsat,*iod,udi,
                *sync);
    }
    *hsize=i;
    return nsat;
}
/* decode SSR 7: phase bias --------------------------------------------------*/
static int decode_ssr7(rtcm_t *rtcm, nav_t* nav, int sys, int subtype)
{
    const uint8_t *sigs;
    double udint,bias,std=0.0,pbias[MAXCODE],stdpb[MAXCODE];
    int i,j,k,type,mode,sync,iod,nsat,prn,sat,nbias,np,mw,offp,sii,swl;
    int dispe,sdc,yaw_ang,yaw_rate;
    
    type=getbitu(rtcm->buff,24,12);
    
    if ((nsat=decode_ssr7_head(rtcm,nav,sys,subtype,&sync,&iod,&udint,&dispe,&mw,
                               &i))<0) {
        trace(2,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
    switch (sys) {
        case SYS_GPS: np=6; offp=  0; sigs=ssr_sig_gps; break;
        case SYS_GLO: np=5; offp=  0; sigs=ssr_sig_glo; break;
        case SYS_GAL: np=6; offp=  0; sigs=ssr_sig_gal; break;
        case SYS_QZS: np=4; offp=192; sigs=ssr_sig_qzs; break;
        case SYS_CMP: np=6; offp=  1; sigs=ssr_sig_cmp; break;
        default: return sync?0:10;
    }
    if (subtype>0) { /* IGS SSR */
        np=6;
        if      (sys==SYS_CMP) offp=0;
        else if (sys==SYS_SBS) offp=119;
    }
    for (j=0;j<nsat&&i+5+17+np<=rtcm->len*8;j++) {
        prn     =getbitu(rtcm->buff,i,np)+offp; i+=np;
        nbias   =getbitu(rtcm->buff,i, 5);      i+= 5;
        yaw_ang =getbitu(rtcm->buff,i, 9);      i+= 9;
        yaw_rate=getbits(rtcm->buff,i, 8);      i+= 8;
        
        for (k=0;k<MAXCODE;k++) pbias[k]=stdpb[k]=0.0;
        for (k=0;k<nbias&&i+((subtype==0)?49:32)<=rtcm->len*8;k++) {
            mode=getbitu(rtcm->buff,i, 5); i+= 5;
            sii =getbitu(rtcm->buff,i, 1); i+= 1; /* integer-indicator */
            swl =getbitu(rtcm->buff,i, 2); i+= 2; /* WL integer-indicator */
            sdc =getbitu(rtcm->buff,i, 4); i+= 4; /* discontinuity counter */
            bias=getbits(rtcm->buff,i,20); i+=20; /* phase bias (m) */
            if (subtype==0) {
                std=getbitu(rtcm->buff,i,17); i+=17; /* phase bias std-dev (m) */
            }
            if (sigs[mode]) {
                pbias[sigs[mode]-1]=bias*0.0001; /* (m) */
                stdpb[sigs[mode]-1]=std *0.0001; /* (m) */
            }
            else {
                trace(2,"rtcm3 %d not supported mode: mode=%d\n",type,mode);
            }
        }
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 %d satellite number error: prn=%d\n",type,prn);
            continue;
        }
        nav->ssr[sat-1].t0 [5]=rtcm->time;
        nav->ssr[sat-1].udi[5]=udint;
        nav->ssr[sat-1].iod[5]=iod;
        nav->ssr[sat-1].yaw_ang =yaw_ang / 256.0*180.0; /* (deg) */
        nav->ssr[sat-1].yaw_rate=yaw_rate/8192.0*180.0; /* (deg/s) */
        
        for (k=0;k<MAXCODE;k++) {
            nav->ssr[sat-1].pbias[k]=pbias[k];
            nav->ssr[sat-1].stdpb[k]=(float)stdpb[k];
        }
    }
    return 20;
}
/* get signal index ----------------------------------------------------------*/
static void sigindex(int sys, const uint8_t *code, int n, const char *opt,
                     int *idx)
{
    int i,nex,pri,pri_h[8]={0},index[8]={0},ex[32]={0};
    
    /* test code priority */
    for (i=0;i<n;i++) {
        if (!code[i]) continue;
        
        if (idx[i]>=NFREQ) { /* save as extended signal if idx >= NFREQ */
            ex[i]=1;
            continue;
        }
        /* code priority */
        pri=getcodepri(sys,code[i],opt);
        
        /* select highest priority signal */
        if (pri>pri_h[idx[i]]) {
            if (index[idx[i]]) ex[index[idx[i]]-1]=1;
            pri_h[idx[i]]=pri;
            index[idx[i]]=i+1;
        }
        else ex[i]=1;
    }
    /* signal index in obs data */
    for (i=nex=0;i<n;i++) {
        if (ex[i]==0) ;
        else if (nex<NEXOBS) idx[i]=NFREQ+nex++;
        else { /* no space in obs data */
            trace(2,"rtcm msm: no space in obs data sys=%d code=%d\n",sys,code[i]);
            idx[i]=-1;
        }
#if 0 /* for debug */
        trace(2,"sig pos: sys=%d code=%d ex=%d idx=%d\n",sys,code[i],ex[i],idx[i]);
#endif
    }
}
/* save obs data in MSM message ----------------------------------------------*/
static void save_msm_obs(rtcm_t *rtcm, nav_t *nav, int sys, msm_h_t *h, const double *r,
                         const double *pr, const double *cp, const double *rr,
                         const double *rrf, const double *cnr, const int *lock,
                         const int *ex, const int *half)
{
    const char *sig[32];
    double tt,freq;
    uint8_t code[32];
    char *msm_type="",*q=NULL;
    int i,j,k,type,prn,sat,fcn,index=0,idx[32];
    
    type=getbitu(rtcm->buff,24,12);
    
    switch (sys) {
        case SYS_GPS: msm_type=q=rtcm->msmtype[0]; break;
        case SYS_GLO: msm_type=q=rtcm->msmtype[1]; break;
        case SYS_GAL: msm_type=q=rtcm->msmtype[2]; break;
        case SYS_QZS: msm_type=q=rtcm->msmtype[3]; break;
        case SYS_SBS: msm_type=q=rtcm->msmtype[4]; break;
        case SYS_CMP: msm_type=q=rtcm->msmtype[5]; break;
        case SYS_IRN: msm_type=q=rtcm->msmtype[6]; break;
    }
    /* id to signal */
    for (i=0;i<h->nsig;i++) {
        switch (sys) {
            case SYS_GPS: sig[i]=msm_sig_gps[h->sigs[i]-1]; break;
            case SYS_GLO: sig[i]=msm_sig_glo[h->sigs[i]-1]; break;
            case SYS_GAL: sig[i]=msm_sig_gal[h->sigs[i]-1]; break;
            case SYS_QZS: sig[i]=msm_sig_qzs[h->sigs[i]-1]; break;
            case SYS_SBS: sig[i]=msm_sig_sbs[h->sigs[i]-1]; break;
            case SYS_CMP: sig[i]=msm_sig_cmp[h->sigs[i]-1]; break;
            case SYS_IRN: sig[i]=msm_sig_irn[h->sigs[i]-1]; break;
            default: sig[i]=""; break;
        }
        /* signal to rinex obs type */
        code[i]=obs2code(sig[i]);
        idx[i]=code2idx(sys,code[i]);
        
        if (code[i]!=CODE_NONE) {
            if (q) q+=sprintf(q,"L%s%s",sig[i],i<h->nsig-1?",":"");
        }
        else {
            if (q) q+=sprintf(q,"(%d)%s",h->sigs[i],i<h->nsig-1?",":"");
            
            trace(2,"rtcm3 %d: unknown signal id=%2d\n",type,h->sigs[i]);
        }
    }
    trace(3,"rtcm3 %d: signals=%s\n",type,msm_type);
    
    /* get signal index */
    sigindex(sys,code,h->nsig,rtcm->opt,idx);
    
    for (i=j=0;i<h->nsat;i++) {
        
        prn=h->sats[i];
        if      (sys==SYS_QZS) prn+=MINPRNQZS-1;
        else if (sys==SYS_SBS) prn+=MINPRNSBS-1;
        
        if ((sat=satno(sys,prn))) {
            tt=timediff(rtcm->obs.data[0].time,rtcm->time);
            if (rtcm->obsflag||fabs(tt)>1E-9) {
                rtcm->obs.n=rtcm->obsflag=0;
            }
            index=obsindex(&rtcm->obs,rtcm->time,sat);
        }
        else {
            trace(2,"rtcm3 %d satellite error: prn=%d\n",type,prn);
        }
        fcn=0;
        if (sys==SYS_GLO) {
            fcn=-8; /* no glonass fcn info */
            if (ex&&ex[i]<=13) {
                fcn=ex[i]-7;
                if (!nav->glo_fcn[prn-1]) {
                    nav->glo_fcn[prn-1]=fcn+8; /* fcn+8 */
                }
            }
            else if (nav->geph[prn-1].sat==sat) {
                fcn= nav->geph[prn-1].frq;
            }
            else if (nav->glo_fcn[prn-1]>0) {
                fcn= nav->glo_fcn[prn-1]-8;
            }
            else if ((fcn=get_default_glo_frq(prn))==NULL_GLO_FRQ)
                fcn=-8;
            else
                fcn=fcn;
        }
        for (k=0;k<h->nsig;k++) {
            if (!h->cellmask[k+i*h->nsig]) continue;
            
            if (sat&&index>=0&&idx[k]>=0) {
                freq=fcn<-7?0.0:code2freq(sys,code[k],fcn);
                
                /* pseudorange (m) */
                if (r[i]!=0.0&&pr[j]>-1E12) {
                    rtcm->obs.data[index].P[idx[k]]=r[i]+pr[j];
                }
                /* carrier-phase (cycle) */
                if (r[i]!=0.0&&cp[j]>-1E12) {
                    rtcm->obs.data[index].L[idx[k]]=(r[i]+cp[j])*freq/CLIGHT;
                }
                /* doppler (hz) */
                if (rr&&rrf&&rrf[j]>-1E12) {
                    rtcm->obs.data[index].D[idx[k]]=
                        (float)(-(rr[i]+rrf[j])*freq/CLIGHT);
                }
                rtcm->obs.data[index].LLI[idx[k]]=
                    lossoflock(rtcm,sat,idx[k],lock[j])+(half[j]?3:0);
                rtcm->obs.data[index].SNR [idx[k]]=(uint16_t)(cnr[j]/SNR_UNIT+0.5);
                rtcm->obs.data[index].code[idx[k]]=code[k];
            }
            j++;
        }
    }
}
/* decode type MSM message header --------------------------------------------*/
static int decode_msm_head(rtcm_t *rtcm, int sys, int *sync, int *iod,
                           msm_h_t *h, int *hsize)
{
    msm_h_t h0={0};
    double tow,tod;
    char *msg,tstr[64];
    int i=24,j,dow,mask,staid,type,ncell=0;
    
    type=getbitu(rtcm->buff,i,12); i+=12;
    
    *h=h0;
    if (i+157<=rtcm->len*8) {
        staid     =getbitu(rtcm->buff,i,12);       i+=12;
        
        if (sys==SYS_GLO) {
            dow   =getbitu(rtcm->buff,i, 3);       i+= 3;
            tod   =getbitu(rtcm->buff,i,27)*0.001; i+=27;
            adjday_glot(rtcm,tod);
        }
        else if (sys==SYS_CMP) {
            tow   =getbitu(rtcm->buff,i,30)*0.001; i+=30;
            tow+=14.0; /* BDT -> GPST */
            adjweek(rtcm,tow);
        }
        else {
            tow   =getbitu(rtcm->buff,i,30)*0.001; i+=30;
            adjweek(rtcm,tow);
        }
        *sync     =getbitu(rtcm->buff,i, 1);       i+= 1;
        *iod      =getbitu(rtcm->buff,i, 3);       i+= 3;
        h->time_s =getbitu(rtcm->buff,i, 7);       i+= 7;
        h->clk_str=getbitu(rtcm->buff,i, 2);       i+= 2;
        h->clk_ext=getbitu(rtcm->buff,i, 2);       i+= 2;
        h->smooth =getbitu(rtcm->buff,i, 1);       i+= 1;
        h->tint_s =getbitu(rtcm->buff,i, 3);       i+= 3;
        for (j=1;j<=64;j++) {
            mask=getbitu(rtcm->buff,i,1); i+=1;
            if (mask) h->sats[h->nsat++]=j;
        }
        for (j=1;j<=32;j++) {
            mask=getbitu(rtcm->buff,i,1); i+=1;
            if (mask) h->sigs[h->nsig++]=j;
        }
    }
    else {
        trace(2,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
    /* test station id */
    if (!test_staid(rtcm,staid)) return -1;
    
    if (h->nsat*h->nsig>64) {
        trace(2,"rtcm3 %d number of sats and sigs error: nsat=%d nsig=%d\n",
              type,h->nsat,h->nsig);
        return -1;
    }
    if (i+h->nsat*h->nsig>rtcm->len*8) {
        trace(2,"rtcm3 %d length error: len=%d nsat=%d nsig=%d\n",type,
              rtcm->len,h->nsat,h->nsig);
        return -1;
    }
    for (j=0;j<h->nsat*h->nsig;j++) {
        h->cellmask[j]=getbitu(rtcm->buff,i,1); i+=1;
        if (h->cellmask[j]) ncell++;
    }
    *hsize=i;
    
    time2str(rtcm->time,tstr,2);
    trace(4,"decode_head_msm: time=%s sys=%d staid=%d nsat=%d nsig=%d sync=%d iod=%d ncell=%d\n",
          tstr,sys,staid,h->nsat,h->nsig,*sync,*iod,ncell);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," staid=%4d %s nsat=%2d nsig=%2d iod=%2d ncell=%2d sync=%d",
                staid,tstr,h->nsat,h->nsig,*iod,ncell,*sync);
    }
    return ncell;
}
/* decode unsupported MSM message --------------------------------------------*/
static int decode_msm0(rtcm_t *rtcm, nav_t *nav, int sys)
{
    msm_h_t h={0};
    int i,sync,iod;
    if (decode_msm_head(rtcm,sys,&sync,&iod,&h,&i)<0) return -1;
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode MSM 4: full pseudorange and phaserange plus CNR --------------------*/
static int decode_msm4(rtcm_t *rtcm, nav_t *nav, int sys)
{
    msm_h_t h={0};
    double r[64],pr[64],cp[64],cnr[64];
    int i,j,type,sync,iod,ncell,rng,rng_m,prv,cpv,lock[64],half[64];
    
    type=getbitu(rtcm->buff,24,12);
    
    /* decode msm header */
    if ((ncell=decode_msm_head(rtcm,sys,&sync,&iod,&h,&i))<0) return -1;
    
    if (i+h.nsat*18+ncell*48>rtcm->len*8) {
        trace(2,"rtcm3 %d length error: nsat=%d ncell=%d len=%d\n",type,h.nsat,
              ncell,rtcm->len);
        return -1;
    }
    for (j=0;j<h.nsat;j++) r[j]=0.0;
    for (j=0;j<ncell;j++) pr[j]=cp[j]=-1E16;
    
    /* decode satellite data */
    for (j=0;j<h.nsat;j++) { /* range */
        rng  =getbitu(rtcm->buff,i, 8); i+= 8;
        if (rng!=255) r[j]=rng*RANGE_MS;
    }
    for (j=0;j<h.nsat;j++) {
        rng_m=getbitu(rtcm->buff,i,10); i+=10;
        if (r[j]!=0.0) r[j]+=rng_m*P2_10*RANGE_MS;
    }
    /* decode signal data */
    for (j=0;j<ncell;j++) { /* pseudorange */
        prv=getbits(rtcm->buff,i,15); i+=15;
        if (prv!=-16384) pr[j]=prv*P2_24*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* phaserange */
        cpv=getbits(rtcm->buff,i,22); i+=22;
        if (cpv!=-2097152) cp[j]=cpv*P2_29*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* lock time */
        lock[j]=getbitu(rtcm->buff,i,4); i+=4;
    }
    for (j=0;j<ncell;j++) { /* half-cycle ambiguity */
        half[j]=getbitu(rtcm->buff,i,1); i+=1;
    }
    for (j=0;j<ncell;j++) { /* cnr */
        cnr[j]=getbitu(rtcm->buff,i,6)*1.0; i+=6;
    }
    /* save obs data in msm message */
    save_msm_obs(rtcm,nav,sys,&h,r,pr,cp,NULL,NULL,cnr,lock,NULL,half);
    
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode MSM 5: full pseudorange, phaserange, phaserangerate and CNR --------*/
static int decode_msm5(rtcm_t *rtcm, nav_t* nav, int sys)
{
    msm_h_t h={0};
    double r[64],rr[64],pr[64],cp[64],rrf[64],cnr[64];
    int i,j,type,sync,iod,ncell,rng,rng_m,rate,prv,cpv,rrv,lock[64];
    int ex[64],half[64];
    
    type=getbitu(rtcm->buff,24,12);
    
    /* decode msm header */
    if ((ncell=decode_msm_head(rtcm,sys,&sync,&iod,&h,&i))<0) return -1;
    
    if (i+h.nsat*36+ncell*63>rtcm->len*8) {
        trace(2,"rtcm3 %d length error: nsat=%d ncell=%d len=%d\n",type,h.nsat,
              ncell,rtcm->len);
        return -1;
    }
    for (j=0;j<h.nsat;j++) {
        r[j]=rr[j]=0.0; ex[j]=15;
    }
    for (j=0;j<ncell;j++) pr[j]=cp[j]=rrf[j]=-1E16;
    
    /* decode satellite data */
    for (j=0;j<h.nsat;j++) { /* range */
        rng  =getbitu(rtcm->buff,i, 8); i+= 8;
        if (rng!=255) r[j]=rng*RANGE_MS;
    }
    for (j=0;j<h.nsat;j++) { /* extended info */
        ex[j]=getbitu(rtcm->buff,i, 4); i+= 4;
    }
    for (j=0;j<h.nsat;j++) {
        rng_m=getbitu(rtcm->buff,i,10); i+=10;
        if (r[j]!=0.0) r[j]+=rng_m*P2_10*RANGE_MS;
    }
    for (j=0;j<h.nsat;j++) { /* phaserangerate */
        rate =getbits(rtcm->buff,i,14); i+=14;
        if (rate!=-8192) rr[j]=rate*1.0;
    }
    /* decode signal data */
    for (j=0;j<ncell;j++) { /* pseudorange */
        prv=getbits(rtcm->buff,i,15); i+=15;
        if (prv!=-16384) pr[j]=prv*P2_24*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* phaserange */
        cpv=getbits(rtcm->buff,i,22); i+=22;
        if (cpv!=-2097152) cp[j]=cpv*P2_29*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* lock time */
        lock[j]=getbitu(rtcm->buff,i,4); i+=4;
    }
    for (j=0;j<ncell;j++) { /* half-cycle ambiguity */
        half[j]=getbitu(rtcm->buff,i,1); i+=1;
    }
    for (j=0;j<ncell;j++) { /* cnr */
        cnr[j]=getbitu(rtcm->buff,i,6)*1.0; i+=6;
    }
    for (j=0;j<ncell;j++) { /* phaserangerate */
        rrv=getbits(rtcm->buff,i,15); i+=15;
        if (rrv!=-16384) rrf[j]=rrv*0.0001;
    }
    /* save obs data in msm message */
    save_msm_obs(rtcm,nav,sys,&h,r,pr,cp,rr,rrf,cnr,lock,ex,half);
    
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode MSM 6: full pseudorange and phaserange plus CNR (high-res) ---------*/
static int decode_msm6(rtcm_t *rtcm, nav_t* nav, int sys)
{
    msm_h_t h={0};
    double r[64],pr[64],cp[64],cnr[64];
    int i,j,type,sync,iod,ncell,rng,rng_m,prv,cpv,lock[64],half[64];
    
    type=getbitu(rtcm->buff,24,12);
    
    /* decode msm header */
    if ((ncell=decode_msm_head(rtcm,sys,&sync,&iod,&h,&i))<0) return -1;
    
    if (i+h.nsat*18+ncell*65>rtcm->len*8) {
        trace(2,"rtcm3 %d length error: nsat=%d ncell=%d len=%d\n",type,h.nsat,
              ncell,rtcm->len);
        return -1;
    }
    for (j=0;j<h.nsat;j++) r[j]=0.0;
    for (j=0;j<ncell;j++) pr[j]=cp[j]=-1E16;
    
    /* decode satellite data */
    for (j=0;j<h.nsat;j++) { /* range */
        rng  =getbitu(rtcm->buff,i, 8); i+= 8;
        if (rng!=255) r[j]=rng*RANGE_MS;
    }
    for (j=0;j<h.nsat;j++) {
        rng_m=getbitu(rtcm->buff,i,10); i+=10;
        if (r[j]!=0.0) r[j]+=rng_m*P2_10*RANGE_MS;
    }
    /* decode signal data */
    for (j=0;j<ncell;j++) { /* pseudorange */
        prv=getbits(rtcm->buff,i,20); i+=20;
        if (prv!=-524288) pr[j]=prv*P2_29*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* phaserange */
        cpv=getbits(rtcm->buff,i,24); i+=24;
        if (cpv!=-8388608) cp[j]=cpv*P2_31*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* lock time */
        lock[j]=getbitu(rtcm->buff,i,10); i+=10;
    }
    for (j=0;j<ncell;j++) { /* half-cycle ambiguity */
        half[j]=getbitu(rtcm->buff,i,1); i+=1;
    }
    for (j=0;j<ncell;j++) { /* cnr */
        cnr[j]=getbitu(rtcm->buff,i,10)*0.0625; i+=10;
    }
    /* save obs data in msm message */
    save_msm_obs(rtcm,nav,sys,&h,r,pr,cp,NULL,NULL,cnr,lock,NULL,half);
    
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode MSM 7: full pseudorange, phaserange, phaserangerate and CNR (h-res) */
static int decode_msm7(rtcm_t *rtcm, nav_t* nav, int sys)
{
    msm_h_t h={0};
    double r[64],rr[64],pr[64],cp[64],rrf[64],cnr[64];
    int i,j,type,sync,iod,ncell,rng,rng_m,rate,prv,cpv,rrv,lock[64];
    int ex[64],half[64];
    
    type=getbitu(rtcm->buff,24,12);
    
    /* decode msm header */
    if ((ncell=decode_msm_head(rtcm,sys,&sync,&iod,&h,&i))<0) return -1;
    
    if (i+h.nsat*36+ncell*80>rtcm->len*8) {
        trace(2,"rtcm3 %d length error: nsat=%d ncell=%d len=%d\n",type,h.nsat,
              ncell,rtcm->len);
        return -1;
    }
    for (j=0;j<h.nsat;j++) {
        r[j]=rr[j]=0.0; ex[j]=15;
    }
    for (j=0;j<ncell;j++) pr[j]=cp[j]=rrf[j]=-1E16;
    
    /* decode satellite data */
    for (j=0;j<h.nsat;j++) { /* range */
        rng  =getbitu(rtcm->buff,i, 8); i+= 8;
        if (rng!=255) r[j]=rng*RANGE_MS;
    }
    for (j=0;j<h.nsat;j++) { /* extended info */
        ex[j]=getbitu(rtcm->buff,i, 4); i+= 4;
    }
    for (j=0;j<h.nsat;j++) {
        rng_m=getbitu(rtcm->buff,i,10); i+=10;
        if (r[j]!=0.0) r[j]+=rng_m*P2_10*RANGE_MS;
    }
    for (j=0;j<h.nsat;j++) { /* phaserangerate */
        rate =getbits(rtcm->buff,i,14); i+=14;
        if (rate!=-8192) rr[j]=rate*1.0;
    }
    /* decode signal data */
    for (j=0;j<ncell;j++) { /* pseudorange */
        prv=getbits(rtcm->buff,i,20); i+=20;
        if (prv!=-524288) pr[j]=prv*P2_29*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* phaserange */
        cpv=getbits(rtcm->buff,i,24); i+=24;
        if (cpv!=-8388608) cp[j]=cpv*P2_31*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* lock time */
        lock[j]=getbitu(rtcm->buff,i,10); i+=10;
    }
    for (j=0;j<ncell;j++) { /* half-cycle amiguity */
        half[j]=getbitu(rtcm->buff,i,1); i+=1;
    }
    for (j=0;j<ncell;j++) { /* cnr */
        cnr[j]=getbitu(rtcm->buff,i,10)*0.0625; i+=10;
    }
    for (j=0;j<ncell;j++) { /* phaserangerate */
        rrv=getbits(rtcm->buff,i,15); i+=15;
        if (rrv!=-16384) rrf[j]=rrv*0.0001;
    }
    /* save obs data in msm message */
    save_msm_obs(rtcm,nav,sys,&h,r,pr,cp,rr,rrf,cnr,lock,ex,half);
    
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode type 1230: GLONASS L1 and L2 code-phase biases ---------------------*/
static int decode_type1230(rtcm_t *rtcm)
{
    int i=24+12,j,staid,align,mask,bias;
    
    if (i+20>=rtcm->len*8) {
        trace(2,"rtcm3 1230: length error len=%d\n",rtcm->len);
        return -1;
    }
    staid=getbitu(rtcm->buff,i,12); i+=12;
    align=getbitu(rtcm->buff,i, 1); i+= 1+3;
    mask =getbitu(rtcm->buff,i, 4); i+= 4;
    
    if (rtcm->outtype) {
        sprintf(rtcm->msgtype+strlen(rtcm->msgtype),
                " staid=%4d align=%d mask=0x%X",staid,align,mask);
    }
    /* test station ID */
    if (!test_staid(rtcm,staid)) return -1;
    
    rtcm->sta.glo_cp_align=align;
    for (j=0;j<4;j++) {
        rtcm->sta.glo_cp_bias[j]=0.0;
    }
    for (j=0;j<4&&i+16<=rtcm->len*8;j++) {
        if (!(mask&(1<<(3-j)))) continue;
        bias=getbits(rtcm->buff,i,16); i+=16;
        if (bias!=-32768) {
            rtcm->sta.glo_cp_bias[j]=bias*0.02;
        }
    }
    return 5;
}
/* decode type 4073: proprietary message Mitsubishi Electric -----------------*/
static int decode_type4073(rtcm_t *rtcm, nav_t *nav)
{
    int i=24+12,subtype;
    
    subtype=getbitu(rtcm->buff,i,4); i+=4;
    
    if (rtcm->outtype) {
        sprintf(rtcm->msgtype+strlen(rtcm->msgtype)," subtype=%d",subtype);
    }
    trace(2,"rtcm3 4073: unsupported message subtype=%d\n",subtype);
    return 0;
}
/* decode type 4076: proprietary message IGS ---------------------------------*/
static int decode_type4076(rtcm_t *rtcm, nav_t *nav)
{
    int i=24+12,ver,subtype;
    
    if (i+3+8>=rtcm->len*8) {
        trace(2,"rtcm3 4076: length error len=%d\n",rtcm->len);
        return -1;
    }
    ver    =getbitu(rtcm->buff,i,3); i+=3;
    subtype=getbitu(rtcm->buff,i,8); i+=8;
    
    if (rtcm->outtype) {
        sprintf(rtcm->msgtype+strlen(rtcm->msgtype)," ver=%d subtype=%3d",ver,
                subtype);
    }
    switch (subtype) {
        case  21: return decode_ssr1(rtcm,nav,SYS_GPS,subtype);
        case  22: return decode_ssr2(rtcm,nav,SYS_GPS,subtype);
        case  23: return decode_ssr4(rtcm,nav,SYS_GPS,subtype);
        case  24: return decode_ssr6(rtcm,nav,SYS_GPS,subtype);
        case  25: return decode_ssr3(rtcm,nav,SYS_GPS,subtype);
        case  26: return decode_ssr7(rtcm,nav,SYS_GPS,subtype);
        case  27: return decode_ssr5(rtcm,nav,SYS_GPS,subtype);
        case  41: return decode_ssr1(rtcm,nav,SYS_GLO,subtype);
        case  42: return decode_ssr2(rtcm,nav,SYS_GLO,subtype);
        case  43: return decode_ssr4(rtcm,nav,SYS_GLO,subtype);
        case  44: return decode_ssr6(rtcm,nav,SYS_GLO,subtype);
        case  45: return decode_ssr3(rtcm,nav,SYS_GLO,subtype);
        case  46: return decode_ssr7(rtcm,nav,SYS_GLO,subtype);
        case  47: return decode_ssr5(rtcm,nav,SYS_GLO,subtype);
        case  61: return decode_ssr1(rtcm,nav,SYS_GAL,subtype);
        case  62: return decode_ssr2(rtcm,nav,SYS_GAL,subtype);
        case  63: return decode_ssr4(rtcm,nav,SYS_GAL,subtype);
        case  64: return decode_ssr6(rtcm,nav,SYS_GAL,subtype);
        case  65: return decode_ssr3(rtcm,nav,SYS_GAL,subtype);
        case  66: return decode_ssr7(rtcm,nav,SYS_GAL,subtype);
        case  67: return decode_ssr5(rtcm,nav,SYS_GAL,subtype);
        case  81: return decode_ssr1(rtcm,nav,SYS_QZS,subtype);
        case  82: return decode_ssr2(rtcm,nav,SYS_QZS,subtype);
        case  83: return decode_ssr4(rtcm,nav,SYS_QZS,subtype);
        case  84: return decode_ssr6(rtcm,nav,SYS_QZS,subtype);
        case  85: return decode_ssr3(rtcm,nav,SYS_QZS,subtype);
        case  86: return decode_ssr7(rtcm,nav,SYS_QZS,subtype);
        case  87: return decode_ssr5(rtcm,nav,SYS_QZS,subtype);
        case 101: return decode_ssr1(rtcm,nav,SYS_CMP,subtype);
        case 102: return decode_ssr2(rtcm,nav,SYS_CMP,subtype);
        case 103: return decode_ssr4(rtcm,nav,SYS_CMP,subtype);
        case 104: return decode_ssr6(rtcm,nav,SYS_CMP,subtype);
        case 105: return decode_ssr3(rtcm,nav,SYS_CMP,subtype);
        case 106: return decode_ssr7(rtcm,nav,SYS_CMP,subtype);
        case 107: return decode_ssr5(rtcm,nav,SYS_CMP,subtype);
        case 121: return decode_ssr1(rtcm,nav,SYS_SBS,subtype);
        case 122: return decode_ssr2(rtcm,nav,SYS_SBS,subtype);
        case 123: return decode_ssr4(rtcm,nav,SYS_SBS,subtype);
        case 124: return decode_ssr6(rtcm,nav,SYS_SBS,subtype);
        case 125: return decode_ssr3(rtcm,nav,SYS_SBS,subtype);
        case 126: return decode_ssr7(rtcm,nav,SYS_SBS,subtype);
        case 127: return decode_ssr5(rtcm,nav,SYS_SBS,subtype);
    }
    trace(2,"rtcm3 4076: unsupported message subtype=%d\n",subtype);
    return 0;
}
/* decode RTCM ver.3 message -------------------------------------------------*/
extern int decode_rtcm3(rtcm_t *rtcm, nav_t* nav)
{
	eph_t new_eph={0};
	geph_t new_geph={0};
	eph_t *eph=&new_eph;
	geph_t *geph=&new_geph;
    double tow;
    int ret=0,prn=0,type=getbitu(rtcm->buff,24,12),week;
    
    trace(3,"decode_rtcm3: len=%3d type=%d\n",rtcm->len,type);
    
    if (rtcm->outtype) {
        sprintf(rtcm->msgtype,"RTCM %4d (%4d):",type,rtcm->len);
    }
    /* real-time input option */
    if (strstr(rtcm->opt,"-RT_INP")) {
        tow=time2gpst(utc2gpst(timeget()),&week);
        rtcm->time=gpst2time(week,floor(tow));
    }
    switch (type) {
        case 1001: ret=decode_type1001(rtcm); break; /* not supported */
        case 1002: ret=decode_type1002(rtcm); break;
        case 1003: ret=decode_type1003(rtcm); break; /* not supported */
        case 1004: ret=decode_type1004(rtcm); break;
        case 1005: ret=decode_type1005(rtcm); break;
        case 1006: ret=decode_type1006(rtcm); break;
        case 1007: ret=decode_type1007(rtcm); break;
        case 1008: ret=decode_type1008(rtcm); break;
        case 1009: ret=decode_type1009(rtcm); break; /* not supported */
        case 1010: ret=decode_type1010(rtcm,nav); break;
        case 1011: ret=decode_type1011(rtcm); break; /* not supported */
        case 1012: ret=decode_type1012(rtcm,nav); break;
        case 1013: ret=decode_type1013(rtcm); break; /* not supported */
        case 1019: ret=decode_type1019(rtcm,eph); break;
        case 1020: ret=decode_type1020(rtcm,geph); break;
        case 1021: ret=decode_type1021(rtcm); break; /* not supported */
        case 1022: ret=decode_type1022(rtcm); break; /* not supported */
        case 1023: ret=decode_type1023(rtcm); break; /* not supported */
        case 1024: ret=decode_type1024(rtcm); break; /* not supported */
        case 1025: ret=decode_type1025(rtcm); break; /* not supported */
        case 1026: ret=decode_type1026(rtcm); break; /* not supported */
        case 1027: ret=decode_type1027(rtcm); break; /* not supported */
        case 1029: ret=decode_type1029(rtcm); break;
        case 1030: ret=decode_type1030(rtcm); break; /* not supported */
        case 1031: ret=decode_type1031(rtcm); break; /* not supported */
        case 1032: ret=decode_type1032(rtcm); break; /* not supported */
        case 1033: ret=decode_type1033(rtcm); break;
        case 1034: ret=decode_type1034(rtcm); break; /* not supported */
        case 1035: ret=decode_type1035(rtcm); break; /* not supported */
        case 1037: ret=decode_type1037(rtcm); break; /* not supported */
        case 1038: ret=decode_type1038(rtcm); break; /* not supported */
        case 1039: ret=decode_type1039(rtcm); break; /* not supported */
        case 1041: ret=decode_type1041(rtcm,eph); break;
        case 1044: ret=decode_type1044(rtcm,eph); break;
        case 1045: ret=decode_type1045(rtcm,eph); break;
        case 1046: ret=decode_type1046(rtcm,eph); break;
        case   63: ret=decode_type1042(rtcm,eph); break; /* RTCM draft */
        case 1042: ret=decode_type1042(rtcm,eph); break;
        case 1057: ret=decode_ssr1(rtcm,nav,SYS_GPS,0); break;
        case 1058: ret=decode_ssr2(rtcm,nav,SYS_GPS,0); break;
        case 1059: ret=decode_ssr3(rtcm,nav,SYS_GPS,0); break;
        case 1060: ret=decode_ssr4(rtcm,nav,SYS_GPS,0); break;
        case 1061: ret=decode_ssr5(rtcm,nav,SYS_GPS,0); break;
        case 1062: ret=decode_ssr6(rtcm,nav,SYS_GPS,0); break;
        case 1063: ret=decode_ssr1(rtcm,nav,SYS_GLO,0); break;
        case 1064: ret=decode_ssr2(rtcm,nav,SYS_GLO,0); break;
        case 1065: ret=decode_ssr3(rtcm,nav,SYS_GLO,0); break;
        case 1066: ret=decode_ssr4(rtcm,nav,SYS_GLO,0); break;
        case 1067: ret=decode_ssr5(rtcm,nav,SYS_GLO,0); break;
        case 1068: ret=decode_ssr6(rtcm,nav,SYS_GLO,0); break;
        case 1071: ret=decode_msm0(rtcm,nav,SYS_GPS); break; /* not supported */
        case 1072: ret=decode_msm0(rtcm,nav,SYS_GPS); break; /* not supported */
        case 1073: ret=decode_msm0(rtcm,nav,SYS_GPS); break; /* not supported */
        case 1074: ret=decode_msm4(rtcm,nav,SYS_GPS); break;
        case 1075: ret=decode_msm5(rtcm,nav,SYS_GPS); break;
        case 1076: ret=decode_msm6(rtcm,nav,SYS_GPS); break;
        case 1077: ret=decode_msm7(rtcm,nav,SYS_GPS); break;
        case 1081: ret=decode_msm0(rtcm,nav,SYS_GLO); break; /* not supported */
        case 1082: ret=decode_msm0(rtcm,nav,SYS_GLO); break; /* not supported */
        case 1083: ret=decode_msm0(rtcm,nav,SYS_GLO); break; /* not supported */
        case 1084: ret=decode_msm4(rtcm,nav,SYS_GLO); break;
        case 1085: ret=decode_msm5(rtcm,nav,SYS_GLO); break;
        case 1086: ret=decode_msm6(rtcm,nav,SYS_GLO); break;
        case 1087: ret=decode_msm7(rtcm,nav,SYS_GLO); break;
        case 1091: ret=decode_msm0(rtcm,nav,SYS_GAL); break; /* not supported */
        case 1092: ret=decode_msm0(rtcm,nav,SYS_GAL); break; /* not supported */
        case 1093: ret=decode_msm0(rtcm,nav,SYS_GAL); break; /* not supported */
        case 1094: ret=decode_msm4(rtcm,nav,SYS_GAL); break;
        case 1095: ret=decode_msm5(rtcm,nav,SYS_GAL); break;
        case 1096: ret=decode_msm6(rtcm,nav,SYS_GAL); break;
        case 1097: ret=decode_msm7(rtcm,nav,SYS_GAL); break;
        case 1101: ret=decode_msm0(rtcm,nav,SYS_SBS); break; /* not supported */
        case 1102: ret=decode_msm0(rtcm,nav,SYS_SBS); break; /* not supported */
        case 1103: ret=decode_msm0(rtcm,nav,SYS_SBS); break; /* not supported */
        case 1104: ret=decode_msm4(rtcm,nav,SYS_SBS); break;
        case 1105: ret=decode_msm5(rtcm,nav,SYS_SBS); break;
        case 1106: ret=decode_msm6(rtcm,nav,SYS_SBS); break;
        case 1107: ret=decode_msm7(rtcm,nav,SYS_SBS); break;
        case 1111: ret=decode_msm0(rtcm,nav,SYS_QZS); break; /* not supported */
        case 1112: ret=decode_msm0(rtcm,nav,SYS_QZS); break; /* not supported */
        case 1113: ret=decode_msm0(rtcm,nav,SYS_QZS); break; /* not supported */
        case 1114: ret=decode_msm4(rtcm,nav,SYS_QZS); break;
        case 1115: ret=decode_msm5(rtcm,nav,SYS_QZS); break;
        case 1116: ret=decode_msm6(rtcm,nav,SYS_QZS); break;
        case 1117: ret=decode_msm7(rtcm,nav,SYS_QZS); break;
        case 1121: ret=decode_msm0(rtcm,nav,SYS_CMP); break; /* not supported */
        case 1122: ret=decode_msm0(rtcm,nav,SYS_CMP); break; /* not supported */
        case 1123: ret=decode_msm0(rtcm,nav,SYS_CMP); break; /* not supported */
        case 1124: ret=decode_msm4(rtcm,nav,SYS_CMP); break;
        case 1125: ret=decode_msm5(rtcm,nav,SYS_CMP); break;
        case 1126: ret=decode_msm6(rtcm,nav,SYS_CMP); break;
        case 1127: ret=decode_msm7(rtcm,nav,SYS_CMP); break;
        case 1131: ret=decode_msm0(rtcm,nav,SYS_IRN); break; /* not supported */
        case 1132: ret=decode_msm0(rtcm,nav,SYS_IRN); break; /* not supported */
        case 1133: ret=decode_msm0(rtcm,nav,SYS_IRN); break; /* not supported */
        case 1134: ret=decode_msm4(rtcm,nav,SYS_IRN); break;
        case 1135: ret=decode_msm5(rtcm,nav,SYS_IRN); break;
        case 1136: ret=decode_msm6(rtcm,nav,SYS_IRN); break;
        case 1137: ret=decode_msm7(rtcm,nav,SYS_IRN); break;
        case 1230: ret=decode_type1230(rtcm);     break;
        case 1240: ret=decode_ssr1(rtcm,nav,SYS_GAL,0); break; /* draft */
        case 1241: ret=decode_ssr2(rtcm,nav,SYS_GAL,0); break; /* draft */
        case 1242: ret=decode_ssr3(rtcm,nav,SYS_GAL,0); break; /* draft */
        case 1243: ret=decode_ssr4(rtcm,nav,SYS_GAL,0); break; /* draft */
        case 1244: ret=decode_ssr5(rtcm,nav,SYS_GAL,0); break; /* draft */
        case 1245: ret=decode_ssr6(rtcm,nav,SYS_GAL,0); break; /* draft */
        case 1246: ret=decode_ssr1(rtcm,nav,SYS_QZS,0); break; /* draft */
        case 1247: ret=decode_ssr2(rtcm,nav,SYS_QZS,0); break; /* draft */
        case 1248: ret=decode_ssr3(rtcm,nav,SYS_QZS,0); break; /* draft */
        case 1249: ret=decode_ssr4(rtcm,nav,SYS_QZS,0); break; /* draft */
        case 1250: ret=decode_ssr5(rtcm,nav,SYS_QZS,0); break; /* draft */
        case 1251: ret=decode_ssr6(rtcm,nav,SYS_QZS,0); break; /* draft */
        case 1252: ret=decode_ssr1(rtcm,nav,SYS_SBS,0); break; /* draft */
        case 1253: ret=decode_ssr2(rtcm,nav,SYS_SBS,0); break; /* draft */
        case 1254: ret=decode_ssr3(rtcm,nav,SYS_SBS,0); break; /* draft */
        case 1255: ret=decode_ssr4(rtcm,nav,SYS_SBS,0); break; /* draft */
        case 1256: ret=decode_ssr5(rtcm,nav,SYS_SBS,0); break; /* draft */
        case 1257: ret=decode_ssr6(rtcm,nav,SYS_SBS,0); break; /* draft */
        case 1258: ret=decode_ssr1(rtcm,nav,SYS_CMP,0); break; /* draft */
        case 1259: ret=decode_ssr2(rtcm,nav,SYS_CMP,0); break; /* draft */
        case 1260: ret=decode_ssr3(rtcm,nav,SYS_CMP,0); break; /* draft */
        case 1261: ret=decode_ssr4(rtcm,nav,SYS_CMP,0); break; /* draft */
        case 1262: ret=decode_ssr5(rtcm,nav,SYS_CMP,0); break; /* draft */
        case 1263: ret=decode_ssr6(rtcm,nav,SYS_CMP,0); break; /* draft */
        case   11: ret=decode_ssr7(rtcm,nav,SYS_GPS,0); break; /* tentative */
        case   12: ret=decode_ssr7(rtcm,nav,SYS_GAL,0); break; /* tentative */
        case   13: ret=decode_ssr7(rtcm,nav,SYS_QZS,0); break; /* tentative */
        case   14: ret=decode_ssr7(rtcm,nav,SYS_CMP,0); break; /* tentative */
        case 4073: ret=decode_type4073(rtcm,nav); break;
        case 4076: ret=decode_type4076(rtcm,nav); break;
    }
	if (ret==2&&eph->sat>0&&satsys(eph->sat,&prn)!=SYS_GLO)
	{
        if (type==1045)
            nav->eph[prn-1+MAXSAT]=*eph;
        else
		    nav->eph[eph->sat-1]=*eph;
		nav->ephsat=eph->sat;
		nav->ephset=0;
        nav->n=MAX_SAT_EPH;
	}
	if (ret==2&&geph->sat>0&&satsys(geph->sat,&prn)==SYS_GLO)
	{
		nav->geph[prn-1]=*geph;
		nav->ephsat=geph->sat;
		nav->ephset=0;
        nav->ng=MAX_GLO_EPH;
	}
    if (ret>=0) {
        if      (1001<=type&&type<=1299) rtcm->nmsg3[type-1000]++; /*   1-299 */
        else if (4070<=type&&type<=4099) rtcm->nmsg3[type-3770]++; /* 300-329 */
        else rtcm->nmsg3[0]++; /* other */
    }
    return ret;
}

/* from rtcm3e.c */
#if 0
/* SSR update intervals ------------------------------------------------------*/
static const double ssrudint[16]={
    1,2,5,10,15,30,60,120,240,300,600,900,1800,3600,7200,10800
};
#endif
/* set sign-magnitude bits ---------------------------------------------------*/
static void setbitg(uint8_t *buff, int pos, int len, int32_t value)
{
    setbitu(buff,pos,1,value<0?1:0);
    setbitu(buff,pos+1,len-1,value<0?-value:value);
}
/* set signed 38 bit field ---------------------------------------------------*/
static void set38bits(uint8_t *buff, int pos, double value)
{
    int word_h=(int)floor(value/64.0);
    uint32_t word_l=(uint32_t)(value-word_h*64.0);
    setbits(buff,pos  ,32,word_h);
    setbitu(buff,pos+32,6,word_l);
}
/* lock time -----------------------------------------------------------------*/
static int locktime(gtime_t time, gtime_t *lltime, uint8_t LLI)
{
    if (!lltime->time||(LLI&1)) *lltime=time;
    return (int)timediff(time,*lltime);
}
/* lock time in double -------------------------------------------------------*/
static double locktime_d(gtime_t time, gtime_t *lltime, uint8_t LLI)
{
    if (!lltime->time||(LLI&1)) *lltime=time;
    return timediff(time,*lltime);
}
/* GLONASS frequency channel number in RTCM (FCN+7,-1:error) -----------------*/
static int fcn_glo(int sat, nav_t *nav)
{
    int prn;
    
    if (satsys(sat,&prn)!=SYS_GLO) {
        return -1;
    }
    if (nav->geph[prn-1].sat==sat) {
        return nav->geph[prn-1].frq+7;
    }
    if (nav->glo_fcn[prn-1]>0) { /* fcn+8 (0: no data) */
        return nav->glo_fcn[prn-1]-8+7;
    }
    return -1;
}
/* lock time indicator (ref [17] table 3.4-2) --------------------------------*/
static int to_lock(int lock)
{
    if (lock<0  ) return 0;
    if (lock<24 ) return lock;
    if (lock<72 ) return (lock+24  )/2;
    if (lock<168) return (lock+120 )/4;
    if (lock<360) return (lock+408 )/8;
    if (lock<744) return (lock+1176)/16;
    if (lock<937) return (lock+3096)/32;
    return 127;
}
/* MSM lock time indicator (ref [17] table 3.5-74) ---------------------------*/
static int to_msm_lock(double lock)
{
    if (lock<0.032  ) return 0;
    if (lock<0.064  ) return 1;
    if (lock<0.128  ) return 2;
    if (lock<0.256  ) return 3;
    if (lock<0.512  ) return 4;
    if (lock<1.024  ) return 5;
    if (lock<2.048  ) return 6;
    if (lock<4.096  ) return 7;
    if (lock<8.192  ) return 8;
    if (lock<16.384 ) return 9;
    if (lock<32.768 ) return 10;
    if (lock<65.536 ) return 11;
    if (lock<131.072) return 12;
    if (lock<262.144) return 13;
    if (lock<524.288) return 14;
    return 15;
}
/* MSM lock time indicator with extended-resolution (ref [17] table 3.5-76) --*/
static int to_msm_lock_ex(double lock)
{
    int lock_ms = (int)(lock * 1000.0);
    
    if (lock<0.0      ) return 0;
    if (lock<0.064    ) return lock_ms;
    if (lock<0.128    ) return (lock_ms+64       )/2;
    if (lock<0.256    ) return (lock_ms+256      )/4;
    if (lock<0.512    ) return (lock_ms+768      )/8;
    if (lock<1.024    ) return (lock_ms+2048     )/16;
    if (lock<2.048    ) return (lock_ms+5120     )/32;
    if (lock<4.096    ) return (lock_ms+12288    )/64;
    if (lock<8.192    ) return (lock_ms+28672    )/128;
    if (lock<16.384   ) return (lock_ms+65536    )/256;
    if (lock<32.768   ) return (lock_ms+147456   )/512;
    if (lock<65.536   ) return (lock_ms+327680   )/1024;
    if (lock<131.072  ) return (lock_ms+720896   )/2048;
    if (lock<262.144  ) return (lock_ms+1572864  )/4096;
    if (lock<524.288  ) return (lock_ms+3407872  )/8192;
    if (lock<1048.576 ) return (lock_ms+7340032  )/16384;
    if (lock<2097.152 ) return (lock_ms+15728640 )/32768;
    if (lock<4194.304 ) return (lock_ms+33554432 )/65536;
    if (lock<8388.608 ) return (lock_ms+71303168 )/131072;
    if (lock<16777.216) return (lock_ms+150994944)/262144;
    if (lock<33554.432) return (lock_ms+318767104)/524288;
    if (lock<67108.864) return (lock_ms+671088640)/1048576;
    return 704;
}
/* L1 code indicator GPS -----------------------------------------------------*/
static int to_code1_gps(uint8_t code)
{
    switch (code) {
        case CODE_L1C: return 0; /* L1 C/A */
        case CODE_L1P:
        case CODE_L1W:
        case CODE_L1Y:
        case CODE_L1N: return 1; /* L1 P(Y) direct */
    }
    return 0;
}
/* L2 code indicator GPS -----------------------------------------------------*/
static int to_code2_gps(uint8_t code)
{
    switch (code) {
        case CODE_L2C:
        case CODE_L2S:
        case CODE_L2L:
        case CODE_L2X: return 0; /* L2 C/A or L2C */
        case CODE_L2P:
        case CODE_L2Y: return 1; /* L2 P(Y) direct */
        case CODE_L2D: return 2; /* L2 P(Y) cross-correlated */
        case CODE_L2W:
        case CODE_L2N: return 3; /* L2 correlated P/Y */
    }
    return 0;
}
/* L1 code indicator GLONASS -------------------------------------------------*/
static int to_code1_glo(uint8_t code)
{
    switch (code) {
        case CODE_L1C: return 0; /* L1 C/A */
        case CODE_L1P: return 1; /* L1 P */
    }
    return 0;
}
/* L2 code indicator GLONASS -------------------------------------------------*/
static int to_code2_glo(uint8_t code)
{
    switch (code) {
        case CODE_L2C: return 0; /* L2 C/A */
        case CODE_L2P: return 1; /* L2 P */
    }
    return 0;
}
/* carrier-phase - pseudorange in cycle --------------------------------------*/
static double cp_pr(double cp, double pr_cyc)
{
    return fmod(cp-pr_cyc+750.0,1500.0)-750.0;
}
/* generate obs field data GPS -----------------------------------------------*/
static void gen_obs_gps(rtcm_t *rtcm, const obsd_t *data, int *code1, int *pr1,
                        int *ppr1, int *lock1, int *amb, int *cnr1, int *code2,
                        int *pr21, int *ppr2, int *lock2, int *cnr2)
{
    double lam1,lam2,pr1c=0.0,ppr;
    int lt1,lt2;
    
    lam1=CLIGHT/FREQ1;
    lam2=CLIGHT/FREQ2;
    *pr1=*amb=0;
    if (ppr1) *ppr1=0xFFF80000; /* invalid values */
    if (pr21) *pr21=0xFFFFE000;
    if (ppr2) *ppr2=0xFFF80000;
    
    /* L1 peudorange */
    if (data->P[0]!=0.0&&data->code[0]) {
        *amb=(int)floor(data->P[0]/PRUNIT_GPS);
        *pr1=ROUND((data->P[0]-*amb*PRUNIT_GPS)/0.02);
        pr1c=*pr1*0.02+*amb*PRUNIT_GPS;
    }
    /* L1 phaserange - L1 pseudorange */
    if (data->P[0]!=0.0&&data->L[0]!=0.0&&data->code[0]) {
        ppr=cp_pr(data->L[0],pr1c/lam1);
        if (ppr1) *ppr1=ROUND(ppr*lam1/0.0005);
    }
    /* L2 -L1 pseudorange */
    if (data->P[0]!=0.0&&data->P[1]!=0.0&&data->code[0]&&data->code[1]&&
        fabs(data->P[1]-pr1c)<=163.82) {
        if (pr21) *pr21=ROUND((data->P[1]-pr1c)/0.02);
    }
    /* L2 phaserange - L1 pseudorange */
    if (data->P[0]!=0.0&&data->L[1]!=0.0&&data->code[0]&&data->code[1]) {
        ppr=cp_pr(data->L[1],pr1c/lam2);
        if (ppr2) *ppr2=ROUND(ppr*lam2/0.0005);
    }
    lt1=locktime(data->time,rtcm->lltime[data->sat-1]  ,data->LLI[0]);
    lt2=locktime(data->time,rtcm->lltime[data->sat-1]+1,data->LLI[1]);
    
    if (lock1) *lock1=to_lock(lt1);
    if (lock2) *lock2=to_lock(lt2);
    if (cnr1 ) *cnr1=ROUND(data->SNR[0]*SNR_UNIT/0.25);
    if (cnr2 ) *cnr2=ROUND(data->SNR[1]*SNR_UNIT/0.25);
    if (code1) *code1=to_code1_gps(data->code[0]);
    if (code2) *code2=to_code2_gps(data->code[1]);
}
/* generate obs field data GLONASS -------------------------------------------*/
static void gen_obs_glo(rtcm_t *rtcm, const obsd_t *data, int fcn, int *code1,
                        int *pr1, int *ppr1, int *lock1, int *amb, int *cnr1,
                        int *code2, int *pr21, int *ppr2, int *lock2, int *cnr2)
{
    double lam1=0.0,lam2=0.0,pr1c=0.0,ppr;
    int lt1,lt2;
    
    if (fcn>=0) { /* fcn+7 */
        lam1=CLIGHT/(FREQ1_GLO+DFRQ1_GLO*(fcn-7));
        lam2=CLIGHT/(FREQ2_GLO+DFRQ2_GLO*(fcn-7));
    }
    *pr1=*amb=0;
    if (ppr1) *ppr1=0xFFF80000; /* invalid values */
    if (pr21) *pr21=0xFFFFE000;
    if (ppr2) *ppr2=0xFFF80000;
    
    /* L1 pseudorange */
    if (data->P[0]!=0.0) {
        *amb=(int)floor(data->P[0]/PRUNIT_GLO);
        *pr1=ROUND((data->P[0]-*amb*PRUNIT_GLO)/0.02);
        pr1c=*pr1*0.02+*amb*PRUNIT_GLO;
    }
    /* L1 phaserange - L1 pseudorange */
    if (data->P[0]!=0.0&&data->L[0]!=0.0&&data->code[0]&&lam1>0.0) {
        ppr=cp_pr(data->L[0],pr1c/lam1);
        if (ppr1) *ppr1=ROUND(ppr*lam1/0.0005);
    }
    /* L2 -L1 pseudorange */
    if (data->P[0]!=0.0&&data->P[1]!=0.0&&data->code[0]&&data->code[1]&&
        fabs(data->P[1]-pr1c)<=163.82) {
        if (pr21) *pr21=ROUND((data->P[1]-pr1c)/0.02);
    }
    /* L2 phaserange - L1 pseudorange */
    if (data->P[0]!=0.0&&data->L[1]!=0.0&&data->code[0]&&data->code[1]&&
        lam2>0.0) {
        ppr=cp_pr(data->L[1],pr1c/lam2);
        if (ppr2) *ppr2=ROUND(ppr*lam2/0.0005);
    }
    lt1=locktime(data->time,rtcm->lltime[data->sat-1]  ,data->LLI[0]);
    lt2=locktime(data->time,rtcm->lltime[data->sat-1]+1,data->LLI[1]);
    
    if (lock1) *lock1=to_lock(lt1);
    if (lock2) *lock2=to_lock(lt2);
    if (cnr1 ) *cnr1=ROUND(data->SNR[0]*SNR_UNIT/0.25);
    if (cnr2 ) *cnr2=ROUND(data->SNR[1]*SNR_UNIT/0.25);
    if (code1) *code1=to_code1_glo(data->code[0]);
    if (code2) *code2=to_code2_glo(data->code[1]);
}
/* encode RTCM header --------------------------------------------------------*/
static int encode_head(int type, rtcm_t *rtcm, int sys, int sync, int nsat)
{
    double tow;
    int i=24,week,epoch;
    
    trace(4,"encode_head: type=%d sync=%d sys=%d nsat=%d\n",type,sync,sys,nsat);
    
    setbitu(rtcm->buff,i,12,type       ); i+=12; /* message no */
    setbitu(rtcm->buff,i,12,rtcm->staid); i+=12; /* ref station id */
    
    if (sys==SYS_GLO) {
        tow=time2gpst(timeadd(gpst2utc(rtcm->time),10800.0),&week);
        epoch=ROUND(fmod(tow,86400.0)/0.001);
        setbitu(rtcm->buff,i,27,epoch); i+=27; /* glonass epoch time */
    }
    else {
        tow=time2gpst(rtcm->time,&week);
        epoch=ROUND(tow/0.001);
        setbitu(rtcm->buff,i,30,epoch); i+=30; /* gps epoch time */
    }
    setbitu(rtcm->buff,i, 1,sync); i+= 1; /* synchronous gnss flag */
    setbitu(rtcm->buff,i, 5,nsat); i+= 5; /* no of satellites */
    setbitu(rtcm->buff,i, 1,0   ); i+= 1; /* smoothing indicator */
    setbitu(rtcm->buff,i, 3,0   ); i+= 3; /* smoothing interval */
    return i;
}
/* encode type 1001: basic L1-only GPS RTK observables -----------------------*/
static int encode_type1001(rtcm_t *rtcm, int sync)
{
    int i,j,nsat=0,sys,prn;
    int code1,pr1,ppr1,lock1,amb;
    
    trace(3,"encode_type1001: sync=%d\n",sync);
    
    for (j=0;j<rtcm->obs.n&&nsat<MAXOBS;j++) {
        sys=satsys(rtcm->obs.data[j].sat,&prn);
        if (!(sys&(SYS_GPS|SYS_SBS))) continue;
        nsat++;
    }
    /* encode header */
    i=encode_head(1001,rtcm,SYS_GPS,sync,nsat);
    
    for (j=0;j<rtcm->obs.n&&nsat<MAXOBS;j++) {
        sys=satsys(rtcm->obs.data[j].sat,&prn);
        if (!(sys&(SYS_GPS|SYS_SBS))) continue;
        
        if (sys==SYS_SBS) prn-=80; /* 40-58: sbas 120-138 */
        
        /* generate obs field data gps */
        gen_obs_gps(rtcm,rtcm->obs.data+j,&code1,&pr1,&ppr1,&lock1,&amb,NULL,
                    NULL,NULL,NULL,NULL,NULL);
        
        setbitu(rtcm->buff,i, 6,prn  ); i+= 6;
        setbitu(rtcm->buff,i, 1,code1); i+= 1;
        setbitu(rtcm->buff,i,24,pr1  ); i+=24;
        setbits(rtcm->buff,i,20,ppr1 ); i+=20;
        setbitu(rtcm->buff,i, 7,lock1); i+= 7;
    }
    rtcm->nbit=i;
    return 1;
}
/* encode type 1002: extended L1-only GPS RTK observables --------------------*/
static int encode_type1002(rtcm_t *rtcm, int sync)
{
    int i,j,nsat=0,sys,prn;
    int code1,pr1,ppr1,lock1,amb,cnr1;
    
    trace(3,"encode_type1002: sync=%d\n",sync);
    
    for (j=0;j<rtcm->obs.n&&nsat<MAXOBS;j++) {
        sys=satsys(rtcm->obs.data[j].sat,&prn);
        if (!(sys&(SYS_GPS|SYS_SBS))) continue;
        nsat++;
    }
    /* encode header */
    i=encode_head(1002,rtcm,SYS_GPS,sync,nsat);
    
    for (j=0;j<rtcm->obs.n&&nsat<MAXOBS;j++) {
        sys=satsys(rtcm->obs.data[j].sat,&prn);
        if (!(sys&(SYS_GPS|SYS_SBS))) continue;
        
        if (sys==SYS_SBS) prn-=80; /* 40-58: sbas 120-138 */
        
        /* generate obs field data gps */
        gen_obs_gps(rtcm,rtcm->obs.data+j,&code1,&pr1,&ppr1,&lock1,&amb,&cnr1,
                    NULL,NULL,NULL,NULL,NULL);
        
        setbitu(rtcm->buff,i, 6,prn  ); i+= 6;
        setbitu(rtcm->buff,i, 1,code1); i+= 1;
        setbitu(rtcm->buff,i,24,pr1  ); i+=24;
        setbits(rtcm->buff,i,20,ppr1 ); i+=20;
        setbitu(rtcm->buff,i, 7,lock1); i+= 7;
        setbitu(rtcm->buff,i, 8,amb  ); i+= 8;
        setbitu(rtcm->buff,i, 8,cnr1 ); i+= 8;
    }
    rtcm->nbit=i;
    return 1;
}
/* encode type 1003: basic L1&L2 GPS RTK observables -------------------------*/
static int encode_type1003(rtcm_t *rtcm, int sync)
{
    int i,j,nsat=0,sys,prn;
    int code1,pr1,ppr1,lock1,amb,code2,pr21,ppr2,lock2;
    
    trace(3,"encode_type1003: sync=%d\n",sync);
    
    for (j=0;j<rtcm->obs.n&&nsat<MAXOBS;j++) {
        sys=satsys(rtcm->obs.data[j].sat,&prn);
        if (!(sys&(SYS_GPS|SYS_SBS))) continue;
        nsat++;
    }
    /* encode header */
    i=encode_head(1003,rtcm,SYS_GPS,sync,nsat);
    
    for (j=0;j<rtcm->obs.n&&nsat<MAXOBS;j++) {
        sys=satsys(rtcm->obs.data[j].sat,&prn);
        if (!(sys&(SYS_GPS|SYS_SBS))) continue;
        
        if (sys==SYS_SBS) prn-=80; /* 40-58: sbas 120-138 */
        
        /* generate obs field data gps */
        gen_obs_gps(rtcm,rtcm->obs.data+j,&code1,&pr1,&ppr1,&lock1,&amb,
                    NULL,&code2,&pr21,&ppr2,&lock2,NULL);
        
        setbitu(rtcm->buff,i, 6,prn  ); i+= 6;
        setbitu(rtcm->buff,i, 1,code1); i+= 1;
        setbitu(rtcm->buff,i,24,pr1  ); i+=24;
        setbits(rtcm->buff,i,20,ppr1 ); i+=20;
        setbitu(rtcm->buff,i, 7,lock1); i+= 7;
        setbitu(rtcm->buff,i, 2,code2); i+= 2;
        setbits(rtcm->buff,i,14,pr21 ); i+=14;
        setbits(rtcm->buff,i,20,ppr2 ); i+=20;
        setbitu(rtcm->buff,i, 7,lock2); i+= 7;
    }
    rtcm->nbit=i;
    return 1;
}
/* encode type 1004: extended L1&L2 GPS RTK observables ----------------------*/
static int encode_type1004(rtcm_t *rtcm, int sync)
{
    int i,j,nsat=0,sys,prn;
    int code1,pr1,ppr1,lock1,amb,cnr1,code2,pr21,ppr2,lock2,cnr2;
    
    trace(3,"encode_type1004: sync=%d\n",sync);
    
    for (j=0;j<rtcm->obs.n&&nsat<MAXOBS;j++) {
        sys=satsys(rtcm->obs.data[j].sat,&prn);
        if (!(sys&(SYS_GPS|SYS_SBS))) continue;
        nsat++;
    }
    /* encode header */
    i=encode_head(1004,rtcm,SYS_GPS,sync,nsat);
    
    for (j=0;j<rtcm->obs.n&&nsat<MAXOBS;j++) {
        sys=satsys(rtcm->obs.data[j].sat,&prn);
        if (!(sys&(SYS_GPS|SYS_SBS))) continue;
        
        if (sys==SYS_SBS) prn-=80; /* 40-58: sbas 120-138 */
        
        /* generate obs field data gps */
        gen_obs_gps(rtcm,rtcm->obs.data+j,&code1,&pr1,&ppr1,&lock1,&amb,
                    &cnr1,&code2,&pr21,&ppr2,&lock2,&cnr2);
        
        setbitu(rtcm->buff,i, 6,prn  ); i+= 6;
        setbitu(rtcm->buff,i, 1,code1); i+= 1;
        setbitu(rtcm->buff,i,24,pr1  ); i+=24;
        setbits(rtcm->buff,i,20,ppr1 ); i+=20;
        setbitu(rtcm->buff,i, 7,lock1); i+= 7;
        setbitu(rtcm->buff,i, 8,amb  ); i+= 8;
        setbitu(rtcm->buff,i, 8,cnr1 ); i+= 8;
        setbitu(rtcm->buff,i, 2,code2); i+= 2;
        setbits(rtcm->buff,i,14,pr21 ); i+=14;
        setbits(rtcm->buff,i,20,ppr2 ); i+=20;
        setbitu(rtcm->buff,i, 7,lock2); i+= 7;
        setbitu(rtcm->buff,i, 8,cnr2 ); i+= 8;
    }
    rtcm->nbit=i;
    return 1;
}
/* encode type 1005: stationary RTK reference station ARP --------------------*/
static int encode_type1005(rtcm_t *rtcm, int sync)
{
    double *p=rtcm->sta.pos;
    int i=24;
    
    trace(3,"encode_type1005: sync=%d\n",sync);
    
    setbitu(rtcm->buff,i,12,1005       ); i+=12; /* message no */
    setbitu(rtcm->buff,i,12,rtcm->staid); i+=12; /* ref station id */
    setbitu(rtcm->buff,i, 6,0          ); i+= 6; /* itrf realization year */
    setbitu(rtcm->buff,i, 1,1          ); i+= 1; /* gps indicator */
    setbitu(rtcm->buff,i, 1,1          ); i+= 1; /* glonass indicator */
    setbitu(rtcm->buff,i, 1,0          ); i+= 1; /* galileo indicator */
    setbitu(rtcm->buff,i, 1,0          ); i+= 1; /* ref station indicator */
    set38bits(rtcm->buff,i,p[0]/0.0001 ); i+=38; /* antenna ref point ecef-x */
    setbitu(rtcm->buff,i, 1,1          ); i+= 1; /* oscillator indicator */
    setbitu(rtcm->buff,i, 1,0          ); i+= 1; /* reserved */
    set38bits(rtcm->buff,i,p[1]/0.0001 ); i+=38; /* antenna ref point ecef-y */
    setbitu(rtcm->buff,i, 2,0          ); i+= 2; /* quarter cycle indicator */
    set38bits(rtcm->buff,i,p[2]/0.0001 ); i+=38; /* antenna ref point ecef-z */
    rtcm->nbit=i;
    return 1;
}
/* encode type 1006: stationary RTK reference station ARP with height --------*/
static int encode_type1006(rtcm_t *rtcm, int sync)
{
    double *p=rtcm->sta.pos;
    int i=24,hgt=0;
    
    trace(3,"encode_type1006: sync=%d\n",sync);
    
    if (0.0<=rtcm->sta.hgt&&rtcm->sta.hgt<=6.5535) {
        hgt=ROUND(rtcm->sta.hgt/0.0001);
    }
    else {
        trace(2,"antenna height error: h=%.4f\n",rtcm->sta.hgt);
    }
    setbitu(rtcm->buff,i,12,1006       ); i+=12; /* message no */
    setbitu(rtcm->buff,i,12,rtcm->staid); i+=12; /* ref station id */
    setbitu(rtcm->buff,i, 6,0          ); i+= 6; /* itrf realization year */
    setbitu(rtcm->buff,i, 1,1          ); i+= 1; /* gps indicator */
    setbitu(rtcm->buff,i, 1,1          ); i+= 1; /* glonass indicator */
    setbitu(rtcm->buff,i, 1,0          ); i+= 1; /* galileo indicator */
    setbitu(rtcm->buff,i, 1,0          ); i+= 1; /* ref station indicator */
    set38bits(rtcm->buff,i,p[0]/0.0001 ); i+=38; /* antenna ref point ecef-x */
    setbitu(rtcm->buff,i, 1,1          ); i+= 1; /* oscillator indicator */
    setbitu(rtcm->buff,i, 1,0          ); i+= 1; /* reserved */
    set38bits(rtcm->buff,i,p[1]/0.0001 ); i+=38; /* antenna ref point ecef-y */
    setbitu(rtcm->buff,i, 2,0          ); i+= 2; /* quarter cycle indicator */
    set38bits(rtcm->buff,i,p[2]/0.0001 ); i+=38; /* antenna ref point ecef-z */
    setbitu(rtcm->buff,i,16,hgt        ); i+=16; /* antenna height */
    rtcm->nbit=i;
    return 1;
}
/* encode type 1007: antenna descriptor --------------------------------------*/
static int encode_type1007(rtcm_t *rtcm, int sync)
{
    int i=24,j,antsetup=rtcm->sta.antsetup;
    int n=(int)MIN(strlen(rtcm->sta.antdes),31);
    
    trace(3,"encode_type1007: sync=%d\n",sync);
    
    setbitu(rtcm->buff,i,12,1007       ); i+=12; /* message no */
    setbitu(rtcm->buff,i,12,rtcm->staid); i+=12; /* ref station id */
    
    /* antenna descriptor */
    setbitu(rtcm->buff,i,8,n); i+=8;
    for (j=0;j<n;j++) {
        setbitu(rtcm->buff,i,8,rtcm->sta.antdes[j]); i+=8;
    }
    setbitu(rtcm->buff,i,8,antsetup); i+=8; /* antetnna setup id */
    rtcm->nbit=i;
    return 1;
}
/* encode type 1008: antenna descriptor & serial number ----------------------*/
static int encode_type1008(rtcm_t *rtcm, int sync)
{
    int i=24,j,antsetup=rtcm->sta.antsetup;
    int n=(int)MIN(strlen(rtcm->sta.antdes),31);
    int m=(int)MIN(strlen(rtcm->sta.antsno),31);
    
    trace(3,"encode_type1008: sync=%d\n",sync);
    
    setbitu(rtcm->buff,i,12,1008       ); i+=12; /* message no */
    setbitu(rtcm->buff,i,12,rtcm->staid); i+=12; /* ref station id */
    
    /* antenna descriptor */
    setbitu(rtcm->buff,i,8,n); i+=8;
    for (j=0;j<n;j++) {
        setbitu(rtcm->buff,i,8,rtcm->sta.antdes[j]); i+=8;
    }
    setbitu(rtcm->buff,i,8,antsetup); i+=8; /* antenna setup id */
    
    /* antenna serial number */
    setbitu(rtcm->buff,i,8,m); i+=8;
    for (j=0;j<m;j++) {
        setbitu(rtcm->buff,i,8,rtcm->sta.antsno[j]); i+=8;
    }
    rtcm->nbit=i;
    return 1;
}
/* encode type 1009: basic L1-only GLONASS RTK observables -------------------*/
static int encode_type1009(rtcm_t *rtcm, nav_t *nav, int sync)
{
    int i,j,nsat=0,sat,prn,fcn;
    int code1,pr1,ppr1,lock1,amb;
    
    for (j=0;j<rtcm->obs.n&&nsat<MAXOBS;j++) {
        sat=rtcm->obs.data[j].sat;
        if (satsys(sat,&prn)!=SYS_GLO) continue;
        if ((fcn=fcn_glo(sat,nav))<0) continue; /* fcn+7 */
        nsat++;
    }
    /* encode header */
    i=encode_head(1009,rtcm,SYS_GLO,sync,nsat);
    
    for (j=0;j<rtcm->obs.n&&nsat<MAXOBS;j++) {
        sat=rtcm->obs.data[j].sat;
        if (satsys(sat,&prn)!=SYS_GLO) continue;
        if ((fcn=fcn_glo(sat,nav))<0) continue; /* fcn+7 */
        
        /* generate obs field data glonass */
        gen_obs_glo(rtcm,rtcm->obs.data+j,fcn,&code1,&pr1,&ppr1,&lock1,&amb,
                    NULL,NULL,NULL,NULL,NULL,NULL);
        
        setbitu(rtcm->buff,i, 6,prn  ); i+= 6;
        setbitu(rtcm->buff,i, 1,code1); i+= 1;
        setbitu(rtcm->buff,i, 5,fcn  ); i+= 5; /* fcn+7 */
        setbitu(rtcm->buff,i,25,pr1  ); i+=25;
        setbits(rtcm->buff,i,20,ppr1 ); i+=20;
        setbitu(rtcm->buff,i, 7,lock1); i+= 7;
    }
    rtcm->nbit=i;
    return 1;
}
/* encode type 1010: extended L1-only GLONASS RTK observables ----------------*/
static int encode_type1010(rtcm_t *rtcm, nav_t* nav, int sync)
{
    int i,j,nsat=0,sat,prn,fcn;
    int code1,pr1,ppr1,lock1,amb,cnr1;
    
    trace(3,"encode_type1010: sync=%d\n",sync);
    
    for (j=0;j<rtcm->obs.n&&nsat<MAXOBS;j++) {
        sat=rtcm->obs.data[j].sat;
        if (satsys(sat,&prn)!=SYS_GLO) continue;
        if ((fcn=fcn_glo(sat,nav))<0) continue; /* fcn+7 */
        nsat++;
    }
    /* encode header */
    i=encode_head(1010,rtcm,SYS_GLO,sync,nsat);
    
    for (j=0;j<rtcm->obs.n&&nsat<MAXOBS;j++) {
        sat=rtcm->obs.data[j].sat;
        if (satsys(sat,&prn)!=SYS_GLO) continue;
        if ((fcn=fcn_glo(sat,nav))<0) continue; /* fcn+7 */
        
        /* generate obs field data glonass */
        gen_obs_glo(rtcm,rtcm->obs.data+j,fcn,&code1,&pr1,&ppr1,&lock1,&amb,
                    &cnr1,NULL,NULL,NULL,NULL,NULL);
        
        setbitu(rtcm->buff,i, 6,prn  ); i+= 6;
        setbitu(rtcm->buff,i, 1,code1); i+= 1;
        setbitu(rtcm->buff,i, 5,fcn  ); i+= 5; /* fcn+7 */
        setbitu(rtcm->buff,i,25,pr1  ); i+=25;
        setbits(rtcm->buff,i,20,ppr1 ); i+=20;
        setbitu(rtcm->buff,i, 7,lock1); i+= 7;
        setbitu(rtcm->buff,i, 7,amb  ); i+= 7;
        setbitu(rtcm->buff,i, 8,cnr1 ); i+= 8;
    }
    rtcm->nbit=i;
    return 1;
}
/* encode type 1011: basic  L1&L2 GLONASS RTK observables --------------------*/
static int encode_type1011(rtcm_t *rtcm, nav_t *nav, int sync)
{
    int i,j,nsat=0,sat,prn,fcn;
    int code1,pr1,ppr1,lock1,amb,code2,pr21,ppr2,lock2;
    
    trace(3,"encode_type1011: sync=%d\n",sync);
    
    for (j=0;j<rtcm->obs.n&&nsat<MAXOBS;j++) {
        sat=rtcm->obs.data[j].sat;
        if (satsys(sat,&prn)!=SYS_GLO) continue;
        if ((fcn=fcn_glo(sat,nav))<0) continue; /* fcn+7 */
        nsat++;
    }
    /* encode header */
    i=encode_head(1011,rtcm,SYS_GLO,sync,nsat);
    
    for (j=0;j<rtcm->obs.n&&nsat<MAXOBS;j++) {
        sat=rtcm->obs.data[j].sat;
        if (satsys(sat,&prn)!=SYS_GLO) continue;
        if ((fcn=fcn_glo(sat,nav))<0) continue; /* fcn+7 */
        
        /* generate obs field data glonass */
        gen_obs_glo(rtcm,rtcm->obs.data+j,fcn,&code1,&pr1,&ppr1,&lock1,&amb,
                    NULL,&code2,&pr21,&ppr2,&lock2,NULL);
        
        setbitu(rtcm->buff,i, 6,prn  ); i+= 6;
        setbitu(rtcm->buff,i, 1,code1); i+= 1;
        setbitu(rtcm->buff,i, 5,fcn  ); i+= 5; /* fcn+7 */
        setbitu(rtcm->buff,i,25,pr1  ); i+=25;
        setbits(rtcm->buff,i,20,ppr1 ); i+=20;
        setbitu(rtcm->buff,i, 7,lock1); i+= 7;
        setbitu(rtcm->buff,i, 2,code2); i+= 2;
        setbits(rtcm->buff,i,14,pr21 ); i+=14;
        setbits(rtcm->buff,i,20,ppr2 ); i+=20;
        setbitu(rtcm->buff,i, 7,lock2); i+= 7;
    }
    rtcm->nbit=i;
    return 1;
}
/* encode type 1012: extended L1&L2 GLONASS RTK observables ------------------*/
static int encode_type1012(rtcm_t *rtcm, nav_t *nav, int sync)
{
    int i,j,nsat=0,sat,prn,fcn;
    int code1,pr1,ppr1,lock1,amb,cnr1,code2,pr21,ppr2,lock2,cnr2;
    
    trace(3,"encode_type1012: sync=%d\n",sync);
    
    for (j=0;j<rtcm->obs.n&&nsat<MAXOBS;j++) {
        sat=rtcm->obs.data[j].sat;
        if (satsys(sat,&prn)!=SYS_GLO) continue;
        if ((fcn=fcn_glo(sat,nav))<0) continue;  /* fcn+7 */
        nsat++;
    }
    /* encode header */
    i=encode_head(1012,rtcm,SYS_GLO,sync,nsat);
    
    for (j=0;j<rtcm->obs.n&&nsat<MAXOBS;j++) {
        sat=rtcm->obs.data[j].sat;
        if (satsys(sat,&prn)!=SYS_GLO) continue;
        if ((fcn=fcn_glo(sat,nav))<0) continue; /* fcn+7 */
        
        /* generate obs field data glonass */
        gen_obs_glo(rtcm,rtcm->obs.data+j,fcn,&code1,&pr1,&ppr1,&lock1,&amb,
                    &cnr1,&code2,&pr21,&ppr2,&lock2,&cnr2);
        
        setbitu(rtcm->buff,i, 6,prn  ); i+= 6;
        setbitu(rtcm->buff,i, 1,code1); i+= 1;
        setbitu(rtcm->buff,i, 5,fcn  ); i+= 5; /* fcn+7 */
        setbitu(rtcm->buff,i,25,pr1  ); i+=25;
        setbits(rtcm->buff,i,20,ppr1 ); i+=20;
        setbitu(rtcm->buff,i, 7,lock1); i+= 7;
        setbitu(rtcm->buff,i, 7,amb  ); i+= 7;
        setbitu(rtcm->buff,i, 8,cnr1 ); i+= 8;
        setbitu(rtcm->buff,i, 2,code2); i+= 2;
        setbits(rtcm->buff,i,14,pr21 ); i+=14;
        setbits(rtcm->buff,i,20,ppr2 ); i+=20;
        setbitu(rtcm->buff,i, 7,lock2); i+= 7;
        setbitu(rtcm->buff,i, 8,cnr2 ); i+= 8;
    }
    rtcm->nbit=i;
    return 1;
}
/* encode type 1019: GPS ephemerides -----------------------------------------*/
static int encode_type1019(rtcm_t *rtcm, eph_t *eph, int sync)
{
    uint32_t sqrtA,e;
    int i=24,prn,week,toe,toc,i0,OMG0,omg,M0,deln,idot,OMGd,crs,crc;
    int cus,cuc,cis,cic,af0,af1,af2,tgd;
    
    trace(3,"encode_type1019: sync=%d\n",sync);
    
    if (satsys(eph->sat,&prn)!=SYS_GPS) return 0;

    week=eph->week%1024;
    toe  =ROUND(eph->toes/16.0);
    toc  =ROUND(time2gpst(eph->toc,NULL)/16.0);
    sqrtA=ROUND_U(sqrt(eph->A)/P2_19);
    e    =ROUND_U(eph->e/P2_33);
    i0   =ROUND(eph->i0  /P2_31/SC2RAD);
    OMG0 =ROUND(eph->OMG0/P2_31/SC2RAD);
    omg  =ROUND(eph->omg /P2_31/SC2RAD);
    M0   =ROUND(eph->M0  /P2_31/SC2RAD);
    deln =ROUND(eph->deln/P2_43/SC2RAD);
    idot =ROUND(eph->idot/P2_43/SC2RAD);
    OMGd =ROUND(eph->OMGd/P2_43/SC2RAD);
    crs  =ROUND(eph->crs/P2_5 );
    crc  =ROUND(eph->crc/P2_5 );
    cus  =ROUND(eph->cus/P2_29);
    cuc  =ROUND(eph->cuc/P2_29);
    cis  =ROUND(eph->cis/P2_29);
    cic  =ROUND(eph->cic/P2_29);
    af0  =ROUND(eph->f0 /P2_31);
    af1  =ROUND(eph->f1 /P2_43);
    af2  =ROUND(eph->f2 /P2_55);
    tgd  =ROUND(eph->tgd[0]/P2_31);
    
    setbitu(rtcm->buff,i,12,1019     ); i+=12;
    setbitu(rtcm->buff,i, 6,prn      ); i+= 6;
    setbitu(rtcm->buff,i,10,week     ); i+=10;
    setbitu(rtcm->buff,i, 4,eph->sva ); i+= 4;
    setbitu(rtcm->buff,i, 2,eph->code); i+= 2;
    setbits(rtcm->buff,i,14,idot     ); i+=14;
    setbitu(rtcm->buff,i, 8,eph->iode); i+= 8;
    setbitu(rtcm->buff,i,16,toc      ); i+=16;
    setbits(rtcm->buff,i, 8,af2      ); i+= 8;
    setbits(rtcm->buff,i,16,af1      ); i+=16;
    setbits(rtcm->buff,i,22,af0      ); i+=22;
    setbitu(rtcm->buff,i,10,eph->iodc); i+=10;
    setbits(rtcm->buff,i,16,crs      ); i+=16;
    setbits(rtcm->buff,i,16,deln     ); i+=16;
    setbits(rtcm->buff,i,32,M0       ); i+=32;
    setbits(rtcm->buff,i,16,cuc      ); i+=16;
    setbitu(rtcm->buff,i,32,e        ); i+=32;
    setbits(rtcm->buff,i,16,cus      ); i+=16;
    setbitu(rtcm->buff,i,32,sqrtA    ); i+=32;
    setbitu(rtcm->buff,i,16,toe      ); i+=16;
    setbits(rtcm->buff,i,16,cic      ); i+=16;
    setbits(rtcm->buff,i,32,OMG0     ); i+=32;
    setbits(rtcm->buff,i,16,cis      ); i+=16;
    setbits(rtcm->buff,i,32,i0       ); i+=32;
    setbits(rtcm->buff,i,16,crc      ); i+=16;
    setbits(rtcm->buff,i,32,omg      ); i+=32;
    setbits(rtcm->buff,i,24,OMGd     ); i+=24;
    setbits(rtcm->buff,i, 8,tgd      ); i+= 8;
    setbitu(rtcm->buff,i, 6,eph->svh ); i+= 6;
    setbitu(rtcm->buff,i, 1,eph->flag); i+= 1;
    setbitu(rtcm->buff,i, 1,eph->fit>0.0?0:1); i+=1;
    rtcm->nbit=i;
    return 1;
}
/* encode type 1020: GLONASS ephemerides -------------------------------------*/
static int encode_type1020(rtcm_t *rtcm, geph_t* geph, int sync)
{
    gtime_t time;
    double ep[6];
    int i=24,j,prn,tk_h,tk_m,tk_s,tb,pos[3],vel[3],acc[3],gamn,taun,dtaun;
    int fcn,NT;
    
    trace(3,"encode_type1020: sync=%d\n",sync);

    if (satsys(geph->sat,&prn)!=SYS_GLO) return 0;
   
    fcn=geph->frq+7;
    
    /* time of frame within day (utc(su) + 3 hr) */
    time=timeadd(gpst2utc(geph->tof),10800.0);
    time2epoch(time,ep);
    tk_h=(int)ep[3];
    tk_m=(int)ep[4];
    tk_s=ROUND(ep[5]/30.0);
    
    /* # of days since jan 1 in leap year */
    ep[0]=floor(ep[0]/4.0)*4.0; ep[1]=ep[2]=1.0;
    ep[3]=ep[4]=ep[5]=0.0;
    NT=(int)floor(timediff(time,epoch2time(ep))/86400.+1.0);
    
    /* index of time interval within day (utc(su) + 3 hr) */
    time=timeadd(gpst2utc(geph->toe),10800.0);
    time2epoch(time,ep);
    tb=ROUND((ep[3]*3600.0+ep[4]*60.0+ep[5])/900.0);
    
    for (j=0;j<3;j++) {
        pos[j]=ROUND(geph->pos[j]/P2_11/1E3);
        vel[j]=ROUND(geph->vel[j]/P2_20/1E3);
        acc[j]=ROUND(geph->acc[j]/P2_30/1E3);
    }
    gamn =ROUND(geph->gamn /P2_40);
    taun =ROUND(geph->taun /P2_30);
    dtaun=ROUND(geph->dtaun/P2_30);
    
    setbitu(rtcm->buff,i,12,1020     ); i+=12;
    setbitu(rtcm->buff,i, 6,prn      ); i+= 6;
    setbitu(rtcm->buff,i, 5,fcn      ); i+= 5;
    setbitu(rtcm->buff,i, 4,0        ); i+= 4; /* almanac health,P1 */
    setbitu(rtcm->buff,i, 5,tk_h     ); i+= 5;
    setbitu(rtcm->buff,i, 6,tk_m     ); i+= 6;
    setbitu(rtcm->buff,i, 1,tk_s     ); i+= 1;
    setbitu(rtcm->buff,i, 1,geph->svh); i+= 1; /* Bn */
    setbitu(rtcm->buff,i, 1,0        ); i+= 1; /* P2 */
    setbitu(rtcm->buff,i, 7,tb       ); i+= 7;
    setbitg(rtcm->buff,i,24,vel[0]   ); i+=24;
    setbitg(rtcm->buff,i,27,pos[0]   ); i+=27;
    setbitg(rtcm->buff,i, 5,acc[0]   ); i+= 5;
    setbitg(rtcm->buff,i,24,vel[1]   ); i+=24;
    setbitg(rtcm->buff,i,27,pos[1]   ); i+=27;
    setbitg(rtcm->buff,i, 5,acc[1]   ); i+= 5;
    setbitg(rtcm->buff,i,24,vel[2]   ); i+=24;
    setbitg(rtcm->buff,i,27,pos[2]   ); i+=27;
    setbitg(rtcm->buff,i, 5,acc[2]   ); i+= 5;
    setbitu(rtcm->buff,i, 1,0        ); i+= 1; /* P3 */
    setbitg(rtcm->buff,i,11,gamn     ); i+=11;
    setbitu(rtcm->buff,i, 3,0        ); i+= 3; /* P,ln */
    setbitg(rtcm->buff,i,22,taun     ); i+=22;
    setbitg(rtcm->buff,i, 5,dtaun    ); i+= 5;
    setbitu(rtcm->buff,i, 5,geph->age); i+= 5; /* En */
    setbitu(rtcm->buff,i, 1,0        ); i+= 1; /* P4 */
    setbitu(rtcm->buff,i, 4,0        ); i+= 4; /* FT */
    setbitu(rtcm->buff,i,11,NT       ); i+=11;
    setbitu(rtcm->buff,i, 2,0        ); i+= 2; /* M */
    setbitu(rtcm->buff,i, 1,0        ); i+= 1; /* flag for additional data */
    setbitu(rtcm->buff,i,11,0        ); i+=11; /* NA */
    setbitu(rtcm->buff,i,32,0        ); i+=32; /* tauc */
    setbitu(rtcm->buff,i, 5,0        ); i+= 5; /* N4 */
    setbitu(rtcm->buff,i,22,0        ); i+=22; /* taugps */
    setbitu(rtcm->buff,i, 1,0        ); i+= 1; /* ln */
    setbitu(rtcm->buff,i, 7,0        ); i+= 7;
    rtcm->nbit=i;
    return 1;
}
/* encode type 1033: receiver and antenna descriptor -------------------------*/
static int encode_type1033(rtcm_t *rtcm, int sync)
{
    int i=24,j,antsetup=rtcm->sta.antsetup;
    int n=(int)MIN(strlen(rtcm->sta.antdes ),31);
    int m=(int)MIN(strlen(rtcm->sta.antsno ),31);
    int I=(int)MIN(strlen(rtcm->sta.rectype),31);
    int J=(int)MIN(strlen(rtcm->sta.recver ),31);
    int K=(int)MIN(strlen(rtcm->sta.recsno ),31);
    
    trace(3,"encode_type1033: sync=%d\n",sync);
    
    setbitu(rtcm->buff,i,12,1033       ); i+=12;
    setbitu(rtcm->buff,i,12,rtcm->staid); i+=12;
    
    setbitu(rtcm->buff,i,8,n); i+= 8;
    for (j=0;j<n;j++) {
        setbitu(rtcm->buff,i,8,rtcm->sta.antdes[j]); i+=8;
    }
    setbitu(rtcm->buff,i,8,antsetup); i+= 8;
    
    setbitu(rtcm->buff,i,8,m); i+= 8;
    for (j=0;j<m;j++) {
        setbitu(rtcm->buff,i,8,rtcm->sta.antsno[j]); i+=8;
    }
    setbitu(rtcm->buff,i,8,I); i+= 8;
    for (j=0;j<I;j++) {
        setbitu(rtcm->buff,i,8,rtcm->sta.rectype[j]); i+=8;
    }
    setbitu(rtcm->buff,i,8,J); i+= 8;
    for (j=0;j<J;j++) {
        setbitu(rtcm->buff,i,8,rtcm->sta.recver[j]); i+=8;
    }
    setbitu(rtcm->buff,i,8,K); i+= 8;
    for (j=0;j<K;j++) {
        setbitu(rtcm->buff,i,8,rtcm->sta.recsno[j]); i+=8;
    }
    rtcm->nbit=i;
    return 1;
}
/* encode type 1041: NavIC/IRNSS ephemerides ---------------------------------*/
static int encode_type1041(rtcm_t *rtcm, eph_t *eph, int sync)
{
    uint32_t sqrtA,e;
    int i=24,prn,week,toe,toc,i0,OMG0,omg,M0,deln,idot,OMGd,crs,crc;
    int cus,cuc,cis,cic,af0,af1,af2,tgd;
    
    trace(3,"encode_type1041: sync=%d\n",sync);
    
    if (satsys(eph->sat,&prn)!=SYS_IRN) return 0;
    week=eph->week%1024;
    toe  =ROUND(eph->toes/16.0);
    toc  =ROUND(time2gpst(eph->toc,NULL)/16.0);
    sqrtA=ROUND_U(sqrt(eph->A)/P2_19);
    e    =ROUND_U(eph->e/P2_33);
    i0   =ROUND(eph->i0  /P2_31/SC2RAD);
    OMG0 =ROUND(eph->OMG0/P2_31/SC2RAD);
    omg  =ROUND(eph->omg /P2_31/SC2RAD);
    M0   =ROUND(eph->M0  /P2_31/SC2RAD);
    deln =ROUND(eph->deln/P2_41/SC2RAD);
    idot =ROUND(eph->idot/P2_43/SC2RAD);
    OMGd =ROUND(eph->OMGd/P2_41/SC2RAD);
    crs  =ROUND(eph->crs/0.0625);
    crc  =ROUND(eph->crc/0.0625);
    cus  =ROUND(eph->cus/P2_28);
    cuc  =ROUND(eph->cuc/P2_28);
    cis  =ROUND(eph->cis/P2_28);
    cic  =ROUND(eph->cic/P2_28);
    af0  =ROUND(eph->f0 /P2_31);
    af1  =ROUND(eph->f1 /P2_43);
    af2  =ROUND(eph->f2 /P2_55);
    tgd  =ROUND(eph->tgd[0]/P2_31);
    
    setbitu(rtcm->buff,i,12,1041     ); i+=12;
    setbitu(rtcm->buff,i, 6,prn      ); i+= 6;
    setbitu(rtcm->buff,i,10,week     ); i+=10;
    setbits(rtcm->buff,i,22,af0      ); i+=22;
    setbits(rtcm->buff,i,16,af1      ); i+=16;
    setbits(rtcm->buff,i, 8,af2      ); i+= 8;
    setbitu(rtcm->buff,i, 4,eph->sva ); i+= 4;
    setbitu(rtcm->buff,i,16,toc      ); i+=16;
    setbits(rtcm->buff,i, 8,tgd      ); i+= 8;
    setbits(rtcm->buff,i,22,deln     ); i+=22;
    setbitu(rtcm->buff,i, 8,eph->iode); i+= 8+10; /* IODEC */
    setbitu(rtcm->buff,i, 2,eph->svh ); i+= 2; /* L5+Sflag */
    setbits(rtcm->buff,i,15,cuc      ); i+=15;
    setbits(rtcm->buff,i,15,cus      ); i+=15;
    setbits(rtcm->buff,i,15,cic      ); i+=15;
    setbits(rtcm->buff,i,15,cis      ); i+=15;
    setbits(rtcm->buff,i,15,crc      ); i+=15;
    setbits(rtcm->buff,i,15,crs      ); i+=15;
    setbits(rtcm->buff,i,14,idot     ); i+=14;
    setbits(rtcm->buff,i,32,M0       ); i+=32;
    setbitu(rtcm->buff,i,16,toe      ); i+=16;
    setbitu(rtcm->buff,i,32,e        ); i+=32;
    setbitu(rtcm->buff,i,32,sqrtA    ); i+=32;
    setbits(rtcm->buff,i,32,OMG0     ); i+=32;
    setbits(rtcm->buff,i,32,omg      ); i+=32;
    setbits(rtcm->buff,i,22,OMGd     ); i+=22;
    setbits(rtcm->buff,i,32,i0       ); i+=32+4;
    rtcm->nbit=i;
    return 1;
}
/* encode type 1044: QZSS ephemerides ----------------------------------------*/
static int encode_type1044(rtcm_t *rtcm, eph_t *eph, int sync)
{
    uint32_t sqrtA,e;
    int i=24,prn,week,toe,toc,i0,OMG0,omg,M0,deln,idot,OMGd,crs,crc;
    int cus,cuc,cis,cic,af0,af1,af2,tgd;
    
    trace(3,"encode_type1044: sync=%d\n",sync);
    
    if (satsys(eph->sat,&prn)!=SYS_QZS) return 0;
    week=eph->week%1024;
    toe  =ROUND(eph->toes/16.0);
    toc  =ROUND(time2gpst(eph->toc,NULL)/16.0);
    sqrtA=ROUND_U(sqrt(eph->A)/P2_19);
    e    =ROUND_U(eph->e/P2_33);
    i0   =ROUND(eph->i0  /P2_31/SC2RAD);
    OMG0 =ROUND(eph->OMG0/P2_31/SC2RAD);
    omg  =ROUND(eph->omg /P2_31/SC2RAD);
    M0   =ROUND(eph->M0  /P2_31/SC2RAD);
    deln =ROUND(eph->deln/P2_43/SC2RAD);
    idot =ROUND(eph->idot/P2_43/SC2RAD);
    OMGd =ROUND(eph->OMGd/P2_43/SC2RAD);
    crs  =ROUND(eph->crs/P2_5 );
    crc  =ROUND(eph->crc/P2_5 );
    cus  =ROUND(eph->cus/P2_29);
    cuc  =ROUND(eph->cuc/P2_29);
    cis  =ROUND(eph->cis/P2_29);
    cic  =ROUND(eph->cic/P2_29);
    af0  =ROUND(eph->f0 /P2_31);
    af1  =ROUND(eph->f1 /P2_43);
    af2  =ROUND(eph->f2 /P2_55);
    tgd  =ROUND(eph->tgd[0]/P2_31);
    
    setbitu(rtcm->buff,i,12,1044     ); i+=12;
    setbitu(rtcm->buff,i, 4,prn-192  ); i+= 4;
    setbitu(rtcm->buff,i,16,toc      ); i+=16;
    setbits(rtcm->buff,i, 8,af2      ); i+= 8;
    setbits(rtcm->buff,i,16,af1      ); i+=16;
    setbits(rtcm->buff,i,22,af0      ); i+=22;
    setbitu(rtcm->buff,i, 8,eph->iode); i+= 8;
    setbits(rtcm->buff,i,16,crs      ); i+=16;
    setbits(rtcm->buff,i,16,deln     ); i+=16;
    setbits(rtcm->buff,i,32,M0       ); i+=32;
    setbits(rtcm->buff,i,16,cuc      ); i+=16;
    setbitu(rtcm->buff,i,32,e        ); i+=32;
    setbits(rtcm->buff,i,16,cus      ); i+=16;
    setbitu(rtcm->buff,i,32,sqrtA    ); i+=32;
    setbitu(rtcm->buff,i,16,toe      ); i+=16;
    setbits(rtcm->buff,i,16,cic      ); i+=16;
    setbits(rtcm->buff,i,32,OMG0     ); i+=32;
    setbits(rtcm->buff,i,16,cis      ); i+=16;
    setbits(rtcm->buff,i,32,i0       ); i+=32;
    setbits(rtcm->buff,i,16,crc      ); i+=16;
    setbits(rtcm->buff,i,32,omg      ); i+=32;
    setbits(rtcm->buff,i,24,OMGd     ); i+=24;
    setbits(rtcm->buff,i,14,idot     ); i+=14;
    setbitu(rtcm->buff,i, 2,eph->code); i+= 2;
    setbitu(rtcm->buff,i,10,week     ); i+=10;
    setbitu(rtcm->buff,i, 4,eph->sva ); i+= 4;
    setbitu(rtcm->buff,i, 6,eph->svh ); i+= 6;
    setbits(rtcm->buff,i, 8,tgd      ); i+= 8;
    setbitu(rtcm->buff,i,10,eph->iodc); i+=10;
    setbitu(rtcm->buff,i, 1,eph->fit==2.0?0:1); i+=1;
    rtcm->nbit=i;
    return 1;
}
/* encode type 1045: Galileo F/NAV satellite ephemerides ---------------------*/
static int encode_type1045(rtcm_t *rtcm, eph_t *eph, int sync)
{
    uint32_t sqrtA,e;
    int i=24,prn,week,toe,toc,i0,OMG0,omg,M0,deln,idot,OMGd,crs,crc;
    int cus,cuc,cis,cic,af0,af1,af2,bgd1,bgd2,oshs,osdvs;
    
    trace(3,"encode_type1045: sync=%d\n",sync);
    
    if (satsys(eph->sat,&prn)!=SYS_GAL) return 0;
    week=(eph->week-1024)%4096; /* gst-week = gal-week - 1024 */
    toe  =ROUND(eph->toes/60.0);
    toc  =ROUND(time2gpst(eph->toc,NULL)/60.0);
    sqrtA=ROUND_U(sqrt(eph->A)/P2_19);
    e    =ROUND_U(eph->e/P2_33);
    i0   =ROUND(eph->i0  /P2_31/SC2RAD);
    OMG0 =ROUND(eph->OMG0/P2_31/SC2RAD);
    omg  =ROUND(eph->omg /P2_31/SC2RAD);
    M0   =ROUND(eph->M0  /P2_31/SC2RAD);
    deln =ROUND(eph->deln/P2_43/SC2RAD);
    idot =ROUND(eph->idot/P2_43/SC2RAD);
    OMGd =ROUND(eph->OMGd/P2_43/SC2RAD);
    crs  =ROUND(eph->crs/P2_5 );
    crc  =ROUND(eph->crc/P2_5 );
    cus  =ROUND(eph->cus/P2_29);
    cuc  =ROUND(eph->cuc/P2_29);
    cis  =ROUND(eph->cis/P2_29);
    cic  =ROUND(eph->cic/P2_29);
    af0  =ROUND(eph->f0 /P2_34);
    af1  =ROUND(eph->f1 /P2_46);
    af2  =ROUND(eph->f2 /P2_59);
    bgd1 =ROUND(eph->tgd[0]/P2_32); /* E5a/E1 */
    bgd2 =ROUND(eph->tgd[1]/P2_32); /* E5b/E1 */
    oshs =(eph->svh>>4)&3;          /* E5a SVH */
    osdvs=(eph->svh>>3)&1;          /* E5a DVS */
    setbitu(rtcm->buff,i,12,1045     ); i+=12;
    setbitu(rtcm->buff,i, 6,prn      ); i+= 6;
    setbitu(rtcm->buff,i,12,week     ); i+=12;
    setbitu(rtcm->buff,i,10,eph->iode); i+=10;
    setbitu(rtcm->buff,i, 8,eph->sva ); i+= 8;
    setbits(rtcm->buff,i,14,idot     ); i+=14;
    setbitu(rtcm->buff,i,14,toc      ); i+=14;
    setbits(rtcm->buff,i, 6,af2      ); i+= 6;
    setbits(rtcm->buff,i,21,af1      ); i+=21;
    setbits(rtcm->buff,i,31,af0      ); i+=31;
    setbits(rtcm->buff,i,16,crs      ); i+=16;
    setbits(rtcm->buff,i,16,deln     ); i+=16;
    setbits(rtcm->buff,i,32,M0       ); i+=32;
    setbits(rtcm->buff,i,16,cuc      ); i+=16;
    setbitu(rtcm->buff,i,32,e        ); i+=32;
    setbits(rtcm->buff,i,16,cus      ); i+=16;
    setbitu(rtcm->buff,i,32,sqrtA    ); i+=32;
    setbitu(rtcm->buff,i,14,toe      ); i+=14;
    setbits(rtcm->buff,i,16,cic      ); i+=16;
    setbits(rtcm->buff,i,32,OMG0     ); i+=32;
    setbits(rtcm->buff,i,16,cis      ); i+=16;
    setbits(rtcm->buff,i,32,i0       ); i+=32;
    setbits(rtcm->buff,i,16,crc      ); i+=16;
    setbits(rtcm->buff,i,32,omg      ); i+=32;
    setbits(rtcm->buff,i,24,OMGd     ); i+=24;
    setbits(rtcm->buff,i,10,bgd1     ); i+=10;
    setbitu(rtcm->buff,i, 2,oshs     ); i+= 2; /* E5a SVH */
    setbitu(rtcm->buff,i, 1,osdvs    ); i+= 1; /* E5a DVS */
    setbitu(rtcm->buff,i, 7,0        ); i+= 7; /* reserved */
    rtcm->nbit=i;
    return 1;
}
/* encode type 1046: Galileo I/NAV satellite ephemerides ---------------------*/
static int encode_type1046(rtcm_t *rtcm, eph_t *eph, int sync)
{
    uint32_t sqrtA,e;
    int i=24,prn,week,toe,toc,i0,OMG0,omg,M0,deln,idot,OMGd,crs,crc;
    int cus,cuc,cis,cic,af0,af1,af2,bgd1,bgd2,oshs1,osdvs1,oshs2,osdvs2;
    
    trace(3,"encode_type1046: sync=%d\n",sync);
    
    if (satsys(eph->sat,&prn)!=SYS_GAL) return 0;
    week=(eph->week-1024)%4096; /* gst-week = gal-week - 1024 */
    toe  =ROUND(eph->toes/60.0);
    toc  =ROUND(time2gpst(eph->toc,NULL)/60.0);
    sqrtA=ROUND_U(sqrt(eph->A)/P2_19);
    e    =ROUND_U(eph->e/P2_33);
    i0   =ROUND(eph->i0  /P2_31/SC2RAD);
    OMG0 =ROUND(eph->OMG0/P2_31/SC2RAD);
    omg  =ROUND(eph->omg /P2_31/SC2RAD);
    M0   =ROUND(eph->M0  /P2_31/SC2RAD);
    deln =ROUND(eph->deln/P2_43/SC2RAD);
    idot =ROUND(eph->idot/P2_43/SC2RAD);
    OMGd =ROUND(eph->OMGd/P2_43/SC2RAD);
    crs  =ROUND(eph->crs/P2_5 );
    crc  =ROUND(eph->crc/P2_5 );
    cus  =ROUND(eph->cus/P2_29);
    cuc  =ROUND(eph->cuc/P2_29);
    cis  =ROUND(eph->cis/P2_29);
    cic  =ROUND(eph->cic/P2_29);
    af0  =ROUND(eph->f0 /P2_34);
    af1  =ROUND(eph->f1 /P2_46);
    af2  =ROUND(eph->f2 /P2_59);
    bgd1 =ROUND(eph->tgd[0]/P2_32); /* E5a/E1 */
    bgd2 =ROUND(eph->tgd[1]/P2_32); /* E5b/E1 */
    oshs1 =(eph->svh>>7)&3;         /* E5b SVH */
    osdvs1=(eph->svh>>6)&1;         /* E5b DVS */
    oshs2 =(eph->svh>>1)&3;         /* E1 SVH */
    osdvs2=(eph->svh>>0)&1;         /* E1 DVS */
    setbitu(rtcm->buff,i,12,1046     ); i+=12;
    setbitu(rtcm->buff,i, 6,prn      ); i+= 6;
    setbitu(rtcm->buff,i,12,week     ); i+=12;
    setbitu(rtcm->buff,i,10,eph->iode); i+=10;
    setbitu(rtcm->buff,i, 8,eph->sva ); i+= 8;
    setbits(rtcm->buff,i,14,idot     ); i+=14;
    setbitu(rtcm->buff,i,14,toc      ); i+=14;
    setbits(rtcm->buff,i, 6,af2      ); i+= 6;
    setbits(rtcm->buff,i,21,af1      ); i+=21;
    setbits(rtcm->buff,i,31,af0      ); i+=31;
    setbits(rtcm->buff,i,16,crs      ); i+=16;
    setbits(rtcm->buff,i,16,deln     ); i+=16;
    setbits(rtcm->buff,i,32,M0       ); i+=32;
    setbits(rtcm->buff,i,16,cuc      ); i+=16;
    setbitu(rtcm->buff,i,32,e        ); i+=32;
    setbits(rtcm->buff,i,16,cus      ); i+=16;
    setbitu(rtcm->buff,i,32,sqrtA    ); i+=32;
    setbitu(rtcm->buff,i,14,toe      ); i+=14;
    setbits(rtcm->buff,i,16,cic      ); i+=16;
    setbits(rtcm->buff,i,32,OMG0     ); i+=32;
    setbits(rtcm->buff,i,16,cis      ); i+=16;
    setbits(rtcm->buff,i,32,i0       ); i+=32;
    setbits(rtcm->buff,i,16,crc      ); i+=16;
    setbits(rtcm->buff,i,32,omg      ); i+=32;
    setbits(rtcm->buff,i,24,OMGd     ); i+=24;
    setbits(rtcm->buff,i,10,bgd1     ); i+=10;
    setbits(rtcm->buff,i,10,bgd2     ); i+=10;
    setbitu(rtcm->buff,i, 2,oshs1    ); i+= 2; /* E5b SVH */
    setbitu(rtcm->buff,i, 1,osdvs1   ); i+= 1; /* E5b DVS */
    setbitu(rtcm->buff,i, 2,oshs2    ); i+= 2; /* E1 SVH */
    setbitu(rtcm->buff,i, 1,osdvs2   ); i+= 1; /* E1 DVS */
    rtcm->nbit=i;
    return 1;
}
/* encode type 1042: Beidou ephemerides --------------------------------------*/
static int encode_type1042(rtcm_t *rtcm, eph_t *eph, int sync)
{
    uint32_t sqrtA,e;
    int i=24,prn,week,toe,toc,i0,OMG0,omg,M0,deln,idot,OMGd,crs,crc;
    int cus,cuc,cis,cic,af0,af1,af2,tgd1,tgd2;
    
    trace(3,"encode_type1042: sync=%d\n",sync);
    
    if (satsys(eph->sat,&prn)!=SYS_CMP) return 0;
    week =eph->week%8192;
    toe  =ROUND(eph->toes/8.0);
    toc  =ROUND(time2bdt(gpst2bdt(eph->toc),NULL)/8.0); /* gpst -> bdt */
    sqrtA=ROUND_U(sqrt(eph->A)/P2_19);
    e    =ROUND_U(eph->e/P2_33);
    i0   =ROUND(eph->i0  /P2_31/SC2RAD);
    OMG0 =ROUND(eph->OMG0/P2_31/SC2RAD);
    omg  =ROUND(eph->omg /P2_31/SC2RAD);
    M0   =ROUND(eph->M0  /P2_31/SC2RAD);
    deln =ROUND(eph->deln/P2_43/SC2RAD);
    idot =ROUND(eph->idot/P2_43/SC2RAD);
    OMGd =ROUND(eph->OMGd/P2_43/SC2RAD);
    crs  =ROUND(eph->crs/P2_6 );
    crc  =ROUND(eph->crc/P2_6 );
    cus  =ROUND(eph->cus/P2_31);
    cuc  =ROUND(eph->cuc/P2_31);
    cis  =ROUND(eph->cis/P2_31);
    cic  =ROUND(eph->cic/P2_31);
    af0  =ROUND(eph->f0 /P2_33);
    af1  =ROUND(eph->f1 /P2_50);
    af2  =ROUND(eph->f2 /P2_66);
    tgd1 =ROUND(eph->tgd[0]/1E-10);
    tgd2 =ROUND(eph->tgd[1]/1E-10);
    
    setbitu(rtcm->buff,i,12,1042     ); i+=12;
    setbitu(rtcm->buff,i, 6,prn      ); i+= 6;
    setbitu(rtcm->buff,i,13,week     ); i+=13;
    setbitu(rtcm->buff,i, 4,eph->sva ); i+= 4;
    setbits(rtcm->buff,i,14,idot     ); i+=14;
    setbitu(rtcm->buff,i, 5,eph->iode); i+= 5;
    setbitu(rtcm->buff,i,17,toc      ); i+=17;
    setbits(rtcm->buff,i,11,af2      ); i+=11;
    setbits(rtcm->buff,i,22,af1      ); i+=22;
    setbits(rtcm->buff,i,24,af0      ); i+=24;
    setbitu(rtcm->buff,i, 5,eph->iodc); i+= 5;
    setbits(rtcm->buff,i,18,crs      ); i+=18;
    setbits(rtcm->buff,i,16,deln     ); i+=16;
    setbits(rtcm->buff,i,32,M0       ); i+=32;
    setbits(rtcm->buff,i,18,cuc      ); i+=18;
    setbitu(rtcm->buff,i,32,e        ); i+=32;
    setbits(rtcm->buff,i,18,cus      ); i+=18;
    setbitu(rtcm->buff,i,32,sqrtA    ); i+=32;
    setbitu(rtcm->buff,i,17,toe      ); i+=17;
    setbits(rtcm->buff,i,18,cic      ); i+=18;
    setbits(rtcm->buff,i,32,OMG0     ); i+=32;
    setbits(rtcm->buff,i,18,cis      ); i+=18;
    setbits(rtcm->buff,i,32,i0       ); i+=32;
    setbits(rtcm->buff,i,18,crc      ); i+=18;
    setbits(rtcm->buff,i,32,omg      ); i+=32;
    setbits(rtcm->buff,i,24,OMGd     ); i+=24;
    setbits(rtcm->buff,i,10,tgd1     ); i+=10;
    setbits(rtcm->buff,i,10,tgd2     ); i+=10;
    setbitu(rtcm->buff,i, 1,eph->svh ); i+= 1;
    rtcm->nbit=i;
    return 1;
}
/* encode type 63: Beidou ephemerides (RTCM draft) ---------------------------*/
static int encode_type63(rtcm_t *rtcm, eph_t *eph, int sync)
{
    uint32_t sqrtA,e;
    int i=24,prn,week,toe,toc,i0,OMG0,omg,M0,deln,idot,OMGd,crs,crc;
    int cus,cuc,cis,cic,af0,af1,af2,tgd1,tgd2;
    
    trace(3,"encode_type63: sync=%d\n",sync);
    
    if (satsys(eph->sat,&prn)!=SYS_CMP) return 0;
    week =eph->week%8192;
    toe  =ROUND(eph->toes/8.0);
    toc  =ROUND(time2bdt(gpst2bdt(eph->toc),NULL)/8.0); /* gpst -> bdt */
    sqrtA=ROUND_U(sqrt(eph->A)/P2_19);
    e    =ROUND_U(eph->e/P2_33);
    i0   =ROUND(eph->i0  /P2_31/SC2RAD);
    OMG0 =ROUND(eph->OMG0/P2_31/SC2RAD);
    omg  =ROUND(eph->omg /P2_31/SC2RAD);
    M0   =ROUND(eph->M0  /P2_31/SC2RAD);
    deln =ROUND(eph->deln/P2_43/SC2RAD);
    idot =ROUND(eph->idot/P2_43/SC2RAD);
    OMGd =ROUND(eph->OMGd/P2_43/SC2RAD);
    crs  =ROUND(eph->crs/P2_6 );
    crc  =ROUND(eph->crc/P2_6 );
    cus  =ROUND(eph->cus/P2_31);
    cuc  =ROUND(eph->cuc/P2_31);
    cis  =ROUND(eph->cis/P2_31);
    cic  =ROUND(eph->cic/P2_31);
    af0  =ROUND(eph->f0 /P2_33);
    af1  =ROUND(eph->f1 /P2_50);
    af2  =ROUND(eph->f2 /P2_66);
    tgd1 =ROUND(eph->tgd[0]/1E-10);
    tgd2 =ROUND(eph->tgd[1]/1E-10);
    
    setbitu(rtcm->buff,i,12,63       ); i+=12;
    setbitu(rtcm->buff,i, 6,prn      ); i+= 6;
    setbitu(rtcm->buff,i,13,week     ); i+=13;
    setbitu(rtcm->buff,i, 4,eph->sva ); i+= 4;
    setbits(rtcm->buff,i,14,idot     ); i+=14;
    setbitu(rtcm->buff,i, 5,eph->iode); i+= 5;
    setbitu(rtcm->buff,i,17,toc      ); i+=17;
    setbits(rtcm->buff,i,11,af2      ); i+=11;
    setbits(rtcm->buff,i,22,af1      ); i+=22;
    setbits(rtcm->buff,i,24,af0      ); i+=24;
    setbitu(rtcm->buff,i, 5,eph->iodc); i+= 5;
    setbits(rtcm->buff,i,18,crs      ); i+=18;
    setbits(rtcm->buff,i,16,deln     ); i+=16;
    setbits(rtcm->buff,i,32,M0       ); i+=32;
    setbits(rtcm->buff,i,18,cuc      ); i+=18;
    setbitu(rtcm->buff,i,32,e        ); i+=32;
    setbits(rtcm->buff,i,18,cus      ); i+=18;
    setbitu(rtcm->buff,i,32,sqrtA    ); i+=32;
    setbitu(rtcm->buff,i,17,toe      ); i+=17;
    setbits(rtcm->buff,i,18,cic      ); i+=18;
    setbits(rtcm->buff,i,32,OMG0     ); i+=32;
    setbits(rtcm->buff,i,18,cis      ); i+=18;
    setbits(rtcm->buff,i,32,i0       ); i+=32;
    setbits(rtcm->buff,i,18,crc      ); i+=18;
    setbits(rtcm->buff,i,32,omg      ); i+=32;
    setbits(rtcm->buff,i,24,OMGd     ); i+=24;
    setbits(rtcm->buff,i,10,tgd1     ); i+=10;
    setbits(rtcm->buff,i,10,tgd2     ); i+=10;
    setbitu(rtcm->buff,i, 1,eph->svh ); i+= 1;
    rtcm->nbit=i;
    return 1;
}
/* encode SSR header ---------------------------------------------------------*/
static int encode_ssr_head(int type, rtcm_t *rtcm, int sys, int subtype,
                           int nsat, int sync, int iod, double udint, int refd,
                           int provid, int solid)
{
    double tow;
    int i=24,msgno,epoch,week,udi,ns;
    
    trace(4,"encode_ssr_head: type=%d sys=%d subtype=%d nsat=%d sync=%d iod=%d "
          "udint=%.0f\n",type,sys,subtype,nsat,sync,iod,udint);
    
    if (subtype==0) { /* RTCM SSR */
        ns=(sys==SYS_QZS)?4:6;
        switch (sys) {
            case SYS_GPS: msgno=(type==7)?11:1056+type; break;
            case SYS_GLO: msgno=(type==7)? 0:1062+type; break;
            case SYS_GAL: msgno=(type==7)?12:1239+type; break; /* draft */
            case SYS_QZS: msgno=(type==7)?13:1245+type; break; /* draft */
            case SYS_CMP: msgno=(type==7)?14:1257+type; break; /* draft */
            case SYS_SBS: msgno=(type==7)? 0:1251+type; break; /* draft */
            default: return 0;
        }
        if (msgno==0) {
            return 0;
        }
        setbitu(rtcm->buff,i,12,msgno); i+=12; /* message type */
        
        if (sys==SYS_GLO) {
            tow=time2gpst(timeadd(gpst2utc(rtcm->time),10800.0),&week);
            epoch=ROUND(tow)%86400;
            setbitu(rtcm->buff,i,17,epoch); i+=17; /* GLONASS epoch time */
        }
        else {
            tow=time2gpst(rtcm->time,&week);
            epoch=ROUND(tow)%604800;
            setbitu(rtcm->buff,i,20,epoch); i+=20; /* GPS epoch time */
        }
    }
    else { /* IGS SSR */
        ns=6;
        tow=time2gpst(rtcm->time,&week);
        epoch=ROUND(tow)%604800;
        setbitu(rtcm->buff,i,12,4076   ); i+=12; /* message type */
        setbitu(rtcm->buff,i, 3,1      ); i+= 3; /* version */
        setbitu(rtcm->buff,i, 8,subtype); i+= 8; /* subtype */
        setbitu(rtcm->buff,i,20,epoch  ); i+=20; /* SSR epoch time */
    }
    for (udi=0;udi<15;udi++) {
        if (ssrudint[udi]>=udint) break;
    }
    setbitu(rtcm->buff,i, 4,udi    ); i+= 4; /* update interval */
    setbitu(rtcm->buff,i, 1,sync   ); i+= 1; /* multiple message indicator */
    if (subtype==0&&(type==1||type==4)) {
        setbitu(rtcm->buff,i,1,refd); i+= 1; /* satellite ref datum */
    }
    setbitu(rtcm->buff,i, 4,iod    ); i+= 4; /* IOD SSR */
    setbitu(rtcm->buff,i,16,provid ); i+=16; /* provider ID */
    setbitu(rtcm->buff,i, 4,solid  ); i+= 4; /* solution ID */
    if (subtype>0&&(type==1||type==4)) {
        setbitu(rtcm->buff,i,1,refd); i+= 1; /* global/regional CRS indicator */
    }
    if (type==7) {
        setbitu(rtcm->buff,i,1,0); i+=1; /* dispersive bias consistency ind */
        setbitu(rtcm->buff,i,1,0); i+=1; /* MW consistency indicator */
    }
    setbitu(rtcm->buff,i,ns,nsat); i+=ns; /* no of satellites */
    return i;
}
/* SSR signal and tracking mode IDs ------------------------------------------*/
static  const int codes_gps[32]={
    CODE_L1C,CODE_L1P,CODE_L1W,CODE_L1S,CODE_L1L,CODE_L2C,CODE_L2D,CODE_L2S,
    CODE_L2L,CODE_L2X,CODE_L2P,CODE_L2W,       0,       0,CODE_L5I,CODE_L5Q
};
static const int codes_glo[32]={
    CODE_L1C,CODE_L1P,CODE_L2C,CODE_L2P,CODE_L4A,CODE_L4B,CODE_L6A,CODE_L6B,
    CODE_L3I,CODE_L3Q
};
static const int codes_gal[32]={
    CODE_L1A,CODE_L1B,CODE_L1C,       0,       0,CODE_L5I,CODE_L5Q,       0,
    CODE_L7I,CODE_L7Q,       0,CODE_L8I,CODE_L8Q,       0,CODE_L6A,CODE_L6B,
    CODE_L6C
};
static const int codes_qzs[32]={
    CODE_L1C,CODE_L1S,CODE_L1L,CODE_L2S,CODE_L2L,       0,CODE_L5I,CODE_L5Q,
           0,CODE_L6S,CODE_L6L,       0,       0,       0,       0,       0,
           0,CODE_L6E
};
static const int codes_bds[32]={
    CODE_L2I,CODE_L2Q,       0,CODE_L6I,CODE_L6Q,       0,CODE_L7I,CODE_L7Q,
           0,CODE_L1D,CODE_L1P,       0,CODE_L5D,CODE_L5P,       0,CODE_L1A,
           0,       0,CODE_L6A
};
static const int codes_sbs[32]={
    CODE_L1C,CODE_L5I,CODE_L5Q
};
/* encode SSR 1: orbit corrections -------------------------------------------*/
static int encode_ssr1(rtcm_t *rtcm, nav_t *nav, int sys, int subtype, int sync)
{
    double udint=0.0;
    int i,j,iod=0,nsat,prn,iode,iodcrc,refd=0,np,ni,nj,offp,deph[3],ddeph[3];
    
    trace(3,"encode_ssr1: sys=%d subtype=%d sync=%d\n",sys,subtype,sync);
    
    switch (sys) {
        case SYS_GPS: np=6; ni= 8; nj= 0; offp=  0; break;
        case SYS_GLO: np=5; ni= 8; nj= 0; offp=  0; break;
        case SYS_GAL: np=6; ni=10; nj= 0; offp=  0; break;
        case SYS_QZS: np=4; ni= 8; nj= 0; offp=192; break;
        case SYS_CMP: np=6; ni=10; nj=24; offp=  1; break;
        case SYS_SBS: np=6; ni= 9; nj=24; offp=120; break;
        default: return 0;
    }
    if (subtype>0) { /* IGS SSR */
        np=6; ni=8; nj=0;
        if      (sys==SYS_CMP) offp=0;
        else if (sys==SYS_SBS) offp=119;
    }
    /* number of satellites */
    for (j=nsat=0;j<MAXSAT;j++) {
        if (satsys(j+1,&prn)!=sys||!nav->ssr[j].update) continue;
        nsat++;
        udint=nav->ssr[j].udi[0];
        iod  =nav->ssr[j].iod[0];
        refd =nav->ssr[j].refd;
    }
    /* encode SSR header */
    i=encode_ssr_head(1,rtcm,sys,subtype,nsat,sync,iod,udint,refd,0,0);
    
    for (j=0;j<MAXSAT;j++) {
        if (satsys(j+1,&prn)!=sys||!nav->ssr[j].update) continue;
        
        iode=nav->ssr[j].iode;      /* SBAS/BDS: toe/t0 modulo */
        iodcrc=nav->ssr[j].iodcrc;  /* SBAS/BDS: IOD CRC */
        
        if (subtype>0) { /* IGS SSR */
            iode&=0xFF;
        }
        deph [0]=ROUND(nav->ssr[j].deph [0]/1E-4);
        deph [1]=ROUND(nav->ssr[j].deph [1]/4E-4);
        deph [2]=ROUND(nav->ssr[j].deph [2]/4E-4);
        ddeph[0]=ROUND(nav->ssr[j].ddeph[0]/1E-6);
        ddeph[1]=ROUND(nav->ssr[j].ddeph[1]/4E-6);
        ddeph[2]=ROUND(nav->ssr[j].ddeph[2]/4E-6);
        
        setbitu(rtcm->buff,i,np,prn-offp); i+=np; /* satellite ID */
        setbitu(rtcm->buff,i,ni,iode    ); i+=ni; /* IODE */
        setbitu(rtcm->buff,i,nj,iodcrc  ); i+=nj; /* IODCRC */
        setbits(rtcm->buff,i,22,deph [0]); i+=22; /* delta radial */
        setbits(rtcm->buff,i,20,deph [1]); i+=20; /* delta along-track */
        setbits(rtcm->buff,i,20,deph [2]); i+=20; /* delta cross-track */
        setbits(rtcm->buff,i,21,ddeph[0]); i+=21; /* dot delta radial */
        setbits(rtcm->buff,i,19,ddeph[1]); i+=19; /* dot delta along-track */
        setbits(rtcm->buff,i,19,ddeph[2]); i+=19; /* dot delta cross-track */
    }
    rtcm->nbit=i;
    return 1;
}
/* encode SSR 2: clock corrections -------------------------------------------*/
static int encode_ssr2(rtcm_t *rtcm, nav_t *nav, int sys, int subtype, int sync)
{
    double udint=0.0;
    int i,j,iod=0,nsat,prn,np,offp,dclk[3];
    
    trace(3,"encode_ssr2: sys=%d subtype=%d sync=%d\n",sys,subtype,sync);
    
    switch (sys) {
        case SYS_GPS: np=6; offp=  0; break;
        case SYS_GLO: np=5; offp=  0; break;
        case SYS_GAL: np=6; offp=  0; break;
        case SYS_QZS: np=4; offp=192; break;
        case SYS_CMP: np=6; offp=  1; break;
        case SYS_SBS: np=6; offp=120; break;
        default: return 0;
    }
    if (subtype>0) { /* IGS SSR */
        np=6;
        if      (sys==SYS_CMP) offp=0;
        else if (sys==SYS_SBS) offp=119;
    }
    /* number of satellites */
    for (j=nsat=0;j<MAXSAT;j++) {
        if (satsys(j+1,&prn)!=sys||!nav->ssr[j].update) continue;
        nsat++;
        udint=nav->ssr[j].udi[1];
        iod  =nav->ssr[j].iod[1];
    }
    /* encode SSR header */
    i=encode_ssr_head(2,rtcm,sys,subtype,nsat,sync,iod,udint,0,0,0);
    
    for (j=0;j<MAXSAT;j++) {
        if (satsys(j+1,&prn)!=sys||!nav->ssr[j].update) continue;
        
        dclk[0]=ROUND(nav->ssr[j].dclk[0]/1E-4);
        dclk[1]=ROUND(nav->ssr[j].dclk[1]/1E-6);
        dclk[2]=ROUND(nav->ssr[j].dclk[2]/2E-8);
        
        setbitu(rtcm->buff,i,np,prn-offp); i+=np; /* satellite ID */
        setbits(rtcm->buff,i,22,dclk[0] ); i+=22; /* delta clock C0 */
        setbits(rtcm->buff,i,21,dclk[1] ); i+=21; /* delta clock C1 */
        setbits(rtcm->buff,i,27,dclk[2] ); i+=27; /* delta clock C2 */
    }
    rtcm->nbit=i;
    return 1;
}
/* encode SSR 3: satellite code biases ---------------------------------------*/
static int encode_ssr3(rtcm_t *rtcm, nav_t *nav, int sys, int subtype, int sync)
{
    const int *codes;
    double udint=0.0;
    int i,j,k,iod=0,nsat,prn,nbias,np,offp;
    int code[MAXCODE],bias[MAXCODE];
    
    trace(3,"encode_ssr3: sys=%d subtype=%d sync=%d\n",sys,subtype,sync);
    
    switch (sys) {
        case SYS_GPS: np=6; offp=  0; codes=codes_gps; break;
        case SYS_GLO: np=5; offp=  0; codes=codes_glo; break;
        case SYS_GAL: np=6; offp=  0; codes=codes_gal; break;
        case SYS_QZS: np=4; offp=192; codes=codes_qzs; break;
        case SYS_CMP: np=6; offp=  1; codes=codes_bds; break;
        case SYS_SBS: np=6; offp=120; codes=codes_sbs; break;
        default: return 0;
    }
    if (subtype>0) { /* IGS SSR */
        np=6;
        if      (sys==SYS_CMP) offp=0;
        else if (sys==SYS_SBS) offp=119;
    }
    /* number of satellites */
    for (j=nsat=0;j<MAXSAT;j++) {
        if (satsys(j+1,&prn)!=sys||!nav->ssr[j].update) continue;
        nsat++;
        udint=nav->ssr[j].udi[4];
        iod  =nav->ssr[j].iod[4];
    }
    /* encode SSR header */
    i=encode_ssr_head(3,rtcm,sys,subtype,nsat,sync,iod,udint,0,0,0);
    
    for (j=nsat=0;j<MAXSAT;j++) {
        if (satsys(j+1,&prn)!=sys||!nav->ssr[j].update) continue;
        
        for (k=nbias=0;k<32;k++) {
            if (!codes[k]||nav->ssr[j].cbias[codes[k]-1]==0.0) continue;
            code[nbias]=k;
            bias[nbias++]=ROUND(nav->ssr[j].cbias[codes[k]-1]/0.01);
        }
        setbitu(rtcm->buff,i,np,prn-offp); i+=np; /* satellite ID */
        setbitu(rtcm->buff,i, 5,nbias);    i+= 5; /* number of code biases */
        
        for (k=0;k<nbias;k++) {
            setbitu(rtcm->buff,i, 5,code[k]); i+= 5; /* signal indicator */
            setbits(rtcm->buff,i,14,bias[k]); i+=14; /* code bias */
        }
    }
    rtcm->nbit=i;
    return 1;
}
/* encode SSR 4: combined orbit and clock corrections ------------------------*/
static int encode_ssr4(rtcm_t *rtcm, nav_t *nav, int sys, int subtype, int sync)
{
    double udint=0.0;
    int i,j,iod=0,nsat,prn,iode,iodcrc,refd=0,np,ni,nj,offp;
    int deph[3],ddeph[3],dclk[3];
    
    trace(3,"encode_ssr4: sys=%d subtype=%d sync=%d\n",sys,subtype,sync);
    
    switch (sys) {
        case SYS_GPS: np=6; ni= 8; nj= 0; offp=  0; break;
        case SYS_GLO: np=5; ni= 8; nj= 0; offp=  0; break;
        case SYS_GAL: np=6; ni=10; nj= 0; offp=  0; break;
        case SYS_QZS: np=4; ni= 8; nj= 0; offp=192; break;
        case SYS_CMP: np=6; ni=10; nj=24; offp=  1; break;
        case SYS_SBS: np=6; ni= 9; nj=24; offp=120; break;
        default: return 0;
    }
    if (subtype>0) { /* IGS SSR */
        np=6; ni=8; nj=0;
        if      (sys==SYS_CMP) offp=0;
        else if (sys==SYS_SBS) offp=119;
    }
    /* number of satellites */
    for (j=nsat=0;j<MAXSAT;j++) {
        if (satsys(j+1,&prn)!=sys||!nav->ssr[j].update) continue;
        nsat++;
        udint=nav->ssr[j].udi[0];
        iod  =nav->ssr[j].iod[0];
        refd =nav->ssr[j].refd;
    }
    /* encode SSR header */
    i=encode_ssr_head(4,rtcm,sys,subtype,nsat,sync,iod,udint,refd,0,0);
    
    for (j=0;j<MAXSAT;j++) {
        if (satsys(j+1,&prn)!=sys||!nav->ssr[j].update) continue;
        
        iode=nav->ssr[j].iode;
        iodcrc=nav->ssr[j].iodcrc;
        
        if (subtype>0) { /* IGS SSR */
            iode&=0xFF;
        }
        deph [0]=ROUND(nav->ssr[j].deph [0]/1E-4);
        deph [1]=ROUND(nav->ssr[j].deph [1]/4E-4);
        deph [2]=ROUND(nav->ssr[j].deph [2]/4E-4);
        ddeph[0]=ROUND(nav->ssr[j].ddeph[0]/1E-6);
        ddeph[1]=ROUND(nav->ssr[j].ddeph[1]/4E-6);
        ddeph[2]=ROUND(nav->ssr[j].ddeph[2]/4E-6);
        dclk [0]=ROUND(nav->ssr[j].dclk [0]/1E-4);
        dclk [1]=ROUND(nav->ssr[j].dclk [1]/1E-6);
        dclk [2]=ROUND(nav->ssr[j].dclk [2]/2E-8);
        
        setbitu(rtcm->buff,i,np,prn-offp); i+=np; /* satellite ID */
        setbitu(rtcm->buff,i,ni,iode    ); i+=ni; /* IODE */
        setbitu(rtcm->buff,i,nj,iodcrc  ); i+=nj; /* IODCRC */
        setbits(rtcm->buff,i,22,deph [0]); i+=22; /* delta raidal */
        setbits(rtcm->buff,i,20,deph [1]); i+=20; /* delta along-track */
        setbits(rtcm->buff,i,20,deph [2]); i+=20; /* delta cross-track */
        setbits(rtcm->buff,i,21,ddeph[0]); i+=21; /* dot delta radial */
        setbits(rtcm->buff,i,19,ddeph[1]); i+=19; /* dot delta along-track */
        setbits(rtcm->buff,i,19,ddeph[2]); i+=19; /* dot delta cross-track */
        setbits(rtcm->buff,i,22,dclk [0]); i+=22; /* delta clock C0 */
        setbits(rtcm->buff,i,21,dclk [1]); i+=21; /* delta clock C1 */
        setbits(rtcm->buff,i,27,dclk [2]); i+=27; /* delta clock C2 */
    }
    rtcm->nbit=i;
    return 1;
}
/* encode SSR 5: URA ---------------------------------------------------------*/
static int encode_ssr5(rtcm_t *rtcm, nav_t *nav, int sys, int subtype, int sync)
{
    double udint=0.0;
    int i,j,nsat,iod=0,prn,ura,np,offp;
    
    trace(3,"encode_ssr5: sys=%d subtype=%d sync=%d\n",sys,subtype,sync);
    
    switch (sys) {
        case SYS_GPS: np=6; offp=  0; break;
        case SYS_GLO: np=5; offp=  0; break;
        case SYS_GAL: np=6; offp=  0; break;
        case SYS_QZS: np=4; offp=192; break;
        case SYS_CMP: np=6; offp=  1; break;
        case SYS_SBS: np=6; offp=120; break;
        default: return 0;
    }
    if (subtype>0) { /* IGS SSR */
        np=6;
        if      (sys==SYS_CMP) offp=0;
        else if (sys==SYS_SBS) offp=119;
    }
    /* number of satellites */
    for (j=nsat=0;j<MAXSAT;j++) {
        if (satsys(j+1,&prn)!=sys||!nav->ssr[j].update) continue;
        nsat++;
        udint=nav->ssr[j].udi[3];
        iod  =nav->ssr[j].iod[3];
    }
    /* encode ssr header */
    i=encode_ssr_head(5,rtcm,sys,subtype,nsat,sync,iod,udint,0,0,0);
    
    for (j=0;j<MAXSAT;j++) {
        if (satsys(j+1,&prn)!=sys||!nav->ssr[j].update) continue;
        
        ura=nav->ssr[j].ura;
        setbitu(rtcm->buff,i,np,prn-offp); i+=np; /* satellite id */
        setbitu(rtcm->buff,i, 6,ura     ); i+= 6; /* ssr ura */
    }
    rtcm->nbit=i;
    return 1;
}
/* encode SSR 6: high rate clock correction ----------------------------------*/
static int encode_ssr6(rtcm_t *rtcm, nav_t *nav, int sys, int subtype, int sync)
{
    double udint=0.0;
    int i,j,nsat,iod=0,prn,hrclk,np,offp;
    
    trace(3,"encode_ssr6: sys=%d subtype=%d sync=%d\n",sys,subtype,sync);
    
    switch (sys) {
        case SYS_GPS: np=6; offp=  0; break;
        case SYS_GLO: np=5; offp=  0; break;
        case SYS_GAL: np=6; offp=  0; break;
        case SYS_QZS: np=4; offp=192; break;
        case SYS_CMP: np=6; offp=  1; break;
        case SYS_SBS: np=6; offp=120; break;
        default: return 0;
    }
    if (subtype>0) { /* IGS SSR */
        np=6;
        if      (sys==SYS_CMP) offp=0;
        else if (sys==SYS_SBS) offp=119;
    }
    /* number of satellites */
    for (j=nsat=0;j<MAXSAT;j++) {
        if (satsys(j+1,&prn)!=sys||!nav->ssr[j].update) continue;
        nsat++;
        udint=nav->ssr[j].udi[2];
        iod  =nav->ssr[j].iod[2];
    }
    /* encode SSR header */
    i=encode_ssr_head(6,rtcm,sys,subtype,nsat,sync,iod,udint,0,0,0);
    
    for (j=0;j<MAXSAT;j++) {
        if (satsys(j+1,&prn)!=sys||!nav->ssr[j].update) continue;
        
        hrclk=ROUND(nav->ssr[j].hrclk/1E-4);
        
        setbitu(rtcm->buff,i,np,prn-offp); i+=np; /* satellite ID */
        setbits(rtcm->buff,i,22,hrclk   ); i+=22; /* high rate clock corr */
    }
    rtcm->nbit=i;
    return 1;
}
/* encode SSR 7: satellite phase biases --------------------------------------*/
static int encode_ssr7(rtcm_t *rtcm, nav_t *nav, int sys, int subtype, int sync)
{
    const int *codes;
    double udint=0.0;
    int i,j,k,iod=0,nsat,prn,nbias,np,offp;
    int code[MAXCODE],pbias[MAXCODE],stdpb[MAXCODE],yaw_ang,yaw_rate;
    
    trace(3,"encode_ssr7: sys=%d subtype=%d sync=%d\n",sys,subtype,sync);
    
    switch (sys) {
        case SYS_GPS: np=6; offp=  0; codes=codes_gps; break;
        case SYS_GLO: np=5; offp=  0; codes=codes_glo; break;
        case SYS_GAL: np=6; offp=  0; codes=codes_gal; break;
        case SYS_QZS: np=4; offp=192; codes=codes_qzs; break;
        case SYS_CMP: np=6; offp=  1; codes=codes_bds; break;
        case SYS_SBS: np=6; offp=120; codes=codes_sbs; break;
        default: return 0;
    }
    if (subtype>0) { /* IGS SSR */
        np=6;
        if      (sys==SYS_CMP) offp=0;
        else if (sys==SYS_SBS) offp=119;
    }
    /* number of satellites */
    for (j=nsat=0;j<MAXSAT;j++) {
        if (satsys(j+1,&prn)!=sys||!nav->ssr[j].update) continue;
        nsat++;
        udint=nav->ssr[j].udi[5];
        iod  =nav->ssr[j].iod[5];
    }
    /* encode SSR header */
    i=encode_ssr_head(7,rtcm,sys,subtype,nsat,sync,iod,udint,0,0,0);
    
    for (j=nsat=0;j<MAXSAT;j++) {
        if (satsys(j+1,&prn)!=sys||!nav->ssr[j].update) continue;
        
        for (k=nbias=0;k<32;k++) {
            if (!codes[k]||nav->ssr[j].pbias[codes[k]-1]==0.0) continue;
            code[nbias]=k;
            pbias[nbias  ]=ROUND(nav->ssr[j].pbias[codes[k]-1]/0.0001);
            stdpb[nbias++]=ROUND(nav->ssr[j].stdpb[codes[k]-1]/0.0001);
        }
        yaw_ang =ROUND(nav->ssr[j].yaw_ang /180.0* 256.0);
        yaw_rate=ROUND(nav->ssr[j].yaw_rate/180.0*8192.0);
        setbitu(rtcm->buff,i,np,prn-offp); i+=np; /* satellite ID */
        setbitu(rtcm->buff,i, 5,nbias);    i+= 5; /* number of code biases */
        setbitu(rtcm->buff,i, 9,yaw_ang);  i+= 9; /* yaw angle */
        setbits(rtcm->buff,i, 8,yaw_rate); i+= 8; /* yaw rate */
        
        for (k=0;k<nbias;k++) {
            setbitu(rtcm->buff,i, 5,code[k] ); i+= 5; /* signal indicator */
            setbitu(rtcm->buff,i, 1,0       ); i+= 1; /* integer-indicator */
            setbitu(rtcm->buff,i, 2,0       ); i+= 2; /* WL integer-indicator */
            setbitu(rtcm->buff,i, 4,0       ); i+= 4; /* discont counter */
            setbits(rtcm->buff,i,20,pbias[k]); i+=20; /* phase bias */
            if (subtype==0) {
                setbits(rtcm->buff,i,17,stdpb[k]); i+=17; /* std-dev ph-bias */
            }
        }
    }
    rtcm->nbit=i;
    return 1;
}
/* satellite no to MSM satellite ID ------------------------------------------*/
static int to_satid(int sys, int sat)
{
    int prn;
    
    if (satsys(sat,&prn)!=sys) return 0;
    
    if      (sys==SYS_QZS) prn-=MINPRNQZS-1;
    else if (sys==SYS_SBS) prn-=MINPRNSBS-1;
    
    return prn;
}
/* observation code to MSM signal ID -----------------------------------------*/
static int to_sigid(int sys, uint8_t code)
{
    const char **msm_sig;
    char *sig;
    int i;
    
    /* signal conversion for undefined signal by rtcm */
    if (sys==SYS_GPS) {
        if      (code==CODE_L1Y) code=CODE_L1P;
        else if (code==CODE_L1M) code=CODE_L1P;
        else if (code==CODE_L1N) code=CODE_L1P;
        else if (code==CODE_L2D) code=CODE_L2P;
        else if (code==CODE_L2Y) code=CODE_L2P;
        else if (code==CODE_L2M) code=CODE_L2P;
        else if (code==CODE_L2N) code=CODE_L2P;
    }
    if (!*(sig=code2obs(code))) return 0;
    
    switch (sys) {
        case SYS_GPS: msm_sig=msm_sig_gps; break;
        case SYS_GLO: msm_sig=msm_sig_glo; break;
        case SYS_GAL: msm_sig=msm_sig_gal; break;
        case SYS_QZS: msm_sig=msm_sig_qzs; break;
        case SYS_SBS: msm_sig=msm_sig_sbs; break;
        case SYS_CMP: msm_sig=msm_sig_cmp; break;
        case SYS_IRN: msm_sig=msm_sig_irn; break;
        default: return 0;
    }
    for (i=0;i<32;i++) {
        if (!strcmp(sig,msm_sig[i])) return i+1;
    }
    return 0;
}
/* generate MSM satellite, signal and cell index -----------------------------*/
static void gen_msm_index(rtcm_t *rtcm, int sys, int *nsat, int *nsig,
                          int *ncell, uint8_t *sat_ind, uint8_t *sig_ind,
                          uint8_t *cell_ind)
{
    int i,j,sat,sig,cell;
    
    *nsat=*nsig=*ncell=0;
    
    /* generate satellite and signal index */
    for (i=0;i<rtcm->obs.n;i++) {
        if (!(sat=to_satid(sys,rtcm->obs.data[i].sat))) continue;
        
        for (j=0;j<NFREQ+NEXOBS;j++) {
            if (!(sig=to_sigid(sys,rtcm->obs.data[i].code[j]))) continue;
            
            sat_ind[sat-1]=sig_ind[sig-1]=1;
        }
    }
    for (i=0;i<64;i++) {
        if (sat_ind[i]) sat_ind[i]=++(*nsat);
    }
    for (i=0;i<32;i++) {
        if (sig_ind[i]) sig_ind[i]=++(*nsig);
    }
    /* generate cell index */
    for (i=0;i<rtcm->obs.n;i++) {
        if (!(sat=to_satid(sys,rtcm->obs.data[i].sat))) continue;
        
        for (j=0;j<NFREQ+NEXOBS;j++) {
            if (!(sig=to_sigid(sys,rtcm->obs.data[i].code[j]))) continue;
            
            cell=sig_ind[sig-1]-1+(sat_ind[sat-1]-1)*(*nsig);
            cell_ind[cell]=1;
        }
    }
    for (i=0;i<*nsat*(*nsig);i++) {
        if (cell_ind[i]&&*ncell<64) cell_ind[i]=++(*ncell);
    }
}
/* generate MSM satellite data fields ----------------------------------------*/
static void gen_msm_sat(rtcm_t *rtcm, nav_t *nav, int sys, int nsat, const uint8_t *sat_ind,
                        double *rrng, double *rrate, uint8_t *info)
{
    obsd_t *data;
    double freq;
    int i,j,k,sat,sig,fcn;
    
    for (i=0;i<64;i++) rrng[i]=rrate[i]=0.0;
    
    for (i=0;i<rtcm->obs.n;i++) {
        data=rtcm->obs.data+i;
        fcn=fcn_glo(data->sat,nav); /* fcn+7 */
        
        if (!(sat=to_satid(sys,data->sat))) continue;
        
        for (j=0;j<NFREQ+NEXOBS;j++) {
            if (!(sig=to_sigid(sys,data->code[j]))) continue;
            k=sat_ind[sat-1]-1;
            freq=code2freq(sys,data->code[j],fcn-7);
            
            /* rough range (ms) and rough phase-range-rate (m/s) */
            if (rrng[k]==0.0&&data->P[j]!=0.0) {
                rrng[k]=ROUND( data->P[j]/RANGE_MS/P2_10)*RANGE_MS*P2_10;
            }
            if (rrate[k]==0.0&&data->D[j]!=0.0&&freq>0.0) {
                rrate[k]=ROUND(-data->D[j]*CLIGHT/freq)*1.0;
            }
            /* extended satellite info */
            if (info) info[k]=sys!=SYS_GLO?0:(fcn<0?15:fcn);
        }
    }
}
/* generate MSM signal data fields -------------------------------------------*/
static void gen_msm_sig(rtcm_t *rtcm, nav_t *nav, int sys, int nsat, int nsig, int ncell,
                        const uint8_t *sat_ind, const uint8_t *sig_ind,
                        const uint8_t *cell_ind, const double *rrng,
                        const double *rrate, double *psrng, double *phrng,
                        double *rate, double *lock, uint8_t *half, float *cnr)
{
    obsd_t *data;
    double freq,lambda,psrng_s,phrng_s,rate_s,lt;
    int i,j,k,sat,sig,fcn,cell,LLI;
    
    for (i=0;i<ncell;i++) {
        if (psrng) psrng[i]=0.0;
        if (phrng) phrng[i]=0.0;
        if (rate ) rate [i]=0.0;
    }
    for (i=0;i<rtcm->obs.n;i++) {
        data=rtcm->obs.data+i;
        fcn=fcn_glo(data->sat,nav); /* fcn+7 */
        
        if (!(sat=to_satid(sys,data->sat))) continue;
        
        for (j=0;j<NFREQ+NEXOBS;j++) {
            if (!(sig=to_sigid(sys,data->code[j]))) continue;
            
            k=sat_ind[sat-1]-1;
            if ((cell=cell_ind[sig_ind[sig-1]-1+k*nsig])>=64) continue;
            
            freq=code2freq(sys,data->code[j],fcn-7);
            lambda=freq==0.0?0.0:CLIGHT/freq;
            psrng_s=data->P[j]==0.0?0.0:data->P[j]-rrng[k];
            phrng_s=data->L[j]==0.0||lambda<=0.0?0.0: data->L[j]*lambda-rrng [k];
            rate_s =data->D[j]==0.0||lambda<=0.0?0.0:-data->D[j]*lambda-rrate[k];
            
            /* subtract phase - psudorange integer cycle offset */
            LLI=data->LLI[j];
            if ((LLI&1)||fabs(phrng_s-rtcm->cp[data->sat-1][j])>1171.0) {
                rtcm->cp[data->sat-1][j]=ROUND(phrng_s/lambda)*lambda;
                LLI|=1;
            }
            phrng_s-=rtcm->cp[data->sat-1][j];
            
            lt=locktime_d(data->time,rtcm->lltime[data->sat-1]+j,LLI);
            
            if (psrng&&psrng_s!=0.0) psrng[cell-1]=psrng_s;
            if (phrng&&phrng_s!=0.0) phrng[cell-1]=phrng_s;
            if (rate &&rate_s !=0.0) rate [cell-1]=rate_s;
            if (lock) lock[cell-1]=lt;
            if (half) half[cell-1]=(data->LLI[j]&2)?1:0;
            if (cnr ) cnr [cell-1]=(float)(data->SNR[j]*SNR_UNIT);
        }
    }
}
/* encode MSM header ---------------------------------------------------------*/
static int encode_msm_head(int type, rtcm_t *rtcm, nav_t *nav, int sys, int sync, int *nsat,
                           int *ncell, double *rrng, double *rrate,
                           uint8_t *info, double *psrng, double *phrng,
                           double *rate, double *lock, uint8_t *half,
                           float *cnr)
{
    double tow;
    uint8_t sat_ind[64]={0},sig_ind[32]={0},cell_ind[32*64]={0};
    uint32_t dow,epoch;
    int i=24,j,nsig=0;
    
    switch (sys) {
        case SYS_GPS: type+=1070; break;
        case SYS_GLO: type+=1080; break;
        case SYS_GAL: type+=1090; break;
        case SYS_QZS: type+=1110; break;
        case SYS_SBS: type+=1100; break;
        case SYS_CMP: type+=1120; break;
        case SYS_IRN: type+=1130; break;
        default: return 0;
    }
    /* generate msm satellite, signal and cell index */
    gen_msm_index(rtcm,sys,nsat,&nsig,ncell,sat_ind,sig_ind,cell_ind);
    
    if (sys==SYS_GLO) {
        /* GLONASS time (dow + tod-ms) */
        tow=time2gpst(timeadd(gpst2utc(rtcm->time),10800.0),NULL);
        dow=(uint32_t)(tow/86400.0);
        epoch=(dow<<27)+ROUND_U(fmod(tow,86400.0)*1E3);
    }
    else if (sys==SYS_CMP) {
        /* BDS time (tow-ms) */
        epoch=ROUND_U(time2gpst(gpst2bdt(rtcm->time),NULL)*1E3);
    }
    else {
        /* GPS, QZSS, Galileo and IRNSS time (tow-ms) */
        epoch=ROUND_U(time2gpst(rtcm->time,NULL)*1E3);
    }
    /* encode msm header (ref [15] table 3.5-78) */
    setbitu(rtcm->buff,i,12,type       ); i+=12; /* message number */
    setbitu(rtcm->buff,i,12,rtcm->staid); i+=12; /* reference station id */
    setbitu(rtcm->buff,i,30,epoch      ); i+=30; /* epoch time */
    setbitu(rtcm->buff,i, 1,sync       ); i+= 1; /* multiple message bit */
    setbitu(rtcm->buff,i, 3,rtcm->seqno); i+= 3; /* issue of data station */
    setbitu(rtcm->buff,i, 7,0          ); i+= 7; /* reserved */
    setbitu(rtcm->buff,i, 2,0          ); i+= 2; /* clock streering indicator */
    setbitu(rtcm->buff,i, 2,0          ); i+= 2; /* external clock indicator */
    setbitu(rtcm->buff,i, 1,0          ); i+= 1; /* smoothing indicator */
    setbitu(rtcm->buff,i, 3,0          ); i+= 3; /* smoothing interval */
    
    /* satellite mask */
    for (j=0;j<64;j++) {
        setbitu(rtcm->buff,i,1,sat_ind[j]?1:0); i+=1;
    }
    /* signal mask */
    for (j=0;j<32;j++) {
        setbitu(rtcm->buff,i,1,sig_ind[j]?1:0); i+=1;
    }
    /* cell mask */
    for (j=0;j<*nsat*nsig&&j<64;j++) {
        setbitu(rtcm->buff,i,1,cell_ind[j]?1:0); i+=1;
    }
    /* generate msm satellite data fields */
    gen_msm_sat(rtcm,nav,sys,*nsat,sat_ind,rrng,rrate,info);
    
    /* generate msm signal data fields */
    gen_msm_sig(rtcm,nav,sys,*nsat,nsig,*ncell,sat_ind,sig_ind,cell_ind,rrng,rrate,
                psrng,phrng,rate,lock,half,cnr);
    
    return i;
}
/* encode rough range integer ms ---------------------------------------------*/
static int encode_msm_int_rrng(rtcm_t *rtcm, int i, const double *rrng,
                               int nsat)
{
    uint32_t int_ms;
    int j;
    
    for (j=0;j<nsat;j++) {
        if (rrng[j]==0.0) {
            int_ms=255;
        }
        else if (rrng[j]<0.0||rrng[j]>RANGE_MS*255.0) {
            trace(2,"msm rough range overflow %s rrng=%.3f\n",
                 time_str(rtcm->time,0),rrng[j]);
            int_ms=255;
        }
        else {
            int_ms=ROUND_U(rrng[j]/RANGE_MS/P2_10)>>10;
        }
        setbitu(rtcm->buff,i,8,int_ms); i+=8;
    }
    return i;
}
/* encode rough range modulo 1 ms --------------------------------------------*/
static int encode_msm_mod_rrng(rtcm_t *rtcm, int i, const double *rrng,
                               int nsat)
{
    uint32_t mod_ms;
    int j;
    
    for (j=0;j<nsat;j++) {
        if (rrng[j]<=0.0||rrng[j]>RANGE_MS*255.0) {
            mod_ms=0;
        }
        else {
            mod_ms=ROUND_U(rrng[j]/RANGE_MS/P2_10)&0x3FFu;
        }
        setbitu(rtcm->buff,i,10,mod_ms); i+=10;
    }
    return i;
}
/* encode extended satellite info --------------------------------------------*/
static int encode_msm_info(rtcm_t *rtcm, int i, const uint8_t *info, int nsat)
{
    int j;
    
    for (j=0;j<nsat;j++) {
        setbitu(rtcm->buff,i,4,info[j]); i+=4;
    }
    return i;
}
/* encode rough phase-range-rate ---------------------------------------------*/
static int encode_msm_rrate(rtcm_t *rtcm, int i, const double *rrate, int nsat)
{
    int j,rrate_val;
    
    for (j=0;j<nsat;j++) {
        if (fabs(rrate[j])>8191.0) {
            trace(2,"msm rough phase-range-rate overflow %s rrate=%.4f\n",
                 time_str(rtcm->time,0),rrate[j]);
            rrate_val=-8192;
        }
        else {
            rrate_val=ROUND(rrate[j]/1.0);
        }
        setbits(rtcm->buff,i,14,rrate_val); i+=14;
    }
    return i;
}
/* encode fine pseudorange ---------------------------------------------------*/
static int encode_msm_psrng(rtcm_t *rtcm, int i, const double *psrng, int ncell)
{
    int j,psrng_val;
    
    for (j=0;j<ncell;j++) {
        if (psrng[j]==0.0) {
            psrng_val=-16384;
        }
        else if (fabs(psrng[j])>292.7) {
            trace(2,"msm fine pseudorange overflow %s psrng=%.3f\n",
                 time_str(rtcm->time,0),psrng[j]);
            psrng_val=-16384;
        }
        else {
            psrng_val=ROUND(psrng[j]/RANGE_MS/P2_24);
        }
        setbits(rtcm->buff,i,15,psrng_val); i+=15;
    }
    return i;
}
/* encode fine pseudorange with extended resolution --------------------------*/
static int encode_msm_psrng_ex(rtcm_t *rtcm, int i, const double *psrng,
                               int ncell)
{
    int j,psrng_val;
    
    for (j=0;j<ncell;j++) {
        if (psrng[j]==0.0) {
            psrng_val=-524288;
        }
        else if (fabs(psrng[j])>292.7) {
            trace(2,"msm fine pseudorange ext overflow %s psrng=%.3f\n",
                 time_str(rtcm->time,0),psrng[j]);
            psrng_val=-524288;
        }
        else {
            psrng_val=ROUND(psrng[j]/RANGE_MS/P2_29);
        }
        setbits(rtcm->buff,i,20,psrng_val); i+=20;
    }
    return i;
}
/* encode fine phase-range ---------------------------------------------------*/
static int encode_msm_phrng(rtcm_t *rtcm, int i, const double *phrng, int ncell)
{
    int j,phrng_val;
    
    for (j=0;j<ncell;j++) {
        if (phrng[j]==0.0) {
            phrng_val=-2097152;
        }
        else if (fabs(phrng[j])>1171.0) {
            trace(2,"msm fine phase-range overflow %s phrng=%.3f\n",
                 time_str(rtcm->time,0),phrng[j]);
            phrng_val=-2097152;
        }
        else {
            phrng_val=ROUND(phrng[j]/RANGE_MS/P2_29);
        }
        setbits(rtcm->buff,i,22,phrng_val); i+=22;
    }
    return i;
}
/* encode fine phase-range with extended resolution --------------------------*/
static int encode_msm_phrng_ex(rtcm_t *rtcm, int i, const double *phrng,
                               int ncell)
{
    int j,phrng_val;
    
    for (j=0;j<ncell;j++) {
        if (phrng[j]==0.0) {
            phrng_val=-8388608;
        }
        else if (fabs(phrng[j])>1171.0) {
            trace(2,"msm fine phase-range ext overflow %s phrng=%.3f\n",
                 time_str(rtcm->time,0),phrng[j]);
            phrng_val=-8388608;
        }
        else {
            phrng_val=ROUND(phrng[j]/RANGE_MS/P2_31);
        }
        setbits(rtcm->buff,i,24,phrng_val); i+=24;
    }
    return i;
}
/* encode lock-time indicator ------------------------------------------------*/
static int encode_msm_lock(rtcm_t *rtcm, int i, const double *lock, int ncell)
{
    int j,lock_val;
    
    for (j=0;j<ncell;j++) {
        lock_val=to_msm_lock(lock[j]);
        setbitu(rtcm->buff,i,4,lock_val); i+=4;
    }
    return i;
}
/* encode lock-time indicator with extended range and resolution -------------*/
static int encode_msm_lock_ex(rtcm_t *rtcm, int i, const double *lock,
                              int ncell)
{
    int j,lock_val;
    
    for (j=0;j<ncell;j++) {
        lock_val=to_msm_lock_ex(lock[j]);
        setbitu(rtcm->buff,i,10,lock_val); i+=10;
    }
    return i;
}
/* encode half-cycle-ambiguity indicator -------------------------------------*/
static int encode_msm_half_amb(rtcm_t *rtcm, int i, const uint8_t *half,
                               int ncell)
{
    int j;
    
    for (j=0;j<ncell;j++) {
        setbitu(rtcm->buff,i,1,half[j]); i+=1;
    }
    return i;
}
/* encode signal CNR ---------------------------------------------------------*/
static int encode_msm_cnr(rtcm_t *rtcm, int i, const float *cnr, int ncell)
{
    int j,cnr_val;
    
    for (j=0;j<ncell;j++) {
        cnr_val=ROUND(cnr[j]/1.0);
        setbitu(rtcm->buff,i,6,cnr_val); i+=6;
    }
    return i;
}
/* encode signal CNR with extended resolution --------------------------------*/
static int encode_msm_cnr_ex(rtcm_t *rtcm, int i, const float *cnr, int ncell)
{
    int j,cnr_val;
    
    for (j=0;j<ncell;j++) {
        cnr_val=ROUND(cnr[j]/0.0625);
        setbitu(rtcm->buff,i,10,cnr_val); i+=10;
    }
    return i;
}
/* encode fine phase-range-rate ----------------------------------------------*/
static int encode_msm_rate(rtcm_t *rtcm, int i, const double *rate, int ncell)
{
    int j,rate_val;
    
    for (j=0;j<ncell;j++) {
        if (rate[j]==0.0) {
            rate_val=-16384;
        }
        else if (fabs(rate[j])>1.6384) {
            trace(2,"msm fine phase-range-rate overflow %s rate=%.3f\n",
                 time_str(rtcm->time,0),rate[j]);
            rate_val=-16384;
        }
        else {
            rate_val=ROUND(rate[j]/0.0001);
        }
        setbitu(rtcm->buff,i,15,rate_val); i+=15;
    }
    return i;
}
/* encode MSM 1: compact pseudorange -----------------------------------------*/
static int encode_msm1(rtcm_t *rtcm, nav_t* nav, int sys, int sync)
{
    double rrng[64],rrate[64],psrng[64];
    int i,nsat,ncell;
    
    trace(3,"encode_msm1: sys=%d sync=%d\n",sys,sync);
    
    /* encode msm header */
    if (!(i=encode_msm_head(1,rtcm,nav,sys,sync,&nsat,&ncell,rrng,rrate,NULL,psrng,
                            NULL,NULL,NULL,NULL,NULL))) {
        return 0;
    }
    /* encode msm satellite data */
    i=encode_msm_mod_rrng(rtcm,i,rrng ,nsat ); /* rough range modulo 1 ms */
    
    /* encode msm signal data */
    i=encode_msm_psrng   (rtcm,i,psrng,ncell); /* fine pseudorange */
    
    rtcm->nbit=i;
    return 1;
}
/* encode MSM 2: compact phaserange ------------------------------------------*/
static int encode_msm2(rtcm_t *rtcm, nav_t* nav, int sys, int sync)
{
    double rrng[64],rrate[64],phrng[64],lock[64];
    uint8_t half[64];
    int i,nsat,ncell;
    
    trace(3,"encode_msm2: sys=%d sync=%d\n",sys,sync);
    
    /* encode msm header */
    if (!(i=encode_msm_head(2,rtcm,nav,sys,sync,&nsat,&ncell,rrng,rrate,NULL,NULL,
                            phrng,NULL,lock,half,NULL))) {
        return 0;
    }
    /* encode msm satellite data */
    i=encode_msm_mod_rrng(rtcm,i,rrng ,nsat ); /* rough range modulo 1 ms */
    
    /* encode msm signal data */
    i=encode_msm_phrng   (rtcm,i,phrng,ncell); /* fine phase-range */
    i=encode_msm_lock    (rtcm,i,lock ,ncell); /* lock-time indicator */
    i=encode_msm_half_amb(rtcm,i,half ,ncell); /* half-cycle-amb indicator */
    
    rtcm->nbit=i;
    return 1;
}
/* encode MSM 3: compact pseudorange and phaserange --------------------------*/
static int encode_msm3(rtcm_t *rtcm, nav_t* nav, int sys, int sync)
{
    double rrng[64],rrate[64],psrng[64],phrng[64],lock[64];
    uint8_t half[64];
    int i,nsat,ncell;
    
    trace(3,"encode_msm3: sys=%d sync=%d\n",sys,sync);
    
    /* encode msm header */
    if (!(i=encode_msm_head(3,rtcm,nav,sys,sync,&nsat,&ncell,rrng,rrate,NULL,psrng,
                            phrng,NULL,lock,half,NULL))) {
        return 0;
    }
    /* encode msm satellite data */
    i=encode_msm_mod_rrng(rtcm,i,rrng ,nsat ); /* rough range modulo 1 ms */
    
    /* encode msm signal data */
    i=encode_msm_psrng   (rtcm,i,psrng,ncell); /* fine pseudorange */
    i=encode_msm_phrng   (rtcm,i,phrng,ncell); /* fine phase-range */
    i=encode_msm_lock    (rtcm,i,lock ,ncell); /* lock-time indicator */
    i=encode_msm_half_amb(rtcm,i,half ,ncell); /* half-cycle-amb indicator */
    
    rtcm->nbit=i;
    return 1;
}
/* encode MSM 4: full pseudorange and phaserange plus CNR --------------------*/
static int encode_msm4(rtcm_t *rtcm, nav_t* nav, int sys, int sync)
{
    double rrng[64],rrate[64],psrng[64],phrng[64],lock[64];
    float cnr[64];
    uint8_t half[64];
    int i,nsat,ncell;
    
    trace(3,"encode_msm4: sys=%d sync=%d\n",sys,sync);
    
    /* encode msm header */
    if (!(i=encode_msm_head(4,rtcm,nav,sys,sync,&nsat,&ncell,rrng,rrate,NULL,psrng,
                            phrng,NULL,lock,half,cnr))) {
        return 0;
    }
    /* encode msm satellite data */
    i=encode_msm_int_rrng(rtcm,i,rrng ,nsat ); /* rough range integer ms */
    i=encode_msm_mod_rrng(rtcm,i,rrng ,nsat ); /* rough range modulo 1 ms */
    
    /* encode msm signal data */
    i=encode_msm_psrng   (rtcm,i,psrng,ncell); /* fine pseudorange */
    i=encode_msm_phrng   (rtcm,i,phrng,ncell); /* fine phase-range */
    i=encode_msm_lock    (rtcm,i,lock ,ncell); /* lock-time indicator */
    i=encode_msm_half_amb(rtcm,i,half ,ncell); /* half-cycle-amb indicator */
    i=encode_msm_cnr     (rtcm,i,cnr  ,ncell); /* signal cnr */
    rtcm->nbit=i;
    return 1;
}
/* encode MSM 5: full pseudorange, phaserange, phaserangerate and CNR --------*/
static int encode_msm5(rtcm_t *rtcm, nav_t* nav, int sys, int sync)
{
    double rrng[64],rrate[64],psrng[64],phrng[64],rate[64],lock[64];
    float cnr[64];
    uint8_t info[64],half[64];
    int i,nsat,ncell;
    
    trace(3,"encode_msm5: sys=%d sync=%d\n",sys,sync);
    
    /* encode msm header */
    if (!(i=encode_msm_head(5,rtcm,nav,sys,sync,&nsat,&ncell,rrng,rrate,info,psrng,
                            phrng,rate,lock,half,cnr))) {
        return 0;
    }
    /* encode msm satellite data */
    i=encode_msm_int_rrng(rtcm,i,rrng ,nsat ); /* rough range integer ms */
    i=encode_msm_info    (rtcm,i,info ,nsat ); /* extended satellite info */
    i=encode_msm_mod_rrng(rtcm,i,rrng ,nsat ); /* rough range modulo 1 ms */
    i=encode_msm_rrate   (rtcm,i,rrate,nsat ); /* rough phase-range-rate */
    
    /* encode msm signal data */
    i=encode_msm_psrng   (rtcm,i,psrng,ncell); /* fine pseudorange */
    i=encode_msm_phrng   (rtcm,i,phrng,ncell); /* fine phase-range */
    i=encode_msm_lock    (rtcm,i,lock ,ncell); /* lock-time indicator */
    i=encode_msm_half_amb(rtcm,i,half ,ncell); /* half-cycle-amb indicator */
    i=encode_msm_cnr     (rtcm,i,cnr  ,ncell); /* signal cnr */
    i=encode_msm_rate    (rtcm,i,rate ,ncell); /* fine phase-range-rate */
    rtcm->nbit=i;
    return 1;
}
/* encode MSM 6: full pseudorange and phaserange plus CNR (high-res) ---------*/
static int encode_msm6(rtcm_t *rtcm, nav_t* nav, int sys, int sync)
{
    double rrng[64],rrate[64],psrng[64],phrng[64],lock[64];
    float cnr[64];
    uint8_t half[64];
    int i,nsat,ncell;
    
    trace(3,"encode_msm6: sys=%d sync=%d\n",sys,sync);
    
    /* encode msm header */
    if (!(i=encode_msm_head(6,rtcm,nav,sys,sync,&nsat,&ncell,rrng,rrate,NULL,psrng,
                            phrng,NULL,lock,half,cnr))) {
        return 0;
    }
    /* encode msm satellite data */
    i=encode_msm_int_rrng(rtcm,i,rrng ,nsat ); /* rough range integer ms */
    i=encode_msm_mod_rrng(rtcm,i,rrng ,nsat ); /* rough range modulo 1 ms */
    
    /* encode msm signal data */
    i=encode_msm_psrng_ex(rtcm,i,psrng,ncell); /* fine pseudorange ext */
    i=encode_msm_phrng_ex(rtcm,i,phrng,ncell); /* fine phase-range ext */
    i=encode_msm_lock_ex (rtcm,i,lock ,ncell); /* lock-time indicator ext */
    i=encode_msm_half_amb(rtcm,i,half ,ncell); /* half-cycle-amb indicator */
    i=encode_msm_cnr_ex  (rtcm,i,cnr  ,ncell); /* signal cnr ext */
    rtcm->nbit=i;
    return 1;
}
/* encode MSM 7: full pseudorange, phaserange, phaserangerate and CNR (h-res) */
static int encode_msm7(rtcm_t *rtcm, nav_t* nav, int sys, int sync)
{
    double rrng[64],rrate[64],psrng[64],phrng[64],rate[64],lock[64];
    float cnr[64];
    uint8_t info[64],half[64];
    int i,nsat,ncell;
    
    trace(3,"encode_msm7: sys=%d sync=%d\n",sys,sync);
    
    /* encode msm header */
    if (!(i=encode_msm_head(7,rtcm,nav,sys,sync,&nsat,&ncell,rrng,rrate,info,psrng,
                            phrng,rate,lock,half,cnr))) {
        return 0;
    }
    /* encode msm satellite data */
    i=encode_msm_int_rrng(rtcm,i,rrng ,nsat ); /* rough range integer ms */
    i=encode_msm_info    (rtcm,i,info ,nsat ); /* extended satellite info */
    i=encode_msm_mod_rrng(rtcm,i,rrng ,nsat ); /* rough range modulo 1 ms */
    i=encode_msm_rrate   (rtcm,i,rrate,nsat ); /* rough phase-range-rate */
    
    /* encode msm signal data */
    i=encode_msm_psrng_ex(rtcm,i,psrng,ncell); /* fine pseudorange ext */
    i=encode_msm_phrng_ex(rtcm,i,phrng,ncell); /* fine phase-range ext */
    i=encode_msm_lock_ex (rtcm,i,lock ,ncell); /* lock-time indicator ext */
    i=encode_msm_half_amb(rtcm,i,half ,ncell); /* half-cycle-amb indicator */
    i=encode_msm_cnr_ex  (rtcm,i,cnr  ,ncell); /* signal cnr ext */
    i=encode_msm_rate    (rtcm,i,rate ,ncell); /* fine phase-range-rate */
    rtcm->nbit=i;
    return 1;
}
/* encode type 1230: GLONASS L1 and L2 code-phase biases ---------------------*/
static int encode_type1230(rtcm_t *rtcm, int sync)
{
    int i=24,j,align,mask=15,bias[4];
    
    trace(3,"encode_type1230: sync=%d\n",sync);
    
    align=rtcm->sta.glo_cp_align;
    
    for (j=0;j<4;j++) {
        bias[j]=ROUND(rtcm->sta.glo_cp_bias[j]/0.02);
        if (bias[j]<=-32768||bias[j]>32767) {
            bias[j]=-32768; /* invalid value */
        }
    }
    setbitu(rtcm->buff,i,12,1230       ); i+=12; /* message no */
    setbitu(rtcm->buff,i,12,rtcm->staid); i+=12; /* station ID */
    setbitu(rtcm->buff,i, 1,align      ); i+= 1; /* GLO code-phase bias ind */
    setbitu(rtcm->buff,i, 3,0          ); i+= 3; /* reserved */
    setbitu(rtcm->buff,i, 4,mask       ); i+= 4; /* GLO FDMA signals mask */
    setbits(rtcm->buff,i,16,bias[0]    ); i+=16; /* GLO C1 code-phase bias */
    setbits(rtcm->buff,i,16,bias[1]    ); i+=16; /* GLO P1 code-phase bias */
    setbits(rtcm->buff,i,16,bias[2]    ); i+=16; /* GLO C2 code-phase bias */
    setbits(rtcm->buff,i,16,bias[3]    ); i+=16; /* GLO P2 code-phase bias */
    rtcm->nbit=i;
    return 1;
}
/* encode type 4073: proprietary message Mitsubishi Electric -----------------*/
static int encode_type4073(rtcm_t *rtcm, nav_t *nav, int subtype, int sync)
{
    trace(2,"rtcm3 4073: unsupported message subtype=%d\n",subtype);
    return 0;
}
/* encode type 4076: proprietary message IGS ---------------------------------*/
static int encode_type4076(rtcm_t *rtcm, nav_t *nav, int subtype, int sync)
{
    switch (subtype) {
        case  21: return encode_ssr1(rtcm,nav,SYS_GPS,subtype,sync);
        case  22: return encode_ssr2(rtcm,nav,SYS_GPS,subtype,sync);
        case  23: return encode_ssr4(rtcm,nav,SYS_GPS,subtype,sync);
        case  24: return encode_ssr6(rtcm,nav,SYS_GPS,subtype,sync);
        case  25: return encode_ssr3(rtcm,nav,SYS_GPS,subtype,sync);
        case  26: return encode_ssr7(rtcm,nav,SYS_GPS,subtype,sync);
        case  27: return encode_ssr5(rtcm,nav,SYS_GPS,subtype,sync);
        case  41: return encode_ssr1(rtcm,nav,SYS_GLO,subtype,sync);
        case  42: return encode_ssr2(rtcm,nav,SYS_GLO,subtype,sync);
        case  43: return encode_ssr4(rtcm,nav,SYS_GLO,subtype,sync);
        case  44: return encode_ssr6(rtcm,nav,SYS_GLO,subtype,sync);
        case  45: return encode_ssr3(rtcm,nav,SYS_GLO,subtype,sync);
        case  46: return encode_ssr7(rtcm,nav,SYS_GLO,subtype,sync);
        case  47: return encode_ssr5(rtcm,nav,SYS_GLO,subtype,sync);
        case  61: return encode_ssr1(rtcm,nav,SYS_GAL,subtype,sync);
        case  62: return encode_ssr2(rtcm,nav,SYS_GAL,subtype,sync);
        case  63: return encode_ssr4(rtcm,nav,SYS_GAL,subtype,sync);
        case  64: return encode_ssr6(rtcm,nav,SYS_GAL,subtype,sync);
        case  65: return encode_ssr3(rtcm,nav,SYS_GAL,subtype,sync);
        case  66: return encode_ssr7(rtcm,nav,SYS_GAL,subtype,sync);
        case  67: return encode_ssr5(rtcm,nav,SYS_GAL,subtype,sync);
        case  81: return encode_ssr1(rtcm,nav,SYS_QZS,subtype,sync);
        case  82: return encode_ssr2(rtcm,nav,SYS_QZS,subtype,sync);
        case  83: return encode_ssr4(rtcm,nav,SYS_QZS,subtype,sync);
        case  84: return encode_ssr6(rtcm,nav,SYS_QZS,subtype,sync);
        case  85: return encode_ssr3(rtcm,nav,SYS_QZS,subtype,sync);
        case  86: return encode_ssr7(rtcm,nav,SYS_QZS,subtype,sync);
        case  87: return encode_ssr5(rtcm,nav,SYS_QZS,subtype,sync);
        case 101: return encode_ssr1(rtcm,nav,SYS_CMP,subtype,sync);
        case 102: return encode_ssr2(rtcm,nav,SYS_CMP,subtype,sync);
        case 103: return encode_ssr4(rtcm,nav,SYS_CMP,subtype,sync);
        case 104: return encode_ssr6(rtcm,nav,SYS_CMP,subtype,sync);
        case 105: return encode_ssr3(rtcm,nav,SYS_CMP,subtype,sync);
        case 106: return encode_ssr7(rtcm,nav,SYS_CMP,subtype,sync);
        case 107: return encode_ssr5(rtcm,nav,SYS_CMP,subtype,sync);
        case 121: return encode_ssr1(rtcm,nav,SYS_SBS,subtype,sync);
        case 122: return encode_ssr2(rtcm,nav,SYS_SBS,subtype,sync);
        case 123: return encode_ssr4(rtcm,nav,SYS_SBS,subtype,sync);
        case 124: return encode_ssr6(rtcm,nav,SYS_SBS,subtype,sync);
        case 125: return encode_ssr3(rtcm,nav,SYS_SBS,subtype,sync);
        case 126: return encode_ssr7(rtcm,nav,SYS_SBS,subtype,sync);
        case 127: return encode_ssr5(rtcm,nav,SYS_SBS,subtype,sync);
    }
    trace(2,"rtcm3 4076: unsupported message subtype=%d\n",subtype);
    return 0;
}
/* encode RTCM ver.3 message -------------------------------------------------*/
extern int encode_rtcm3(rtcm_t *rtcm, nav_t *nav, int type, int subtype, int sync)
{
    int ret=0,prn=0;
	eph_t *eph=0;
	geph_t *geph=0;
	if (nav->ephsat>0)
	{
		if (type==1019)
		{
			if (satsys(nav->ephsat,&prn)!=SYS_GPS) return 0;
		    eph=nav->eph+nav->ephsat-1;
		}
		else if (type==1020)
		{
			if (satsys(nav->ephsat,&prn)!=SYS_GLO) return 0;
			geph=nav->geph+prn-1;
		}
		else if (type==1041)
		{
			if (satsys(nav->ephsat,&prn)!=SYS_IRN) return 0;
		    eph=nav->eph+nav->ephsat-1;
		}
		else if (type==1042||type==63)
		{
			if (satsys(nav->ephsat,&prn)!=SYS_CMP) return 0;
		    eph=nav->eph+nav->ephsat-1;
		}
		else if (type==1044)
		{
			if (satsys(nav->ephsat,&prn)!=SYS_QZS) return 0;
		    eph=nav->eph+nav->ephsat-1;
		}
		else if (type==1045)
		{
			if (satsys(nav->ephsat,&prn)!=SYS_GAL) return 0;
		    eph=nav->eph+nav->ephsat-1+MAXSAT; /* F/NAV */
		}
		else if (type==1046)
		{
			if (satsys(nav->ephsat,&prn)!=SYS_GAL) return 0;
		    eph=nav->eph+nav->ephsat-1+MAXSAT; /* I/NAV */
		}
	}

    trace(3,"encode_rtcm3: type=%d subtype=%d sync=%d\n",type,subtype,sync);
    
    switch (type) {
        case 1001: ret=encode_type1001(rtcm,sync);     break;
        case 1002: ret=encode_type1002(rtcm,sync);     break;
        case 1003: ret=encode_type1003(rtcm,sync);     break;
        case 1004: ret=encode_type1004(rtcm,sync);     break;
        case 1005: ret=encode_type1005(rtcm,sync);     break;
        case 1006: ret=encode_type1006(rtcm,sync);     break;
        case 1007: ret=encode_type1007(rtcm,sync);     break;
        case 1008: ret=encode_type1008(rtcm,sync);     break;
        case 1009: ret=encode_type1009(rtcm,nav,sync); break;
        case 1010: ret=encode_type1010(rtcm,nav,sync); break;
        case 1011: ret=encode_type1011(rtcm,nav,sync); break;
        case 1012: ret=encode_type1012(rtcm,nav,sync); break;
        case 1019: ret=encode_type1019(rtcm,eph,sync); break;
        case 1020: ret=encode_type1020(rtcm,geph,sync);break;
        case 1033: ret=encode_type1033(rtcm,sync);     break;
        case 1041: ret=encode_type1041(rtcm,eph,sync); break;
        case 1042: ret=encode_type1042(rtcm,eph,sync); break;
        case 1044: ret=encode_type1044(rtcm,eph,sync); break;
        case 1045: ret=encode_type1045(rtcm,eph,sync); break;
        case 1046: ret=encode_type1046(rtcm,eph,sync); break;
        case   63: ret=encode_type63  (rtcm,eph,sync); break; /* draft */
        case 1057: ret=encode_ssr1(rtcm,nav,SYS_GPS,0,sync); break;
        case 1058: ret=encode_ssr2(rtcm,nav,SYS_GPS,0,sync); break;
        case 1059: ret=encode_ssr3(rtcm,nav,SYS_GPS,0,sync); break;
        case 1060: ret=encode_ssr4(rtcm,nav,SYS_GPS,0,sync); break;
        case 1061: ret=encode_ssr5(rtcm,nav,SYS_GPS,0,sync); break;
        case 1062: ret=encode_ssr6(rtcm,nav,SYS_GPS,0,sync); break;
        case 1063: ret=encode_ssr1(rtcm,nav,SYS_GLO,0,sync); break;
        case 1064: ret=encode_ssr2(rtcm,nav,SYS_GLO,0,sync); break;
        case 1065: ret=encode_ssr3(rtcm,nav,SYS_GLO,0,sync); break;
        case 1066: ret=encode_ssr4(rtcm,nav,SYS_GLO,0,sync); break;
        case 1067: ret=encode_ssr5(rtcm,nav,SYS_GLO,0,sync); break;
        case 1068: ret=encode_ssr6(rtcm,nav,SYS_GLO,0,sync); break;
        case 1071: ret=encode_msm1(rtcm,nav,SYS_GPS,sync); break;
        case 1072: ret=encode_msm2(rtcm,nav,SYS_GPS,sync); break;
        case 1073: ret=encode_msm3(rtcm,nav,SYS_GPS,sync); break;
        case 1074: ret=encode_msm4(rtcm,nav,SYS_GPS,sync); break;
        case 1075: ret=encode_msm5(rtcm,nav,SYS_GPS,sync); break;
        case 1076: ret=encode_msm6(rtcm,nav,SYS_GPS,sync); break;
        case 1077: ret=encode_msm7(rtcm,nav,SYS_GPS,sync); break;
        case 1081: ret=encode_msm1(rtcm,nav,SYS_GLO,sync); break;
        case 1082: ret=encode_msm2(rtcm,nav,SYS_GLO,sync); break;
        case 1083: ret=encode_msm3(rtcm,nav,SYS_GLO,sync); break;
        case 1084: ret=encode_msm4(rtcm,nav,SYS_GLO,sync); break;
        case 1085: ret=encode_msm5(rtcm,nav,SYS_GLO,sync); break;
        case 1086: ret=encode_msm6(rtcm,nav,SYS_GLO,sync); break;
        case 1087: ret=encode_msm7(rtcm,nav,SYS_GLO,sync); break;
        case 1091: ret=encode_msm1(rtcm,nav,SYS_GAL,sync); break;
        case 1092: ret=encode_msm2(rtcm,nav,SYS_GAL,sync); break;
        case 1093: ret=encode_msm3(rtcm,nav,SYS_GAL,sync); break;
        case 1094: ret=encode_msm4(rtcm,nav,SYS_GAL,sync); break;
        case 1095: ret=encode_msm5(rtcm,nav,SYS_GAL,sync); break;
        case 1096: ret=encode_msm6(rtcm,nav,SYS_GAL,sync); break;
        case 1097: ret=encode_msm7(rtcm,nav,SYS_GAL,sync); break;
        case 1101: ret=encode_msm1(rtcm,nav,SYS_SBS,sync); break;
        case 1102: ret=encode_msm2(rtcm,nav,SYS_SBS,sync); break;
        case 1103: ret=encode_msm3(rtcm,nav,SYS_SBS,sync); break;
        case 1104: ret=encode_msm4(rtcm,nav,SYS_SBS,sync); break;
        case 1105: ret=encode_msm5(rtcm,nav,SYS_SBS,sync); break;
        case 1106: ret=encode_msm6(rtcm,nav,SYS_SBS,sync); break;
        case 1107: ret=encode_msm7(rtcm,nav,SYS_SBS,sync); break;
        case 1111: ret=encode_msm1(rtcm,nav,SYS_QZS,sync); break;
        case 1112: ret=encode_msm2(rtcm,nav,SYS_QZS,sync); break;
        case 1113: ret=encode_msm3(rtcm,nav,SYS_QZS,sync); break;
        case 1114: ret=encode_msm4(rtcm,nav,SYS_QZS,sync); break;
        case 1115: ret=encode_msm5(rtcm,nav,SYS_QZS,sync); break;
        case 1116: ret=encode_msm6(rtcm,nav,SYS_QZS,sync); break;
        case 1117: ret=encode_msm7(rtcm,nav,SYS_QZS,sync); break;
        case 1121: ret=encode_msm1(rtcm,nav,SYS_CMP,sync); break;
        case 1122: ret=encode_msm2(rtcm,nav,SYS_CMP,sync); break;
        case 1123: ret=encode_msm3(rtcm,nav,SYS_CMP,sync); break;
        case 1124: ret=encode_msm4(rtcm,nav,SYS_CMP,sync); break;
        case 1125: ret=encode_msm5(rtcm,nav,SYS_CMP,sync); break;
        case 1126: ret=encode_msm6(rtcm,nav,SYS_CMP,sync); break;
        case 1127: ret=encode_msm7(rtcm,nav,SYS_CMP,sync); break;
        case 1131: ret=encode_msm1(rtcm,nav,SYS_IRN,sync); break;
        case 1132: ret=encode_msm2(rtcm,nav,SYS_IRN,sync); break;
        case 1133: ret=encode_msm3(rtcm,nav,SYS_IRN,sync); break;
        case 1134: ret=encode_msm4(rtcm,nav,SYS_IRN,sync); break;
        case 1135: ret=encode_msm5(rtcm,nav,SYS_IRN,sync); break;
        case 1136: ret=encode_msm6(rtcm,nav,SYS_IRN,sync); break;
        case 1137: ret=encode_msm7(rtcm,nav,SYS_IRN,sync); break;
        case 1230: ret=encode_type1230(rtcm,sync);     break;
        case 1240: ret=encode_ssr1(rtcm,nav,SYS_GAL,0,sync); break; /* draft */
        case 1241: ret=encode_ssr2(rtcm,nav,SYS_GAL,0,sync); break; /* draft */
        case 1242: ret=encode_ssr3(rtcm,nav,SYS_GAL,0,sync); break; /* draft */
        case 1243: ret=encode_ssr4(rtcm,nav,SYS_GAL,0,sync); break; /* draft */
        case 1244: ret=encode_ssr5(rtcm,nav,SYS_GAL,0,sync); break; /* draft */
        case 1245: ret=encode_ssr6(rtcm,nav,SYS_GAL,0,sync); break; /* draft */
        case 1246: ret=encode_ssr1(rtcm,nav,SYS_QZS,0,sync); break; /* draft */
        case 1247: ret=encode_ssr2(rtcm,nav,SYS_QZS,0,sync); break; /* draft */
        case 1248: ret=encode_ssr3(rtcm,nav,SYS_QZS,0,sync); break; /* draft */
        case 1249: ret=encode_ssr4(rtcm,nav,SYS_QZS,0,sync); break; /* draft */
        case 1250: ret=encode_ssr5(rtcm,nav,SYS_QZS,0,sync); break; /* draft */
        case 1251: ret=encode_ssr6(rtcm,nav,SYS_QZS,0,sync); break; /* draft */
        case 1252: ret=encode_ssr1(rtcm,nav,SYS_SBS,0,sync); break; /* draft */
        case 1253: ret=encode_ssr2(rtcm,nav,SYS_SBS,0,sync); break; /* draft */
        case 1254: ret=encode_ssr3(rtcm,nav,SYS_SBS,0,sync); break; /* draft */
        case 1255: ret=encode_ssr4(rtcm,nav,SYS_SBS,0,sync); break; /* draft */
        case 1256: ret=encode_ssr5(rtcm,nav,SYS_SBS,0,sync); break; /* draft */
        case 1257: ret=encode_ssr6(rtcm,nav,SYS_SBS,0,sync); break; /* draft */
        case 1258: ret=encode_ssr1(rtcm,nav,SYS_CMP,0,sync); break; /* draft */
        case 1259: ret=encode_ssr2(rtcm,nav,SYS_CMP,0,sync); break; /* draft */
        case 1260: ret=encode_ssr3(rtcm,nav,SYS_CMP,0,sync); break; /* draft */
        case 1261: ret=encode_ssr4(rtcm,nav,SYS_CMP,0,sync); break; /* draft */
        case 1262: ret=encode_ssr5(rtcm,nav,SYS_CMP,0,sync); break; /* draft */
        case 1263: ret=encode_ssr6(rtcm,nav,SYS_CMP,0,sync); break; /* draft */
        case   11: ret=encode_ssr7(rtcm,nav,SYS_GPS,0,sync); break; /* tentative */
        case   12: ret=encode_ssr7(rtcm,nav,SYS_GAL,0,sync); break; /* tentative */
        case   13: ret=encode_ssr7(rtcm,nav,SYS_QZS,0,sync); break; /* tentative */
        case   14: ret=encode_ssr7(rtcm,nav,SYS_CMP,0,sync); break; /* tentative */
        case 4073: ret=encode_type4073(rtcm,nav,subtype,sync); break;
        case 4076: ret=encode_type4076(rtcm,nav,subtype,sync); break;
    }
    if (ret>0) {
        if      (1001<=type&&type<=1299) rtcm->nmsg3[type-1000]++; /*   1-299 */
        else if (4070<=type&&type<=4099) rtcm->nmsg3[type-3770]++; /* 300-329 */
        else rtcm->nmsg3[0]++; /* other */
    }
    return ret;
}

/* from rtcm.c */

/* input RTCM 3 message from stream --------------------------------------------
* fetch next RTCM 3 message and input a message from byte stream
* args   : rtcm_t *rtcm     IO  rtcm control struct
*          uint8_t data     I   stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 5: input station pos/ant parameters,
*                  10: input ssr messages)
* notes  : before firstly calling the function, time in rtcm control struct has
*          to be set to the approximate time within 1/2 week in order to resolve
*          ambiguity of time in rtcm messages.
*          
*          to specify input options, set rtcm->opt to the following option
*          strings separated by spaces.
*
*          -EPHALL  : input all ephemerides (default: only new)
*          -STA=nnn : input only message with STAID=nnn (default: all)
*          -GLss    : select signal ss for GPS MSM (ss=1C,1P,...)
*          -RLss    : select signal ss for GLO MSM (ss=1C,1P,...)
*          -ELss    : select signal ss for GAL MSM (ss=1C,1B,...)
*          -JLss    : select signal ss for QZS MSM (ss=1C,2C,...)
*          -CLss    : select signal ss for BDS MSM (ss=2I,7I,...)
*          -ILss    : select signal ss for IRN MSM (ss=5A,9A,...)
*          -GALINAV : select I/NAV for Galileo ephemeris (default: all)
*          -GALFNAV : select F/NAV for Galileo ephemeris (default: all)
*
*          supported RTCM 3 messages (ref [7][10][15][16][17][18])
*
*            TYPE       :  GPS   GLONASS Galileo  QZSS     BDS    SBAS    NavIC
*         ----------------------------------------------------------------------
*          OBS COMP L1  : 1001~   1009~     -       -       -       -       -
*              FULL L1  : 1002    1010      -       -       -       -       -
*              COMP L1L2: 1003~   1011~     -       -       -       -       -
*              FULL L1L2: 1004    1012      -       -       -       -       -
*
*          NAV          : 1019    1020    1045**  1044    1042      -     1041
*                           -       -     1046**    -       63*     -       -
*
*          MSM 1        : 1071~   1081~   1091~   1111~   1121~   1101~   1131~
*              2        : 1072~   1082~   1092~   1112~   1122~   1102~   1132~
*              3        : 1073~   1083~   1093~   1113~   1123~   1103~   1133~
*              4        : 1074    1084    1094    1114    1124    1104    1134
*              5        : 1075    1085    1095    1115    1125    1105    1135 
*              6        : 1076    1086    1096    1116    1126    1106    1136 
*              7        : 1077    1087    1097    1117    1127    1107    1137 
*
*          SSR ORBIT    : 1057    1063    1240*   1246*   1258*     -       -
*              CLOCK    : 1058    1064    1241*   1247*   1259*     -       -
*              CODE BIAS: 1059    1065    1242*   1248*   1260*     -       -
*              OBT/CLK  : 1060    1066    1243*   1249*   1261*     -       -
*              URA      : 1061    1067    1244*   1250*   1262*     -       -
*              HR-CLOCK : 1062    1068    1245*   1251*   1263*     -       -
*              PHAS BIAS:   11*     -       12*     13*     14*     -       -
*
*          ANT/RCV INFO : 1007    1008    1033
*          STA POSITION : 1005    1006
*
*          PROPRIETARY  : 4076 (IGS)
*         ----------------------------------------------------------------------
*                            (* draft, ** 1045:F/NAV,1046:I/NAV, ~ only encode)
*
*          for MSM observation data with multiple signals for a frequency,
*          a signal is selected according to internal priority. to select
*          a specified signal, use the input options.
*
*          RTCM 3 message format:
*            +----------+--------+-----------+--------------------+----------+
*            | preamble | 000000 |  length   |    data message    |  parity  |
*            +----------+--------+-----------+--------------------+----------+
*            |<-- 8 --->|<- 6 -->|<-- 10 --->|<--- length x 8 --->|<-- 24 -->|
*            
*-----------------------------------------------------------------------------*/
extern int input_rtcm3(rtcm_t *rtcm, uint8_t data, nav_t *nav)
{
    trace(5,"input_rtcm3: data=%02x\n",data);
    
    /* synchronize frame */
    if (rtcm->nbyte==0) {
        if (data!=RTCM3PREAMB) return 0;
        rtcm->buff[rtcm->nbyte++]=data;
        return 0;
    }
    rtcm->buff[rtcm->nbyte++]=data;
    
    if (rtcm->nbyte==3) {
        rtcm->len=getbitu(rtcm->buff,14,10)+3; /* length without parity */
    }
    if (rtcm->nbyte<3||rtcm->nbyte<rtcm->len+3) return 0;
    rtcm->nbyte=0;
    
    /* check parity */
    if (rtk_crc24q(rtcm->buff,rtcm->len)!=getbitu(rtcm->buff,rtcm->len*8,24)) {
        trace(2,"rtcm3 parity error: len=%d\n",rtcm->len);
        return 0;
    }
    /* decode rtcm3 message */
    return decode_rtcm3(rtcm,nav);
}
extern int input_rtcm3_buff(rtcm_t *rtcm, uint8_t *buff, int nbyte, nav_t *nav)
{
    if (buff[0]!=RTCM3PREAMB||nbyte<6) return 0;
    rtcm->len=getbitu(buff,14,10)+3; /* length without parity */
    if (nbyte<(rtcm->len+3)) return 0;
    
    /* check parity */
    if (rtk_crc24q(buff,rtcm->len)!=getbitu(buff,rtcm->len*8,24)) {
        trace(2,"rtcm3 parity error: len=%d\n",rtcm->len);
        return 0;
    }
    memcpy(rtcm->buff,buff,sizeof(uint8_t)*nbyte);
    /* decode rtcm3 message */
    return decode_rtcm3(rtcm,nav);
}
extern int check_rtcm3_type(uint8_t *buff, int nbyte, int *len, int *crc, int *staid, double *xyz)
{
    int type = 0, i = 0;
    if (buff[0]!=RTCM3PREAMB||nbyte<6) return 0;
    *len = getbitu(buff, 14, 10) + 3; /* length without parity */
    if (nbyte<(*len + 3)) return 0;
    if (rtk_crc24q(buff,*len)!=getbitu(buff,(*len)*8,24))
        *crc = 1;
    else
    {
        *crc = 0;
        type = getbitu(buff, 24, 12);
        if (type == 1071 || type == 1072 || type == 1073 || type == 1074 || type == 1075 || type == 1076 || type == 1077 || /* GPS */
            type == 1081 || type == 1082 || type == 1083 || type == 1084 || type == 1085 || type == 1086 || type == 1087 || /* GLO */
            type == 1091 || type == 1092 || type == 1093 || type == 1094 || type == 1095 || type == 1096 || type == 1097 || /* GAL */
            type == 1101 || type == 1102 || type == 1103 || type == 1104 || type == 1105 || type == 1106 || type == 1107 || /* SBS */
            type == 1111 || type == 1112 || type == 1113 || type == 1114 || type == 1115 || type == 1116 || type == 1117 || /* QZS */
            type == 1121 || type == 1122 || type == 1123 || type == 1124 || type == 1125 || type == 1126 || type == 1127 || /* BDS */
            type == 1131 || type == 1132 || type == 1133 || type == 1134 || type == 1135 || type == 1136 || type == 1137 || /* IRN */
            type == 1001 || type == 1002 || type == 1003 || type == 1004 ||	/* RTCM 2.x */
            type == 1009 || type == 1010 || type == 1011 || type == 1012 || /* RTCM 2.x */
            type == 1005 || type == 1006 || type == 1007 || type == 1008 || type == 1033 || type == 1230)
        {
            *staid = getbitu(buff, 24 + 12, 12);
        }
        else
        {
            *staid = 0;
        }
        if (type == 1005 || type == 1006)
        {
            /* 24 => type, 24+12 => staid, 24+12+12+6+4 => pos */
            i = 24 + 12 + 12 + 6 + 4;
            xyz[0]=getbits_38(buff,i)*0.0001; i+=38+2;
            xyz[1]=getbits_38(buff,i)*0.0001; i+=38+2;
            xyz[2]=getbits_38(buff,i)*0.0001;
        }
    }
    *len+=3; /* total len */
    return type; /* type */
}
extern int change_rtcm3_id(uint8_t *buff, int nbyte, int rcvid)
{
    int len = 0, i = 24, type = 0, staid = 0;
	int crc = 0;
    int ret = 0;
    if (buff[0]!=RTCM3PREAMB||nbyte<6) return ret;
    len=getbitu(buff,14,10)+3; /* length without parity */
    if (nbyte<(len + 3)) return ret;

    i = 24;
    type = getbitu(buff,i,12); i += 12;

    if (type == 1071 || type == 1072 || type == 1073 || type == 1074 || type == 1075 || type == 1076 || type == 1077 || /* GPS */
		type == 1081 || type == 1082 || type == 1083 || type == 1084 || type == 1085 || type == 1086 || type == 1087 || /* GLO */
        type == 1091 || type == 1092 || type == 1093 || type == 1094 || type == 1095 || type == 1096 || type == 1097 || /* GAL */
        type == 1101 || type == 1102 || type == 1103 || type == 1104 || type == 1105 || type == 1106 || type == 1107 || /* SBS */
        type == 1111 || type == 1112 || type == 1113 || type == 1114 || type == 1115 || type == 1116 || type == 1117 || /* QZS */
		type == 1121 || type == 1122 || type == 1123 || type == 1124 || type == 1125 || type == 1126 || type == 1127 || /* BDS */
        type == 1131 || type == 1132 || type == 1133 || type == 1134 || type == 1135 || type == 1136 || type == 1137 || /* IRN */
		type == 1001 || type == 1002 || type == 1003 || type == 1004 ||	/* RTCM 2.x */
		type == 1009 || type == 1010 || type == 1011 || type == 1012 || /* RTCM 2.x */
		type == 1005 || type == 1006 || type == 1007 || type == 1008 || type == 1033 || type == 1230)
    {
        staid = getbitu(buff, i, 12);

		if (staid!=rcvid)
		{
			setbitu(buff,i,12,rcvid); /* new station id */
	        staid = getbitu(buff, i, 12);
			
			/* crc-24q */
			crc=rtk_crc24q(buff,len);
			setbitu(buff,len*8,24,crc);
            ret=1;
		}
    }
    return ret;
}
extern int update_rtcm3_pos(uint8_t *buff, int nbyte, int rcvid, double *p)
{
    int len = 0, i = 24, type = 0, staid = 0;
	int crc = 0;
    int ret = 0;
    if (buff[0]!=RTCM3PREAMB||nbyte<6) return ret;
    len=getbitu(buff,14,10)+3; /* length without parity */
    if (nbyte<(len + 3)) return ret;

    i = 24; /* type */
    type = getbitu(buff,i,12); i += 12;

    if (type == 1005 || type == 1006)
    {
        /* 24 + 12, staid */
        setbitu(buff,i,12,     rcvid); i+=12; /* ref station id */
                                       i+= 6; /* itrf realization year */
                                       i+= 1; /* gps indicator */
                                       i+= 1; /* glonass indicator */
                                       i+= 1; /* galileo indicator */
                                       i+= 1; /* ref station indicator */
        set38bits(buff,i,p[0]/0.0001); i+=38; /* antenna ref point ecef-x */
                                       i+= 1; /* oscillator indicator */
                                       i+= 1; /* reserved */
        set38bits(buff,i,p[1]/0.0001); i+=38; /* antenna ref point ecef-y */
                                       i+= 2; /* quarter cycle indicator */
        set38bits(buff,i,p[2]/0.0001); i+=38; /* antenna ref point ecef-z */

		/* crc-24q */
		crc=rtk_crc24q(buff,len);
		setbitu(buff,len*8,24,crc);
        ret=1;
    }
    return ret;
}
/* generate RTCM 3 message -----------------------------------------------------
* generate RTCM 3 message
* args   : rtcm_t *rtcm     IO  rtcm control struct
*          int    type      I   message type
*          int    subtype   I   message subtype
*          int    sync      I   sync flag (1:another message follows)
* return : status (1:ok,0:error)
* notes  : For rtcm 3 msm, the {nsat} x {nsig} in rtcm->obs should not exceed
*          64. If {nsat} x {nsig} of the input obs data exceeds 64, separate
*          them to multiple ones and call gen_rtcm3() multiple times as user
*          responsibility.
*          ({nsat} = number of valid satellites, {nsig} = number of signals in
*          the obs data) 
*-----------------------------------------------------------------------------*/
extern int gen_rtcm3(rtcm_t *rtcm, nav_t *nav, int type, int subtype, int sync)
{
    uint32_t crc;
    int i=0;
    
    trace(4,"gen_rtcm3: type=%d subtype=%d sync=%d\n",type,subtype,sync);
    
    rtcm->nbit=rtcm->len=rtcm->nbyte=0;
    
    /* set preamble and reserved */
    setbitu(rtcm->buff,i, 8,RTCM3PREAMB); i+= 8;
    setbitu(rtcm->buff,i, 6,0          ); i+= 6;
    setbitu(rtcm->buff,i,10,0          ); i+=10;
    
    /* encode rtcm 3 message body */
    if (!encode_rtcm3(rtcm,nav,type,subtype,sync)) return 0;
    
    /* padding to align 8 bit boundary */
    for (i=rtcm->nbit;i%8;i++) {
        setbitu(rtcm->buff,i,1,0);
    }
    /* message length (header+data) (bytes) */
    if ((rtcm->len=i/8)>=3+1024) {
        trace(2,"generate rtcm 3 message length error len=%d\n",rtcm->len-3);
        rtcm->nbit=rtcm->len=0;
        return 0;
    }
    /* message length without header and parity */
    setbitu(rtcm->buff,14,10,rtcm->len-3);
    
    /* crc-24q */
    crc=rtk_crc24q(rtcm->buff,rtcm->len);
    setbitu(rtcm->buff,i,24,crc);
    
    /* length total (bytes) */
    rtcm->nbyte=rtcm->len+3;
    
    return 1;
}
/* write rtcm3 msm to stream -------------------------------------------------*/
extern int write_rtcm3_msm(rtcm_t *out, nav_t *nav, int msg, int sync, uint8_t *rtcm_buff, int nbyte)
{
    obsd_t *data;
    obs_t obs_tmp=out->obs;
    int i,j,n,ns,sys,nobs,code,nsat=0,nsig=0,nmsg,mask[MAXCODE]={0};
    
    if      (1071<=msg&&msg<=1077) sys=SYS_GPS;
    else if (1081<=msg&&msg<=1087) sys=SYS_GLO;
    else if (1091<=msg&&msg<=1097) sys=SYS_GAL;
    else if (1101<=msg&&msg<=1107) sys=SYS_SBS;
    else if (1111<=msg&&msg<=1117) sys=SYS_QZS;
    else if (1121<=msg&&msg<=1127) sys=SYS_CMP;
    else if (1131<=msg&&msg<=1137) sys=SYS_IRN;
    else return nbyte;
    
    data=obs_tmp.data;
    nobs=obs_tmp.n;
    
    /* count number of satellites and signals */
    for (i=0;i<nobs&&i<MAXOBS;i++) {
        if (satsys(data[i].sat,NULL)!=sys) continue;
        nsat++;
        for (j=0;j<NFREQ+NEXOBS;j++) {
            if (!(code=data[i].code[j])||mask[code-1]) continue;
            mask[code-1]=1;
            nsig++;
        }
    }
    if (nsig>64) return nbyte;
    
    /* pack data to multiple messages if nsat x nsig > 64 */
    if (nsig>0) {
        ns=64/nsig;         /* max number of sats in a message */
        nmsg=(nsat-1)/ns+1; /* number of messages */
    }
    else {
        ns=0;
        nmsg=1;
    }
    out->obs.n;
    
    for (i=j=0;i<nmsg;i++) {
        for (n=0;n<ns&&j<nobs&&j<MAXOBS;j++) {
            if (satsys(data[j].sat,NULL)!=sys) continue;
            out->obs.data[n++]=data[j];
        }
        out->obs.n=n;
        
        if (gen_rtcm3(out,nav,msg,0,i<nmsg-1?1:sync)) {
            memcpy(rtcm_buff+nbyte,out->buff,sizeof(uint8_t)*out->nbyte);
            nbyte+=out->nbyte;
        }
    }
    out->obs=obs_tmp;
    return nbyte;
}

/* write rtcm3 msm to stream -------------------------------------------------*/
extern int write_rtcm3(rtcm_t *out, nav_t* nav, int msg, int sync, uint8_t *rtcm_buff, int nbyte)
{
    if (gen_rtcm3(out,nav,msg,0,0)) {
        memcpy(rtcm_buff+nbyte,out->buff,sizeof(uint8_t)*out->nbyte);
        nbyte+=out->nbyte;
    }
    return nbyte;
}

