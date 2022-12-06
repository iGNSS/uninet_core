#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include "rtcm_buff.h"
#include "EngineVRS.h"

static const unsigned int tbl_CRC24Q[]={
    0x000000,0x864CFB,0x8AD50D,0x0C99F6,0x93E6E1,0x15AA1A,0x1933EC,0x9F7F17,
    0xA18139,0x27CDC2,0x2B5434,0xAD18CF,0x3267D8,0xB42B23,0xB8B2D5,0x3EFE2E,
    0xC54E89,0x430272,0x4F9B84,0xC9D77F,0x56A868,0xD0E493,0xDC7D65,0x5A319E,
    0x64CFB0,0xE2834B,0xEE1ABD,0x685646,0xF72951,0x7165AA,0x7DFC5C,0xFBB0A7,
    0x0CD1E9,0x8A9D12,0x8604E4,0x00481F,0x9F3708,0x197BF3,0x15E205,0x93AEFE,
    0xAD50D0,0x2B1C2B,0x2785DD,0xA1C926,0x3EB631,0xB8FACA,0xB4633C,0x322FC7,
    0xC99F60,0x4FD39B,0x434A6D,0xC50696,0x5A7981,0xDC357A,0xD0AC8C,0x56E077,
    0x681E59,0xEE52A2,0xE2CB54,0x6487AF,0xFBF8B8,0x7DB443,0x712DB5,0xF7614E,
    0x19A3D2,0x9FEF29,0x9376DF,0x153A24,0x8A4533,0x0C09C8,0x00903E,0x86DCC5,
    0xB822EB,0x3E6E10,0x32F7E6,0xB4BB1D,0x2BC40A,0xAD88F1,0xA11107,0x275DFC,
    0xDCED5B,0x5AA1A0,0x563856,0xD074AD,0x4F0BBA,0xC94741,0xC5DEB7,0x43924C,
    0x7D6C62,0xFB2099,0xF7B96F,0x71F594,0xEE8A83,0x68C678,0x645F8E,0xE21375,
    0x15723B,0x933EC0,0x9FA736,0x19EBCD,0x8694DA,0x00D821,0x0C41D7,0x8A0D2C,
    0xB4F302,0x32BFF9,0x3E260F,0xB86AF4,0x2715E3,0xA15918,0xADC0EE,0x2B8C15,
    0xD03CB2,0x567049,0x5AE9BF,0xDCA544,0x43DA53,0xC596A8,0xC90F5E,0x4F43A5,
    0x71BD8B,0xF7F170,0xFB6886,0x7D247D,0xE25B6A,0x641791,0x688E67,0xEEC29C,
    0x3347A4,0xB50B5F,0xB992A9,0x3FDE52,0xA0A145,0x26EDBE,0x2A7448,0xAC38B3,
    0x92C69D,0x148A66,0x181390,0x9E5F6B,0x01207C,0x876C87,0x8BF571,0x0DB98A,
    0xF6092D,0x7045D6,0x7CDC20,0xFA90DB,0x65EFCC,0xE3A337,0xEF3AC1,0x69763A,
    0x578814,0xD1C4EF,0xDD5D19,0x5B11E2,0xC46EF5,0x42220E,0x4EBBF8,0xC8F703,
    0x3F964D,0xB9DAB6,0xB54340,0x330FBB,0xAC70AC,0x2A3C57,0x26A5A1,0xA0E95A,
    0x9E1774,0x185B8F,0x14C279,0x928E82,0x0DF195,0x8BBD6E,0x872498,0x016863,
    0xFAD8C4,0x7C943F,0x700DC9,0xF64132,0x693E25,0xEF72DE,0xE3EB28,0x65A7D3,
    0x5B59FD,0xDD1506,0xD18CF0,0x57C00B,0xC8BF1C,0x4EF3E7,0x426A11,0xC426EA,
    0x2AE476,0xACA88D,0xA0317B,0x267D80,0xB90297,0x3F4E6C,0x33D79A,0xB59B61,
    0x8B654F,0x0D29B4,0x01B042,0x87FCB9,0x1883AE,0x9ECF55,0x9256A3,0x141A58,
    0xEFAAFF,0x69E604,0x657FF2,0xE33309,0x7C4C1E,0xFA00E5,0xF69913,0x70D5E8,
    0x4E2BC6,0xC8673D,0xC4FECB,0x42B230,0xDDCD27,0x5B81DC,0x57182A,0xD154D1,
    0x26359F,0xA07964,0xACE092,0x2AAC69,0xB5D37E,0x339F85,0x3F0673,0xB94A88,
    0x87B4A6,0x01F85D,0x0D61AB,0x8B2D50,0x145247,0x921EBC,0x9E874A,0x18CBB1,
    0xE37B16,0x6537ED,0x69AE1B,0xEFE2E0,0x709DF7,0xF6D10C,0xFA48FA,0x7C0401,
    0x42FA2F,0xC4B6D4,0xC82F22,0x4E63D9,0xD11CCE,0x575035,0x5BC9C3,0xDD8538
};

#define PRUNIT_GPS  299792.458  /* rtcm ver.3 unit of gps pseudorange (m) */
#define PRUNIT_GLO  599584.916  /* rtcm ver.3 unit of glonass pseudorange (m) */
#define RANGE_MS    (CLIGHT*0.001)      /* range in 1 ms */
#define SC2RAD      3.1415926535898     /* semi-circle to radian (IS-GPS) */

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
#define P2_28       3.725290298461914E-09 /* 2^-28 */
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
#define P2_41       4.547473508864641E-13 /* 2^-41 */
#define P2_43       1.136868377216160E-13 /* 2^-43 */
#define P2_46       1.421085471520200E-14 /* 2^-46 */
#define P2_48       3.552713678800501E-15 /* 2^-48 */
#define P2_50       8.881784197001252E-16 /* 2^-50 */
#define P2_55       2.775557561562891E-17 /* 2^-55 */
#define P2_59       1.734723475976810E-18 /* 2^-59 */
#define P2_66       1.355252715606880E-20 /* 2^-66 */


extern unsigned int crc24q (unsigned char *buff, int len)
{
    unsigned int crc=0;
    int i;
    
    for (i=0;i<len;i++) crc=((crc<<8)&0xFFFFFF)^tbl_CRC24Q[(crc>>16)^buff[i]];
    return crc;
}
extern void setbitu(unsigned char *buff, int pos, int len, unsigned int data)
{
    unsigned int mask=1u<<(len-1);
    int i;
    if (len<=0||32<len) return;
    for (i=pos;i<pos+len;i++,mask>>=1) {
        if (data&mask) buff[i/8]|=1u<<(7-i%8); else buff[i/8]&=~(1u<<(7-i%8));
    }
}
extern unsigned int getbitu(unsigned char *buff, int pos, int len)
{
    unsigned int bits=0;
    int i;
    for (i=pos;i<pos+len;i++) bits=(bits<<1)+((buff[i/8]>>(7-i%8))&1u);
    return bits;
}
extern int getbits(unsigned char *buff, int pos, int len)
{
    unsigned int bits=getbitu(buff,pos,len);
    if (len<=0||32<=len||!(bits&(1u<<(len-1)))) return (int)bits;
    return (int)(bits|(~0u<<len)); /* extend sign */
}
/* get sign-magnitude bits ---------------------------------------------------*/
extern double getbitg(unsigned char *buff, int pos, int len)
{
    double value=getbitu(buff,pos+1,len-1);
    return getbitu(buff,pos,1)?-value:value;
}
/* get signed 38bit field ----------------------------------------------------*/
extern double getbits_38(unsigned char *buff, int pos)
{
    return (double)getbits(buff,pos,32)*64.0+getbitu(buff,pos+32,6);
}

extern void setbits(unsigned char *buff, int pos, int len, int data)
{
    if (data<0) data|=1<<(len-1); else data&=~(1<<(len-1)); /* set sign bit */
    setbitu(buff,pos,len,(unsigned int)data);
}
/* set sign-magnitude bits ---------------------------------------------------*/
extern void setbitg(unsigned char *buff, int pos, int len, int value)
{
    setbitu(buff,pos,1,value<0?1:0);
    setbitu(buff,pos+1,len-1,value<0?-value:value);
}
/* set signed 38 bit field ---------------------------------------------------*/
extern void set38bits(unsigned char *buff, int pos, double value)
{
    int word_h=(int)floor(value/64.0);
    unsigned int word_l=(unsigned int)(value-word_h*64.0);
    setbits(buff,pos  ,32,word_h);
    setbitu(buff,pos+32,6,word_l);
}
/* decode type 1005: stationary rtk reference station arp --------------------*/
extern int decode_type1005_(unsigned char *buff, int len, int *staid, int *itrf, double *pos)
{
    int i=24+12;
    
    if (i+140<=len*8) {
        *staid=getbitu(buff,i,12); i+=12;
        *itrf =getbitu(buff,i, 6); i+= 6+4;
        pos[0]=getbits_38(buff,i)*0.0001; i+=38+2;
        pos[1]=getbits_38(buff,i)*0.0001; i+=38+2;
        pos[2]=getbits_38(buff,i)*0.0001;
    }
    else {
        return -1;
    }
    return 5;
}
/* decode type 1006: stationary rtk reference station arp with height --------*/
extern int decode_type1006_(unsigned char *buff, int len, int *staid, int *itrf, double *pos, double *anth)
{
    int i=24+12;
    
    if (i+156<=len*8) {
        *staid=getbitu(buff,i,12); i+=12;
        *itrf =getbitu(buff,i, 6); i+= 6+4;
        pos[0]=getbits_38(buff,i)*0.0001; i+=38+2;
        pos[1]=getbits_38(buff,i)*0.0001; i+=38+2;
        pos[2]=getbits_38(buff,i)*0.0001; i+=38;
        *anth =getbitu(buff,i,16)*0.0001;
    }
    else {
        return -1;
    }
    return 5;
}
/* decode type 1007: antenna descriptor --------------------------------------*/
extern int decode_type1007_(unsigned char *buff, int len, int *staid, char *des, int *setup)
{
    int i=24+12,j,n;
    
    n=getbitu(buff,i+12,8); des[n]='\0';
    
    if (i+28+8*n<=len*8) {
        *staid=getbitu(buff,i,12); i+=12+8;
        for (j=0;j<n&&j<31;j++) {
            des[j]=(char)getbitu(buff,i,8); i+=8;
        }
        *setup=getbitu(buff,i, 8);
    }
    else {
        return -1;
    }
    return 5;
}
/* decode type 1008: antenna descriptor & serial number ----------------------*/
extern int decode_type1008_(unsigned char *buff, int len, int *staid, char *des, int *setup, char *sno)
{
    int i=24+12,j,n,m;
    
    n=getbitu(buff,i+12,8);
    m=getbitu(buff,i+28+8*n,8);

	des[n]='\0';
    sno[m]='\0';
    if (i+36+8*(n+m)<=len*8) {
        *staid=getbitu(buff,i,12); i+=12+8;
        for (j=0;j<n&&j<31;j++) {
            des[j]=(char)getbitu(buff,i,8); i+=8;
        }
        *setup=getbitu(buff,i, 8); i+=8+8;
        for (j=0;j<m&&j<31;j++) {
            sno[j]=(char)getbitu(buff,i,8); i+=8;
        }
    }
    else {
        return -1;
    }
    return 5;
}
/* decode type 1033: receiver and antenna descriptor -------------------------*/
extern int decode_type1033_(unsigned char *buff, int len, int *staid, char *des, int *setup, char *sno, char *rec, char *ver, char *rsn)
{
    int i=24+12,j,n,m,n1,n2,n3;
    
    n =getbitu(buff,i+12,8);
    m =getbitu(buff,i+28+8*n,8);
    n1=getbitu(buff,i+36+8*(n+m),8);
    n2=getbitu(buff,i+44+8*(n+m+n1),8);
    n3=getbitu(buff,i+52+8*(n+m+n1+n2),8);

	des[n ]='\0';
	sno[m ]='\0';
	rec[n1]='\0';
	ver[n2]='\0';
	rsn[n3]='\0';
    
    if (i+60+8*(n+m+n1+n2+n3)<=len*8) {
        *staid=getbitu(buff,i,12); i+=12+8;
        for (j=0;j<n&&j<31;j++) {
            des[j]=(char)getbitu(buff,i,8); i+=8;
        }
        *setup=getbitu(buff,i, 8); i+=8+8;
        for (j=0;j<m&&j<31;j++) {
            sno[j]=(char)getbitu(buff,i,8); i+=8;
        }
        i+=8;
        for (j=0;j<n1&&j<31;j++) {
            rec[j]=(char)getbitu(buff,i,8); i+=8;
        }
        i+=8;
        for (j=0;j<n2&&j<31;j++) {
            ver[j]=(char)getbitu(buff,i,8); i+=8;
        }
        i+=8;
        for (j=0;j<n3&&j<31;j++) {
            rsn[j]=(char)getbitu(buff,i,8); i+=8;
        }
    }
    else {
        return -1;
    }

    return 5;
}
/* decode type 1230: GLONASS L1 and L2 code-phase biases ---------------------*/
extern int decode_type1230_(unsigned char *buff, int len, int *staid, int *align, int *mask, double *glo_bias)
{
    int i=24+12,j,bias;
    
    if (i+20>=len*8) {
        return -1;
    }
    *staid=getbitu(buff,i,12); i+=12;
    *align=getbitu(buff,i, 1); i+= 1+3;
    *mask =getbitu(buff,i, 4); i+= 4;
    
    for (j=0;j<4;j++) {
        glo_bias[j]=0.0;
    }
    for (j=0;j<4&&i+16<=len*8;j++) {
        if (!(*mask&(1<<(3-j)))) continue;
        bias=getbits(buff,i,16); i+=16;
        if (bias!=-32768) {
            glo_bias[j]=bias*0.02;
        }
    }
    return 5;
}

/* decode type 1019: gps ephemerides -----------------------------------------*/
extern int decode_type1019_(unsigned char *buff, int len, sat_eph_t *eph)
{
    double sqrtA=0.0;
    int i=24+12;
    memset(eph,0,sizeof(sat_eph_t));
    
    if (i+476<=len*8) {
        eph->sys   ='G';
        eph->prn   =getbitu(buff,i, 6);              i+= 6;
        eph->week  =getbitu(buff,i,10)+2048;         i+=10;
        eph->sva   =getbitu(buff,i, 4);              i+= 4;
        eph->code  =getbitu(buff,i, 2);              i+= 2;
        eph->idot  =getbits(buff,i,14)*P2_43*SC2RAD; i+=14;
        eph->iode  =getbitu(buff,i, 8);              i+= 8;
        eph->tocs  =getbitu(buff,i,16)*16.0;         i+=16;
        eph->f2    =getbits(buff,i, 8)*P2_55;        i+= 8;
        eph->f1    =getbits(buff,i,16)*P2_43;        i+=16;
        eph->f0    =getbits(buff,i,22)*P2_31;        i+=22;
        eph->iodc  =getbitu(buff,i,10);              i+=10;
        eph->crs   =getbits(buff,i,16)*P2_5;         i+=16;
        eph->deln  =getbits(buff,i,16)*P2_43*SC2RAD; i+=16;
        eph->M0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cuc   =getbits(buff,i,16)*P2_29;        i+=16;
        eph->e     =getbitu(buff,i,32)*P2_33;        i+=32;
        eph->cus   =getbits(buff,i,16)*P2_29;        i+=16;
        sqrtA      =getbitu(buff,i,32)*P2_19;        i+=32;
        eph->toes  =getbitu(buff,i,16)*16.0;         i+=16;
        eph->cic   =getbits(buff,i,16)*P2_29;        i+=16;
        eph->OMG0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cis   =getbits(buff,i,16)*P2_29;        i+=16;
        eph->i0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->crc   =getbits(buff,i,16)*P2_5;         i+=16;
        eph->omg   =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->OMGd  =getbits(buff,i,24)*P2_43*SC2RAD; i+=24;
        eph->tgd[0]=getbits(buff,i, 8)*P2_31;        i+= 8;
        eph->svh   =getbitu(buff,i, 6);              i+= 6;
        eph->flag  =getbitu(buff,i, 1);              i+= 1;
        eph->fit   =getbitu(buff,i, 1)?0.0:4.0; /* 0:4hr,1:>4hr */
    }
    else {
        return -1;
    }
    if (eph->prn>=40) {
        eph->sys='S'; eph->prn+=80;
    }
    //trace_(4,"decode_type1019: prn=%c%3d iode=%4d toe=%10.0f\n",eph->sys,eph->prn,eph->iode,eph->toes);
    
    eph->A=sqrtA*sqrtA;

    return 2;
}
/* decode type 1020: glonass ephemerides -------------------------------------*/
extern int decode_type1020_(unsigned char *buff, int len, glo_eph_t *geph)
{
    double tk_h,tk_m,tk_s;
    int i=24+12,tb,bn;
    memset(geph,0,sizeof(glo_eph_t));
    
    if (i+348<=len*8) {
        geph->sys  ='R';
        geph->prn  =getbitu(buff,i, 6);           i+= 6;
        geph->frq  =getbitu(buff,i, 5)-7;         i+= 5+2+2;
        tk_h       =getbitu(buff,i, 5);           i+= 5;
        tk_m       =getbitu(buff,i, 6);           i+= 6;
        tk_s       =getbitu(buff,i, 1)*30.0;      i+= 1;
        bn         =getbitu(buff,i, 1);           i+= 1+1;
        tb         =getbitu(buff,i, 7);           i+= 7;
        geph->vel[0]=getbitg(buff,i,24)*P2_20*1E3; i+=24;
        geph->pos[0]=getbitg(buff,i,27)*P2_11*1E3; i+=27;
        geph->acc[0]=getbitg(buff,i, 5)*P2_30*1E3; i+= 5;
        geph->vel[1]=getbitg(buff,i,24)*P2_20*1E3; i+=24;
        geph->pos[1]=getbitg(buff,i,27)*P2_11*1E3; i+=27;
        geph->acc[1]=getbitg(buff,i, 5)*P2_30*1E3; i+= 5;
        geph->vel[2]=getbitg(buff,i,24)*P2_20*1E3; i+=24;
        geph->pos[2]=getbitg(buff,i,27)*P2_11*1E3; i+=27;
        geph->acc[2]=getbitg(buff,i, 5)*P2_30*1E3; i+= 5+1;
        geph->gamn  =getbitg(buff,i,11)*P2_40;     i+=11+3;
        geph->taun  =getbitg(buff,i,22)*P2_30;     i+=22;
        geph->dtaun =getbitg(buff,i, 5)*P2_30;     i+=5;
        geph->age   =getbitu(buff,i, 5);
    }
    else {
        return -1;
    }
  
    geph->svh=bn;
    geph->iode=tb&0x7F;
    geph->tofs=tk_h*3600.0+tk_m*60.0+tk_s-10800.0; /* lt->utc */
    geph->toes=tb*900.0-10800.0; /* lt->utc */
    geph->tofs += 18.0; if (geph->tofs<0.0) geph->tofs += 24*3600.0;
    geph->toes += 18.0; if (geph->toes<0.0) geph->toes += 24*3600.0;

    //trace_(4, "decode_type1020: prn=%c%3d tof=%10.0f toe=%10.0f week=%4i\n",'R',geph->prn,geph->tofs,geph->toes,get_week_number_());

    return 2;
}
/* decode type 1041: NavIC/IRNSS ephemerides ---------------------------------*/
static int decode_type1041_(unsigned char *buff, int len, sat_eph_t *eph)
{
    double sqrtA;
    int i=24+12;
    
    if (i+482-12<=len*8) {
        eph->sys   ='I';
        eph->prn   =getbitu(buff,i, 6);              i+= 6;
        eph->week  =getbitu(buff,i,10)+2048;         i+=10;
        eph->f0    =getbits(buff,i,22)*P2_31;        i+=22;
        eph->f1    =getbits(buff,i,16)*P2_43;        i+=16;
        eph->f2    =getbits(buff,i, 8)*P2_55;        i+= 8;
        eph->sva   =getbitu(buff,i, 4);              i+= 4;
        eph->tocs  =getbitu(buff,i,16)*16.0;         i+=16;
        eph->tgd[0]=getbits(buff,i, 8)*P2_31;        i+= 8;
        eph->deln  =getbits(buff,i,22)*P2_41*SC2RAD; i+=22;
        eph->iode  =getbitu(buff,i, 8);              i+= 8+10; /* IODEC */
        eph->svh   =getbitu(buff,i, 2);              i+= 2; /* L5+Sflag */
        eph->cuc   =getbits(buff,i,15)*P2_28;        i+=15;
        eph->cus   =getbits(buff,i,15)*P2_28;        i+=15;
        eph->cic   =getbits(buff,i,15)*P2_28;        i+=15;
        eph->cis   =getbits(buff,i,15)*P2_28;        i+=15;
        eph->crc   =getbits(buff,i,15)*0.0625;       i+=15;
        eph->crs   =getbits(buff,i,15)*0.0625;       i+=15;
        eph->idot  =getbits(buff,i,14)*P2_43*SC2RAD; i+=14;
        eph->M0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->toes  =getbitu(buff,i,16)*16.0;         i+=16;
        eph->e     =getbitu(buff,i,32)*P2_33;        i+=32;
        sqrtA      =getbitu(buff,i,32)*P2_19;        i+=32;
        eph->OMG0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->omg   =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->OMGd  =getbits(buff,i,22)*P2_41*SC2RAD; i+=22;
        eph->i0    =getbits(buff,i,32)*P2_31*SC2RAD;
    }
    else {
        //trace(2,"rtcm3 1041 length error: len=%d\n",len);
        return -1;
    }
    //trace(4,"decode_type1041: prn=%d iode=%d toe=%.0f\n",prn,eph->iode,eph->toes);
    
    eph->A=sqrtA*sqrtA;
    eph->iodc=eph->iode;

    return 2;
}
/* decode type 1042/63: beidou ephemerides -----------------------------------*/
extern int decode_type1042_(unsigned char *buff, int len, sat_eph_t *eph)
{
    double sqrtA;
    int i=24+12;
    memset(eph,0,sizeof(sat_eph_t));
    
    if (i+499<=len*8) {
        eph->sys   ='C';
        eph->prn   =getbitu(buff,i, 6);              i+= 6;
        eph->week  =getbitu(buff,i,13);              i+=13;
        eph->week +=1356;/* BDT week to GPS week */
        eph->sva   =getbitu(buff,i, 4);              i+= 4;
        eph->idot  =getbits(buff,i,14)*P2_43*SC2RAD; i+=14;
        eph->iode  =getbitu(buff,i, 5);              i+= 5; /* AODE */
        eph->tocs  =getbitu(buff,i,17)*8.0;          i+=17;
        eph->f2    =getbits(buff,i,11)*P2_66;        i+=11;
        eph->f1    =getbits(buff,i,22)*P2_50;        i+=22;
        eph->f0    =getbits(buff,i,24)*P2_33;        i+=24;
        eph->iodc  =getbitu(buff,i, 5);              i+= 5; /* AODC */
        eph->crs   =getbits(buff,i,18)*P2_6;         i+=18;
        eph->deln  =getbits(buff,i,16)*P2_43*SC2RAD; i+=16;
        eph->M0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cuc   =getbits(buff,i,18)*P2_31;        i+=18;
        eph->e     =getbitu(buff,i,32)*P2_33;        i+=32;
        eph->cus   =getbits(buff,i,18)*P2_31;        i+=18;
        sqrtA      =getbitu(buff,i,32)*P2_19;        i+=32;
        eph->toes  =getbitu(buff,i,17)*8.0;          i+=17;
        eph->cic   =getbits(buff,i,18)*P2_31;        i+=18;
        eph->OMG0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cis   =getbits(buff,i,18)*P2_31;        i+=18;
        eph->i0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->crc   =getbits(buff,i,18)*P2_6;         i+=18;
        eph->omg   =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->OMGd  =getbits(buff,i,24)*P2_43*SC2RAD; i+=24;
        eph->tgd[0]=getbits(buff,i,10)*1E-10;        i+=10;
        eph->tgd[1]=getbits(buff,i,10)*1E-10;        i+=10;
        eph->svh   =getbitu(buff,i, 1);              i+= 1;
    }
    else {
        return -1;
    }
    //trace_(4,"decode_type1042: prn=%c%3d iode=%4d toe=%10.0f\n",eph->sys,eph->prn,eph->iode,eph->toes);
    
    eph->tocs+=14.0; /* BDT to GPST */
    eph->toes+=14.0; /* BDT to GPST */
    eph->A=sqrtA*sqrtA;

    return 2;
}

/* decode type 1044: qzss ephemerides (ref [15]) -----------------------------*/
extern int decode_type1044_(unsigned char *buff, int len, sat_eph_t *eph)
{
    double sqrtA;
    int i=24+12;
    memset(eph,0,sizeof(sat_eph_t));
    
    if (i+473<=len*8) {
        eph->sys   ='J';
        eph->prn   =getbitu(buff,i, 4);              i+= 4;
        eph->tocs  =getbitu(buff,i,16)*16.0;         i+=16;
        eph->f2    =getbits(buff,i, 8)*P2_55;        i+= 8;
        eph->f1    =getbits(buff,i,16)*P2_43;        i+=16;
        eph->f0    =getbits(buff,i,22)*P2_31;        i+=22;
        eph->iode  =getbitu(buff,i, 8);              i+= 8;
        eph->crs   =getbits(buff,i,16)*P2_5;         i+=16;
        eph->deln  =getbits(buff,i,16)*P2_43*SC2RAD; i+=16;
        eph->M0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cuc   =getbits(buff,i,16)*P2_29;        i+=16;
        eph->e     =getbitu(buff,i,32)*P2_33;        i+=32;
        eph->cus   =getbits(buff,i,16)*P2_29;        i+=16;
        sqrtA      =getbitu(buff,i,32)*P2_19;        i+=32;
        eph->toes  =getbitu(buff,i,16)*16.0;         i+=16;
        eph->cic   =getbits(buff,i,16)*P2_29;        i+=16;
        eph->OMG0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cis   =getbits(buff,i,16)*P2_29;        i+=16;
        eph->i0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->crc   =getbits(buff,i,16)*P2_5;         i+=16;
        eph->omg   =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->OMGd  =getbits(buff,i,24)*P2_43*SC2RAD; i+=24;
        eph->idot  =getbits(buff,i,14)*P2_43*SC2RAD; i+=14;
        eph->code  =getbitu(buff,i, 2);              i+= 2;
        eph->week  =getbitu(buff,i,10)+2048;         i+=10;
        eph->sva   =getbitu(buff,i, 4);              i+= 4;
        eph->svh   =getbitu(buff,i, 6);              i+= 6;
        eph->tgd[0]=getbits(buff,i, 8)*P2_31;        i+= 8;
        eph->iodc  =getbitu(buff,i,10);              i+=10;
        eph->fit   =getbitu(buff,i, 1)?0.0:2.0; /* 0:2hr,1:>2hr */
    }
    else {
        return -1;
    }
    //trace_(4,"decode_type1044: prn=%c%3d iode=%4d toe=%10.0f\n",eph->sys,eph->prn,eph->iode,eph->toes);
    
    eph->A=sqrtA*sqrtA;
    eph->flag=1; /* fixed to 1 */

    return 2;
}
/* decode type 1045: galileo satellite ephemerides (ref [15]) ----------------*/
extern int decode_type1045_(unsigned char *buff, int len, sat_eph_t *eph)
{
    double sqrtA;
    int i=24+12,e5a_hs,e5a_dvs,rsv;
    memset(eph,0,sizeof(sat_eph_t));
    
    if (i+484<=len*8) {
        eph->sys   ='E';
        eph->prn   =getbitu(buff,i, 6);              i+= 6;
        eph->week  =getbitu(buff,i,12);              i+=12; /* gst-week */
        eph->week +=1024; /* gal-week = gst-week + 1024 */
        eph->iode  =getbitu(buff,i,10);              i+=10;
        eph->sva   =getbitu(buff,i, 8);              i+= 8;
        eph->idot  =getbits(buff,i,14)*P2_43*SC2RAD; i+=14;
        eph->tocs  =getbitu(buff,i,14)*60.0;         i+=14;
        eph->f2    =getbits(buff,i, 6)*P2_59;        i+= 6;
        eph->f1    =getbits(buff,i,21)*P2_46;        i+=21;
        eph->f0    =getbits(buff,i,31)*P2_34;        i+=31;
        eph->crs   =getbits(buff,i,16)*P2_5;         i+=16;
        eph->deln  =getbits(buff,i,16)*P2_43*SC2RAD; i+=16;
        eph->M0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cuc   =getbits(buff,i,16)*P2_29;        i+=16;
        eph->e     =getbitu(buff,i,32)*P2_33;        i+=32;
        eph->cus   =getbits(buff,i,16)*P2_29;        i+=16;
        sqrtA      =getbitu(buff,i,32)*P2_19;        i+=32;
        eph->toes  =getbitu(buff,i,14)*60.0;         i+=14;
        eph->cic   =getbits(buff,i,16)*P2_29;        i+=16;
        eph->OMG0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cis   =getbits(buff,i,16)*P2_29;        i+=16;
        eph->i0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->crc   =getbits(buff,i,16)*P2_5;         i+=16;
        eph->omg   =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->OMGd  =getbits(buff,i,24)*P2_43*SC2RAD; i+=24;
        eph->tgd[0]=getbits(buff,i,10)*P2_32;        i+=10; /* E5a/E1 */
        e5a_hs    =getbitu(buff,i, 2);              i+= 2; /* OSHS */
        e5a_dvs   =getbitu(buff,i, 1);              i+= 1; /* OSDVS */
        rsv       =getbitu(buff,i, 7);
    }
    else {
        //trace_(2,"rtcm3 1045 length error: len=%d\n",len);
        return -1;
    }
    //trace_(4,"decode_type1045: prn=%c%3d iode=%4d toe=%10.0f\n",eph->sys,eph->prn,eph->iode,eph->toes);
    
    eph->A=sqrtA*sqrtA;
    eph->svh=(e5a_hs<<4)+(e5a_dvs<<3);
    eph->code=(1<<1)+(1<<8); /* data source = F/NAV+E5a */
    eph->iodc=eph->iode;

    return 2;
}
/* decode type 1046: galileo satellite ephemerides (extension for IGS MGEX) --*/
extern int decode_type1046_(unsigned char *buff, int len, sat_eph_t *eph)
{
    double sqrtA;
    int i=24+12,e5b_hs,e5b_dvs,e1_hs,e1_dvs;
    memset(eph,0,sizeof(sat_eph_t));
    
    if (i+484<=len*8) {
        eph->sys   ='E';
        eph->prn   =getbitu(buff,i, 6);              i+= 6;
        eph->week  =getbitu(buff,i,12);              i+=12;
        eph->week +=1024; /* gal-week = gst-week + 1024 */
        eph->iode  =getbitu(buff,i,10);              i+=10;
        eph->sva   =getbitu(buff,i, 8);              i+= 8;
        eph->idot  =getbits(buff,i,14)*P2_43*SC2RAD; i+=14;
        eph->tocs  =getbitu(buff,i,14)*60.0;         i+=14;
        eph->f2    =getbits(buff,i, 6)*P2_59;        i+= 6;
        eph->f1    =getbits(buff,i,21)*P2_46;        i+=21;
        eph->f0    =getbits(buff,i,31)*P2_34;        i+=31;
        eph->crs   =getbits(buff,i,16)*P2_5;         i+=16;
        eph->deln  =getbits(buff,i,16)*P2_43*SC2RAD; i+=16;
        eph->M0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cuc   =getbits(buff,i,16)*P2_29;        i+=16;
        eph->e     =getbitu(buff,i,32)*P2_33;        i+=32;
        eph->cus   =getbits(buff,i,16)*P2_29;        i+=16;
        sqrtA      =getbitu(buff,i,32)*P2_19;        i+=32;
        eph->toes  =getbitu(buff,i,14)*60.0;         i+=14;
        eph->cic   =getbits(buff,i,16)*P2_29;        i+=16;
        eph->OMG0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->cis   =getbits(buff,i,16)*P2_29;        i+=16;
        eph->i0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->crc   =getbits(buff,i,16)*P2_5;         i+=16;
        eph->omg   =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
        eph->OMGd  =getbits(buff,i,24)*P2_43*SC2RAD; i+=24;
        eph->tgd[0]=getbits(buff,i,10)*P2_32;        i+=10; /* E5a/E1 */
        eph->tgd[1]=getbits(buff,i,10)*P2_32;        i+=10; /* E5b/E1 */
        e5b_hs     =getbitu(buff,i, 2);              i+= 2; /* E5b OSHS */
        e5b_dvs    =getbitu(buff,i, 1);              i+= 1; /* E5b OSDVS */
        e1_hs      =getbitu(buff,i, 2);              i+= 2; /* E1 OSHS */
        e1_dvs     =getbitu(buff,i, 1);              i+= 1; /* E1 OSDVS */
    }
    else {
        return -1;
    }
    //trace_(4,"decode_type1046: prn=%c%3d iode=%3d toe=%10.0f\n",seph->sys,seph->prn,seph->iode,seph->toes);
    
    eph->A=sqrtA*sqrtA;
    eph->svh=(e5b_hs<<7)+(e5b_dvs<<6)+(e1_hs<<1)+(e1_dvs<<0);
    eph->code=(1<<0)+(1<<2)+(1<<9); /* data source = I/NAV+E1+E5b */
    eph->iodc=eph->iode;

    return 2;
}

extern int decode_rtcm_data(unsigned char *buff, int nbyte, int *type, int *crc, int *staid, double *tow, int *sync, int *prn, int *frq, int *week)
{
    int i = 0, ret = 0, len = 0;
    if (nbyte < 3) return 0;
    len=getbitu(buff,14,10)+3; /* length without parity */
    if (nbyte<len+3) return 0;
    i = 24;
    *type = getbitu(buff, i, 12); i += 12;
    /* decode rtcm3 message */
    if ((*type == 1074 || *type == 1075 || *type == 1076 || *type == 1077)|| /* GPS */
        (*type == 1094 || *type == 1095 || *type == 1096 || *type == 1097)|| /* GAL */
        (*type == 1104 || *type == 1105 || *type == 1106 || *type == 1107)|| /* SBS */
        (*type == 1114 || *type == 1115 || *type == 1116 || *type == 1117))   /* QZS */
    {
        /* GPS, GAL, SBS, QZS */
        *staid = getbitu(buff, i, 12);           i += 12;
        *tow   = getbitu(buff, i, 30) * 0.001;   i += 30;
        *sync  = getbitu(buff, i,  1);           i +=  1;
        ret = sync?0:1;
    }
    if (*type == 1084 || *type == 1085 || *type == 1086 || *type == 1087)
    {
        /* GLO */
        *staid = getbitu(buff, i, 12);				  i += 12;
        double dow  = getbitu(buff, i,  3);           i +=  3;
        double tod  = getbitu(buff, i, 27) * 0.001;   i += 27;
        *sync  = getbitu(buff, i,  1);                i +=  1;
        *tow  = dow * 24.0 * 3600.0 + tod - 3.0 * 3600.0 + 18.0;
        ret = sync?0:1;
    }
    if (*type == 1124 || *type == 1125 || *type == 1126 || *type == 1127)
    {
        /* BDS */
        *staid = getbitu(buff, i, 12);           i += 12;
        *tow   = getbitu(buff, i, 30) * 0.001;   i += 30;
        *sync  = getbitu(buff,i, 1);             i +=  1;
        *tow += 14.0; /* BDT -> GPST */
        ret = *sync?0:1;
    }
    if (*type == 1019)
    {
        *prn   =getbitu(buff,i, 6);              i+= 6;
        *week  =getbitu(buff,i,10);              i+=10;
        *week +=2048;
    }
    if (*type == 1020)
    {
        *prn   =getbitu(buff,i, 6);              i+= 6;
        *frq   =getbitu(buff,i, 5)-7;            i+= 5+2+2;
    }
    if (*type == 1042)
    {
        *prn   =getbitu(buff,i, 6);              i+= 6;
        *week  =getbitu(buff,i,13);              i+=13;
        *week +=1356; /* BDT week to GPS week */
    }
    if (*type == 1044)
    {
        *prn   =getbitu(buff,i, 4);              i+= 4+430;
        *week  =getbitu(buff,i,10);              i+=10;
        *week  +=2048;
    }		
    if (*type == 1045|| *type == 1046)
    {
        *prn   =getbitu(buff,i, 6);              i+= 6;
        *week  =getbitu(buff,i,12);              i+=12; /* gst-week */
        *week +=1024 ; /* gal-week = gst-week + 1024 */
    }
    if (*type == 1005|| *type == 1006|| *type == 1007|| *type == 1008)
    {
        *staid=getbitu(buff,i,12); i+=12;
		ret = 5;
    }
    /* check parity */
    if (crc24q(buff, len) != getbitu(buff, len * 8, 24)) {
        *crc = 1;
        return 0;
    }
    return ret;
}
extern int decode_rtcm_type(unsigned char* buff, int nbyte)
{
    int len = 0, i = 24, type = 0;
    if (nbyte<=3) return 0;
    len=getbitu(buff,14,10)+3; /* length without parity */
    if (nbyte<(len+3)) return 0;

    i = 24;
    type = getbitu(buff, i, 12); i += 12;
    return type;
}

/* encode type 1005: stationary rtk reference station arp --------------------*/
extern int encode_type1005(unsigned char* buff, int staid, double* p)
{
    int i=0,nbit=0,len=0;
    unsigned int crc=0;
    /* set preamble and reserved */
    setbitu(buff,i, 8,RTCM3PREAMB); i+= 8;
    setbitu(buff,i, 6,0          ); i+= 6;
    setbitu(buff,i,10,0          ); i+=10;
    /* body */
    setbitu(buff,i,12,1005       ); i+=12; /* message no */
    setbitu(buff,i,12,staid      ); i+=12; /* ref station id */
    setbitu(buff,i, 6,0          ); i+= 6; /* itrf realization year */
    setbitu(buff,i, 1,1          ); i+= 1; /* gps indicator */
    setbitu(buff,i, 1,1          ); i+= 1; /* glonass indicator */
    setbitu(buff,i, 1,0          ); i+= 1; /* galileo indicator */
    setbitu(buff,i, 1,0          ); i+= 1; /* ref station indicator */
    set38bits(buff,i,p[0]/0.0001 ); i+=38; /* antenna ref point ecef-x */
    setbitu(buff,i, 1,1          ); i+= 1; /* oscillator indicator */
    setbitu(buff,i, 1,0          ); i+= 1; /* reserved */
    set38bits(buff,i,p[1]/0.0001 ); i+=38; /* antenna ref point ecef-y */
    setbitu(buff,i, 2,0          ); i+= 2; /* quarter cycle indicator */
    set38bits(buff,i,p[2]/0.0001 ); i+=38; /* antenna ref point ecef-z */
    nbit=i;
    /* padding to align 8 bit boundary */
    for (i=nbit;i%8;i++) {
        setbitu(buff,i,1,0);
    }
    /* message length (header+data) (bytes) */
    if ((len=i/8)>=3+1024) {
        /* generate rtcm 3 message length error */
        return 0;
    }
    /* message length without header and parity */
    setbitu(buff,14,10,len-3);
    
    /* crc-24q */
    crc=crc24q(buff,len);
    setbitu(buff,i,24,crc);
    
    /* length total (bytes) */
    return len+3;
}

/* encode user defined message to store the received time tag */
extern int encode_type4095(unsigned char* buff, int staid, unsigned int week, double tow)
{
    int i = 0, nbit = 0, len = 0;
    unsigned int crc = 0;
    /* set preamble and reserved */
    setbitu(buff, i, 8, RTCM3PREAMB); i += 8;
    setbitu(buff, i, 6, 0); i += 6;
    setbitu(buff, i, 10, 0); i += 10;
    /* body */
    setbitu(buff, i, 12, 4095); i += 12; /* message no */

	setbitu(buff, i, 12, staid); i += 12; /* ref station id */
	setbitu(buff, i, 10, week);                     i += 10; /* week number */
    setbitu(buff, i, 30, (unsigned int)(tow*1000)); i += 30; /* week second */

    nbit = i;
    /* padding to align 8 bit boundary */
    for (i = nbit; i % 8; i++) {
        setbitu(buff, i, 1, 0);
    }
    /* message length (header+data) (bytes) */
    if ((len = i / 8) >= 3 + 1024) {
        /* generate rtcm 3 message length error */
        return 0;
    }
    /* message length without header and parity */
    setbitu(buff, 14, 10, len - 3);

    /* crc-24q */
    crc = crc24q(buff, len);
    setbitu(buff, i, 24, crc);

    /* length total (bytes) */
    return len + 3;
}

/* decode user defined message to store the received time tag */
extern int decode_type4095(unsigned char* buff, int len, int *staid, unsigned int *week, double *tow)
{
    int i = 24 + 12;

    if ((i + (12+10+30)) <= len * 8) {
		*staid = getbitu(buff, i, 12); i += 12; /* station ID */
		*week  = getbitu(buff, i, 10); i += 10; /* week number */
        *week += 2048;
        *tow = getbitu(buff, i, 30)*0.001; /* week second */
    }
    else {
        return -1;
    }
    return 5;
}

/* encode user defined message to store the rover coordinates */
extern int encode_type4094(unsigned char* buff, int vrsid, double* rov_xyz, double* vrs_xyz)
{
    int i = 0, nbit = 0, len = 0;
    unsigned int crc = 0;
    /* set preamble and reserved */
    setbitu(buff, i, 8, RTCM3PREAMB); i += 8;
    setbitu(buff, i, 6, 0); i += 6;
    setbitu(buff, i, 10, 0); i += 10;
    /* body */
    setbitu(buff, i, 12, 4094); i += 12; /* message no */
    setbitu(buff, i, 12, vrsid); i += 12; /* vrs id */
    set38bits(buff, i, rov_xyz[0] / 0.0001); i += 38; /* ecef-x */
    set38bits(buff, i, rov_xyz[1] / 0.0001); i += 38; /* ecef-y */
    set38bits(buff, i, rov_xyz[2] / 0.0001); i += 38; /* ecef-z */
    set38bits(buff, i, vrs_xyz[0] / 0.0001); i += 38; /* ecef-x */
    set38bits(buff, i, vrs_xyz[1] / 0.0001); i += 38; /* ecef-y */
    set38bits(buff, i, vrs_xyz[2] / 0.0001); i += 38; /* ecef-z */
    nbit = i;
    /* padding to align 8 bit boundary */
    for (i = nbit; i % 8; i++) {
        setbitu(buff, i, 1, 0);
    }
    /* message length (header+data) (bytes) */
    if ((len = i / 8) >= 3 + 1024) {
        /* generate rtcm 3 message length error */
        return 0;
    }
    /* message length without header and parity */
    setbitu(buff, 14, 10, len - 3);

    /* crc-24q */
    crc = crc24q(buff, len);
    setbitu(buff, i, 24, crc);

    /* length total (bytes) */
    return len + 3;
}

/* decode user defined message to store the rover coordinates */
extern int decode_type4094(unsigned char* buff, int len, int *vrsid, double* rov_xyz, double* vrs_xyz)
{
    int i = 24 + 12;

    if (i + (12+38*6) <= len * 8) {
        *vrsid = getbitu(buff, i, 12); i += 12;
        rov_xyz[0] = getbits_38(buff, i) * 0.0001; i += 38;
        rov_xyz[1] = getbits_38(buff, i) * 0.0001; i += 38;
        rov_xyz[2] = getbits_38(buff, i) * 0.0001; i += 38;
        vrs_xyz[0] = getbits_38(buff, i) * 0.0001; i += 38;
        vrs_xyz[1] = getbits_38(buff, i) * 0.0001; i += 38;
        vrs_xyz[2] = getbits_38(buff, i) * 0.0001;
    }
    else {
        return -1;
    }
    return 5;
}

extern int change_rtcm_id(unsigned char* buff, int nbyte, int rcvid)
{
    int len = 0, i = 24, type = 0, staid = 0;
	unsigned int crc = 0;
    if (nbyte <= 3) return 0;
    len = getbitu(buff, 14, 10) + 3; /* length without parity */
    if (nbyte < (len + 3)) return 0;

    i = 24;
    type = getbitu(buff, i, 12); i += 12;

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
			crc=crc24q(buff,len);
			setbitu(buff,len*8,24,crc);
		}
    }
    return 0;
}

extern int check_rtcm_type(unsigned char* buff, int nbyte, int* len, int* crc, int *staid)
{
    int type = 0;
    if (nbyte <= 3) return 0;
    if (buff[0] != RTCM3PREAMB) return 0;
    *len = getbitu(buff, 14, 10) + 3; /* length without parity */
    if (nbyte < (*len + 3)) return 0;
    if (crc24q(buff, *len) != getbitu(buff, (*len) * 8, 24))
        *crc = 1;
    else
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
        *staid = getbitu(buff, 24+12, 12);
    }
    else
    {
        *staid = 0;
    }
    *len += 3; /* total len */
    return type; /* type */
}

extern int is_rtcm_eph_type_(int type)
{
    if (type == 1019 || type == 1020 || type == 1041 || type == 1042 || type == 1043 || type == 1044 || type == 1045 || type == 1046)
        return 1;
    else
        return 0;
}
extern int is_rtcm_ssr_type_(int type)
{
    return 0;
}
extern int is_rtcm_obs_type_(int type)
{
    if (type == 1071 || type == 1072 || type == 1073 || type == 1074 || type == 1075 || type == 1076 || type == 1077 || /* GPS */
        type == 1081 || type == 1082 || type == 1083 || type == 1084 || type == 1085 || type == 1086 || type == 1087 || /* GLO */
        type == 1091 || type == 1092 || type == 1093 || type == 1094 || type == 1095 || type == 1096 || type == 1097 || /* GAL */
        type == 1101 || type == 1102 || type == 1103 || type == 1104 || type == 1105 || type == 1106 || type == 1107 || /* SBS */
        type == 1111 || type == 1112 || type == 1113 || type == 1114 || type == 1115 || type == 1116 || type == 1117 || /* QZS */
        type == 1121 || type == 1122 || type == 1123 || type == 1124 || type == 1125 || type == 1126 || type == 1127 || /* BDS */
        type == 1131 || type == 1132 || type == 1133 || type == 1134 || type == 1135 || type == 1136 || type == 1137 || /* IRN */
        type == 1001 || type == 1002 || type == 1003 || type == 1004 ||	/* RTCM 2.x */
        type == 1009 || type == 1010 || type == 1011 || type == 1012) /* RTCM 2.x */
        return 1;
    else
        return 0;
}
extern int is_rtcm_sta_type_(int type)
{
    if (type == 1005 || type == 1006 || type == 1007 || type == 1008 || type == 1033 || type == 1230)
        return 1;
    else
        return 0;
}
extern int is_rtcm_msm4_(int type)
{
    if (type == 1074 || type == 1084 || type == 1094 || type == 1104 || type == 1114 || type == 1124 || type == 1134)
        return 1;
    else
        return 0;
}
extern int is_rtcm_msm5_(int type)
{
    if (type == 1075 || type == 1085 || type == 1095 || type == 1105 || type == 1115 || type == 1125 || type == 1135)
        return 1;
    else
        return 0;
}
extern int is_rtcm_msm6_(int type)
{
    if (type == 1076 || type == 1086 || type == 1096 || type == 1106 || type == 1116 || type == 1126 || type == 1136)
        return 1;
    else
        return 0;
}
extern int is_rtcm_msm7_(int type)
{
    if (type == 1077 || type == 1087 || type == 1097 || type == 1107 || type == 1117 || type == 1127 || type == 1137)
        return 1;
    else
        return 0;
}