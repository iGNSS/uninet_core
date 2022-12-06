//------------------------------------------------------------------------------
#ifndef _GNSS_PROC_PP_RTCM_H_
#define _GNSS_PROC_PP_RTCM_H_
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif 
	/* approximate date for rtcm decoder */
	typedef struct
	{
		int year;
		int mon;
		int day;
		int hour;
	}vdate_t;
	/* vrs coordinate */
	typedef struct
	{
		double xyz[3];
	}vxyz_t;
	//--------------------------------------------------------------------------
	/* main function to process the data */
	void engine_pp_main_rtcm(const char *fname, vdate_t *date, vxyz_t *vxyz, int nxyz);
	/* configure file to process the data */
	void engine_pp_main_ini(const char* fname);
	//--------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif
