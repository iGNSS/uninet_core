//------------------------------------------------------------------------------
#ifndef _GNSS_PROC_PP_H_
#define _GNSS_PROC_PP_H_
//------------------------------------------------------------------------------
#include <vector>

#ifdef __cplusplus
extern "C" {
#endif 

	typedef struct
	{
		char t; // observation type
		char n; // band/frequency => 1, 2, ..., 8
		char a; // attribute => tracking mode or channel, e.g., I, Q, etc
	}raw_obs_type_t;

	typedef struct
	{
		char s;
		std::vector< raw_obs_type_t > obs_type;
	}sys_obs_type_t;

	inline bool operator==(const sys_obs_type_t& lhs, const char& rhs) { return lhs.s == rhs; }
	 
	typedef struct
	{
		char	rcvName_[21];
		char	antType_[21];
		double  antNEU_[3];
		double	rcvXYZ_[3];
		int		rinexVersion_;
		std::vector<sys_obs_type_t> sys_type;
		int obsTypeNum;
		int obsTypeLoc[7]; /* C1, P1, P2, L1, L2, D1, D2 */
		int nobs; /* temp use for number of observation in current epoch */
	}rinex_obs_header_t;

	//--------------------------------------------------------------------------
	void engine_pp_main_rinex(const char *fname);
	//--------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif

#endif
