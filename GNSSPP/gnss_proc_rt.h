#pragma once
//------------------------------------------------------------------------------
#ifndef _GNSS_PROC_RT_H_
#define _GNSS_PROC_RT_H_
//------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAX_BUFF_LEN
#define MAX_BUFF_LEN 1024*10
#endif

	typedef struct
	{
		int port; /* port number */
		char add[60]; /* address*/
		char usr[30]; /* user name */
		char pwd[30]; /* password */
		char mnt[60]; /* mountpoint */
		unsigned int nbyte;
		unsigned char buffer[MAX_BUFF_LEN];
	}ntrip_t;

	void engine_rt_main(const char *fname);

#ifdef __cplusplus
}
#endif

#endif
