#include "vrs.h"
//------------------------------------------------------------------------------
#include "gnss.h"

#include "gnss_obs.h"

#include "gnss_core.h"

#include <stdio.h>
#include <time.h>

#include "gnss_utils.h"

#include "gnss.h"

#ifndef MAX_BASE
#define MAX_BASE 20
#endif

#ifndef MAX_ROVE
#define MAX_ROVE 200
#endif

#ifndef MAX_TYPE
#define MAX_TYPE 20
#endif

typedef struct
{
	int type;
	gtime_t time;
	uint64_t count;
}type_t;

#ifndef MAX_BUF_LEN
#define MAX_BUF_LEN (4096)
#endif


/* main decode engine  */
typedef struct
{
	obs_t obs;
	int staid;
	double xyz[3];
	type_t types[MAX_TYPE];
	int ntype;
	uint64_t numofepoch; /* number of epoch marked sync flag */
	uint64_t numofepoch_wo_sync; /* number of epoch without sync flag */
	uint8_t data[MAX_BUF_LEN];
	int nbyte;
	unsigned long packet_received;
	unsigned long packet_crc_failed;
}connect_t;

typedef struct
{
	connect_t base[MAX_BASE];
	connect_t rove[MAX_ROVE];
	rtcm_t rtcm;
	nav_t nav;
	epoch_t epoch;
	int nb;
	int nr;
	uint64_t byte_received;
	uint64_t byte_crc_failed;
	uint64_t packet_received;
	uint64_t packet_received_current;
}decoder_t;

static decoder_t gDecoder = { 0 };
decoder_t* pDecoder = &gDecoder;

/* main process engine */
static network_t gNetwork;
network_t* pNetwork = &gNetwork;

/*-----------------------------------------------------*/
/* data log */
static uint8_t g_log_opt = 1;
static uint8_t g_raw_opt = 1;
FILE* fRAW = NULL; /* raw data log for post-processing */
FILE* fLOG = NULL; /* process status */

/* open files to write */
static void set_output_file(struct tm* ltm)
{
	/* log data */
	if (!fRAW && g_raw_opt)
	{
		char strTime[128] = { 0 };
		sprintf(strTime, "%04d-%0d-%0d-%02d-%02d-%02d.rtcm3", (int)(1900 + ltm->tm_year), (int)(1 + ltm->tm_mon), (int)(ltm->tm_mday), (int)(ltm->tm_hour), (int)(ltm->tm_min), (int)(ltm->tm_sec));
		fRAW = fopen(strTime, "wb");
	}
	if (!fLOG&& g_log_opt)
	{
		char strTime[128] = { 0 };
		sprintf(strTime, "%04d-%0d-%0d-%02d-%02d-%02d.log", (int)(1900 + ltm->tm_year), (int)(1 + ltm->tm_mon), (int)(ltm->tm_mday), (int)(ltm->tm_hour), (int)(ltm->tm_min), (int)(ltm->tm_sec));
		fLOG = fopen(strTime, "w");
	}
}

/* write the log data */
static void output_log_data(char* buffer, int opt)
{
	if (!fLOG&& g_log_opt)
	{
		time_t now = time(0);
		struct tm* ltm = localtime(&now);
		set_output_file(ltm);
	}
	if (fLOG)
	{
		fprintf(fLOG, "%s", buffer);
		fflush(fLOG);
	}
	if (opt)
	{
		printf("%s", buffer);
	}
}

/* write the raw data */
static void output_raw_data(uint8_t* dat_buff, int len_buff)
{
	if (!fRAW && g_raw_opt)
	{
		time_t now = time(0);
		struct tm* ltm = localtime(&now);
		set_output_file(ltm);
	}
	if (fRAW)
	{
		fwrite((void*)dat_buff, len_buff, sizeof(char), fRAW);
		fflush(fRAW);
	}
}

/*-----------------------------------------------------*/

#ifndef MAX_BUF_LEN
#define MAX_BUF_LEN 4096
#endif

static int update_station_info(decoder_t* decoder, int staid)
{
	/* manage stations (check the index and try to add it if not found), return the index */
	int index =-1;
	int i = 0;
	connect_t* connect = decoder->base + i;
	if (staid == 0) return index;
	for (; i < decoder->nb; ++i, ++connect)
	{
		if (connect->staid == staid)
		{
			index = i;
			break;
		}
	}
	if (i == decoder->nb)
	{
		/* new station */
		if (decoder->nb < MAX_BASE)
		{
			/* add new */
			memset(connect, 0, sizeof(connect_t));
			connect->staid = staid;
			index = decoder->nb;
			++decoder->nb;
		}
		else
		{
			/* find the best location to add new */
			i = 0;
			connect = decoder->base + i;
			for (; i < decoder->nb; ++i, ++connect)
			{
				if (connect->staid == 0)
				{
					memset(connect, 0, sizeof(connect_t));
					connect->staid = staid;
					index = i;
					break;
				}
			}
			if (i == decoder->nb)
			{
				/* failed */
			}
		}
	}
	return index;
}

static int update_station_coordinate(decoder_t* decoder, int staid, double* pos)
{
	/* get coordinate */
	connect_t* connect = 0;
	int index = update_station_info(decoder, staid);
	if (index < 0) return -1;
	connect = decoder->base + index;
	connect->xyz[0] = pos[0];
	connect->xyz[1] = pos[1];
	connect->xyz[2] = pos[2];
	return index;
}

static void process_station_observation(network_t *network, int staid, double* xyz, obs_t *obs, nav_t* nav, epoch_t *epoch)
{
	memset(epoch, 0, sizeof(epoch_t));
	if (obsnav2epoch(obs, nav, epoch) > 0)
	{
		/* assign coordinate */
		epoch->pos[0] = xyz[0];
		epoch->pos[1] = xyz[1];
		epoch->pos[2] = xyz[2];
		add_obs_to_network(network, staid, epoch);
	}
	/* clean the data */
	memset(obs, 0, sizeof(obs_t));
	
}

static int update_station_observation(decoder_t* decoder, network_t *network, int staid, obs_t* obs, int completed)
{
	connect_t* connect = 0;
	obsd_t* obsd = 0;
	int i = 0;
	double dt = 0;
	int index = update_station_info(decoder, staid);
	if (index < 0) return -1;
	if (obs->n == 0) return 0;
	connect = decoder->base + index;
	for (i = 0, obsd = obs->data + i; i < obs->n; ++i, ++obsd)
	{
		if (connect->obs.n > 0)
		{
			/* check new epoch or not */
			dt = timediff(obsd->time, connect->obs.data[0].time);
			if (fabs(dt) > 0.001)
			{
				/* epoch is completed by missed data wiithout sync flag */
				++connect->numofepoch_wo_sync;
				process_station_observation(network, staid, connect->xyz, &connect->obs, &decoder->nav, &decoder->epoch);
			}
		}
		addobs(&connect->obs, obsd);
	}
	if (completed) /* data is completed based on sync flag */
	{
		++connect->numofepoch;
		process_station_observation(network, staid, connect->xyz, &connect->obs, &decoder->nav, &decoder->epoch);
	}
	return i;
}

/* update rtcm decode type stats */
static void update_type_stat(connect_t* connect, int type, gtime_t t)
{
	int i = 0;
	for (i = 0; i < connect->ntype; ++i)
	{
		if (connect->types[i].type == type)
		{
			connect->types[i].time = t;
			++connect->types[i].count;
			break;
		}
	}
	if (i == connect->ntype)
	{
		/* */
		if (connect->ntype < MAX_TYPE)
		{
			connect->types[i].type = type;
			connect->types[i].time = t;
			++connect->types[i].count;
			++connect->ntype;
		}
		else
		{
			/* reach maximum */
		}
	}
}
/* decode the rtcm data */
static int process_rtcm_buff(decoder_t *decoder, network_t *network, int type, uint8_t* buffer, int len)
{
	int staid = 0;
	int ret = 0, i = 0, j = 0;
	int index = 0;
	rtcm_t* rtcm = &decoder->rtcm;
	nav_t* nav = &decoder->nav;
	connect_t* connect = 0;
	rtcm->staid = 0;
	memset(&rtcm->sta, 0, sizeof(sta_t));
	ret = input_rtcm3_buff(rtcm, buffer, len, nav);
	/* update stats */
	if (type > 0)
	{
		if (rtcm->staid > 0)
		{
			index = update_station_info(decoder, rtcm->staid);
			if (index >= 0)
			{
				connect = decoder->base + index;
				update_type_stat(connect, type, rtcm->time);
			}
			else
			{
#ifdef _WIN32
				printf("cannot add stations\n");
#endif
			}
		}
		else
		{

		}
	}
	/* decode rtcm data */
	/* sta coordinate */
	if ((type==1005 || type==1006) && ret == 5)
	{
		if (rtcm->staid == 0 || fabs(rtcm->sta.pos[0]) < 0.01 || fabs(rtcm->sta.pos[1]) < 0.01 || fabs(rtcm->sta.pos[2]) < 0.01)
		{
			/* bad station info */
		}
		else
		{
			update_station_coordinate(decoder, rtcm->staid, rtcm->sta.pos);
		}
	}
	/* sta info */
	else if (type == 1007 || type == 1008 || type == 1033)
	{
		/* do nothing now */
	}
	/* eph */
	else if (type == 1019||type == 1020 || type == 1041||type == 1042 || type == 1044 || type == 1045 || type == 1046)
	{
		/* do nothing now */
	}
	/* obs */
	else if ((type >= 1074&&type<=1077) || (type >= 1084 && type <= 1087) || (type >= 1094 && type <= 1097) || (type >= 1104 && type <= 1107) || (type >= 1114 && type <= 1117) || (type >= 1124 && type <= 1127))
	{
		update_station_observation(decoder, network, rtcm->staid, &rtcm->obs, ret==1);
	}
	/* ssr */
	else 
	{
	}
	return ret;
}

static int add_buff_to_connect(connect_t* connect, uint8_t data, int *plen, int *crc, int *staid, double *xyz_rt)
{
	int type = 0;
	if (connect->nbyte >= MAX_BUF_LEN)
	{
		connect->nbyte = 0;
	}
	if (connect->nbyte == 0)
	{
		if (data != 0xD3) return type;
	}
	connect->data[connect->nbyte++] = data;
	if ((type = check_rtcm3_type(connect->data, connect->nbyte, plen, crc, staid, xyz_rt)) > 0)
	{
		connect->nbyte = 0;
	}
	return type;
}

/* set the rtcm data buffer to the engine */
extern int set_rtcm_data_buff(int rcvid, uint8_t* buffer, int nbyte, double *xyz)
{
	int type = 0, crc = 0, staid = 0, sync = 0, prn = 0, frq = 0, week = 0, plen = 0;
	double tow = 0.0, xyz_rt[3] = { 0 };
	int ret = 0;
	time_t now = time(0);
	struct tm* ltm = localtime(&now);
	int loc = 0;
	int idxofpacket = 0;
	int byte_crc_failed = 0;
	char log_buff[255] = { 0 };
	connect_t* connect = 0;
	int index = update_station_info(pDecoder, rcvid); if (index < 0) return 0;
	connect = pDecoder->base + index;
	/* seperate the buffer into various message type */
	for (loc=0;loc<nbyte;++loc)
	{
		/* check rtcm */
		type = add_buff_to_connect(connect, buffer[loc], &plen, &crc, &staid, xyz_rt);
		if (type > 0 && !crc)
		{
			if ((type == 1005 || type == 1006) && xyz != NULL && !(fabs(xyz[0]) < 0.01 || fabs(xyz[1]) < 0.01 || fabs(xyz[0]) < 0.01))
			{
				update_rtcm3_pos(connect->data, plen, rcvid, xyz);
				xyz_rt[0] -= xyz[0];
				xyz_rt[1] -= xyz[1];
				xyz_rt[2] -= xyz[2];
				if (fabs(xyz_rt[0] * xyz_rt[0] + xyz_rt[1] * xyz_rt[1] + xyz_rt[2] * xyz_rt[2]) > 0.001)
				{
					sprintf(log_buff, "coordinate difference %4i,%4i,%10.4f,%10.4f,%10.4f\n", staid, rcvid, xyz_rt[0], xyz_rt[1], xyz_rt[2]);
					output_log_data(log_buff, 1);
				}
			}
			if (rcvid > 0 && staid != rcvid && change_rtcm3_id(connect->data, plen, rcvid)) /* replace station ID */
			{
				//printf("rtcm ID %4i in the data was replaced with %4i\n", staid, rcvid);
			}
			/* only output data if CRC passed */
			if (g_log_opt)
				printf("%04d-%0d-%0d-%02d-%02d-%02d,%04i,%04i,%04i,%i,%i,%04i,%04i\n", 1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec, rcvid, staid, type, sync, crc, plen, nbyte);
			output_raw_data(connect->data, plen);
			/* process the rtcm data */
			process_rtcm_buff(pDecoder, pNetwork, type, connect->data, plen);
			/* skip the processed buffer */
			++idxofpacket;
			++connect->packet_received;
		}
		else if (type>0 && crc)
		{
			/* crc failed */
			++byte_crc_failed;
			++connect->packet_crc_failed;
		}
	}
	/* keep stats */
	pDecoder->byte_received += nbyte;
	pDecoder->byte_crc_failed += byte_crc_failed;
	pDecoder->packet_received_current = idxofpacket;
	pDecoder->packet_received += idxofpacket;
	return idxofpacket;
}

/* add rover coordinate and information */
extern int add_vrs_rover_data(int vrsid, double* xyz)
{
	char log_buffer[255] = { 0 };
	printf("rove: %04i,%14.4f,%14.4f,%14.4f\n", vrsid, xyz[0], xyz[1], xyz[2]);
	return add_vrs_to_network(pNetwork, vrsid, xyz);
}

/* get the rtcm buffer for the rover */
extern int get_vrs_rove_buff(int vrsid, uint8_t* buffer)
{
	int ngps = 0, nglo = 0, ngal = 0, nbds = 0, nqzs = 0;
	int i = 0, sys = 0, prn = 0, nbyte = 0, ret = 0;
	obs_t obs_new = { 0 };
	obsd_t* obsd = pDecoder->rtcm.obs.data + i;
	if (get_vrs_from_network(pNetwork, vrsid, &pDecoder->epoch))
	{
		if (epoch2obs(&pDecoder->epoch, &pDecoder->rtcm.obs))
		{
			/* encode the rtcm data into buffer */
			for (; i < pDecoder->rtcm.obs.n; ++i, ++obsd)
			{
				sys = satsys(obsd->sat, &prn);
				if (sys == SYS_GPS) ++ngps;
				else if (sys == SYS_GLO) ++nglo;
				else if (sys == SYS_GAL) ++ngal;
				else if (sys == SYS_CMP) ++nbds;
				else if (sys == SYS_QZS) ++nqzs;
			}
			pDecoder->rtcm.staid = vrsid;
			if (ngps > 0) nbyte = write_rtcm3_msm(&pDecoder->rtcm, &pDecoder->nav, 1074, (nglo + ngal + nbds + nqzs) > 0, buffer, nbyte);
			if (nglo > 0) nbyte = write_rtcm3_msm(&pDecoder->rtcm, &pDecoder->nav, 1084, (ngal + nbds + nqzs) > 0, buffer, nbyte);
			if (ngal > 0) nbyte = write_rtcm3_msm(&pDecoder->rtcm, &pDecoder->nav, 1094, (nbds + nqzs) > 0, buffer, nbyte);
			if (nbds > 0) nbyte = write_rtcm3_msm(&pDecoder->rtcm, &pDecoder->nav, 1124, nqzs > 0, buffer, nbyte);
			if (nqzs > 0) nbyte = write_rtcm3_msm(&pDecoder->rtcm, &pDecoder->nav, 1114, 0, buffer, nbyte);
			if (fabs(pDecoder->epoch.pos[0]) < 0.001 || fabs(pDecoder->epoch.pos[1]) < 0.001 || fabs(pDecoder->epoch.pos[2]) < 0.001)
			{

			}
			else
			{
				pDecoder->rtcm.sta.pos[0] = pDecoder->epoch.pos[0];
				pDecoder->rtcm.sta.pos[1] = pDecoder->epoch.pos[1];
				pDecoder->rtcm.sta.pos[2] = pDecoder->epoch.pos[2];
				nbyte = write_rtcm3(&pDecoder->rtcm, &pDecoder->nav, 1005, 0, buffer, nbyte);
			}
			memset(&pDecoder->rtcm.obs, 0, sizeof(obs_t));
			for (i = 0; i < nbyte; ++i)
			{
				ret = input_rtcm3(&pDecoder->rtcm, buffer[i], &pDecoder->nav);
				if (ret == 1)
				{
					prn = 0;
				}
			}
			prn = 0;
		}
	}
	return nbyte;
}

extern void del_vrs_rove_data(int vrsid)
{
	int i = 0;
	for (; i < pDecoder->nr; ++i)
	{
		if (pDecoder->rove[i].staid == vrsid)
		{
			memset(pDecoder->rove + i, 0, sizeof(connect_t));
		}
	}
	del_vrs_from_network(pNetwork, vrsid);
}

/* delete base station */
extern void del_vrs_base_data(int staid)
{
	int i = 0;
	for (; i < pDecoder->nb; ++i)
	{
		if (pDecoder->base[i].staid == staid)
		{
			memset(pDecoder->base + i, 0, sizeof(connect_t));
		}
	}
	del_bas_from_network(pNetwork, staid);
}

/* reset the system, clear all variables in memory */
extern void system_reset()
{
	network_init(pNetwork);
}

/* house keeping when system exist */
extern void system_exit()
{
	/* house keeping */
	if (fRAW) fclose(fRAW);
	if (fLOG) fclose(fLOG);
}

/* set the approximate time for offline process */
extern void set_appr_time(int year, int mon, int day, int hour)
{
	double ep[6] = { year, mon, day, hour, 0, 0 };
	pDecoder->rtcm.time = epoch2time(ep);
}

/* control raw data output */
extern void set_raw_data_option(int opt)
{
	if (opt)
	{
		if (!g_raw_opt)
			g_raw_opt = 1;
	}
	else
	{
		if (g_raw_opt)
		{
			if (fRAW) fclose(fRAW);
			g_raw_opt = 0;
		}
	}
}
/* control log data output */
extern void set_log_data_option(int opt)
{
	if (opt)
	{
		if (!g_log_opt)
		{
			g_log_opt = 1;
		}
	}
	else
	{
		if (g_log_opt)
		{
			if (fLOG) fclose(fLOG);
			g_log_opt = 0;
		}
	}
}

extern void system_status_output(FILE* fout)
{
	if (!fout) return;
	int i = 0, j = 0;
	fprintf(fout, "%Iu,total received bytes\r\n", pDecoder->byte_received);
	fprintf(fout, "%Iu,total received bytes with crc failed\r\n", pDecoder->byte_crc_failed);
	fprintf(fout, "%Iu,total packets for current epoch\r\n", pDecoder->packet_received_current);
	fprintf(fout, "%Iu,total packets\r\n", pDecoder->packet_received);
	fprintf(fout, "\r\n");
	for (i = 0; i < pDecoder->nb; ++i)
	{
		fprintf(fout, "%4i,%Iu,%Iu,total epochs with and without sync flag\r\n", pDecoder->base[i].staid, pDecoder->base[i].numofepoch, pDecoder->base[i].numofepoch_wo_sync);
		for (j = 0; j < pDecoder->base[i].ntype; ++j)
		{
			fprintf(fout, "%4i,%4i,%Iu,total rtcm type received\r\n", pDecoder->base[i].staid, pDecoder->base[i].types[j].type, pDecoder->base[i].types[j].count);
		}
	}
	fflush(fout);
}