#include "gnss_utils.h"
#include "ephemeris.h"

extern char sys2char(int sys)
{
	char ret = ' ';
	if (sys == SYS_GPS)
		ret = 'G';
	else if (sys == SYS_GLO)
		ret = 'R';
	else if (sys == SYS_GAL)
		ret = 'E';
	else if (sys == SYS_CMP)
		ret = 'C';
	else if (sys == SYS_QZS)
		ret = 'J';
	return ret;
}
extern int addobs(obs_t* obs, obsd_t* obsd)
{
	int i = 0, j = 0, nsat = 0;
	obsd_t* dat = obs->data + i;
	for (; i < obs->n; ++i, ++dat)
	{
		if (dat->sat == obsd->sat)
		{
			if (fabs(timediff(dat->time, obsd->time)) > 0.001)
			{
				/* different time tag, reset */
				memset(dat, 0, sizeof(obsd_t));
			}
			for (j = 0; j < (NFREQ + NEXOBS); ++j)
			{
				if (obsd->code[j] > 0)
				{
					dat->code[j] = obsd->code[j];
					dat->D[j] = obsd->D[j];
					dat->L[j] = obsd->L[j];
					dat->LLI[j] = obsd->LLI[j];
					dat->P[j] = obsd->P[j];
					dat->SNR[j] = obsd->SNR[j];
				}
			}
			dat->time = obsd->time;
			break;
		}
	}
	if (i == obs->n)
	{
		/* new satellite */
		if (obs->n < MAXOBS)
		{
			obs->data[obs->n] = *obsd;
			++obs->n;
		}
		else
		{
			/* cannot add */
		}
	}
	return nsat;
}


extern int addepoch(epoch_t* epoch, obsd_t* obsd)
{
	int i = 0, j = 0, prn = 0, nsat = 0;
	sat_obs_t* satobs = epoch->obs + i;
	gtime_t cur_time = gpst2time(epoch->wk, epoch->ws);
	if (epoch->n>0 && fabs(timediff(cur_time, obsd->time)) > 0.001)
	{
		/* different time tag, reset */
		memset(epoch, 0, sizeof(epoch_t));
		epoch->ws = time2gpst(obsd->time, &epoch->wk);
	}
	for (; i < epoch->n; ++i, ++satobs)
	{
		if (satobs->sat == obsd->sat)
		{
			for (j = 0; j < (NFREQ + NEXOBS); ++j)
			{
				if (obsd->code[j] > 0)
				{
					satobs->code[j] = obsd->code[j];
					satobs->D[j] = obsd->D[j];
					satobs->L[j] = obsd->L[j];
					satobs->LLI[j] = obsd->LLI[j];
					satobs->P[j] = obsd->P[j];
					satobs->SNR[j] = obsd->SNR[j];
				}
			}
			break;
		}
	}
	if (i == epoch->n)
	{
		/* new satellite */
		if (epoch->n < MAX_SAT)
		{
			if (epoch->n == 0)
			{
				epoch->ws = time2gpst(obsd->time, &epoch->wk);
			}
			satobs = epoch->obs + epoch->n;
			memset(satobs, 0, sizeof(sat_obs_t));
			satobs->sat = obsd->sat;
			satobs->sys = sys2char(satsys(obsd->sat, &prn));
			satobs->prn = prn;
			if (satobs->sys != ' ')
			{
				for (j = 0; j < (NFREQ + NEXOBS); ++j)
				{
					if (j >= MAX_FRQ) continue;
					satobs->code[j] = obsd->code[j];
					satobs->P[j] = obsd->P[j];
					satobs->L[j] = obsd->L[j];
					satobs->D[j] = obsd->D[j];
					satobs->SNR[j] = obsd->SNR[j];
					satobs->LLI[j] = obsd->LLI[j];
					satobs->wave[j] = sat2wave(obsd->sat, obsd->code[j], NULL);
				}
				++epoch->n;
			}
		}
		else
		{
			/* cannot add */
		}
	}
	return nsat;
}
extern int obs2epoch(obs_t* obs, epoch_t* epoch)
{
	int i = 0, j = 0, prn = 0;
	obsd_t* dat = obs->data + i;
	sat_obs_t* satobs = epoch->obs + 0;
	memset(epoch, 0, sizeof(epoch_t));
	for (; i < obs->n; ++i, ++dat)
	{
		if (epoch->n == 0)
		{
			epoch->ws = time2gpst(obs->data[i].time, &epoch->wk);
		}
		satobs = epoch->obs + epoch->n;
		memset(satobs, 0, sizeof(sat_obs_t));
		satobs->sat = dat->sat;
		satobs->sys = sys2char(satsys(dat->sat, &prn));
		satobs->prn = prn;
		if (satobs->sys == ' ') continue;
		for (j = 0; j < (NFREQ + NEXOBS); ++j)
		{
			if (j >= MAX_FRQ) continue;
			satobs->code[j] = dat->code[j];
			satobs->P[j] = dat->P[j];
			satobs->L[j] = dat->L[j];
			satobs->D[j] = dat->D[j];
			satobs->SNR[j] = dat->SNR[j];
			satobs->LLI[j] = dat->LLI[j];
			satobs->wave[j] = sat2wave(dat->sat, dat->code[j], NULL);
		}
		if (epoch->n< MAX_SAT)
			++epoch->n;
	}
	return epoch->n;
}

extern int obsnav2epoch(obs_t* obs, nav_t* nav, epoch_t* epoch)
{
	int i = 0, j = 0, prn = 0;
	obsd_t* dat = obs->data + i;
	sat_obs_t* satobs = epoch->obs + 0;
	sat_vec_t* satvec = epoch->vec + 0;
	memset(epoch, 0, sizeof(epoch_t));
	for (; i < obs->n; ++i, ++dat)
	{
		if (epoch->n == 0)
		{
			epoch->ws = time2gpst(obs->data[i].time, &epoch->wk);
		}
		satobs = epoch->obs + epoch->n;
		satvec = epoch->vec + epoch->n;
		memset(satobs, 0, sizeof(sat_obs_t));
		memset(satvec, 0, sizeof(sat_vec_t));
		satobs->sat = dat->sat;
		satobs->sys = sys2char(satsys(dat->sat, &prn));
		satobs->prn = prn;
		if (satobs->sys == ' ') continue;
		for (j = 0; j < (NFREQ + NEXOBS); ++j)
		{
			if (j >= MAX_FRQ) continue;
			satobs->code[j] = dat->code[j];
			satobs->P[j] = dat->P[j];
			satobs->L[j] = dat->L[j];
			satobs->D[j] = dat->D[j];
			satobs->SNR[j] = dat->SNR[j];
			satobs->LLI[j] = dat->LLI[j];
			satobs->wave[j] = sat2wave(dat->sat, dat->code[j], nav);
		}
		satposs(obs->data[i].time, obs->data + i, 1, nav, 0, satvec->rs, satvec->dts, &satvec->var, &satvec->svh);
		if (epoch->n < MAX_SAT)
			++epoch->n;
	}
	return epoch->n;
}
extern int epoch2obs(epoch_t* epoch, obs_t* obs)
{
	int i = 0, j = 0;
	sat_obs_t* satobs = epoch->obs + 0;
	sat_vec_t* satvec = epoch->vec + 0;
	obsd_t* dat = obs->data + 0;
	gtime_t cur_time = { 0 };
	memset(obs, 0, sizeof(obs_t));
	for (; i < epoch->n; ++i, ++satobs, ++satvec)
	{
		if (i == 0)
		{
			cur_time = gpst2time(epoch->wk, epoch->ws);
		}
		dat = obs->data + obs->n;
		memset(dat, 0, sizeof(obsd_t));
		dat->sat = satobs->sat;
		dat->time = cur_time;
		for (j = 0; j < MAX_FRQ; ++j)
		{
			if (j >= (NFREQ + NEXOBS)) continue;
			dat->code[j] = satobs->code[j];
			dat->P[j] = satobs->P[j];
			dat->L[j] = satobs->L[j];
			dat->D[j] = satobs->D[j];
			dat->SNR[j] = satobs->SNR[j];
			dat->LLI[j] = satobs->LLI[j];
		}
		++obs->n;
	}
	return obs->n;
}