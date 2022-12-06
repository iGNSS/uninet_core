#include "pch.h"
#include "gnss_core.h"
#include "rtcm_buff.h"
#include "base_setting.h"

#include <algorithm>


extern double ConvertToTimeGPS(int year, int mon, int day, int hour, int min, double sec, int* wn)
{
	if (year < 80)
		year += 2000;
	else if (year > 80 && year < 1900)
		year += 1900;

	int totalDay = (year < 1981) ? (0) : (360);
	for (int yearIndex = 1981; yearIndex < year; ++yearIndex)
	{
		totalDay += 365;
		if ((yearIndex % 4 == 0 && yearIndex % 100 != 0) || yearIndex % 400 == 0)
			++totalDay;
	}
	int dayPerMon[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
	for (int monIndex = 0; monIndex < (mon - 1); ++monIndex)
		totalDay += dayPerMon[monIndex];
	if (mon > 2 && ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0))
		++totalDay;
	totalDay += day;
	*wn = totalDay / 7;
	totalDay -= *wn * 7;
	return  totalDay * 24.0 * 3600.0 + hour * 3600.0 + min * 60.0 + sec;
}

extern double ConvertFromTimeGPS(int wn, double ws, int* year, int* mon, int* day, int* hour, int* min)
{

	unsigned short	dayPerMon[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

	int wnr = (int)(ws / (7.0 * 24.0 * 3600.0));
	wn += wnr;
	ws -= wnr * 7.0 * 24.0 * 3600.0;

	int	weekMin = (int)(ws / 60.0);
	double sec = ws - weekMin * 60.0;
	int	weekHour = weekMin / 60;
	*min = weekMin - weekHour * 60;
	int	weekDay = weekHour / 24;
	*hour = weekHour - weekDay * 24;

	int	totalDay = weekDay + wn * 7;
	if (totalDay < 360)
		*year = 1980;
	else
	{
		*year = 1981;
		totalDay -= 360;
		while (1)
		{
			if (totalDay < 365) break;
			if ((*year % 4 == 0 && *year % 100 != 0) || *year % 400 == 0)
				--totalDay;
			totalDay -= 365;
			++(*year);
		}
	}
	if (totalDay <= dayPerMon[0])
		*mon = 1;
	else
	{
		totalDay -= dayPerMon[0];
		if ((*year % 4 == 0 && *year % 100 != 0) || *year % 400 == 0)
			--totalDay;
		*mon = 2;
		while (1)
		{
			if (totalDay <= dayPerMon[*mon - 1])
			{
				break;
			}
			else
			{
				totalDay -= dayPerMon[*mon - 1];
				++(*mon);
			}
		}
	}
	if (*mon == 2 && ((*year % 4 == 0 && *year % 100 != 0) || *year % 400 == 0))
		++totalDay;
	*day = totalDay;
	return sec;
}

extern void xyz2blh_(const double* xyz, double* blh)
{
	// ecef xyz => blh
	double a = ae_WGS84, finv = finv_WGS84;
	double f = 1.0 / finv, e2 = 2 * f - f * f;
	double x = xyz[0], y = xyz[1], z = xyz[2], lat, lon, ht;
	double R = sqrt(x * x + y * y + z * z);
	double ang = 0.0;
	double lat1 = 0.0;
	double Rw = 0.0;
	double Rn = 0.0;
	if (fabs(z) < 1.0e-5)
	{
		lat = 0.0;
	}
	else
	{
		ang = atan(fabs(z / sqrt(x * x + y * y))) * ((z < 0.0) ? -1.0 : 1.0);
		//if (z<0.0) ang = -ang;
		lat1 = ang;
		Rw = sqrt(1 - e2 * sin(lat1) * sin(lat1));
		Rn = 0.0;
		lat = atan(fabs(tan(ang) * (1 + (a * e2 * sin(lat1)) / (z * Rw))));
		if (z < 0.0) lat = -lat;
		while (fabs(lat - lat1) > 1e-12)
		{
			lat1 = lat;
			Rw = sqrt(1 - e2 * sin(lat1) * sin(lat1));
			lat = atan(fabs(tan(ang) * (1 + (a * e2 * sin(lat1)) / (z * Rw))));
			if (z < 0.0) lat = -lat;
		}
		if (lat > PI) lat = lat - 2.0 * PI;
	}
	if (fabs(x) < 1e-5)
	{
		if (y >= 0.0)
			lon = PI / 2.0;
		else
			lon = 3.0 * PI / 2.0;
	}
	else
	{
		lon = atan(fabs(y / x));
		if (x > 0.0)
		{
			if (y >= 0.0)
				lon = lon;
			else
				lon = 2.0 * PI - lon;
		}
		else
		{
			if (y >= 0.0)
				lon = PI - lon;
			else
				lon = PI + lon;
		}
	}
	Rw = sqrt(1 - e2 * sin(lat) * sin(lat));
	Rn = a / Rw;
	ht = R * cos(ang) / cos(lat) - Rn;
	if (lon > PI) lon = lon - 2.0 * PI;
	blh[0] = lat;
	blh[1] = lon;
	blh[2] = ht;
	return;
}

void blh2xyz_(const double* blh, double* xyz)
{
	// lat, lon, ht => ecef xyz
	double a = ae_WGS84, finv = finv_WGS84;
	double f = 1.0 / finv, e2 = 2 * f - f * f;
	double lat = blh[0], lon = blh[1], ht = blh[2];
	double Rw = sqrt(1 - e2 * sin(lat) * sin(lat));
	double Rn = a / Rw;
	xyz[0] = (Rn + ht) * cos(lat) * cos(lon);
	xyz[1] = (Rn + ht) * cos(lat) * sin(lon);
	xyz[2] = (Rn * (1 - e2) + ht) * sin(lat);
	return;
}

/* baseline distance between two ECEF coordinates */
double baseline_distance(double* xyz1, double* xyz2)
{
	double dxyz[3] = { xyz2[0] - xyz1[0], xyz2[1] - xyz1[1], xyz2[2] - xyz1[2] };
	return sqrt(dxyz[0] * dxyz[0] + dxyz[1] * dxyz[1] + dxyz[2] * dxyz[2]);
}

/* add GNSS ephemeris data to network database */
int add_eph_to_network(network_t* network, char* eph)
{
	int ret = 0;
	if (((sat_eph_t*)eph)->sys == 'R')
	{
		/* GLO*/
		glo_eph_t* seph = (glo_eph_t*)eph;
		std::vector<geph_t>::iterator psat = network->glo_ephs.begin();
		for (; psat != network->glo_ephs.end(); ++psat)
		{
			if (psat->prn == seph->prn) break;
		}
		if (psat == network->glo_ephs.end())
		{
			geph_t cur_eph = { 0 };
			cur_eph.prn = seph->prn;
			cur_eph.eph[1] = *seph;
			network->glo_ephs.push_back(cur_eph);
			ret = 1;
		}
		else
		{
			double dt = seph->toes - psat->eph[1].toes;
			if (fabs(dt) > 0.1)
			{
				psat->eph[0] = psat->eph[1];
				psat->eph[1] = *seph;
				ret = 2;
			}
		}
	}
	else
	{
		sat_eph_t* seph = (sat_eph_t*)eph;
		std::vector<seph_t>::iterator psat = network->sat_ephs.begin();
		for (; psat != network->sat_ephs.end(); ++psat)
		{
			if (psat->sys == seph->sys && psat->prn == seph->prn) break;
		}
		if (psat == network->sat_ephs.end())
		{
			seph_t cur_eph = { 0 };
			cur_eph.sys = seph->sys;
			cur_eph.prn = seph->prn;
			cur_eph.eph[1] = *seph;
			network->sat_ephs.push_back(cur_eph);
			ret = 1;
		}
		else
		{
			double dt = seph->toes - psat->eph[1].toes;
			if (fabs(dt) > 0.1)
			{
				psat->eph[0] = psat->eph[1];
				psat->eph[1] = *seph;
				ret = 2;
			}
		}
	}
	return ret;
}
/* add GNSS observation data to network database */
int add_obs_to_network(network_t* network, int staid, int type, double time, sat_obs_t* obs, unsigned int nobs)
{
	int ret = 0;
	std::vector<base_t>::iterator base = network->bases.begin();
	for (; base != network->bases.end(); ++base)
	{
		if (base->ID == staid) break;
	}
	if (base == network->bases.end())
	{
		/* new station */
		sobs_t new_sobs;
		new_sobs.type = type;
		eobs_t epoch;
		epoch.time = time;
		for (unsigned int i = 0; i < nobs; ++i)
			epoch.obs.push_back(obs[i]);
		new_sobs.eobs.push_back(epoch);
		base_t new_base;
		new_base.ID = staid;
		new_base.conn_ID = 0;
		new_base.pos[0] = 0;
		new_base.pos[1] = 0;
		new_base.pos[2] = 0;
		new_base.ht = 0;
		new_base.sobs.push_back(new_sobs);
		network->bases.push_back(new_base);
		ret = 1;
	}
	else
	{
		/* existing station */
		std::vector<sobs_t>::iterator sobs = base->sobs.begin();
		for (; sobs != base->sobs.end(); ++sobs)
		{
			if (sobs->type == type) break;
		}
		if (sobs == base->sobs.end())
		{
			/* new rtcm type */
			sobs_t new_sobs;
			new_sobs.type = type;
			eobs_t epoch;
			epoch.time = time;
			for (unsigned int i = 0; i < nobs; ++i)
				epoch.obs.push_back(obs[i]);
			new_sobs.eobs.push_back(epoch);
			base->sobs.push_back(new_sobs);
			ret = 2;
		}
		else 
		{
			std::vector<eobs_t>::iterator eobs = sobs->eobs.begin();
			for (; eobs != sobs->eobs.end(); ++eobs)
			{
				double dt = time - eobs->time;
				if (fabs(dt) < 0.005)
				{
					/* same epoch */
					for (unsigned int i = 0; i < nobs; ++i)
					{
						/* find the data existing or not */
						std::vector<sat_obs_t>::iterator cur_obs = eobs->obs.begin();
						for (; cur_obs != eobs->obs.end(); ++cur_obs)
						{
							if (cur_obs->sys == obs[i].sys && cur_obs->prn == obs[i].prn && cur_obs->code == obs[i].code) break;
						}
						if (cur_obs == eobs->obs.end())
							eobs->obs.push_back(obs[i]);
						else
							*cur_obs = obs[i];
					}
					ret = 4;
					break;
				}
			}
			if (eobs == sobs->eobs.end())
			{
				/* process the last epoch */
				network_processor(network);
				/* new epoch */
				eobs_t epoch;
				epoch.time = time;
				for (unsigned int i = 0; i < nobs; ++i)
					epoch.obs.push_back(obs[i]);
				sobs->eobs.push_back(epoch);
				/* clean too old data */
				if (sobs->eobs.size() > 5)
				{
					sobs->eobs.erase(sobs->eobs.begin());
				}
				/* try to get the rover coordinate */
				ret = 3;
			}
		}
	}
	return ret;
}
/* add vrs data to network database */
int add_vrs_to_network(network_t* network, int vrsid, double* rov_xyz, double* vrs_xyz)
{
	int ret = 0;
	std::vector<rove_t>::iterator rove = network->roves.begin();
	for (; rove != network->roves.end(); ++rove)
	{
		if (rove->ID == vrsid) break;
	}
	if (rove == network->roves.end())
	{
		/* new vrs statation */
		rove_t new_rove;
		new_rove.ID = vrsid;
		new_rove.base_ID = 0;
		new_rove.flag = 0;
		new_rove.pos[0] = rov_xyz[0];
		new_rove.pos[1] = rov_xyz[1];
		new_rove.pos[2] = rov_xyz[2];
		new_rove.vrs_pos[0] = vrs_xyz[0];
		new_rove.vrs_pos[1] = vrs_xyz[1];
		new_rove.vrs_pos[2] = vrs_xyz[2];
		network->roves.push_back(new_rove);
		ret = 1;
	}
	else
	{
		/* existing vrs station, update coordinate */
		double dpos[4] = { rov_xyz[0] - rove->pos[0], rov_xyz[1] - rove->pos[1], rov_xyz[2] - rove->pos[2], 0 };
		dpos[3] = sqrt(dpos[0] * dpos[0] + dpos[1] * dpos[1] + dpos[2] * dpos[2]);

		rove->pos[0] = rov_xyz[0];
		rove->pos[1] = rov_xyz[1];
		rove->pos[2] = rov_xyz[2];
		rove->vrs_pos[0] = vrs_xyz[0];
		rove->vrs_pos[1] = vrs_xyz[1];
		rove->vrs_pos[2] = vrs_xyz[2];
		ret = 2;
	}
	return ret;
}
/* update the base station coordinate, will check it is lat(radian),lon(radian), ht or ECEF XYZ */
int add_bas_to_network(network_t* network, int staid, double* pos)
{
	int ret = 0;
	double xyz[4] = { pos[0], pos[1], pos[2], sqrt(pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]) };
	if (fabs(pos[0]) <= HALF_PI && fabs(pos[1]) <= TWO_PI)
	{
		/* coordinate is lat, lon and ht */
		blh2xyz_(pos, xyz);
		ret = 1;
	}
	else if (xyz[3] > 6300000.0)
	{
		/* valid ECEF XYZ */
		ret = 2;
	}
	if (ret > 0)
	{
		std::vector<base_t>::iterator base = network->bases.begin();
		for (; base != network->bases.end(); ++base)
		{
			if (base->ID == staid) break;
		}
		if (base != network->bases.end())
		{
			base->pos[0] = xyz[0];
			base->pos[1] = xyz[1];
			base->pos[2] = xyz[2];
			ret += 2;
		}
		else
		{
			base_t new_base;
			new_base.ID = staid;
			new_base.conn_ID = 0;
			new_base.pos[0] = xyz[0];
			new_base.pos[1] = xyz[1];
			new_base.pos[2] = xyz[2];
			new_base.ht = 0;
			network->bases.push_back(new_base);
		}
	}
	return ret;
}

/* delete bas station from network database */
void del_bas_from_network(network_t *network, int staid)
{
	std::vector<base_t>::iterator base = network->bases.begin();
	for (; base != network->bases.end(); ++base)
	{
		if (base->ID == staid)
		{
			base = network->bases.erase(base);
			break;
		}
	}
}

typedef struct
{
	double time;
	size_t n;
}obs_time_t;

bool operator == (const obs_time_t& t1, const obs_time_t& t2) { return fabs(t1.time - t2.time) < 0.001; }
bool operator <  (const obs_time_t& t1, const obs_time_t& t2) { return t1 == t2 ? (t1.n > t2.n) : (t1.time < t2.time); }

/* process the network data */
/* 1. evaluate the satellite orbit (position, velocity, acceleration, clock bias, clock drift) */
/* 2. receiver based GNSS data processing */
/* 3. form baselines */
/* 4. baseline process */
/* 5. vrs modeling */
/* 6. generate vrs measurement for each vrs rove */
void network_processor(network_t* network)
{
	/* 6. generate vrs measurement for each vrs rove */
	/* currently, just use the nearest base station */
	std::vector<rove_t>::iterator rove = network->roves.begin();
	for (; rove != network->roves.end(); ++rove)
	{
		/* clean the previous epoch data */
		/* find the best base station to generate the correction for the rove */
		size_t bestL = 0;
		size_t currL = 0;
		std::vector<base_t>::iterator base = network->bases.begin();
		double bestD = baseline_distance(rove->vrs_pos, base->pos);
		++base;
		++currL;
		for (; base != network->bases.end(); ++base, ++currL)
		{
			double currD = baseline_distance(rove->vrs_pos, base->pos);
			if (currD < bestD)
			{
				bestL = currL;
				bestD = currD;
			}
		}
		/* */
		base = network->bases.begin() + bestL;
		/* backup old obs */
		eobs_t old_obs = rove->obs;
		/* clear old obs */
		rove->obs.obs.clear();
		rove->obs.time = 0;
#if 0
		/* search new time tag with most observations */
		std::vector<sobs_t>::iterator pobs = base->sobs.begin();
		std::vector<obs_time_t> vobs_time;
		std::vector<obs_time_t>::iterator pobs_time;
		for (; pobs != base->sobs.end(); ++pobs)
		{
			/* search all epochs for current rtcm type */
			std::vector< eobs_t>::iterator peobs = pobs->eobs.begin();
			for (; peobs != pobs->eobs.end(); ++peobs)
			{
				for (pobs_time = vobs_time.begin(); pobs_time != vobs_time.end(); ++pobs_time)
				{
					if (fabs(pobs_time->time - peobs->time) < 0.001)
					{
						/* exist epoch, add obs */
						pobs_time->n += peobs->obs.size();
						break;
					}
				}
				/* new epoch */
				if (pobs_time == vobs_time.end())
				{
					obs_time_t cur_obs_time;
					cur_obs_time.n = 0;
					cur_obs_time.time = pobs_time->time;
					vobs_time.push_back(cur_obs_time);
				}
			}
		}
		std::sort(vobs_time.begin(), vobs_time.end());
		if (vobs_time.size() > 0)
		{
			/* use the */
		}
#endif
		//rove->obs = base->sobs;
		rove->base_ID = base->ID;
		printf("%3i,%3i,%10.4f\n", rove->ID, rove->base_ID, bestD);
	}
}
/* initize network */
void network_init(network_t* network)
{
	network->bases.clear();
	network->conns.clear();
	network->glo_ephs.clear();
	network->roves.clear();
	network->sat_ephs.clear();
	memset(&network->total_packet_received, 0, sizeof(network->total_packet_received));
}

int get_vrs_from_network_(network_t* network, int vrsid, double* vrs_xyz, eobs_t* obs)
{
	std::vector<rove_t>::iterator rove = network->roves.begin();
	for (; rove != network->roves.end(); ++rove)
	{
		if (rove->ID == vrsid) break;
	}
	if (rove != network->roves.end())
	{
		vrs_xyz[0] = rove->vrs_pos[0];
		vrs_xyz[1] = rove->vrs_pos[1];
		vrs_xyz[2] = rove->vrs_pos[2];
		*obs = rove->obs;
	}
	else
	{
	}
	return obs->obs.size();
}

/* get the rtcm encode buffer for the current vrs */
int get_vrs_from_network(network_t* network, int vrsid, unsigned char* buffer)
{
	int nbyte = 0;
	std::vector<rove_t>::iterator rove = network->roves.begin();
	for (; rove != network->roves.end(); ++rove)
	{
		if (rove->ID == vrsid) break;
	}
	if (rove != network->roves.end())
	{
		/* encode the position data */
		nbyte += encode_type1005(buffer + nbyte, vrsid, rove->vrs_pos);
		/* encode the observation data => TBD */
#ifdef _DEBUG
		double blh[3] = { 0 };
		xyz2blh_(rove->vrs_pos, blh);
		printf("vrs_data: %3i,%3i,%3i,%14.4f,%14.4f,%14.4f,%14.9f,%14.9f,%10.4f\n", rove->ID, rove->base_ID, nbyte, rove->vrs_pos[0], rove->vrs_pos[1], rove->vrs_pos[2], blh[0]*R2D, blh[1] * R2D, blh[2]);
#endif
	}
	else
	{
#ifdef _DEBUG
		printf("vrs_data: %3i, not found\n", vrsid);
#endif
	}
	return nbyte;
}

/* delete the vrs rove data */
void del_vrs_from_network(network_t* network, int vrsid)
{
	std::vector<rove_t>::iterator rove = network->roves.begin();
	for (; rove != network->roves.end(); ++rove)
	{
		if (rove->ID == vrsid)
		{
			rove = network->roves.erase(rove);
#ifdef _DEBUG
			printf("vrs_data: %3i removed\n", vrsid);
#endif
			break;
		}
	}
}