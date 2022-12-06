#include "gnss_core.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "gmodel.h"
#include "gtime.h"

#define TIME_TOL 0.001

extern int week_number(double time)
{
	return (int)floor(time / (7 * 24 * 3600) + 0.5);
}

extern double week_second(double time)
{
	time -= week_number(time) * 7 * 24 * 3600.0;
	return time;
}
extern int is_time_same(double ws1, double ws2) 
{ 
	int ret = 0;
	double dt = ws1 - ws2;
	if (fabs(dt) <= TIME_TOL)
		ret = 1;
	else
	{
		dt -= floor(dt / (7 * 24 * 3600) + 0.5) * 7 * 24 * 3600.0;
		if (fabs(dt) <= TIME_TOL)
			ret = 1;
	}
	return ret;
}
extern int is_time_less(double ws1, double ws2)
{
	int ret = 0;
	double dt = ws1 - ws2;
	if (dt <-TIME_TOL)
		ret = 1;
	else
	{
		dt -= floor(dt / (7 * 24 * 3600) + 0.5) * 7 * 24 * 3600.0;
		if (dt <-TIME_TOL)
			ret = 1;
	}
	return ret;
}

/* baseline distance between two ECEF coordinates */
extern double baseline_distance(double* xyz1, double* xyz2)
{
	double dxyz[3] = { xyz2[0] - xyz1[0], xyz2[1] - xyz1[1], xyz2[2] - xyz1[2] };
	return sqrt(dxyz[0] * dxyz[0] + dxyz[1] * dxyz[1] + dxyz[2] * dxyz[2]);
}

extern double median_data(double* data, int n)
{
	int i = 0;
	int j = 0;
	double tmp = 0;
	for (i = 0; i < n; ++i)
	{
		for (j = i + 1; j < n; ++j)
		{
			if (data[j] < data[i])
			{
				/* switch to get the smallest at the current location */
				tmp = data[i];
				data[i] = data[j];
				data[j] = tmp;
			}
		}
	}
	return data[n / 2];
}

/* inner product ---------------------------------------------------------------
* inner product of vectors
* args   : double *a,*b     I   vector a,b (n x 1)
*          int    n         I   size of vector a,b
* return : a'*b
*-----------------------------------------------------------------------------*/
static double dot(const double *a, const double *b, int n)
{
    double c=0.0;
    
    while (--n>=0) c+=a[n]*b[n];
    return c;
}
/* euclid norm -----------------------------------------------------------------
* euclid norm of vector
* args   : double *a        I   vector a (n x 1)
*          int    n         I   size of vector a
* return : || a ||
*-----------------------------------------------------------------------------*/
static double norm(const double *a, int n)
{
    return sqrt(dot(a,a,n));
}

/* transform ecef to geodetic postion ------------------------------------------
* transform ecef position to geodetic position
* args   : double *r        I   ecef position {x,y,z} (m)
*          double *pos      O   geodetic position {lat,lon,h} (rad,m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
static void ecef2pos(const double *r, double *pos)
{
    double e2=FE_WGS84*(2.0-FE_WGS84),r2=dot(r,r,2),z,zk,v=RE_WGS84,sinp;
    
    for (z=r[2],zk=0.0;fabs(z-zk)>=1E-4;) {
        zk=z;
        sinp=z/sqrt(r2+z*z);
        v=RE_WGS84/sqrt(1.0-e2*sinp*sinp);
        z=r[2]+v*e2*sinp;
    }
    pos[0]=r2>1E-12?atan(z/sqrt(r2)):(r[2]>0.0?PI/2.0:-PI/2.0);
    pos[1]=r2>1E-12?atan2(r[1],r[0]):0.0;
    pos[2]=sqrt(r2+z*z)-v;
}

/* geometric distance ----------------------------------------------------------
* compute geometric distance and receiver-to-satellite unit vector
* args   : double *rs       I   satellilte position (ecef at transmission) (m)
*          double *rr       I   receiver position (ecef at reception) (m)
*          double *e        O   line-of-sight vector (ecef)
* return : geometric distance (m) (0>:error/no satellite position)
* notes  : distance includes sagnac effect correction
*-----------------------------------------------------------------------------*/
static double geodist_(const double *rs, const double *rr, double *e)
{
    double r;
    int i;
    
    if (norm(rs,3)<RE_WGS84) return -1.0;
    for (i=0;i<3;i++) e[i]=rs[i]-rr[i];
    r=norm(e,3);
    for (i=0;i<3;i++) e[i]/=r;
    return r+OMGE*(rs[0]*rr[1]-rs[1]*rr[0])/CLIGHT;
}
/* ecef to local coordinate transfromation matrix ------------------------------
* compute ecef to local coordinate transfromation matrix
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *E        O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
static void xyz2enu(const double *pos, double *E)
{
    double sinp=sin(pos[0]),cosp=cos(pos[0]),sinl=sin(pos[1]),cosl=cos(pos[1]);
    
    E[0]=-sinl;      E[3]=cosl;       E[6]=0.0;
    E[1]=-sinp*cosl; E[4]=-sinp*sinl; E[7]=cosp;
    E[2]=cosp*cosl;  E[5]=cosp*sinl;  E[8]=sinp;
}
/* multiply matrix -----------------------------------------------------------*/
static void matmul(const char *tr, int n, int k, int m, double alpha, const double *A, const double *B, double beta, double *C)
{
    double d;
    int i,j,x,f=tr[0]=='N'?(tr[1]=='N'?1:2):(tr[1]=='N'?3:4);
    
    for (i=0;i<n;i++) for (j=0;j<k;j++) {
        d=0.0;
        switch (f) {
            case 1: for (x=0;x<m;x++) d+=A[i+x*n]*B[x+j*m]; break;
            case 2: for (x=0;x<m;x++) d+=A[i+x*n]*B[j+x*k]; break;
            case 3: for (x=0;x<m;x++) d+=A[x+i*m]*B[x+j*m]; break;
            case 4: for (x=0;x<m;x++) d+=A[x+i*m]*B[j+x*k]; break;
        }
        if (beta==0.0) C[i+j*n]=alpha*d; else C[i+j*n]=alpha*d+beta*C[i+j*n];
    }
}
/* transform ecef vector to local tangental coordinate -------------------------
* transform ecef vector to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *r        I   vector in ecef coordinate {x,y,z}
*          double *e        O   vector in local tangental coordinate {e,n,u}
* return : none
*-----------------------------------------------------------------------------*/
static void ecef2enu(const double *pos, const double *r, double *e)
{
    double E[9];
    
    xyz2enu(pos,E);
    matmul("NN",3,1,3,1.0,E,r,0.0,e);
}
/* satellite azimuth/elevation angle -------------------------------------------
* compute satellite azimuth/elevation angle
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
*          double *e        I   receiver-to-satellilte unit vevtor (ecef)
*          double *azel     IO  azimuth/elevation {az,el} (rad) (NULL: no output)
*                               (0.0<=azel[0]<2*pi,-pi/2<=azel[1]<=pi/2)
* return : elevation angle (rad)
*-----------------------------------------------------------------------------*/
static double satazel_(const double* pos, const double* e, double* azel)
{
	double az = 0.0, el = PI / 2.0, enu[3];

	if (pos[2] > -RE_WGS84) {
		ecef2enu(pos, e, enu);
		az = dot(enu, enu, 2) < 1E-12 ? 0.0 : atan2(enu[0], enu[1]);
		if (az < 0.0) az += 2 * PI;
		el = asin(enu[2]);
	}
	if (azel) { azel[0] = az; azel[1] = el; }
	return el;
}
/* troposphere model -----------------------------------------------------------
* compute tropospheric delay by standard atmosphere and saastamoinen model
* args   : gtime_t time     I   time
*          double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          double humi      I   relative humidity
* return : tropospheric delay (m)
*-----------------------------------------------------------------------------*/
static double tropmodel_(const double *pos, const double *azel, double humi)
{
    const double temp0=15.0; /* temparature at sea level */
    double hgt,pres,temp,e,z,trph,trpw;
    
    if (pos[2]<-100.0||1E4<pos[2]||azel[1]<=0) return 0.0;
    
    /* standard atmosphere */
    hgt=pos[2]<0.0?0.0:pos[2];
    
    pres=1013.25*pow(1.0-2.2557E-5*hgt,5.2568);
    temp=temp0-6.5E-3*hgt+273.16;
    e=6.108*humi*exp((17.15*temp-4684.0)/(temp-38.45));
    
    /* saastamoninen model */
    z=PI/2.0-azel[1];
    trph=0.0022768*pres/(1.0-0.00266*cos(2.0*pos[0])-0.00028*hgt/1E3)/cos(z);
    trpw=0.002277*(1255.0/temp+0.05)*e/cos(z);
    return trph+trpw;
}

extern int make_vrs_measurement(sat_obs_t *src_obs, sat_vec_t *src_vec, double* src_xyz, int n, double* new_xyz, sat_obs_t *new_obs, sat_vec_t *new_vec)
{
	int i = 0, j = 0, nobs = 0;
	double src_pos[3] = { 0 }; /* src coordinate in lat,lon,ht */
	double new_pos[3] = { 0 }; /* new coordinate in lat,lon,ht */
	double src_azel[2] = { 0 };
	double new_azel[2] = { 0 };
	ecef2pos(src_xyz, src_pos);
	ecef2pos(new_xyz, new_pos);
	for (i = 0; i < n; ++i)
	{
		if (norm(src_vec[i].rs, 3) < 1.0) continue; /* satellite position */
		if (norm(src_vec[i].rs + 3, 3) < 1.0) continue; /* satellite velocity */
		new_obs[nobs] = src_obs[i];
		new_vec[nobs] = src_vec[i];
		/* calculate the source/src/original vector information, unit vector, azimuth/elevation, and troposheric */
		double src_dist = geodist_(src_vec[i].rs, src_xyz, src_vec[i].e);
		satazel_(src_pos, src_vec[i].e, src_azel);
		double src_tro = tropmodel_(src_pos, src_azel, 0.7);
		/* calculate the target/new vector information, unit vector, azimuth/elevation, and troposheric */
		double new_dist = geodist_(new_vec[nobs].rs, new_xyz, new_vec[nobs].e);
		satazel_(new_pos, new_vec[i].e, new_azel);
		double new_tro = tropmodel_(new_pos, new_azel, 0.7);
		double dela_dist = (new_dist + new_tro) - (src_dist + src_tro);
		double dt = dela_dist / CLIGHT;
		double pre_dela_dist = dela_dist;
		while (1)
		{
			new_vec[nobs].rs[0] = src_vec[i].rs[0] + src_vec[i].rs[3] * dt;
			new_vec[nobs].rs[1] = src_vec[i].rs[1] + src_vec[i].rs[4] * dt;
			new_vec[nobs].rs[2] = src_vec[i].rs[2] + src_vec[i].rs[5] * dt;

			new_dist = geodist_(new_vec[nobs].rs, new_xyz, new_vec[nobs].e);
			satazel_(new_pos, new_vec[i].e, new_azel);
			new_tro = tropmodel_(new_pos, new_azel, 0.7);
			dela_dist = (new_dist + new_tro) - (src_dist + src_tro);
			dt = dela_dist / CLIGHT;
			if (fabs(dela_dist - pre_dela_dist) < 1.0e-5)
				break;
			pre_dela_dist = dela_dist;
		}
		for (j = 0; j < MAX_FRQ; ++j)
		{
			if (new_obs[nobs].P[j] != 0.0) new_obs[nobs].P[j] += dela_dist;
			if (new_obs[nobs].L[j] != 0.0 && new_obs[nobs].wave[j] > 0.0)
			{
				new_obs[nobs].L[j] += dela_dist / new_obs[nobs].wave[j];
			}
			else 
			{
				new_obs[nobs].L[j] = 0.0;
			}
			new_obs[nobs].D[j] = 0.0;
		}
		nobs++;
	}
	return nobs;
}

static int add_sta_to_network(network_t* network, int staid)
{
	int index = -1, ib = 0;
	base_t* base = network->bases + ib;
	if (staid == 0) return index; /* ID can not be 0, and need satellites */
	/* check existing station or not */
	for (ib = 0, base = network->bases + ib; ib < network->nb; ++base, ++ib)
	{
		if (base->ID == staid)
		{
			index = ib;
			break;
		}
	}
	if (ib == network->nb) /* new staton */
	{
		if (network->nb < MAX_BASE)
		{
			/* new station */
			base->ID = staid;
			index = network->nb;
			++network->nb;
		}
		else
		{
			/* find the recycle location */
			for (ib = 0, base = network->bases + ib; ib < network->nb; ++base, ++ib)
			{
				if (base->ID == 0)
				{
					base->ID = staid;
					index = ib;
					break;
				}
			}
		}
	}
	return index;
}

/* add GNSS observation data to network database */
extern int add_obs_to_network(network_t* network, int staid, epoch_t *epoch)
{
	int ret = 0;
	int ib = 0, i = 0, j = 0;
	int index = add_sta_to_network(network, staid);
	base_t *base = network->bases + 0;
	double new_time = 0;
	if (staid == 0 || index < 0 || epoch->n == 0) return ret; /* ID can not be 0, and need satellites */
	base = network->bases + index;
	/* existing station */
	for (i = 0; i < MAX_EPOCH; ++i)
	{
		if (base->epochs[i].n == 0) continue;
		if (is_time_same(epoch->ws, base->epochs[i].ws))
		{
			/* exist epoch, update data */
			base->epochs[i] = *epoch; /* may consider to merge the epoch, instead of replace */
			ret = 4;
			break;
		}
		if (is_time_less(epoch->ws, base->epochs[i].ws))
		{
			/* new epoch later then the previous epochs */
			/* insert if not the first epoch */
			if (i > 0)
			{
				/* left shift data before the current location */
				for (j = 1; j < i; ++j)
				{
					base->epochs[j - 1] = base->epochs[j];
				}
				/* inert the data which arrived late, do not update the time tag */
				base->epochs[i - 1] = *epoch;
				++base->numofepoch;
				ret = 5;
			}
		}
	}
	if (i == MAX_EPOCH) /* newest epoch, append at the end */
	{
		/* new epoch */
		/* left shift data before the current location */
		for (j = 1; j < i; ++j)
		{
			base->epochs[j - 1] = base->epochs[j];
		}
		base->epochs[i - 1] = *epoch;
		++base->numofepoch;
		ret = 1;
	}

	if (ret > 0)
	{
		if (fabs(epoch->ws - network->time) > 0.001)
		{
#if 0
			if (network->numofepoch > 0)
			{
				/* update network time tag */
				i = 0;
				new_time = 0.0;
				max_time = 0.0;
				for (ib = 0, base = network->bases + ib; ib < network->nb; ++base, ++ib)
				{
					if (base->epochs[MAX_EPOCH - 1].n > 0)
					{
						if (i == 0)
						{
							new_time = base->epochs[MAX_EPOCH - 1].ws;
						}
						network->ws[i] = base->epochs[MAX_EPOCH - 1].ws - new_time;
						network->ws[i] -= floor(network->ws[i] / (7 * 24 * 3600.0) + 0.5) * (7 * 24 * 3600.0);
						++i;
					}
				}
				if (i > 5)
				{
					new_time += median_data(network->ws, i);
					new_time -= floor(network->ws[i] / (7 * 24 * 3600.0) + 0.5) * (7 * 24 * 3600.0);
				}
				else
				{

				}
				if (fabs(new_time - network->time) > 0.001)
				{
					/* new epoch */
					network_processor(network);
					network->time = new_time;
					network->status = 0;
				}
			}
			else
#endif
			{
				network_processor(network);
				network->time = epoch->ws;
				network->status = 0;
			}
			++network->numofepoch;
		}
		else
		{
			/* old epoch */
		}
	}
	return ret;
}
/* add vrs data to network database, return the index */
extern int add_vrs_to_network(network_t* network, int vrsid, double* xyz)
{
	int ret =-1;
	int ir = 0, bestLoc = 0;
	rove_t* rove = network->roves + ir;
	double bestDis = 0.0;
	double cur_dis = 0.0;
	if (vrsid == 0 || fabs(xyz[0]) < 0.001 || fabs(xyz[1]) < 0.001 || fabs(xyz[2]) < 0.001) return ret; /* ID can not be 0, need valid coordinate */
	for (ir=0, rove = network->roves+ir;ir<network->nr;++ir, ++rove)
	{
		if (rove->ID == vrsid) break;
	}
	if (ir < network->nr) /* existing rove */
	{
		/* update coordinate */
		rove->cur_xyz[0] = xyz[0];
		rove->cur_xyz[1] = xyz[1];
		rove->cur_xyz[2] = xyz[2];
		/* update vrs coordinate or not ? */
		cur_dis = baseline_distance(rove->cur_xyz, rove->vrs_xyz);
		if (cur_dis > 2500.0)
		{
			/* update vrs location using the newest position */
			rove->vrs_xyz[0] = rove->cur_xyz[0];
			rove->vrs_xyz[1] = rove->cur_xyz[1];
			rove->vrs_xyz[2] = rove->cur_xyz[2];
		}
		ret = ir;
	}
	else
	{
		/* new vrs statation */
		/* search nearby coordinate with 2.5 km */
		bestLoc = -1;
		for (ir = 0, rove = network->roves+ir; ir < network->nr; ++ir, ++rove)
		{
			if (rove->ID == 0) continue;
			cur_dis = baseline_distance(xyz, rove->vrs_xyz);
			if (bestLoc < 0 || cur_dis < bestDis)
			{
				bestLoc = ir;
				bestDis = cur_dis;
			}
		}
		if (bestLoc >= 0 && bestDis <= 2500.0) /* find a better ID */
		{
			/* use the current rover */
			rove = network->roves + bestLoc;	//add
			ret = bestLoc;
		}
		else
		{
			/* find the recycle location */
			for (ir = 0, rove = network->roves + ir; ir < network->nr; ++ir, ++rove)
			{
				if (rove->ID == 0)
				{
					rove->ID = vrsid;
					rove->vrs_xyz[0] = xyz[0];
					rove->vrs_xyz[1] = xyz[1];
					rove->vrs_xyz[2] = xyz[2];
					ret = ir;
					break;
				}
			}
			if (ir == network->nr) /* new location */
			{
				if (network->nr < MAX_ROVE)
				{
					rove->ID = vrsid;
					rove->vrs_xyz[0] = xyz[0];
					rove->vrs_xyz[1] = xyz[1];
					rove->vrs_xyz[2] = xyz[2];
					ret = network->nr;
					++network->nr;
				}
				else
				{
					/* can not add the rove station */
				}
			}
		}
	}
	return ret;
}
/* update the base station coordinate, return the index */
extern int add_bas_to_network(network_t* network, int staid, double* xyz)
{
	int ret =-1;
	int ib = 0;
	base_t* base = network->bases + ib;
	if (xyz == 0 || staid == 0 || fabs(xyz[0]) < 0.01 || fabs(xyz[1]) < 0.01 || fabs(xyz[2]) < 0.01) return ret; /* ID cannot be 0 */
	for (ib = 0, base = network->bases + ib; ib < network->nb; ++ib, ++base)
	{
		if (base->ID == staid) break;
	}
	if (ib < network->nb) /* existing station */
	{
		base->xyz[0] = xyz[0];
		base->xyz[1] = xyz[1];
		base->xyz[2] = xyz[2];
		ret = ib;
	}
	else /* new station */
	{
		/* find the location for new station */
		for (ib = 0, base = network->bases + ib; ib < network->nb; ++ib, ++base)
		{
			if (base->ID == 0)
			{
				base->ID = staid;
				base->xyz[0] = xyz[0];
				base->xyz[1] = xyz[1];
				base->xyz[2] = xyz[2];
				ret = ib;
				break;
			}
		}
		if (ib == network->nb) /* no recycle locations, need to add new */
		{
			if (network->nb < MAX_BASE)
			{
				base->ID = staid;
				base->xyz[0] = xyz[0];
				base->xyz[1] = xyz[1];
				base->xyz[2] = xyz[2];
				ret = network->nb;
				++network->nb;
			}else
			{
				/* reach maxium, can not add */
			}
		}
	}
	return ret;
}

/* delete bas station from network database */
extern void del_bas_from_network(network_t *network, int staid)
{
	int ib = 0;
	base_t* base = network->bases + ib;
	for (; ib<network->nb; ++ib, ++base)
	{
		if (base->ID == staid)
		{
			memset(base, 0, sizeof(base_t));
			break;
		}
	}
}

/* delete the vrs rove data from network database */
extern void del_vrs_from_network(network_t* network, int vrsid)
{
	int ir = 0;
	rove_t* rove = network->roves + ir;
	for (; ir < network->nr; ++ir, ++rove)
	{
		if (rove->ID == vrsid)
		{
			memset(rove, 0, sizeof(rove_t));
			break;
		}
	}
}
/* output */
extern int get_vrs_from_network(network_t* network, int vrsid, epoch_t* epoch)
{
	int ir = 0;
	rove_t* rove = network->roves + ir;
	for (; ir < network->nr; ++ir, ++rove)
	{
		if (rove->ID == vrsid) break;
	}
	if (ir < network->nr)
	{
		*epoch = rove->epochs[MAX_EPOCH - 1];
		return epoch->n;
	}
	else
	{
		return 0;
	}
}

/* process the network data */
/* 1. evaluate the satellite orbit (position, velocity, acceleration, clock bias, clock drift) */
/* 2. receiver based GNSS data processing */
/* 3. form baselines */
/* 4. baseline process */
/* 5. vrs modeling */
/* 6. generate vrs measurement for each vrs rove */

static void network_receiver_engine(network_t* network)
{

}

static void network_form_baseline(network_t* network)
{

}

static void network_baseline_engine(network_t* network)
{

}

static void network_vrs_modeling(network_t* network)
{

}

static void network_vrs_generate(network_t* network)
{
	int ir = 0;
	int ib = 0;
	rove_t* rove = network->roves + ir;
	int i = 0;
	int j = 0;
	base_t* base = network->bases + 0;
	base_t* base_can = network->bases + 0;
	epoch_t* epoch = 0;
	int bestLoc = 0;
	double bestDis = 0;
	double currDis = 0;
	double dt = 0;
	for (ir = 0; ir < network->nr; ++ir, ++rove)
	{
		rove->status = 0;
		if (rove->ID == 0) continue;
		/* search the best two stations within 1 seconds of the current time tag */
		bestLoc = -1;
		bestDis = 0;
		for (ib = 0, base = network->bases + ib; ib < network->nb; ++ib, ++base)
		{
			epoch = base->epochs + (MAX_EPOCH - 1);
			if (epoch->n > 0 )
				dt = fabs(epoch->ws - network->time);
			dt -= floor(dt / (7 * 24 * 3600.0) + 0.5) * (7 * 24 * 3600.0);
			if (fabs(dt) > 1.5) continue;
			/* check distance */
			if (fabs(epoch->pos[0]) < 0.001 || fabs(epoch->pos[1]) < 0.001 || fabs(epoch->pos[2]) < 0.001)
			{
				currDis = -1.0;
			}
			else
			{
				currDis = baseline_distance(rove->vrs_xyz, epoch->pos);
			}
			if (bestLoc < 0 || currDis < bestDis || currDis < 0.0)
			{
				bestLoc = ib;
				bestDis = currDis;
			}
		}
		if (bestLoc >= 0) /* can set maximum limitation here */
		{
			/* find the best base station */
			/* generate the correction for the rove */
			base = network->bases + bestLoc;
			rove->baseID = base->ID;
			/* generate vrs data */
			for (j = 1; j < MAX_EPOCH; ++j)
			{
				rove->epochs[j - 1] = rove->epochs[j];
			}
			epoch_t* rov_epoch = rove->epochs + (MAX_EPOCH - 1);
			epoch_t* bas_epoch = base->epochs + (MAX_EPOCH - 1);
			if (fabs(bas_epoch->pos[0]) < 0.001 || fabs(bas_epoch->pos[1]) < 0.001 || fabs(bas_epoch->pos[2]) < 0.001)
			{
				*rov_epoch = *bas_epoch;
			}
			else
			{
				rov_epoch->pos[0] = rove->vrs_xyz[0];
				rov_epoch->pos[1] = rove->vrs_xyz[1];
				rov_epoch->pos[2] = rove->vrs_xyz[2];
				rov_epoch->n = make_vrs_measurement(bas_epoch->obs, bas_epoch->vec, bas_epoch->pos, bas_epoch->n, rove->vrs_xyz, rov_epoch->obs, rov_epoch->vec);
			}
			rove->status = 1;
		}
	}
	return;
}
extern void network_processor(network_t* network)
{
#ifdef _WIN32
	printf("%10.3f,%u, vrs engine\n", network->time, network->numofepoch);
#endif
	/* 1. evaluate the satellite orbit (position, velocity, acceleration, clock bias, clock drift) */
	/* this step is done in another module */
	/* 2. receiver based GNSS data processing */
	network_receiver_engine(network);
	/* 3. form baselines */
	network_form_baseline(network);
	/* 4. baseline process */
	network_baseline_engine(network);
	/* 5. vrs modeling */
	network_vrs_modeling(network);
	/* 6. generate vrs measurement for each vrs rove using the nearst base station */
	network_vrs_generate(network);
}
/* initize network */
extern void network_init(network_t* network)
{
	memset(network->bases, 0, sizeof(base_t) * MAX_BASE);
	memset(network->roves, 0, sizeof(rove_t) * MAX_ROVE);
	network->nb = 0;
	network->nr = 0;
	network->numofepoch = 0;
	network->time = 0;
	network->status = 0;
	memset(network->ws, 0, sizeof(network->ws));
}