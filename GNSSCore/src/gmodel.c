#include "gmodel.h"

#include <math.h>

#ifndef ae_WGS84
#define ae_WGS84 6378137.0
#endif

#ifndef finv_WGS84
#define finv_WGS84 298.257223563
#endif

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
