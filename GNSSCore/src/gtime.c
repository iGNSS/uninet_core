#include "gtime.h"

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