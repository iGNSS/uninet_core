// GNSSPP.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>

#include "gnss_proc_pp.h"
#include "gnss_proc_rt.h"
#include "gnss_proc_pp_rtcm.h"

#define MAXFIELD 100

static int parse_fields(char* const buffer, char** val)
{
	char* p, * q;
	int n = 0;

	/* parse fields */
	for (p = buffer; *p && n < MAXFIELD; p = q + 1) {
		if ((q = strchr(p, ',')) || (q = strchr(p, '*')) || (q = strchr(p, '\n'))) {
			val[n++] = p; *q = '\0';
		}
		else
		{
			if (p != NULL && strlen(p) > 0)
				val[n++] = p;
			break;
		}
	}
	return n;
}


static int proc_config_file(const char* fname)
{
	FILE* fINI = fopen(fname, "r");
	char buffer[1024] = { 0 };
	char* val[MAXFIELD];
	int line = 0;
	char inifname[255] = { 0 };
	while (fINI != NULL && !feof(fINI))
	{
		if (fgets(buffer, sizeof(buffer), fINI) == NULL) break;
		char* temp = strchr(buffer, '#'); /* # => start comments */
		if (temp != NULL) temp[0] = '\0';
		temp = strchr(buffer, '\n');
		if (temp != NULL) temp[0] = '\0';
		temp = strchr(buffer, '\r');
		if (temp != NULL) temp[0] = '\0';
		if (strlen(buffer) < 1) continue;
		int num = parse_fields(buffer, val);

		if (num < 2) continue;
		int type = atoi(val[0]);
		if (type == 1)
		{
			engine_pp_main_rinex(val[1]);
		}
		else if (type == 2)
		{
			engine_rt_main(val[1]);
		}
		else if (type == 3)
		{
			engine_pp_main_ini(val[1]);
		}
		else if (type == 4)
		{
			strcpy(inifname, val[1]);
		}
		++line;
	}
	if (fINI) fclose(fINI);
	return line;
}

int main(int argc, char* argv[])
{
	engine_pp_main_ini("D:\\rtk\\uninet0313\\GNSSPP\\2022-3-12-20-05-09.ini");
	if (argc < 2)
	{
		printf("gnsspp project_file_name\n");
		proc_config_file("data.ini");
	}
	else 
	{
		proc_config_file(argv[1]);
	}
	return 0;
}

