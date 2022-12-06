//------------------------------------------------------------------------------
#include "gnss_proc_pp.h"
//------------------------------------------------------------------------------
#include <stdio.h>
#include <string.h>
#include <vector>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <ctime>
//------------------------------------------------------------------------------

//#include "EngineVRS.h"
#include "gobs.h"
#include "geph.h"
#include "gtime.h"

#pragma warning (disable:4996)
#pragma warning (disable:0266)

#ifndef MAX_OBS_NUM
#define MAX_OBS_NUM 300
#endif

typedef struct
{
	double time;
	double dt;
	sat_obs_t dat[MAX_OBS_NUM];
	unsigned int n;
}sat_epoch_t;



void readRinexNavData(const char* fname, std::vector<sat_eph_t> &sat_eph, std::vector<glo_eph_t>& glo_eph)
{
	FILE* fRinexNav = fopen(fname, "r");
	if (fRinexNav == NULL) return;

	// Read Data from Rinex Navigation Data File
	bool endOfHeader = false;
	char buffer[255];
	bool isReadNextLine = false;
	int rinexVersion = 0;
	while (!feof(fRinexNav))
	{
		if (!isReadNextLine) fgets(buffer, sizeof(buffer), fRinexNav);
		if (strstr(buffer, "RINEX VERSION / TYPE") != NULL)
		{
			/* merage file with multi header, need to the header always */
			rinexVersion = 0;
			endOfHeader = false;
		}
		if (!endOfHeader)       //Check End of Header
		{
			if (strstr(buffer, "RINEX VERSION / TYPE") != NULL)
			{
				char temp[21] = { 0 };
				strncpy(temp, buffer, sizeof(char) * 20);
				temp[20] = '\0';
				rinexVersion = (int)(atof(temp) * 100);
				continue;
			}
			if (strstr(buffer, "END OF HEADER") != NULL)
			{
				if (rinexVersion == 0) break; /* no valid rinex version, skip the file */
				endOfHeader = true;
				continue;
			}
			continue;
		}
		if (buffer[0] == ' ' && buffer[1] == ' ') continue;
		for (int i = 0; i < (int)strlen(buffer); ++i)
		{
			if (buffer[i] == 'D' || buffer[i] == 'd')
				buffer[i] = 'E';
			//buffer[i] = toupper(buffer[i]);
		}

		char tempBuffer[255];

		char sysID = 'G';
		int satID = 0, year = 0, mon = 0, day = 0, hour = 0, min = 0;
		double sec = 0.0;

		if (rinexVersion < 300)
		{
			/* I2(PRN),1X,I2.2(YY),1X,I2(MM),1X,I2(DD),1X,I2(HOUR),1X,I2(MIN),F5.1(SEC),3D19.12(SV CLOCK BIAS),3D19.12(SV CLOCK DRIFT),3D19.12(SV CLOCK DRIFT RATE)
			*/
			strncpy(tempBuffer, buffer + 0, sizeof(char) * 2); tempBuffer[2] = '\0'; satID = atoi(tempBuffer);
			strncpy(tempBuffer, buffer + 0 * 3 + 3, sizeof(char) * 3); tempBuffer[3] = '\0'; year = atoi(tempBuffer);
			strncpy(tempBuffer, buffer + 1 * 3 + 3, sizeof(char) * 3); tempBuffer[3] = '\0'; mon = atoi(tempBuffer);
			strncpy(tempBuffer, buffer + 2 * 3 + 3, sizeof(char) * 3); tempBuffer[3] = '\0'; day = atoi(tempBuffer);
			strncpy(tempBuffer, buffer + 3 * 3 + 3, sizeof(char) * 3); tempBuffer[3] = '\0'; hour = atoi(tempBuffer);
			strncpy(tempBuffer, buffer + 4 * 3 + 3, sizeof(char) * 3); tempBuffer[3] = '\0'; min = atoi(tempBuffer);
			strncpy(tempBuffer, buffer + 5 * 3 + 3, sizeof(char) * 5); tempBuffer[5] = '\0'; sec = atof(tempBuffer);
		}
		else
		{
			/* A1(SYS),I2.2(PRN),1X,I4(TOC YYYY),1X,I2.2(MON),1X,I2.2(DAY),1X,I2.2(HOUR),1X,I2.2(MIN),1X,I2.2(SEC),3D19.12(SV CLOCK BIAS),3D19.12(SV CLOCK DRIFT),3D19.12(SV CLOCK DRIFT RATE)
			* G01 2021 06 11 00 00 00 6.623202934861E-04-1.170974428533E-11 0.000000000000E+00
			*/
			sysID = buffer[0];
			strncpy(tempBuffer, buffer + 1, sizeof(char) * 2); tempBuffer[2] = '\0'; satID = atoi(tempBuffer);
			strncpy(tempBuffer, buffer + 3, sizeof(char) * 5); tempBuffer[5] = '\0'; year = atoi(tempBuffer);
			strncpy(tempBuffer, buffer + 0 * 3 + 8, sizeof(char) * 3); tempBuffer[3] = '\0'; mon = atoi(tempBuffer);
			strncpy(tempBuffer, buffer + 1 * 3 + 8, sizeof(char) * 3); tempBuffer[3] = '\0'; day = atoi(tempBuffer);
			strncpy(tempBuffer, buffer + 2 * 3 + 8, sizeof(char) * 3); tempBuffer[3] = '\0'; hour = atoi(tempBuffer);
			strncpy(tempBuffer, buffer + 3 * 3 + 8, sizeof(char) * 3); tempBuffer[3] = '\0'; min = atoi(tempBuffer);
			strncpy(tempBuffer, buffer + 4 * 3 + 8, sizeof(char) * 3); tempBuffer[3] = '\0'; sec = atof(tempBuffer);
		}

		int weekNum = 0;
		double weekSec = ConvertToTimeGPS(year, mon, day, hour, min, sec, &weekNum);

		double brdc_data[32] = { 0 };

		brdc_data[0] = weekSec; // Time ==> GPS Second
		//brdc_data[0] += weekNum * 7.0 * 24.0 * 3600.0;

		int loc = 23;
		if (rinexVersion < 300) loc = 22;

		//Satellite Clk Bias, drift , drift rate (1, 2, 3)
		strncpy(tempBuffer, buffer + 0 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[0 + 1] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 1 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[1 + 1] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 2 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[2 + 1] = atof(tempBuffer);

		fgets(buffer, sizeof(buffer), fRinexNav); //2nd Line

		for (int i = 0; i < (int)strlen(buffer); ++i)
		{
			if (buffer[i] == 'D' || buffer[i] == 'd')
				buffer[i] = 'E';
			//buffer[i] = toupper(buffer[i]);
		}

		/* 2.x 3X,4D19.12
		*  3.x 4X,4D19.12
		*/
		loc = 4;
		if (rinexVersion < 300) loc = 3;
		//IODE Issue of Data, Ephemeris, Crs, Delt n, M0 (3, 22, 41, 60)
		strncpy(tempBuffer, buffer + 0 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[0 + 4] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 1 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[1 + 4] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 2 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[2 + 4] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 3 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[3 + 4] = atof(tempBuffer);

		fgets(buffer, sizeof(buffer), fRinexNav); //3rd Line

		for (int i = 0; i < (int)strlen(buffer); ++i)
		{
			if (buffer[i] == 'D' || buffer[i] == 'd')
				buffer[i] = 'E';
			//buffer[i] = toupper(buffer[i]);
		}

		// Cuc, e Eccentricity, Cus, sqrt(A) (3, 22, 41, 60)
		strncpy(tempBuffer, buffer + 0 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[0 + 8] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 1 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[1 + 8] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 2 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[2 + 8] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 3 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[3 + 8] = atof(tempBuffer);

		fgets(buffer, sizeof(buffer), fRinexNav); //4th Line

		for (int i = 0; i < (int)strlen(buffer); ++i)
		{
			if (buffer[i] == 'D' || buffer[i] == 'd')
				buffer[i] = 'E';
			//buffer[i] = toupper(buffer[i]);
		}

		// Toe Time of Ephemeris, Cic, COMEGA, CIS (3, 22, 41, 60)
		strncpy(tempBuffer, buffer + 0 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[0 + 12] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 1 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[1 + 12] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 2 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[2 + 12] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 3 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[3 + 12] = atof(tempBuffer);

		fgets(buffer, sizeof(buffer), fRinexNav); //5th Line

		for (int i = 0; i < (int)strlen(buffer); ++i)
		{
			if (buffer[i] == 'D' || buffer[i] == 'd')
				buffer[i] = 'E';
			//buffer[i] = toupper(buffer[i]);
		}

		if (sysID == 'C')
			int oo = 0;

		if ((rinexVersion < 300 && (buffer[0] != ' ' || buffer[1] != ' ')) || (rinexVersion >= 300 && sysID == 'R') || (rinexVersion >= 300 && sysID == 'S'))
		{
			// check is GLONASS navigation message or not
			if (rinexVersion < 300)
			{
				sysID = 'R';
			}

			/* UTC time to GPS time */
			glo_eph_t geph = { 0 };
			double tow = brdc_data[0] + 18.0;
			double toc = floor((tow + 450.0) / 900.0) * 900;
			int dow = (int)floor(tow / 86400.0);
			double tod = rinexVersion < 300 ? brdc_data[3] : fmod(brdc_data[3], 86400.0);
			double tof = tod + dow * 86400.0;
			/* IODE = Tb (7bit), Tb =index of UTC+3H within current day */
			geph.iode = (int)(fmod(brdc_data[0] + 10800.0, 86400.0) / 900.0 + 0.5);

			geph.sys = sysID;
			geph.prn = satID;
			geph.toes = brdc_data[0]+18.0;
			geph.taun =-brdc_data[1];
			geph.gamn = brdc_data[2];
			geph.tofs = brdc_data[3]+18.0;
			geph.pos[0] = brdc_data[4] * 1e3;
			geph.vel[0] = brdc_data[5] * 1e3;
			geph.acc[0] = brdc_data[6] * 1e3;
			geph.svh = (int)brdc_data[7];
			geph.pos[1] = brdc_data[8] * 1e3;
			geph.vel[1] = brdc_data[9] * 1e3;
			geph.acc[1] = brdc_data[10] * 1e3;
			geph.frq = (int)brdc_data[11];
			geph.pos[2] = brdc_data[12] * 1e3;
			geph.vel[2] = brdc_data[13] * 1e3;
			geph.acc[2] = brdc_data[14] * 1e3;
			geph.age = (int)brdc_data[15];

			glo_eph.push_back(geph);

			isReadNextLine = true;

			continue;
		}

		// i0, Crc, omega, OMEGA DOT (3, 22, 41, 60)
		strncpy(tempBuffer, buffer + 0 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[0 + 16] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 1 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[1 + 16] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 2 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[2 + 16] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 3 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[3 + 16] = atof(tempBuffer);

		fgets(buffer, sizeof(buffer), fRinexNav); //6th Line

		for (int i = 0; i < (int)strlen(buffer); ++i)
		{
			if (buffer[i] == 'D' || buffer[i] == 'd')
				buffer[i] = 'E';
			//buffer[i] = toupper(buffer[i]);
		}

		// IDOT, Codes on L2 channel, GPS Week # (to go with TOE) Continuous, L2 P data flag (3, 22, 41, 60)
		strncpy(tempBuffer, buffer + 0 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[0 + 20] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 1 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[1 + 20] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 2 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[2 + 20] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 3 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[3 + 20] = atof(tempBuffer);

		/*	- SV accuracy (meters) See GPS ICD Section 20.3.3.3.1.3 use specified equations to define nominal values, N = 0-6: use 2(1+N/2) (round to one decimal place i.e. 2.8, 5.7 and 11.3) , N= 7-15:use 2 (N-2),8192 specifies use at own risk
			- SV health (bits 17-22 w 3 sf 1)
			- TGD (seconds)
			- IODC Issue of Data, Clock
		*/
		fgets(buffer, sizeof(buffer), fRinexNav); //7th Line
		strncpy(tempBuffer, buffer + 0 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[0 + 24] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 1 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[1 + 24] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 2 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[2 + 24] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 3 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[3 + 24] = atof(tempBuffer);

		/*	- Transmission time of message **) (sec of GPS week, derived e.g. from Zcount in Hand Over Word (HOW))
			- Fit Interval in hours; bit 17 w 10 sf 2 + IODC & Table 20-XII of GPS ICD. (BNK if Unknown).
			- Spare(x2) (see Section 6.4)
		*/
		fgets(buffer, sizeof(buffer), fRinexNav); //8th Line
		strncpy(tempBuffer, buffer + 0 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[0 + 28] = atof(tempBuffer);
		strncpy(tempBuffer, buffer + 1 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[1 + 28] = atof(tempBuffer);
		//strncpy(tempBuffer, buffer + 2 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[2 + 28] = atof(tempBuffer);
		//strncpy(tempBuffer, buffer + 3 * 19 + loc, sizeof(char) * 19); tempBuffer[19] = '\0'; brdc_data[3 + 28] = atof(tempBuffer);

		if (rinexVersion < 300)
		{
			sysID = 'G';
		}

		sat_eph_t eph = { 0 };
		eph.sys = sysID;
		eph.prn = satID;
		eph.tocs = brdc_data[0];
		eph.f0 = brdc_data[1];
		eph.f1 = brdc_data[2];
		eph.f2 = brdc_data[3];
		eph.iode = (int)brdc_data[4];
		eph.crs = brdc_data[5];
		eph.deln = brdc_data[6];
		eph.M0 = brdc_data[7];
		eph.cuc = brdc_data[8];
		eph.e = brdc_data[9];
		eph.cus = brdc_data[10];
		eph.A = brdc_data[11] * brdc_data[11];
		eph.toes = brdc_data[12];// +weekNum * 7.0 * 24.0 * 3600.0;
		eph.cic = brdc_data[13];
		eph.OMG0 = brdc_data[14];
		eph.cis = brdc_data[15];
		eph.i0 = brdc_data[16];
		eph.crc = brdc_data[17];
		eph.omg = brdc_data[18];
		eph.OMGd = brdc_data[19];
		eph.idot = brdc_data[20];

		if (sysID == 'G'|| sysID == 'J')
		{
			eph.iodc = (int)brdc_data[27];
			eph.week = (int)brdc_data[22];
			eph.code = (int)brdc_data[21];
			eph.svh = (int)brdc_data[25];
			eph.sva = (int)brdc_data[24]; /* need to convert */
			eph.flag = (int)brdc_data[23]; /* L2P data flag */
			eph.tgd[0] = brdc_data[26]; /* TGD second */
			if (sysID == 'G')
				eph.fit = brdc_data[29];
			else
				eph.fit = brdc_data[29] == 0.0 ? 1.0 : 2.0;
		}
		else if (sysID == 'E')
		{
			eph.iode = (int)brdc_data[4];      /* IODnav */
			//eph.toes = brdc_data[12];      /* Toe (s) in Galileo week */
			eph.week = (int)brdc_data[22];      /* Galileo week = GPS week */

			eph.code = (int)brdc_data[21];      /* brdc_data sources */
										  /* bit 0 set: I/NAV E1-B */
										  /* bit 1 set: F/NAV E5a-I */
										  /* bit 2 set: F/NAV E5b-I */
										  /* bit 8 set: af0-af2 toc are for E5a.E1 */
										  /* bit 9 set: af0-af2 toc are for E5b.E1 */
			eph.svh = (int)brdc_data[25];      /* sv health */
										  /* bit     0: E1B DVS */
										  /* bit   1-2: E1B HS */
										  /* bit     3: E5a DVS */
										  /* bit   4-5: E5a HS */
										  /* bit     6: E5b DVS */
										  /* bit   7-8: E5b HS */
			eph.sva = (int)brdc_data[24]; /* sisa (m->index) */

			eph.tgd[0] = brdc_data[26];      /* BGD E5a/E1 */
			eph.tgd[1] = brdc_data[27];      /* BGD E5b/E1 */
		}
		else if (sysID == 'C') { /* BeiDou v.3.02 */
			/* BDT to GPS time */
			eph.tocs += 14.0;  /* bdt -> gpst */
			//eph.iode = (int)brdc_data[4];      /* AODE */
			eph.iodc = (int)brdc_data[29];      /* AODC */
			//eph.toes = brdc_data[12];      /* Toe (s) in BDT week */
			eph.week = (int)brdc_data[22]+1356;      /* bdt week */
			//eph.toe = bdt2gpst(bdt2time(eph.week, brdc_data[12])); /* BDT -> GPST */
			//eph.ttr = bdt2gpst(bdt2time(eph.week, brdc_data[28])); /* BDT -> GPST */
			//eph.toe = adjweek(eph.toe, toc);
			//eph.ttr = adjweek(eph.ttr, toc);
			eph.toes = brdc_data[12] + 14.0;
			eph.ttr = brdc_data[28] + 14.0;

			eph.svh = (int)brdc_data[25];      /* satH1 */
			eph.sva = (int)brdc_data[24];  /* URA index (m->index) */

			eph.tgd[0] = brdc_data[26];      /* TGD1 B1/B3 */
			eph.tgd[1] = brdc_data[27];      /* TGD2 B2/B3 */
		}
		else if (sysID == 'I') { /* IRNSS v.3.03 */
			eph.iode = (int)brdc_data[4];      /* IODEC */
			eph.toes = brdc_data[12];      /* Toe (s) in IRNSS week */
			eph.week = (int)brdc_data[22];      /* IRNSS week */
			//eph.toe = adjweek(gpst2time(eph.week, brdc_data[12]), toc);
			//eph.ttr = adjweek(gpst2time(eph.week, brdc_data[28]), toc);
			eph.svh = (int)brdc_data[25];      /* SV health */
			eph.sva = (int)brdc_data[24];  /* URA index (m->index) */
			eph.tgd[0] = brdc_data[26];      /* TGD */
		}

		sat_eph.push_back(eph);

		isReadNextLine = false;

	}
	fclose(fRinexNav);
	return;
}

#pragma warning (default:4996)
	int find_obs_location(sat_obs_t* obs, int n, unsigned char sys, unsigned char prn, unsigned char code)
	{
		int i = 0;
		sat_obs_t* pobs = 0;
		for (i = 0, pobs = obs + i; i < n; ++i, ++pobs)
		{
			if (pobs->sys == sys && pobs->prn == prn && pobs->code == code) break;
		}
		return i;
	}

	int read_project_file(const char* fname, 
		char *outputdir,
		char* refinidir,
		std::vector<std::string>& vRefObsFileName_,
		std::vector<std::string>& vRovObsFileName_,
		std::vector<std::string>& vBrdcFileName_,
		std::vector<std::string>& vSp3FileName_,
		std::vector<std::string>& vCorsIniFileName_,
		std::vector<std::string>& vUserIniFileName_,
		std::vector<std::string>& vAntFileName_)
	{
		char buffer[1024] = { 0 };
		FILE* fPRJ = fopen(fname, "r");
		if (fPRJ == NULL) return 0;
		while (!feof(fPRJ))
		{
			memset(buffer, '\0', sizeof(buffer));
			fgets(buffer, sizeof(buffer), fPRJ);
			char* temp = strchr(buffer, ';');
			if (temp != NULL) temp[0] = '\0';
			temp = strchr(buffer, '\n');
			if (temp != NULL) temp[0] = '\0';
			temp = strchr(buffer, '=');
			if (temp == NULL) continue;
			int nstr = (int)strlen(buffer);
			int nkey = (int)(temp - buffer);
			temp[0] = '\0';
			for (int i = 0; i < nkey; ++i)
				buffer[i] = tolower(buffer[i]);
			if (strstr(buffer, "refrnx") != NULL)
			{
				std::rotate(buffer + 0, buffer + nkey + 1, buffer + nstr);
				buffer[nstr - nkey - 1] = '\0';
				while (buffer[0] == ' ')
				{
					std::rotate(buffer + 0, buffer + 1, buffer + strlen(buffer));
					buffer[strlen(buffer) - 1] = '\0';
				}
				std::string fileName(buffer);
				vRefObsFileName_.push_back(fileName);
				continue;
			}
			if (strstr(buffer, "rovrnx") != NULL)
			{
				std::rotate(buffer + 0, buffer + nkey + 1, buffer + nstr);
				buffer[nstr - nkey - 1] = '\0';
				while (buffer[0] == ' ')
				{
					std::rotate(buffer + 0, buffer + 1, buffer + strlen(buffer));
					buffer[strlen(buffer) - 1] = '\0';
				}
				std::string fileName(buffer);
				vRovObsFileName_.push_back(fileName);
				continue;
			}
			if (strstr(buffer, "brdc") != NULL)
			{
				std::rotate(buffer + 0, buffer + nkey + 1, buffer + nstr);
				buffer[nstr - nkey - 1] = '\0';
				while (buffer[0] == ' ')
				{
					std::rotate(buffer + 0, buffer + 1, buffer + strlen(buffer));
					buffer[strlen(buffer) - 1] = '\0';
				}
				std::string fileName(buffer);
				vBrdcFileName_.push_back(fileName);
				continue;
			}
			if (strstr(buffer, "sp3") != NULL)
			{
				std::rotate(buffer + 0, buffer + nkey + 1, buffer + nstr);
				buffer[nstr - nkey - 1] = '\0';
				while (buffer[0] == ' ')
				{
					std::rotate(buffer + 0, buffer + 1, buffer + strlen(buffer));
					buffer[strlen(buffer) - 1] = '\0';
				}
				std::string fileName(buffer);
				vSp3FileName_.push_back(fileName);
				continue;
			}
			if (strstr(buffer, "antmodel") != NULL)
			{
				std::rotate(buffer + 0, buffer + nkey + 1, buffer + nstr);
				buffer[nstr - nkey - 1] = '\0';
				while (buffer[0] == ' ')
				{
					std::rotate(buffer + 0, buffer + 1, buffer + strlen(buffer));
					buffer[strlen(buffer) - 1] = '\0';
				}
				std::string fileName(buffer);
				vAntFileName_.push_back(fileName);
				continue;
			}
			if (strstr(buffer, "outputdir") != NULL)
			{
				std::rotate(buffer + 0, buffer + nkey + 1, buffer + nstr);
				buffer[nstr - nkey - 1] = '\0';
				while (buffer[0] == ' ')
				{
					std::rotate(buffer + 0, buffer + 1, buffer + strlen(buffer));
					buffer[strlen(buffer) - 1] = '\0';
				}
				strncpy(outputdir, buffer, strlen(buffer));
				continue;
			}
			if (strstr(buffer, "refinidir") != NULL)
			{
				std::rotate(buffer + 0, buffer + nkey + 1, buffer + nstr);
				buffer[nstr - nkey - 1] = '\0';
				while (buffer[0] == ' ')
				{
					std::rotate(buffer + 0, buffer + 1, buffer + strlen(buffer));
					buffer[strlen(buffer) - 1] = '\0';
				}
				strncpy(refinidir, buffer, strlen(buffer));
				continue;
			}
			if (strstr(buffer, "corsrefini") != NULL)
			{
				std::rotate(buffer + 0, buffer + nkey + 1, buffer + nstr);
				buffer[nstr - nkey - 1] = '\0';
				while (buffer[0] == ' ')
				{
					std::rotate(buffer + 0, buffer + 1, buffer + strlen(buffer));
					buffer[strlen(buffer) - 1] = '\0';
				}
				std::string fileName(buffer);
				vCorsIniFileName_.push_back(fileName);
				continue;
			}
			if (strstr(buffer, "userrefini") != NULL)
			{
				std::rotate(buffer + 0, buffer + nkey + 1, buffer + nstr);
				buffer[nstr - nkey - 1] = '\0';
				while (buffer[0] == ' ')
				{
					std::rotate(buffer + 0, buffer + 1, buffer + strlen(buffer));
					buffer[strlen(buffer) - 1] = '\0';
				}
				std::string fileName(buffer);
				vUserIniFileName_.push_back(fileName);
				continue;
			}
		}
		fclose(fPRJ);
		return 1;
	}

	static int check_sat_eph(int wn, double tow, sat_obs_t* obs, std::vector<sat_eph_t>& sat_eph, std::vector<glo_eph_t>& glo_eph)
	{
		int ret = 0;
		double time = wn * 7 * 24 * 3600 + tow;
		/* push eph */
		if (obs->sys != 'R')
		{

			std::vector<sat_eph_t>::iterator pEphG = sat_eph.begin();
			while (pEphG != sat_eph.end())
			{
				if (pEphG->sys != obs->sys|| pEphG->prn != obs->prn)
				{
					++pEphG;
					continue;
				}
				double toes = pEphG->toes + pEphG->week * 7.0 * 24.0 * 3600.0;
				if (toes < (time + 100.0))
				{
					//add_sat_eph(&(*pEphG));
					++ret;
					pEphG = sat_eph.erase(pEphG);
				}
				else
				{
					++pEphG;
				}
			}
		}
		else
		{
			/* push glo eph */
			std::vector<glo_eph_t>::iterator pEphR = glo_eph.begin();
			while (pEphR != glo_eph.end())
			{
				if (pEphR->prn != obs->prn)
				{
					++pEphR;
					continue;
				}
				double tocs = pEphR->toes + wn * 7.0 * 24.0 * 3600.0;
				if (tocs < (time + 16 * 60.0))
				{
					//add_glo_eph(&(*pEphR));
					++ret;
					pEphR = glo_eph.erase(pEphR);
				}
				else
				{
					++pEphR;
				}
			}
		}
		return ret;
	}
	//--------------------------------------------------------------------------
	using namespace std;
	//--------------------------------------------------------------------------
	void engine_pp_main_rinex(const char *fname)
	{
		using namespace std;
		std::vector<std::string> vRefObsFileName_,
								 vRovObsFileName_,
								 vBrdcFileName_,
								 vSp3FileName_,
								 vCorsIniFileName_,
								 vUserIniFileName_,
								 vAntFileName_;
		char buffer[1024] = { 0 };
		char outputdir[255] = { 0 };
		char refinidir[255] = { 0 };
		if (!read_project_file(fname, outputdir, refinidir, vRefObsFileName_, vRovObsFileName_, vBrdcFileName_, vSp3FileName_, vCorsIniFileName_, vUserIniFileName_, vAntFileName_)) return;
		// must have data and broadcast orbit
		if (vRefObsFileName_.size()==0||vBrdcFileName_.size()==0) return;
		int maxRef = (int)vRefObsFileName_.size(),
			maxRov = (int)vRovObsFileName_.size(),
			maxRCV = maxRef+maxRov;

		std::vector<sat_eph_t> sat_eph;
		std::vector<glo_eph_t> glo_eph;

		//TEngineRTK engineRTK;
		//TRefRcvDataSet refRcvDataSet;
		for (int i = 0; i < (int)vBrdcFileName_.size(); ++i)
			readRinexNavData(vBrdcFileName_[i].c_str(), sat_eph, glo_eph);
		//--------------------------------------------------------------------------	   
		clock_t st = clock();
		//--------------------------------------------------------------------------	   
		FILE * fLOG = fopen("rtk.log", "a");
		//--------------------------------------------------------------------------	   
		std::vector<FILE *> vFile(maxRCV);
		std::vector<rinex_obs_header_t> vRinexObsHeader(maxRCV);
		std::vector<sat_epoch_t> vEpoch(maxRCV);
		//--------------------------------------------------------------------------	   
		// read observation header information
		for (int idxRcv=0;idxRcv<maxRCV;++idxRcv)
		{
			char fileName[255] = { 0 };
			if (idxRcv<maxRef)
				strcpy(fileName, vRefObsFileName_[idxRcv       ].c_str());
			else
				strcpy(fileName, vRovObsFileName_[idxRcv-maxRef].c_str());
			vFile[idxRcv] = fopen(fileName, "r");
			if (vFile[idxRcv]==NULL) continue;
			// reading data
			std::vector<rinex_obs_header_t>::iterator pObsHeader = vRinexObsHeader.begin()+idxRcv;
			char * result = strrchr(fileName, '\\');
			if (result!=NULL)
				strncpy(fileName, result+1, strlen(result));
			result = strrchr(fileName, '.');
			if (result != NULL) result[0] = '\0';
			int nn = (int)strlen(fileName);
			if (nn > 20) nn = 20;
			strncpy(pObsHeader->rcvName_, fileName, sizeof(char)*nn);
			pObsHeader->rcvName_[nn] = '\0';
			bool isEOF = false;
			while (!feof(vFile[idxRcv]))
			{
				memset(buffer, '\0', sizeof(buffer));
				fgets(buffer, sizeof(buffer), vFile[idxRcv]);
				if (strlen(buffer)<60) continue;
				// gps rinex observation header information
				if (strstr(buffer, "RINEX VERSION / TYPE")!=NULL)
				{
					buffer[20] = '\0'; pObsHeader->rinexVersion_ = (int)(atof(buffer)*100);
				}
				else if (strstr(buffer+60, "MARKER NAME")!=NULL)
				{
					char markName_[21];
					strncpy(markName_, buffer+0, sizeof(char)*20);	markName_[20] = '\0';
				}
				else if (strstr(buffer+60, "REC # / TYPE / VERS")!=NULL )
				{
					char rcvNum_[21], rcvType_[41];
					strncpy(rcvNum_ , buffer+ 0, sizeof(char)*20);    rcvNum_ [20]    = '\0';
					strncpy(rcvType_, buffer+20, sizeof(char)*40);    rcvType_[40]    = '\0';
				}
				else if (strstr(buffer+60, "ANT # / TYPE")!=NULL)
				{
					char antNum_[21];
					strncpy(antNum_ , buffer+ 0, sizeof(char)*20);    antNum_ [20]     = '\0';
					strncpy(pObsHeader->antType_, buffer+20, sizeof(char)*20);    pObsHeader->antType_[20]     = '\0';
				}
				else if (strstr(buffer+60, "APPROX POSITION XYZ")!=NULL)
				{
					pObsHeader->rcvXYZ_[0] = atof(buffer+ 0);
					pObsHeader->rcvXYZ_[1] = atof(buffer+14);
					pObsHeader->rcvXYZ_[2] = atof(buffer+29);
				}
				else if (strstr(buffer+60, "ANTENNA: DELTA H/E/N")!=NULL)
				{
					pObsHeader->antNEU_[2] = atof(buffer+ 0);
					pObsHeader->antNEU_[1] = atof(buffer+14);
					pObsHeader->antNEU_[0] = atof(buffer+29);
				}
				else if (strstr(buffer+60, "# / TYPES OF OBSERV")!=NULL)
				{
					pObsHeader->obsTypeNum = (unsigned short)atoi(buffer);
					std::string lineReadStr(buffer);
					lineReadStr = lineReadStr.substr(6,54);
					if (pObsHeader->obsTypeNum>9)
					{
						fgets(buffer, sizeof(buffer), vFile[idxRcv]);
						std::string addLineReadStr= buffer;
						lineReadStr += addLineReadStr.substr(6,54);
					}
					for (unsigned short i=0;i<pObsHeader->obsTypeNum;++i)
					{
						std::string curObsType = lineReadStr.substr(i*6+4,2);
							 if (curObsType=="C1") pObsHeader->obsTypeLoc[0] = i+1; /* C1C */
						else if (curObsType=="P1") pObsHeader->obsTypeLoc[1] = i+1; /* C1P/C1W */
						else if (curObsType=="P2") pObsHeader->obsTypeLoc[2] = i+1; /* C2W/C2P */
						else if (curObsType=="L1") pObsHeader->obsTypeLoc[3] = i+1; /* L1C/L1W */
						else if (curObsType=="L2") pObsHeader->obsTypeLoc[4] = i+1; /* L2C */
						else if (curObsType=="D1") pObsHeader->obsTypeLoc[5] = i+1; /* D1 */
						else if (curObsType=="D2") pObsHeader->obsTypeLoc[6] = i+1; /* D2 */
					}
				}
				else if (strstr(buffer + 60, "SYS / # / OBS TYPES") != NULL)
				{
					char keystr[255];
					char sysID = buffer[0];
					strncpy(keystr, buffer + 3, sizeof(char) * 3); keystr[3] = '\0'; int obsTypeNum = atoi(keystr);
					strncpy(keystr, buffer + 6, sizeof(char)*(13 * 4));
					int num_of_line = obsTypeNum / 13;
					for (int i = 1; i <= num_of_line; ++i)
					{
						fgets(buffer, sizeof(buffer), vFile[idxRcv]);
						for (int j = (int)strlen(buffer) - 1; i<60; ++i)
							buffer[i] = ' ';
						strncpy(keystr+ num_of_line*(13*4), buffer + 6, sizeof(char)*(13 * 4));
					}
					keystr[obsTypeNum * 4] = '\0';
					sys_obs_type_t sysObsType;
					sysObsType.s = sysID;
					for (int i = 0; i<obsTypeNum; ++i)
					{
						raw_obs_type_t rawObsType;
						rawObsType.t = keystr[i * 4 + 1];
						rawObsType.n = keystr[i * 4 + 2];
						rawObsType.a = keystr[i * 4 + 3];
						sysObsType.obs_type.push_back(rawObsType);
					}
					if (sysObsType.obs_type.size()>0)
						pObsHeader->sys_type.push_back(sysObsType);

					int kk = 0;
				}
				else if (strstr(buffer+60, "INTERVAL")!=NULL)
					int interval_ = atoi(buffer);
				else if (strstr(buffer+60, "END OF HEADER")!=NULL )  
				{
					isEOF = true;
					break;
				}
			}
			if (!isEOF)
			{
				// file have problem to read
				fclose(vFile[idxRcv]);
				vFile[idxRcv] = NULL;
				pObsHeader->rinexVersion_ = 0;
				memset(pObsHeader->rcvName_, 0, sizeof(pObsHeader->rcvName_));
				memset(pObsHeader->antType_, 0, sizeof(pObsHeader->antType_));
				pObsHeader->antNEU_[0] = pObsHeader->antNEU_[1] = pObsHeader->antNEU_[2] = 0;
				pObsHeader->rcvXYZ_[0] = pObsHeader->rcvXYZ_[1] = pObsHeader->rcvXYZ_[2] = 0;
			}
		}
		// update coordinates and antenna type from reference station files
		for (int idxRcv=0;idxRcv<maxRCV;++idxRcv)
		{
			std::vector<rinex_obs_header_t>::iterator pObsHeader = vRinexObsHeader.begin() + idxRcv;
			if (pObsHeader->rinexVersion_>0)
			{
				for (unsigned short i=0;i<strlen(pObsHeader->rcvName_);++i)
					pObsHeader->rcvName_[i] = tolower(pObsHeader->rcvName_[i]);
				//refRcvDataSet.RefRcvData(pObsHeader->RcvName(), pObsHeader->rcvXYZ_, pObsHeader->antType_, 0.0, true);
			}
		}
		//--------------------------------------------------------------------------
		// read observation data
		double preTime = 0.0;
		//--------------------------------------------------------------------------
		unsigned long numOfEpoch = 0;
		//--------------------------------------------------------------------------
		//--------------------------------------------------------------------------
		while (true)
		{
			//----------------------------------------------------------------------
			double curTime = 0.0;
			//----------------------------------------------------------------------
			for (int idxRcv=0;idxRcv<maxRCV;++idxRcv)
			{
				if (vFile[idxRcv]==NULL) continue;
				std::vector<rinex_obs_header_t>::iterator pObsHeader = vRinexObsHeader.begin()+idxRcv;
				std::vector<sat_epoch_t>::iterator pObsData   = vEpoch.begin() +idxRcv;
				//----------------------------------------------------------------------
				if (pObsData->time!=0.0&&pObsData->time >preTime)
				{
					if (curTime==0.0||curTime>pObsData->time)
						curTime = pObsData->time;
					continue;
				}
				//----------------------------------------------------------------------
				while (!feof(vFile[idxRcv]))
				{
					char tempBuffer[255];
					memset(buffer, 0, sizeof(buffer));
					unsigned long curLOC = 0L;
					fgets(buffer, sizeof(buffer), vFile[idxRcv]);
					/* */
					int idxSAT = 0;
					/* */
					if (pObsHeader->rinexVersion_ < 300)
					{
						for (size_t i = strlen(buffer) - 1; i < 80; ++i)
							buffer[i] = ' ';
						// Read Epoch Information
						bool isTime = !(buffer[0] != ' ' || buffer[2] == ' ' ||
							buffer[3] != ' ' || buffer[5] == ' ' ||
							buffer[6] != ' ' || buffer[8] == ' ' ||
							buffer[9] != ' ' || buffer[11] == ' ' ||
							buffer[12] != ' ' || buffer[14] == ' ');
						if (!isTime) continue;
						strncpy(tempBuffer, buffer + 28, sizeof(char) * 1); tempBuffer[1] = '\0'; unsigned short flag = (unsigned short)atoi(tempBuffer);
						if (buffer[28] != '0') continue;
						strncpy(tempBuffer, buffer + 29, sizeof(char) * 3); tempBuffer[3] = '\0'; unsigned short satNum = (unsigned short)atoi(tempBuffer);
						if (satNum == 0) continue;
						strncpy(tempBuffer, buffer + 0 * 3 + 1, sizeof(char) * 2); tempBuffer[2] = '\0'; unsigned short year = (unsigned short)atoi(tempBuffer);
						strncpy(tempBuffer, buffer + 1 * 3 + 1, sizeof(char) * 2); tempBuffer[2] = '\0'; unsigned short mon = (unsigned short)atoi(tempBuffer);
						strncpy(tempBuffer, buffer + 2 * 3 + 1, sizeof(char) * 2); tempBuffer[2] = '\0'; unsigned short day = (unsigned short)atoi(tempBuffer);
						strncpy(tempBuffer, buffer + 3 * 3 + 1, sizeof(char) * 2); tempBuffer[2] = '\0'; unsigned short hour = (unsigned short)atoi(tempBuffer);
						strncpy(tempBuffer, buffer + 4 * 3 + 1, sizeof(char) * 2); tempBuffer[2] = '\0'; unsigned short min = (unsigned short)atoi(tempBuffer);
						strncpy(tempBuffer, buffer + 5 * 3 + 1, sizeof(char) * 11); tempBuffer[11] = '\0'; double         sec = atof(tempBuffer);
						strncpy(tempBuffer, buffer + 68, sizeof(char) * 12); tempBuffer[12] = '\0'; double         cdt = atof(tempBuffer);

						int weekNum = 0;
						double weekSec = ConvertToTimeGPS(year, mon, day, hour, min, sec, &weekNum);

						pObsData->dt = cdt;
						pObsData->time = weekNum * 7.0 * 24.0 * 3600.0 + weekSec;
						pObsData->n = 0;
						memset(pObsData->dat, 0, sizeof(sat_obs_t)* MAX_OBS_NUM);

						std::string lineReadStr = buffer;

						std::string satInfoStr = lineReadStr.substr(32, 36);
						int num_of_line = satNum / 12;
						if (num_of_line > 2)
							int kk = 0;
						for (int idxI = 1; idxI <= num_of_line; ++idxI)
						{
							if ((satNum - idxI * 12) > 0)
							{
								// Read additional line for satellite prns
								curLOC = ftell(vFile[idxRcv]);
								fgets(buffer, sizeof(buffer), vFile[idxRcv]);
								for (size_t i = strlen(buffer) - 1; i < 80; ++i)
									buffer[i] = ' ';
								// Read Epoch Information
								isTime = !(buffer[0] != ' ' || buffer[2] == ' ' ||
									buffer[3] != ' ' || buffer[5] == ' ' ||
									buffer[6] != ' ' || buffer[8] == ' ' ||
									buffer[9] != ' ' || buffer[11] == ' ' ||
									buffer[12] != ' ' || buffer[14] == ' ');
								if (isTime)
								{
									fseek(vFile[idxRcv], curLOC, SEEK_SET);
									break; // jump out
								}
								std::string addLineReadStr = buffer;
								satInfoStr += addLineReadStr.substr(32, 36);
								if (cdt == 0.0) cdt = atof(buffer + 68);
							}
						}
						//--------------------------------------------------------------
						idxSAT = 0;
						for (unsigned short satIndex = 0; satIndex < satNum; ++satIndex)
						{
							//----------------------------------------------------------
							// Init Data
							char			cType = satInfoStr[satIndex * 3];
							unsigned short	satID = atoi(satInfoStr.substr(satIndex * 3 + 1, 2).c_str());
							if (cType == '1')
							{
								cType = 'S';
								satID -= 100;
							}
							else if (cType != 'G'&&cType != 'R'&&cType != 'S'&&cType != 'E' && cType != 'C' && cType != 'J')
								cType = 'G';

							// G => GPS,  R => GLONASS, S => Geostationary signal payload,  E => Galileo
							curLOC = ftell(vFile[idxRcv]);
							fgets(buffer, sizeof(buffer), vFile[idxRcv]);
							for (size_t i = strlen(buffer) - 1; i < 80; ++i)
								buffer[i] = ' ';
							// Read Epoch Information
							isTime = !(buffer[0] != ' ' || buffer[2] == ' ' ||
								buffer[3] != ' ' || buffer[5] == ' ' ||
								buffer[6] != ' ' || buffer[8] == ' ' ||
								buffer[9] != ' ' || buffer[11] == ' ' ||
								buffer[12] != ' ' || buffer[14] == ' ');
							if (isTime)
							{
								fseek(vFile[idxRcv], curLOC, SEEK_SET);
								break; // jump out
							}

							lineReadStr = buffer;
							lineReadStr = lineReadStr.substr(0, 80);

							// Judge Empty or not
							num_of_line = pObsHeader->obsTypeNum / 5;
							for (int idxI = 1; idxI <= num_of_line; ++idxI)
							{
								if ((pObsHeader->obsTypeNum - idxI * 5) > 0)
								{
									curLOC = ftell(vFile[idxRcv]);
									fgets(buffer, sizeof(buffer), vFile[idxRcv]);
									for (size_t i = strlen(buffer) - 1; i < 80; ++i)
										buffer[i] = ' ';
									// Read Epoch Information
									isTime = !(buffer[0] != ' ' || buffer[2] == ' ' ||
										buffer[3] != ' ' || buffer[5] == ' ' ||
										buffer[6] != ' ' || buffer[8] == ' ' ||
										buffer[9] != ' ' || buffer[11] == ' ' ||
										buffer[12] != ' ' || buffer[14] == ' ');
									if (isTime)
									{
										fseek(vFile[idxRcv], curLOC, SEEK_SET);
										break; // jump out
									}
									std::string addLineReadStr = std::string(buffer);
									lineReadStr += addLineReadStr;
									lineReadStr = lineReadStr.substr(0, pObsHeader->obsTypeNum * 16);
								}
							}
							for (unsigned short i = 0; i < (pObsHeader->obsTypeNum * 16); ++i)
							{
								if (lineReadStr[i] == '\n')
									lineReadStr[i] = ' ';
							}

							double OBS[7];
							unsigned short LLI[7], SNR[7];
							memset(OBS, '\0', sizeof(double) * 7);
							memset(LLI, '\0', sizeof(unsigned short) * 7);
							memset(SNR, '\0', sizeof(unsigned short) * 7);
							for (unsigned short i = 0; i < 7; ++i)	// Only read CA, P1, P2, L1, L2
							{
								if (pObsHeader->obsTypeLoc[i] == 0) continue;
								OBS[i] = atof(lineReadStr.substr((pObsHeader->obsTypeLoc[i] - 1) * 16, 14).c_str());
								LLI[i] = atoi(lineReadStr.substr((pObsHeader->obsTypeLoc[i] - 1) * 16 + 14, 1).c_str());
								SNR[i] = atoi(lineReadStr.substr((pObsHeader->obsTypeLoc[i] - 1) * 16 + 15, 1).c_str());
							}

							pObsData->time = weekNum * 7.0 * 24.0 * 3600.0 + weekSec;
							pObsData->dt = cdt;

							int pre_obs_num = pObsData->n;
							if (fabs(OBS[0]) > 0.01) /* 1C */
							{
								sat_obs_t satRawObsData = { 0 };

								satRawObsData.sys = (unsigned char)cType;
								satRawObsData.prn = (unsigned char)satID;

								satRawObsData.P = OBS[0];
								satRawObsData.L = OBS[3];
								satRawObsData.D = OBS[5];
								satRawObsData.S = (unsigned char)SNR[3];
								satRawObsData.code = 2;

								if (pObsData->n < MAX_OBS_NUM)
								{
									pObsData->dat[pObsData->n++] = satRawObsData;
								}
								else
								{
									/* reach to the maximum observation number */
									printf("error in adding more observation data\r\n");
								}
							}
							else if (fabs(OBS[1]) > 0.01) /* 1P */
							{
								sat_obs_t satRawObsData = { 0 };

								satRawObsData.sys = (unsigned char)cType;
								satRawObsData.prn = (unsigned char)satID;

								satRawObsData.P = OBS[1];
								satRawObsData.L = OBS[3];
								satRawObsData.D = OBS[5];
								satRawObsData.S = (unsigned char)SNR[3];
								satRawObsData.code = 3;

								if (pObsData->n < MAX_OBS_NUM)
								{
									pObsData->dat[pObsData->n++] = satRawObsData;
								}
								else
								{
									/* reach to the maximum observation number */
									printf("error in adding more observation data\r\n");
								}
							}
							if (fabs(OBS[2]) > 0.01) /* 2P */
							{
								sat_obs_t satRawObsData = { 0 };

								satRawObsData.sys = (unsigned char)cType;
								satRawObsData.prn = (unsigned char)satID;

								satRawObsData.P = OBS[2];
								satRawObsData.L = OBS[4];
								satRawObsData.D = OBS[6];
								satRawObsData.S = (unsigned char)SNR[4];
								satRawObsData.code = 9;

								if (pObsData->n < MAX_OBS_NUM)
								{
									pObsData->dat[pObsData->n++] = satRawObsData;
								}
								else
								{
									/* reach to the maximum observation number */
									printf("error in adding more observation data\r\n");
								}
							}
							//----------------------------------------------------------
						}
					}
					else
					{
						// Read Epoch Information
						if (buffer[0] != '>') continue;

						char tempBuffer[255];

						strncpy(tempBuffer, buffer + 31, sizeof(char) * 1); tempBuffer[4] = '\0'; int flag = atoi(tempBuffer);

						strncpy(tempBuffer, buffer + 32, sizeof(char) * 3); tempBuffer[3] = '\0'; int satNum = atoi(tempBuffer);

						if (satNum == 0) continue;
						if (flag != 0 && flag != 1)
						{
							for (int i = 0; i<satNum; ++i)
							{
								if (feof(vFile[idxRcv])) break;
								long curFileLOC = ftell(vFile[idxRcv]);
								fgets(buffer, sizeof(buffer), vFile[idxRcv]);
								if (strlen(buffer)>0 && buffer[0] == '>')
								{
									fseek(vFile[idxRcv], curFileLOC, SEEK_SET);
									break;
								}
							}
							continue;
						}

						strncpy(tempBuffer, buffer + 2, sizeof(char) * 4); tempBuffer[4] = '\0'; int y = atoi(tempBuffer);
						strncpy(tempBuffer, buffer + 0 * 3 + 7, sizeof(char) * 2); tempBuffer[2] = '\0'; int m = atoi(tempBuffer);
						strncpy(tempBuffer, buffer + 1 * 3 + 7, sizeof(char) * 2); tempBuffer[2] = '\0'; int d = atoi(tempBuffer);
						strncpy(tempBuffer, buffer + 2 * 3 + 7, sizeof(char) * 2); tempBuffer[2] = '\0'; int hh = atoi(tempBuffer);
						strncpy(tempBuffer, buffer + 3 * 3 + 7, sizeof(char) * 2); tempBuffer[2] = '\0'; int mm = atoi(tempBuffer);
						strncpy(tempBuffer, buffer + 4 * 3 + 7, sizeof(char) * 11); tempBuffer[11] = '\0'; double ss = atof(tempBuffer);
						strncpy(tempBuffer, buffer + 40, sizeof(char) * 15); tempBuffer[15] = '\0'; double dt = atof(tempBuffer);

						int weekNum = 0;
						double weekSec = ConvertToTimeGPS(y, m, d, hh, mm, ss, &weekNum);

						pObsData->dt = dt;
						pObsData->time = weekNum * 7.0 * 24.0 * 3600.0 + weekSec;
						pObsData->n = 0;
						memset(pObsData->dat, 0, sizeof(sat_obs_t)* MAX_OBS_NUM);

						std::vector<sys_obs_type_t>::const_iterator pSOT;
						std::vector<raw_obs_type_t>::const_iterator pROT;
						std::vector<sat_obs_t>::iterator pROD;

						for (int satIdx = 0; satIdx<satNum; ++satIdx)
						{
							if (feof(vFile[idxRcv])) break;
							fgets(buffer, sizeof(buffer), vFile[idxRcv]);
							if (strlen(buffer) == 0) continue;
							if (buffer[0] == '>')
							{
								long curFileLOC = ftell(vFile[idxRcv]);
								fseek(vFile[idxRcv], curFileLOC, SEEK_SET);
								break;
							}
							char s = toupper(buffer[0]);
							pSOT = std::find(pObsHeader->sys_type.begin(), pObsHeader->sys_type.end(), s);
							if (pSOT == pObsHeader->sys_type.end()) continue;
							strncpy(tempBuffer, buffer + 1, sizeof(char) * 2); tempBuffer[2] = '\0'; int prn = atoi(tempBuffer);

							if (prn <= 0) continue;

							double curOBS[7] = { 0.0 };

							int index = 0;
							for (pROT = pSOT->obs_type.begin(), index = 0; pROT != pSOT->obs_type.end(); ++pROT, ++index)
							{
								/*
								G   16 L1C L1W L2W L2L C1C C1W C2W C2L D1C D1W D2W D2L S1C  SYS / #  / OBS TYPES
									S1W S2W S2L                                          SYS / #  / OBS TYPES
									*/

								strncpy(tempBuffer, buffer + index * 16 + 3, sizeof(char) * 14); tempBuffer[14] = '\0'; double OBS = atof(tempBuffer); if (OBS == 0.0) continue;
								strncpy(tempBuffer, buffer + index * 16 + 3 + 14, sizeof(char) * 1); tempBuffer[1] = '\0'; int    LLI = atoi(tempBuffer);
								strncpy(tempBuffer, buffer + index * 16 + 3 + 14 + 1, sizeof(char) * 1); tempBuffer[1] = '\0'; int    SS = atoi(tempBuffer);
								sat_obs_t satObsData = { 0 };
								satObsData.sys = s;
								satObsData.prn = prn;
								if (s == 'G')
								{
									/*
G   18 C1C C2P C5I C7X C8X C6X L1C L2P L5I L7X L8X L6X S1C  SYS / # / OBS TYPES
	   S2P S5I S7X S8X S6X                                  SYS / # / OBS TYPES

									*/
									/* 1C, 2P, 5I*/
									if (pROT->a == 'C')
									{
										if (pROT->n == '1') /* 1C */
											satObsData.code = 2;
									}
									else if (pROT->a == 'P')
									{
										if (pROT->n == '2') /* 2P */
											satObsData.code = 9;
									}
									else if (pROT->a == 'I')
									{
										if (pROT->n == '5') /* 5I */
											satObsData.code = 22;
									}
								}
								else if (s == 'R')
								{
									/*
	R    7 C1C C1P C2P L1C L2P S1C S2P                          SYS / # / OBS TYPES

										*/
										/* 1C, 2P*/
									if (pROT->a == 'C')
									{
										if (pROT->n == '1') /* 1C */
											satObsData.code = 2;
									}
									else if (pROT->a == 'P')
									{
										if (pROT->n == '2') /* 2P */
											satObsData.code = 9;
									}
								}
								else if (s == 'E')
								{
									/*
E   15 C1X C5X C7X C8X C6X L1X L5X L7X L8X L6X S1X S5X S7X  SYS / # / OBS TYPES
	   S8X S6X                                              SYS / # / OBS TYPES
									*/
									/* 1X, 5X, 7X */
									if (pROT->a == 'X')
									{
										if (pROT->n == '1') /* 1X */
											satObsData.code = 5;
										else if (pROT->n == '5') /* 5X */
											satObsData.code = 24;
										else if (pROT->n == '7') /* 7X */
											satObsData.code = 16;
									}
								}
								else if (s == 'C')
								{
									/*
C    9 C1I C7I C6I L1I L7I L6I S1I S7I S6I                  SYS / # / OBS TYPES
									*/
									/* 1I/2I, 7I, 6I */
									if (pROT->a == 'I')
									{
										if ((pROT->n == '1' && pObsHeader->rinexVersion_ <= 302) || (pROT->n == '2' && pObsHeader->rinexVersion_ > 302)) /* 1I -> 2I (3.0.4, 3.0.5) */
											satObsData.code = 2;
										else if (pROT->n == '7') /* 7I */
											satObsData.code = 14;
										else if (pROT->n == '6') /* 6I */
											satObsData.code = 8;
									}
								}
								if (satObsData.code > 0)
								{
									int idx_obs = find_obs_location(pObsData->dat, pObsData->n, satObsData.sys, satObsData.prn, satObsData.code);
									if (idx_obs == pObsData->n)
									{
										/* new observation data */
										if (pObsData->n == MAX_OBS_NUM)
										{
											/* reach to maximum */
											continue;
										}
										pObsData->dat[pObsData->n] = satObsData;
										pObsData->n++;
									}
									if (pROT->t == 'C')
										pObsData->dat[idx_obs].P = OBS;
									else if (pROT->t == 'L')
									{
										pObsData->dat[idx_obs].L = OBS;
										pObsData->dat[idx_obs].lock = LLI;
										if (pObsData->dat[idx_obs].S == 0&& SS !=0)
											pObsData->dat[idx_obs].S = SS;
									}
									else if (pROT->t == 'D')
										pObsData->dat[idx_obs].D = OBS;
									else if (pROT->t == 'S')
										pObsData->dat[idx_obs].S = (int)OBS;
								}
							}
						}

					}
					if (pObsData->n > 0)
					{
						//printf("%3i,%2i,%2i,%10.7f,%3i\n", idxRcv, hour, min, sec, pObsData->NumOfSat());
						if (curTime == 0.0 || curTime > pObsData->time)
							curTime = pObsData->time;
						break;
					}
				}
			}
			//----------------------------------------------------------------------

			if (curTime==0.0) break; // finish
			//----------------------------------------------------------------------
			// process current epoch data
			//----------------------------------------------------------------------
			int year, month, day, hour, min;
			double sec;
			//----------------------------------------------------------------------
			int wn = (int)(curTime / (7.0 * 24.0 * 3600.0));
			sec = ConvertFromTimeGPS(0, curTime, &year, &month, &day, &hour, &min);
			double useTime = (double)(clock()-st)/CLOCKS_PER_SEC;
			sprintf(buffer, "%4i,%10.3f,%4i,%2i,%2i,%2i,%2i,%6.3f,%10.3f,%6u,%6.2f\n", wn, curTime-wn*7.0*24.0*3600.0, year, month, day, hour, min, sec, useTime, numOfEpoch, (numOfEpoch>0)?(useTime/ numOfEpoch):(0.0));
			printf("%s", buffer);
			//----------------------------------------------------------------------

			/* push obs */
			for (int idxRcv=0;idxRcv<maxRCV;++idxRcv)
			{
				std::vector<rinex_obs_header_t>::iterator pObsHeader = vRinexObsHeader.begin() + idxRcv;
				std::vector<sat_epoch_t>::iterator pObsData = vEpoch.begin() + idxRcv;
				if (fabs(pObsData->time - curTime) < 1.0e-3)
				{
					sat_obs_t* pobs = 0;
					unsigned int idxobs = 0;
					for (idxobs = 0, pobs = pObsData->dat + idxobs; idxobs < pObsData->n; ++idxobs, ++pobs)
					{
						check_sat_eph(0, curTime, pobs, sat_eph, glo_eph);
					}

					//add_rcv_dat(idxRcv + 1, pObsHeader->rcvXYZ_, pObsHeader->rcvName_, pObsHeader->antType_);

					//add_obs_dat(idxRcv + 1, pObsData->time, pObsData->dat, pObsData->n);

				}
			}
			//----------------------------------------------------------------------
			preTime = curTime;
			//----------------------------------------------------------------------
			++numOfEpoch;
			//----------------------------------------------------------------------
			if (numOfEpoch > 3600) break;
		}
		//--------------------------------------------------------------------------
		for (int idxRcv=0;idxRcv<maxRCV;++idxRcv)
		{
			if (vFile[idxRcv]!=NULL)
				fclose(vFile[idxRcv]);
		}
		//----------------------------------------------------------------------
		if (fLOG!=NULL)
		{
			fprintf(fLOG, "%s,%6u,%10.3f\n", fname, numOfEpoch, double((clock()-st))/CLOCKS_PER_SEC);
			fclose(fLOG);
		}
		return;    
	}
	//--------------------------------------------------------------------------
#pragma warning (default:4996)
#pragma warning (default:0266)
//------------------------------------------------------------------------------

