//------------------------------------------------------------------------------
#include <stdio.h>
#include <string>
#include <vector>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <ctime>

#include <winsock2.h>
#include <ws2tcpip.h>
#include <process.h>    /* _beginthread, _endthread */

//#include "EngineVRS.h"

#pragma comment(lib, "ws2_32.lib")




//------------------------------------------------------------------------------
#include "gnss_proc_rt.h"
//------------------------------------------------------------------------------
#pragma warning (disable:4996)
	//--------------------------------------------------------------------------
	using namespace std;
	//--------------------------------------------------------------------------

	unsigned __stdcall rtcm_read_thread(void* ntrip)
	{
		ntrip_t* pntrip = (ntrip_t*)ntrip;
		return pntrip->nbyte;
	}


	void engine_rt_main(const char* fname)
	{
		clock_t st = clock();
		std::vector<ntrip_t> ntrips;
		int maxRcv = (int)ntrips.size();
		int idxRcv = 0;
		int numRcv = 0;
		// Initialize Winsock.
		WSADATA wsaData;
		int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
		if (iResult != NO_ERROR)
			printf("Error at WSAStartup()\n");

		ntrip_t* pntrip = 0;
		ofstream* os = new ofstream[maxRcv];
		for (idxRcv = 0, pntrip = &ntrips[0] + idxRcv; idxRcv < maxRcv; ++idxRcv, ++pntrip)
		{
			char buffer[255];
			sprintf(buffer, "output\\%s.rtcm", pntrip->mnt);
			os[idxRcv].open(buffer, std::ios::out | std::ios::binary);
		}

		HANDLE* aThread = new HANDLE[maxRcv];
		unsigned long numOfEpoch = 0;
		unsigned int threadID = 0;
		// Create worker threads

		numRcv = 0;
		for (idxRcv = 0, pntrip = &ntrips[0] + idxRcv; idxRcv < maxRcv; ++idxRcv, ++pntrip)
		{
			aThread[numRcv] = (HANDLE)_beginthreadex(NULL, 0, &rtcm_read_thread, (void*)pntrip, 0, &threadID);

			if (aThread[numRcv] == NULL)
				printf("%20s, CreateThread error: %d\n", pntrip->mnt, GetLastError());
			else
				++numRcv;
		}
		while (1)
		{

			// Wait for all threads to terminate
			Sleep(100);
			WaitForMultipleObjects(numRcv, aThread, false, INFINITE);
			// Close thread handles

			for (idxRcv = 0, pntrip = &ntrips[0] + idxRcv; idxRcv < maxRcv; ++idxRcv, ++pntrip)
			{
				if (pntrip->nbyte > 0)
				{
					os[idxRcv].write((char*)pntrip->buffer, pntrip->nbyte);
					//set_rtcm_data_buff(idxRcv, pntrip->buffer, pntrip->nbyte);
				}
			}

			//printf("%10i\n\n", numOfEpoch++);
			printf("\n");
		}

		for (idxRcv = 0; idxRcv < numRcv; ++idxRcv)
			CloseHandle(aThread[idxRcv]);

		delete[] aThread;
		for (idxRcv = 0; idxRcv < maxRcv; ++idxRcv)
			os[idxRcv].close();
		delete[] os;
		return;
	}
	//--------------------------------------------------------------------------
#pragma warning (default:4996)
//------------------------------------------------------------------------------

