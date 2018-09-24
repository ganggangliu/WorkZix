#include <string.h>
#include <list>
#include <vector>
#include <stdio.h>
#include <lcm/lcm.h>

#include <boost/timer.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
using namespace boost;

struct LcmDataInfo
{
	std::string szChannelName;
	std::list<posix_time::ptime> TimeList;//microseconds
	std::list<uint32_t> SizeList;//bytes
	void FreashData()
	{
		TimeList.clear();
		SizeList.clear();
	}
	void AddInfo(posix_time::ptime Time, uint32_t nSize)
	{
		TimeList.push_back(Time);
		SizeList.push_back(nSize);
	}
	void GetInfo(uint64_t& nSizeAvg, uint64_t& nFramRate)
	{

		uint64_t nAcc = 0;
		std::list<uint32_t>::iterator ItorS;
		for (ItorS = SizeList.begin(); ItorS != SizeList.end();)
		{
			nAcc += *ItorS;
			ItorS++;
		}
		if (SizeList.size() == 0)
			nSizeAvg = 0;
		else
			nSizeAvg = nAcc/SizeList.size();
		nFramRate = SizeList.size();
		FreashData();
	}
	
};

std::vector<LcmDataInfo> g_VecBuf;
boost::mutex g_mutex;
boost::shared_ptr<boost::thread> g_pThreadDisplay;

void catchall_handler(const lcm_recv_buf_t *rbuf, const char *channel, void *u)
{
	g_mutex.lock();
	posix_time::ptime time_now = posix_time::microsec_clock::universal_time();
	int nChannelInd = -1;
	for (unsigned i = 0; i < g_VecBuf.size(); i++)
	{
		if (std::string(channel).compare(g_VecBuf[i].szChannelName) == 0)
		{
			nChannelInd = i;
			break;
		}
	}
	if (nChannelInd == -1)
	{
		LcmDataInfo InfoT;
		InfoT.szChannelName = std::string(channel);
		InfoT.AddInfo(time_now, rbuf->data_size);
		g_VecBuf.push_back(InfoT);
	}
	else
	{
		g_VecBuf[nChannelInd].AddInfo(time_now, rbuf->data_size);
	}
	g_mutex.unlock();
}

void ThreadDisplay()
{
	int nDistCont = 0;
	while(1)
	{
		nDistCont++;
		boost::thread::sleep(get_system_time() + posix_time::millisec(1000));
 		g_mutex.lock();
 		system("cls");
 		std::cout << nDistCont << std::endl;
 		std::cout << "szChannelName | nSizeAvg | nFramRate | nTotalBytes" << std::endl;
		std::cout << "--------------------------------------------------------------------" << std::endl;
 		for (unsigned int i = 0; i < g_VecBuf.size(); i++)
 		{
 			uint64_t nSizeAvg;
 			uint64_t nFramRate;
 			g_VecBuf[i].GetInfo(nSizeAvg, nFramRate);
 			std::cout << "[" << g_VecBuf[i].szChannelName << "]" << " | " << nSizeAvg << " Bytes/Frame" << " | " 
				<< nFramRate << " fps" << " | " << nSizeAvg*nFramRate << " Bytes/s" << std::endl;
 		}
 		g_mutex.unlock();
	}
}

int main(int argc, char **argv)
{
	lcm_t *lcm = lcm_create(NULL);
	if (!lcm) 
	{
		fprintf(stderr, "couldn't initialize LCM\n");
		return 1;
	}
	lcm_subscribe(lcm, ".*", catchall_handler, NULL);

	g_pThreadDisplay = shared_ptr<thread>(new thread(bind(&ThreadDisplay)));

	while(0 == lcm_handle(lcm)) 
	{
	}

	lcm_destroy(lcm);

	return 0;
}
