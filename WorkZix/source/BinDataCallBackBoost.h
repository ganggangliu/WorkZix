#ifndef BIN_DATA_CALL_BACK_BOOST_H
#define BIN_DATA_CALL_BACK_BOOST_H

#include <stdint.h>
#include <vector>
#include <iostream>
#include <boost/function.hpp>

typedef boost::function<void(unsigned char*,int,void*)> FunctionBinData;

 class CBinDataCallBackBoost
 {
 public:

	 enum State
	 {
		 WAIT_HEAD,
		 FILL_DATA,
		 CHECK,
		 FINISH
	 };

	 CBinDataCallBackBoost();
	 ~CBinDataCallBackBoost();
	 //Set head pattern and length of binary data, 
	 //this must be called before feeding data stream
	 void SetDataPattern(unsigned char* pszHead, int nHeadLen, int nDataLen);

	 //Set callback, if binary data pattern fitted, execute callback function,
	 //this must be called before feeding data stream
	 void SetCallBack(FunctionBinData hCallBack, void* pUser = 0);

	 //Set CRC check, this is optional
	 //CRC check with bytes ranging ['nStartByte', 'nEndByte'] and  byte 'nCheckByte'
	 void SetCrcCheck(int nStartByte, int nEndByte, int nCheckByte);

	 //Fetch data from device, and call this function to parse data
	 void FeedData(unsigned char* pData, int nDataLen);

 private:
	 bool CrcCheck(unsigned char* pdata, int nStart, int nEnd, int nCheckByte);
	 unsigned char* m_pszHeading;
	 int m_nHeadLen;
	 int m_nDataLen;
	 FunctionBinData m_hCallBack;
	 void* m_pUser;

	 bool m_bIsCrcCheck;
	 int m_nCrcStart;
	 int m_nCrcEnd;
	 int m_nCrcCheckByte;

	 unsigned char* m_pszBuff;
	 State m_State;
	 int m_nSearchInd;

	 long m_nTotalFrames;
	 long m_nBadFrames;
 };


#endif 