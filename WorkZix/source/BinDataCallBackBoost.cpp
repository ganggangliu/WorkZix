#include "BinDataCallBackBoost.h"

CBinDataCallBackBoost::CBinDataCallBackBoost()
{
	m_pszHeading = 0;
	m_nHeadLen = 0;
	m_nDataLen = 0;
	m_pUser = 0;

	m_bIsCrcCheck = false;
	m_nCrcStart = 0;
	m_nCrcEnd = 0;
	m_nCrcCheckByte = 0;

	m_pszBuff = 0;
	m_State = WAIT_HEAD;
	m_nSearchInd = 0;

	m_nTotalFrames = 0;
	m_nBadFrames = 0;
}

CBinDataCallBackBoost::~CBinDataCallBackBoost()
{
	if (m_pszHeading)
	{
		delete[] m_pszHeading;
		m_pszHeading = NULL;
	}
	if (m_pszBuff)
	{
		delete[] m_pszBuff;
		m_pszBuff = NULL;
	}
}

void CBinDataCallBackBoost::SetDataPattern(unsigned char* pszHead, int nHeadLen, int nDataLen)
{
	if (m_pszHeading)
	{
		delete[] m_pszHeading;
		m_pszHeading = NULL;
	}
	if (m_pszBuff)
	{
		delete[] m_pszBuff;
		m_pszBuff = NULL;
	}

	m_pszHeading = new unsigned char[nHeadLen];
	memcpy(m_pszHeading, pszHead ,nHeadLen);
	m_nDataLen = nDataLen;
	m_nHeadLen = nHeadLen;

	m_pszBuff = new unsigned char[nDataLen];
	memset(m_pszBuff, 0, nDataLen);

}

void CBinDataCallBackBoost::SetCallBack(FunctionBinData hCallBack, void* pUser)
{
	m_hCallBack = hCallBack;
	m_pUser = pUser;
}

void CBinDataCallBackBoost::SetCrcCheck(int nStartByte, int nEndByte, int nCheckByte)
{
	m_bIsCrcCheck = true;
	m_nCrcStart = nStartByte;
	m_nCrcEnd = nEndByte;
	m_nCrcCheckByte = nCheckByte;
}

void CBinDataCallBackBoost::FeedData(unsigned char* pData, int nDataLen)
{
	for (int i = 0; i < nDataLen; i++)
	{
		if (m_State == WAIT_HEAD)
		{
			if (pData[i] == m_pszHeading[m_nSearchInd])
			{
				m_pszBuff[m_nSearchInd] = pData[i];
				m_nSearchInd++;
			}
			else
			{
				m_nSearchInd = 0;
			}
			if (m_nSearchInd == m_nHeadLen)
			{
				m_State = FILL_DATA;
			}
		}
		else if (m_State == FILL_DATA)
		{
			//m_pszBuff[m_nSearchInd] = pData[i];
			//m_nSearchInd++;
			//if (m_nSearchInd == m_nDataLen)
			//{
			//	m_State = CHECK;
			//}
			int nRestDataLen = nDataLen - i;
			int nRestSearchLen = m_nDataLen - m_nSearchInd;
			if (nRestDataLen < nRestSearchLen)
			{
				memcpy(m_pszBuff+m_nSearchInd,pData+i,nRestDataLen);
				m_nSearchInd += nRestDataLen;
				i = nDataLen;
				break;
			}
			else
			{
				memcpy(m_pszBuff+m_nSearchInd,pData+i,nRestSearchLen);
				m_nSearchInd = m_nDataLen;
				i += nRestSearchLen;
				m_State = CHECK;
			}
		}

		if (m_State == CHECK)
		{
			if (m_bIsCrcCheck)
			{
				if (CrcCheck((unsigned char*)m_pszBuff, m_nCrcStart, m_nCrcEnd, m_nCrcCheckByte))
				{
					m_State = FINISH;
				}
				else
				{
					m_nSearchInd = 0;
					m_State = WAIT_HEAD;
					m_nBadFrames++;
				}
			}
			else
			{
				m_State = FINISH;
			}
		}

		if (m_State == FINISH)
		{
			if (!m_hCallBack.empty())
			{
				m_hCallBack(m_pszBuff, (m_nDataLen+m_nHeadLen), m_pUser);
			}
			m_nSearchInd = 0;
			m_State = WAIT_HEAD;
			m_nTotalFrames++;
		}
	}
}

bool CBinDataCallBackBoost::CrcCheck(unsigned char* pdata, int nStart, int nEnd, int nCheckByte)
{
	unsigned char CS = 0; 
	for (int i = nStart; i< nEnd + 1; i++) 
	{ 
		CS += pdata[i]; 
	}

	return (CS == pdata[nCheckByte]);
}