#include "FileOpr.h"

#ifdef WIN32

#include <windows.h>

using namespace std;

CFileOpr::CFileOpr()
{
	m_nFileSize = 0;
	m_nSysBlockSize = 0;
	m_nSysGranularity = 0;
	m_nFileOffSet = 0;
	m_pMapBuffer = 0;
	m_pFileMap = 0;
}

bool CFileOpr::Open(std::string szFilePath)
{
	HANDLE hFile = CreateFileA(szFilePath.c_str(), GENERIC_READ, 0,
		NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL); 
	if (hFile == INVALID_HANDLE_VALUE)
	{
		return false;
	}

	// 创建文件映射对象
	m_pFileMap = CreateFileMappingA(hFile, NULL, PAGE_READONLY, 0, 0, NULL);
	if (m_pFileMap == NULL)
	{
		CloseHandle(hFile);  
		return false;
	}

	//得到系统分配粒度  
	SYSTEM_INFO SysInfo;  
	GetSystemInfo(&SysInfo);  
	m_nSysGranularity = SysInfo.dwAllocationGranularity;

	//得到文件尺寸  
	DWORD dwFileSizeHigh;  
	m_nFileSize = ::GetFileSize(hFile, &dwFileSizeHigh);  
	m_nFileSize |= (((__int64)dwFileSizeHigh) << 32);
	printf("File size: %lld\n", m_nFileSize);

	//关闭文件对象  
	CloseHandle(hFile);  

	//偏移地址   
	m_nFileOffSet = 0;

	// 块大小
	m_nSysBlockSize = 1000 * m_nSysGranularity;
	if (m_nFileSize < 1000 * m_nSysGranularity)
		m_nSysBlockSize = (uint64_t)m_nFileSize;

	m_pMapBuffer = (LPBYTE)MapViewOfFile(m_pFileMap, FILE_MAP_READ, 
		(DWORD)(m_nFileOffSet >> 32), (DWORD)(m_nFileOffSet & 0xFFFFFFFF), m_nSysBlockSize);
	if (m_pMapBuffer == NULL)
		return false;
	UnmapViewOfFile(m_pMapBuffer);

	return true;
}

int64_t CFileOpr::GetFileSize()
{
	return m_nFileSize;
}

int64_t CFileOpr::ReadData(unsigned char* pBuf, uint64_t nCont, uint64_t nOffSet/* = 0*/)
{
	if (m_pFileMap == NULL || m_nFileSize <= 0)
	{
		return -1;
	}
	uint64_t nPageStart = floorl(nOffSet/m_nSysBlockSize)*m_nSysBlockSize;
	//  	printf("%lld\n", nPageStart);
	uint64_t nReadLen = nOffSet + nCont - nPageStart;
	m_pMapBuffer = (LPBYTE)MapViewOfFile(m_pFileMap, FILE_MAP_READ, 
		(DWORD)(nPageStart >> 32), (DWORD)(nPageStart & 0xFFFFFFFF), nReadLen);
	if (m_pMapBuffer == NULL)
	{
		return -1;
	}

	uint64_t nCopyStart = nOffSet - nPageStart;
	memcpy(pBuf, m_pMapBuffer + nCopyStart, nCont);
	UnmapViewOfFile(m_pMapBuffer);

	return 1;
}

bool CFileOpr::Close()
{
	return true/*CloseHandle(m_pFileMap)*/;
}

#else

using namespace std;

CFileOpr::CFileOpr()
{
	m_nFileSize = 0;
}

bool CFileOpr::Open(std::string szFilePath)
{
	m_fs.open(szFilePath, ios::in|ios::binary);
	if (!m_fs.is_open())
	{
		return false;
	}
	m_fs.seekg(0, ios::end);
	m_nFileSize = m_fs.tellg();
	m_fs.seekg(0, ios::beg);
	m_fs.clear();

	return true;
}

int64_t CFileOpr::GetFileSize()
{
	return m_nFileSize;
}

int64_t CFileOpr::ReadData(unsigned char* pBuf, uint64_t nCont, uint64_t nOffSet/* = 0*/)
{
	m_fs.seekg(nOffSet);
	if (m_fs.eof())
		return -1;
	m_fs.read((char*)pBuf, nCont);
	if (m_fs.eof())
	{
		m_fs.clear();
	}
	return nCont;
}

bool CFileOpr::Close()
{
	m_fs.close();

	return true;
}

#endif