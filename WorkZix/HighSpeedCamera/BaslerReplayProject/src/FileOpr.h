#ifndef FILE_OPR_H
#define FILE_OPR_H

#include <fstream>
#include <string>
#include <cstdint>

#ifdef WIN32

class CFileOpr
{
public:
	CFileOpr();
	bool Open(std::string szFilePath);
	int64_t GetFileSize();
	int64_t ReadData(unsigned char* pBuf, uint64_t nCont, uint64_t nOffSet = 0);
	bool Close();

private:
	uint64_t m_nFileSize;
	uint64_t m_nSysBlockSize;
	uint64_t m_nSysGranularity;
	uint64_t m_nFileOffSet;
	unsigned char* m_pMapBuffer;
	void* m_pFileMap;
};

#else

class CFileOpr
{
public:
	CFileOpr();
	bool Open(std::string szFilePath);
	int64_t GetFileSize();
	int64_t ReadData(unsigned char* pBuf, uint64_t nCont, uint64_t nOffSet = 0);
	bool Close();

private:
	std::ifstream m_fs;
	int64_t m_nFileSize;
};

#endif


#endif