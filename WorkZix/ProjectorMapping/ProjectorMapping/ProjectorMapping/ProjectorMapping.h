
// ProjectorMapping.h : PROJECT_NAME Ӧ�ó������ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"		// ������


// CProjectorMappingApp:
// �йش����ʵ�֣������ ProjectorMapping.cpp
//

class CProjectorMappingApp : public CWinApp
{
public:
	CProjectorMappingApp();

// ��д
public:
	virtual BOOL InitInstance();

// ʵ��

	DECLARE_MESSAGE_MAP()
};

extern CProjectorMappingApp theApp;