
// GoogleEarthDisplay.h : PROJECT_NAME Ӧ�ó������ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"		// ������


// CGoogleEarthDisplayApp:
// �йش����ʵ�֣������ GoogleEarthDisplay.cpp
//

class CGoogleEarthDisplayApp : public CWinApp
{
public:
	CGoogleEarthDisplayApp();

// ��д
public:
	virtual BOOL InitInstance();
	virtual BOOL ExitInstance();

// ʵ��

	DECLARE_MESSAGE_MAP()
};

extern CGoogleEarthDisplayApp theApp;