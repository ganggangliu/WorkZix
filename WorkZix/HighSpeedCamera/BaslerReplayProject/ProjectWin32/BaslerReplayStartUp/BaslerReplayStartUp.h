
// BaslerReplayStartUp.h : PROJECT_NAME Ӧ�ó������ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"		// ������


// CBaslerReplayStartUpApp:
// �йش����ʵ�֣������ BaslerReplayStartUp.cpp
//

class CBaslerReplayStartUpApp : public CWinApp
{
public:
	CBaslerReplayStartUpApp();

// ��д
public:
	virtual BOOL InitInstance();

// ʵ��

	DECLARE_MESSAGE_MAP()
};

extern CBaslerReplayStartUpApp theApp;