#pragma once

#include "OpenCVInc.h"
#include "afxwin.h"

// CDialogProjectWin �Ի���



class CDialogProjectWin : public CDialogEx
{
	DECLARE_DYNAMIC(CDialogProjectWin)

public:
	CDialogProjectWin(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CDialogProjectWin();

// �Ի�������
	enum { IDD = IDD_DIALOG_PROJECT_WIN };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	int Init(CRect DispRect);
	virtual BOOL OnInitDialog();
	afx_msg void OnPaint();
	HWND m_hOpenCvWnd;
	CRect m_DispRect;
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	bool m_bIsFull;
	bool ProjectImage(cv::Mat& image);
	CStatic m_CStatic_Project;

	std::vector<uchar> m_Buf;
	HGLOBAL m_hGlobal;
	void* m_pGlobalBuf;
	IStream* m_pStream;
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	CImage m_imageBuf;

	cv::Mat	m_matBuf;
	unsigned int m_nDispCont;
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	CRITICAL_SECTION m_cs;
};
