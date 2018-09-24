
// GoogleEarthDisplayDlg.h : 头文件
//

#pragma once

#include "afxwin.h"
#include <vector>
#include "GoogleEarthOpr.h"

// CGoogleEarthDisplayDlg 对话框
class CGoogleEarthDisplayDlg : public CDialogEx
{
// 构造
public:
	CGoogleEarthDisplayDlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
	enum { IDD = IDD_GOOGLEEARTHDISPLAY_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedStartge();
	afx_msg void OnBnClickedStopge();
	afx_msg void OnClose();

public:
	void SetGEWindow();
public:
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnBnClickedButFocus();
	afx_msg BOOL PreTranslateMessage(MSG* pMsg); 
	afx_msg void OnBnClickedButReadLog();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnBnClickedButClear();
	CEdit m_CEdit_lat;
	CEdit m_CEdit_Long;
	CButton m_CBut_Google_Start;
	CButton m_CBut_Focus;
	CStatic m_CStatic_Lat;
	CStatic m_CStatic_Lon;
	CButton m_CBut_ReadLog;
	CButton m_CBut_Clear;
	CButton m_CBut_Stop;
	CStatic m_CStetic1;
	CStatic m_CStatic2;
	CStatic m_CStatic_LogId;
	CStatic m_CStatic_TIP;
	std::vector<HWND> m_VecCtl;
//////////////////////////////////////////////////////////////////////////
 	int ReadTrackLog(std::string szPath);
	std::vector<double> m_VecLatLog;
	std::vector<double> m_VecLonLog;
	std::vector<double> m_VecHeadLog;
	int m_nDispCont;
	CGoogleEarthOpr m_GoogleEarthOpr;
	CString m_szModuleFilePath;
	CString m_HistoryFilePath;
};
