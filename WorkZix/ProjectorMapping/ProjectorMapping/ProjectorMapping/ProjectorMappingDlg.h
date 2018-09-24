
// ProjectorMappingDlg.h : 头文件
//

#pragma once

#include "DialogProjectWin.h"
#include "ProjectorCalibrate.h"
#include "DialogMonitorSet.h"

// CProjectorMappingDlg 对话框
class CProjectorMappingDlg : public CDialogEx
{
// 构造
public:
	CProjectorMappingDlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
	enum { IDD = IDD_PROJECTORMAPPING_DIALOG };

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

	CDialogProjectWin* m_pDialogProj;
public:
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	afx_msg void OnBnClickedButtonDispSet();
	afx_msg void OnBnClickedButtonStart();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	cv::VideoCapture m_cap;
	afx_msg void OnBnClickedButtonStartCalib();
	CProjectCalibrate m_CalibOpr;
	bool ProjectImage(cv::Mat image);
	CDialogMonitorSet* m_pDlgMonitorSet;

friend bool WINAPI ProjectImageDisplayCallBack(cv::Mat& image);
afx_msg void OnBnClickedButtonProject();
afx_msg void OnBnClickedButtonProjectVideo();
afx_msg void OnBnClickedButtonCameraView();
afx_msg void OnDestroy();
virtual BOOL DestroyWindow();
afx_msg void OnClose();
};
