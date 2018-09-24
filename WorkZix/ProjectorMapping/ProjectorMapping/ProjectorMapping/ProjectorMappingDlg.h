
// ProjectorMappingDlg.h : ͷ�ļ�
//

#pragma once

#include "DialogProjectWin.h"
#include "ProjectorCalibrate.h"
#include "DialogMonitorSet.h"

// CProjectorMappingDlg �Ի���
class CProjectorMappingDlg : public CDialogEx
{
// ����
public:
	CProjectorMappingDlg(CWnd* pParent = NULL);	// ��׼���캯��

// �Ի�������
	enum { IDD = IDD_PROJECTORMAPPING_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV ֧��


// ʵ��
protected:
	HICON m_hIcon;

	// ���ɵ���Ϣӳ�亯��
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
