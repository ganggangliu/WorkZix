
// TestDlg.h : ͷ�ļ�
//

#pragma once
#include "afxwin.h"

#include "OpenCVInc.h"

// CTestDlg �Ի���
class CTestDlg : public CDialogEx
{
// ����
public:
	CTestDlg(CWnd* pParent = NULL);	// ��׼���캯��

// �Ի�������
	enum { IDD = IDD_TEST_DIALOG };
	CImage img;
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
public:
	CStatic m_CStaticImage;
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	int m_nCont;
	void MatToCImage(cv::Mat& mat, CImage& cimage);
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedButton1();
};
