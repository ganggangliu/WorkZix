/*
=================================================================
Workfile: ATDispatch.h
Date: 2010-02-08 09:26
Author: �¿�
Description: ΪLPDISPATCH���͵Ľӿ��趨���ȡĳ������ֵ��
			 LPDISPATCH���͵Ľӿ���ִ��ĳ������
version: 1.0 
Copyright (C) USER CHENJUN.
=================================================================
*/

#if !defined(AFX_ATDISPATCH_H__4ACCBE6B_62BB_42E4_8343_AF62107CF790__INCLUDED_)
#define AFX_ATDISPATCH_H__4ACCBE6B_62BB_42E4_8343_AF62107CF790__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CATDispatch 
{
public:
	//����һ��CATDispatch�Ͷ���
	CATDispatch(LPDISPATCH pDisp);
	virtual ~CATDispatch();

	//��LPDISPATCH����ת��ΪCATDispatch�Ͷ���
	CATDispatch& operator=(LPDISPATCH pDisp);

	//LPDISPATCH���͵Ľӿ��л�ȡĳ������ֵ	
	VARIANT get(LPOLESTR pwProName, int iParams = -1);

	//LPDISPATCH���͵Ľӿ�������ĳ������ֵ
	VARIANT put(LPOLESTR pwProName, VARIANT vParam);

	//LPDISPATCH���͵Ľӿ���ִ��ĳ������
	VARIANT Function(LPOLESTR pwFunName, const VARIANT vParamCnt, ...);

private:
	//�������Ի��������ƣ�ȡ�����
	DISPID GetPropertyID(LPOLESTR pwProName);
	
	//���ݱ��ȡ�ò����ṹ��
	DISPPARAMS GetDispParams(const VARIANT vParamCnt, va_list& args);
	DISPPARAMS GetDispParams(const VARIANT vParamCnt, ...);

	//�������Ի�������ţ������ṹ�壬ȡ�����Ի򷽷��ķ���ֵ
	VARIANT GetInvokeValue(DISPID dispID, DISPPARAMS dispParams, WORD wFlags);
	
private:
	LPDISPATCH m_pDisp;
};

#endif // !defined(AFX_ATDISPATCH_H__4ACCBE6B_62BB_42E4_8343_AF62107CF790__INCLUDED_)
