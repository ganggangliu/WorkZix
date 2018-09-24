/*
=================================================================
Workfile: ATDispatch.cpp
Date: 2010-02-08 09:26
Author: �¿�
Description: ΪLPDISPATCH���͵Ľӿ��趨���ȡĳ������ֵ��
			 LPDISPATCH���͵Ľӿ���ִ��ĳ������
version: 1.0 
Copyright (C) USER CHENJUN.
=================================================================
*/

#include "stdafx.h"
#include "ATDispatch.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

/*********************************************************************
���� : ����һ��CATDispatch�Ͷ���
���� : 
	LPDISPATCH pDisp : LPDISPATCH��������
���� : NONE
*********************************************************************/
CATDispatch::CATDispatch(LPDISPATCH pDisp) : m_pDisp(pDisp)
{

}

CATDispatch::~CATDispatch()
{

}

/*********************************************************************
���� : ��LPDISPATCH����ת��ΪCATDispatch�Ͷ���
���� : 
	LPDISPATCH pDisp : LPDISPATCH��������
���� : CATDispatch����
*********************************************************************/
CATDispatch& CATDispatch::operator=(LPDISPATCH pDisp)
{
	m_pDisp = pDisp;
	return *this;
}

/*********************************************************************
���� : LPDISPATCH���͵Ľӿ��л�ȡĳ������ֵ	
���� :
	LPOLESTR pwProName	:	���Ե����� 
	int iParams			:	���±�����ԣ�����������(int��)
���� : ����һ���������͵�����
*********************************************************************/
VARIANT CATDispatch::get(LPOLESTR pwProName, int iParams/* = -1 */)
{
	VARIANT vResult;				//���Է��صļ�����
	VariantInit(&vResult);

	//ȡ�õ���ţ�׼�����浽���� 
	DISPID dispID = GetPropertyID(pwProName);

	DISPPARAMS dispParams;
	
	if (iParams == -1)				//�޲�������
	{
		VARIANT args[1];
		args[0].vt = VT_I2;			
		args[0].iVal = 0;			//�����ĸ�����1
		dispParams = GetDispParams(args[0]);
	}
	else							//һ���������ͣ�int�Ͳ�����
	{
		VARIANT args[2];
		args[0].vt = VT_I2;			
		args[0].iVal = 1;			//�����ĸ�����1
		args[1].vt = VT_I2;			
		args[1].iVal = (short)iParams;	//��һ��������ֵ
		dispParams = GetDispParams(args[0], args[1]);
	}

	vResult = GetInvokeValue(dispID, dispParams, DISPATCH_PROPERTYGET);

	if ((iParams != -1) && (dispParams.rgvarg != NULL))
	{
		delete [] dispParams.rgvarg;
	}
	
	return vResult;
}

/*********************************************************************
���� : LPDISPATCH���͵Ľӿ�������ĳ������ֵ	
���� :
	LPOLESTR pwProName	:	���Ե����� 
	VARIANT vParam		:	�����趨��ֵ
	int iParams			:	���±�����ԣ�����������(int��)
���� : ����һ���������͵�����
*********************************************************************/
VARIANT CATDispatch::put(LPOLESTR pwProName, VARIANT vParam)
{
	VARIANT vResult;				//���Է��صļ�����
	VariantInit(&vResult);
	DISPID dispidNamed = DISPID_PROPERTYPUT;

	//ȡ�õ���ţ�׼�����浽���� 
	DISPID dispID = GetPropertyID(pwProName);

	DISPPARAMS dispParams;
	
	dispParams.cArgs = 1;			//��ʾ�����ļ���
	dispParams.cNamedArgs = 1;		//��ʾ���������ļ���
	dispParams.rgdispidNamedArgs = &dispidNamed;//��ʾ���������ĵ���ID
	dispParams.rgvarg = &vParam;	//��ʾ�Բ������������
	
	vResult = GetInvokeValue(dispID, dispParams, DISPATCH_PROPERTYPUT);
	
	return vResult;
}

/*********************************************************************
���� : LPDISPATCH���͵Ľӿ���ִ��ĳ������
���� :
	LPOLESTR pwFunName	:	���������� 
	VARIANT	vParamCnt	:	�����ĸ���(int����)
	...					:	�����б�(��������)
���� : ����һ���������͵�����
*********************************************************************/
VARIANT CATDispatch::Function(LPOLESTR pwFunName, const VARIANT vParamCnt, ...)
{
	VARIANT vResult;				//�������صļ�����
	VariantInit(&vResult);

	//ȡ�õ���ţ�׼�����浽����
	DISPID dispID = GetPropertyID(pwFunName);

	ASSERT(vParamCnt.vt == VT_I2);
	int iParamCnt = vParamCnt.iVal;
	
	va_list args;
	va_start(args, vParamCnt);
	DISPPARAMS dispParams = GetDispParams(vParamCnt, args);
	va_end(args);
	
	vResult = GetInvokeValue(dispID, dispParams, DISPATCH_METHOD);

	if ((iParamCnt > 0) && (dispParams.rgvarg != NULL))
	{
		delete [] dispParams.rgvarg;
	}
	
	return vResult;
}

/*********************************************************************
���� : �������Ի��������ƣ�ȡ�����
���� :
	LPOLESTR pwName : ���Ի��������� 
���� : ���Ի��������
*********************************************************************/
DISPID CATDispatch::GetPropertyID(LPOLESTR pwName)
{
	DISPID dispID;					//ȡ�õ���ţ�׼�����浽���� 

	//������������ȡ�����
	HRESULT hr = m_pDisp->GetIDsOfNames( 
		IID_NULL, 
		&pwName,					//�������Ƶ����� 
		1,							//�������������е�Ԫ�ظ��� 
		LOCALE_SYSTEM_DEFAULT,		//ʹ��ϵͳĬ�ϵ����Ի��� 
		&dispID);					//����ֵ 
	
	//���ʧ�ܣ�˵�����������û��pwName���Ի���
	ASSERT(SUCCEEDED(hr));

	return dispID;
}

/*********************************************************************
���� : �������Ի�������ţ������ṹ�壬ȡ�����Ի򷽷��ķ���ֵ
���� :
	DISPID dispID			: ���Ի��������
	DISPPARAMS dispParams	: �����ṹ��
	WORD wFlags				: ��ʶ������������
		DISPATCH_METHOD			: ��ʶ����
		DISPATCH_PROPERTYGET	: ��ʶ��ȡ����
		DISPATCH_PROPERTYPUT	: ��ʶ��������
���� : ���Ի����ķ���ֵ
*********************************************************************/
VARIANT CATDispatch::GetInvokeValue(DISPID dispID, DISPPARAMS dispParams, WORD wFlags)
{
	VARIANT vResult;				//�������صļ�����
	VariantInit(&vResult);
	VariantClear(&vResult);

	EXCEPINFO excepInfo;			//�쳣��Ϣ
	UINT errArg;					//������Ϣ

	HRESULT hr = m_pDisp->Invoke(	//��������
		dispID,						//������dispIDָ�� 
		IID_NULL, 
		LOCALE_USER_DEFAULT/*LOCALE_SYSTEM_DEFAULT*/,	//ʹ��ϵͳĬ�ϵ����Ի��� 
		wFlags,						//���õ��Ƿ���������
		&dispParams,				//���� 
		&vResult,					//����ֵ 
		&excepInfo,					//�쳣���� 
		&errArg);					//������ 

	ASSERT(SUCCEEDED(hr));			//���ʧ�ܣ�˵���������ݴ���

	return vResult;
}

/*********************************************************************
���� : ���ݱ��ȡ�ò����ṹ��
���� :
	VARIANT vParamCnt		: �����ĸ���
	va_list& args			: �ɱ�����б�
���� : �����ṹ��
*********************************************************************/
DISPPARAMS CATDispatch::GetDispParams(const VARIANT vParamCnt, va_list& args)
{
	ASSERT(vParamCnt.vt == VT_I2);
	int iParamCnt = vParamCnt.iVal;
	
	DISPPARAMS dispParams;

	if (iParamCnt <= 0)				//�޲�������
	{
		dispParams.cArgs = 0;
		dispParams.cNamedArgs = 0;
		dispParams.rgdispidNamedArgs = NULL;
	}
	else							//n������
	{
		dispParams.cArgs = iParamCnt;//��ʾ�����ļ���
		dispParams.cNamedArgs = 0;	//��ʾ���������ļ���
		dispParams.rgdispidNamedArgs = NULL;//��ʾ���������ĵ���ID
		dispParams.rgvarg = new VARIANTARG[iParamCnt];//��ʾ�Բ������������

		for (int i = 0; i < iParamCnt; i++)
		{
			VARIANT va;
			VariantInit(&va);
			
			va = va_arg(args, VARIANT);
			
			ASSERT(va.vt != VT_EMPTY || va.vt != VT_ERROR);

			dispParams.rgvarg[i] = va;				
		}	

		va_end(args);
	}

	return dispParams;
}

/*********************************************************************
���� : ���ݱ��ȡ�ò����ṹ��
���� :
	VARIANT vParamCnt		: �����ĸ���
	...						: �ɱ�����б�
���� : �����ṹ��
*********************************************************************/
DISPPARAMS CATDispatch::GetDispParams(const VARIANT vParamCnt, ...)
{
	ASSERT(vParamCnt.vt == VT_I2);
	int iParamCnt = vParamCnt.iVal;
	
	DISPPARAMS dispParams;

	if (iParamCnt <= 0)				//�޲�������
	{
		dispParams.cArgs = 0;
		dispParams.cNamedArgs = 0;
		dispParams.rgdispidNamedArgs = NULL;
	}
	else							//n������
	{
		dispParams.cArgs = iParamCnt;//��ʾ�����ļ���
		dispParams.cNamedArgs = 0;	//��ʾ���������ļ���
		dispParams.rgdispidNamedArgs = NULL;//��ʾ���������ĵ���ID
		dispParams.rgvarg = new VARIANTARG[iParamCnt];//��ʾ�Բ������������

		va_list args;
		va_start(args, vParamCnt);

		for (int i = 0; i < iParamCnt; i++)
		{
			VARIANT va;
			VariantInit(&va);
			
			va = va_arg(args, VARIANT);
			
			ASSERT(va.vt != VT_EMPTY || va.vt != VT_ERROR);

			dispParams.rgvarg[i] = va;				
		}	

		va_end(args);
	}

	return dispParams;
}



