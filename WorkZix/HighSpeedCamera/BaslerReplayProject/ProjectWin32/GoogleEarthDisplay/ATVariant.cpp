/*
=================================================================
Workfile: ATVariant.cpp
Date: 2010-02-08 09:26
Author: �¿�
Description: VARIANT����ת��Ϊshort��long��CString��LPDISPATCH��
			 ����
version: 1.0 
Copyright (C) USER CHENJUN.
=================================================================
*/


#include "stdafx.h"
#include "ATVariant.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

/*********************************************************************
���� : ����һ��CATVariant�Ͷ���
���� : 
	VARIANT vValue : VARIANT��������
���� : NONE
*********************************************************************/
CATVariant::CATVariant(VARIANT vValue)
{
	VariantInit(&m_vValue);
	VariantClear(&m_vValue);
	
	m_vValue = vValue;
}

CATVariant::~CATVariant()
{

}

/*********************************************************************
���� : ��VARIANT����ת��ΪCATVariant�Ͷ���
���� : 
	VARIANT vValue : VARIANT��������
���� : CATVariant����
*********************************************************************/
CATVariant& CATVariant::operator=(VARIANT vValue)
{
	VariantInit(&m_vValue);
	VariantClear(&m_vValue);

	m_vValue = vValue;
	return *this;
}

/*********************************************************************
���� : ��ȡVARIANT���͵�long������
���� : NONE
���� : VARIANT���͵�long������
*********************************************************************/
long CATVariant::GetLong()
{
	long lRetVal = -1;

	ASSERT(m_vValue.vt == VT_I4);
	
	lRetVal = m_vValue.lVal;

	return lRetVal;
}

/*********************************************************************
���� : ��ȡVARIANT���͵�short������
���� : NONE
���� : VARIANT���͵�short������
*********************************************************************/
short CATVariant::GetShort()
{
	short iRetVal = -1;

	ASSERT(m_vValue.vt == VT_I2);
	
	iRetVal = m_vValue.iVal;

	return iRetVal;
}

/*********************************************************************
���� : ��ȡVARIANT���͵�CString������
���� : NONE
���� : VARIANT���͵�CString������
*********************************************************************/
CString CATVariant::GetString()
{
	CString sRetVal = _T("");

	ASSERT(m_vValue.vt == VT_BSTR || m_vValue.vt == VT_LPSTR);
	
	sRetVal = m_vValue.bstrVal;

	return sRetVal;
}

/*********************************************************************
���� : ��ȡVARIANT���͵�LPDISPATCH������
���� : NONE
���� : VARIANT���͵�LPDISPATCH������
*********************************************************************/
LPDISPATCH CATVariant::GetDispatch()
{
	LPDISPATCH pDispRetVal = NULL;
	
	ASSERT(m_vValue.vt == VT_DISPATCH);
	
	pDispRetVal = m_vValue.pdispVal;

	return pDispRetVal;
}