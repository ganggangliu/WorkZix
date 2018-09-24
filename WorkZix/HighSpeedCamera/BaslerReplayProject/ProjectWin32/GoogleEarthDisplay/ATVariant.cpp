/*
=================================================================
Workfile: ATVariant.cpp
Date: 2010-02-08 09:26
Author: 陈俊
Description: VARIANT类型转换为short、long、CString、LPDISPATCH等
			 类型
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
功能 : 构造一个CATVariant型对象
参数 : 
	VARIANT vValue : VARIANT类型数据
返回 : NONE
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
功能 : 把VARIANT类型转换为CATVariant型对象
参数 : 
	VARIANT vValue : VARIANT类型数据
返回 : CATVariant对象
*********************************************************************/
CATVariant& CATVariant::operator=(VARIANT vValue)
{
	VariantInit(&m_vValue);
	VariantClear(&m_vValue);

	m_vValue = vValue;
	return *this;
}

/*********************************************************************
功能 : 获取VARIANT类型的long型数据
参数 : NONE
返回 : VARIANT类型的long型数据
*********************************************************************/
long CATVariant::GetLong()
{
	long lRetVal = -1;

	ASSERT(m_vValue.vt == VT_I4);
	
	lRetVal = m_vValue.lVal;

	return lRetVal;
}

/*********************************************************************
功能 : 获取VARIANT类型的short型数据
参数 : NONE
返回 : VARIANT类型的short型数据
*********************************************************************/
short CATVariant::GetShort()
{
	short iRetVal = -1;

	ASSERT(m_vValue.vt == VT_I2);
	
	iRetVal = m_vValue.iVal;

	return iRetVal;
}

/*********************************************************************
功能 : 获取VARIANT类型的CString型数据
参数 : NONE
返回 : VARIANT类型的CString型数据
*********************************************************************/
CString CATVariant::GetString()
{
	CString sRetVal = _T("");

	ASSERT(m_vValue.vt == VT_BSTR || m_vValue.vt == VT_LPSTR);
	
	sRetVal = m_vValue.bstrVal;

	return sRetVal;
}

/*********************************************************************
功能 : 获取VARIANT类型的LPDISPATCH型数据
参数 : NONE
返回 : VARIANT类型的LPDISPATCH型数据
*********************************************************************/
LPDISPATCH CATVariant::GetDispatch()
{
	LPDISPATCH pDispRetVal = NULL;
	
	ASSERT(m_vValue.vt == VT_DISPATCH);
	
	pDispRetVal = m_vValue.pdispVal;

	return pDispRetVal;
}