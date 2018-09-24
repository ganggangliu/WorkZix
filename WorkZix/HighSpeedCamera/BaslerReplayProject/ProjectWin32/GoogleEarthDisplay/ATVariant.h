/*
=================================================================
Workfile: ATVariant.h
Date: 2010-02-08 09:26
Author: 陈俊
Description: VARIANT类型转换为short、long、CString、LPDISPATCH等
			 类型
version: 1.0 
Copyright (C) USER CHENJUN.
=================================================================
*/


#if !defined(AFX_ATVARIANT_H__23D81B12_8924_4E48_8623_98464ABC51B1__INCLUDED_)
#define AFX_ATVARIANT_H__23D81B12_8924_4E48_8623_98464ABC51B1__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CATVariant  
{
public:
	//构造一个CATVariant型对象
	CATVariant(VARIANT vValue);
	virtual ~CATVariant();

	//把VARIANT类型转换为CATVariant型对象
	CATVariant& operator=(VARIANT vValue);
	
	//获取VARIANT类型的long型数据
	long GetLong();

	//获取VARIANT类型的short型数据
	short GetShort();

	//获取VARIANT类型的CString型数据
	CString GetString();

	//获取VARIANT类型的LPDISPATCH型数据
	LPDISPATCH GetDispatch();

private:
	VARIANT m_vValue;

};

#endif // !defined(AFX_ATVARIANT_H__23D81B12_8924_4E48_8623_98464ABC51B1__INCLUDED_)
