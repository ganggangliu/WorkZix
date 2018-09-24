/*
=================================================================
Workfile: ATVariant.h
Date: 2010-02-08 09:26
Author: �¿�
Description: VARIANT����ת��Ϊshort��long��CString��LPDISPATCH��
			 ����
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
	//����һ��CATVariant�Ͷ���
	CATVariant(VARIANT vValue);
	virtual ~CATVariant();

	//��VARIANT����ת��ΪCATVariant�Ͷ���
	CATVariant& operator=(VARIANT vValue);
	
	//��ȡVARIANT���͵�long������
	long GetLong();

	//��ȡVARIANT���͵�short������
	short GetShort();

	//��ȡVARIANT���͵�CString������
	CString GetString();

	//��ȡVARIANT���͵�LPDISPATCH������
	LPDISPATCH GetDispatch();

private:
	VARIANT m_vValue;

};

#endif // !defined(AFX_ATVARIANT_H__23D81B12_8924_4E48_8623_98464ABC51B1__INCLUDED_)
