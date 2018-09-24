/*
=================================================================
Workfile: ATDispatch.h
Date: 2010-02-08 09:26
Author: 陈俊
Description: 为LPDISPATCH类型的接口设定或获取某个属性值，
			 LPDISPATCH类型的接口中执行某个方法
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
	//构造一个CATDispatch型对象
	CATDispatch(LPDISPATCH pDisp);
	virtual ~CATDispatch();

	//把LPDISPATCH类型转换为CATDispatch型对象
	CATDispatch& operator=(LPDISPATCH pDisp);

	//LPDISPATCH类型的接口中获取某个属性值	
	VARIANT get(LPOLESTR pwProName, int iParams = -1);

	//LPDISPATCH类型的接口中设置某个属性值
	VARIANT put(LPOLESTR pwProName, VARIANT vParam);

	//LPDISPATCH类型的接口中执行某个方法
	VARIANT Function(LPOLESTR pwFunName, const VARIANT vParamCnt, ...);

private:
	//根据属性或函数的名称，取得序号
	DISPID GetPropertyID(LPOLESTR pwProName);
	
	//根据变参取得参数结构体
	DISPPARAMS GetDispParams(const VARIANT vParamCnt, va_list& args);
	DISPPARAMS GetDispParams(const VARIANT vParamCnt, ...);

	//根据属性或函数的序号，参数结构体，取得属性或方法的返回值
	VARIANT GetInvokeValue(DISPID dispID, DISPPARAMS dispParams, WORD wFlags);
	
private:
	LPDISPATCH m_pDisp;
};

#endif // !defined(AFX_ATDISPATCH_H__4ACCBE6B_62BB_42E4_8343_AF62107CF790__INCLUDED_)
