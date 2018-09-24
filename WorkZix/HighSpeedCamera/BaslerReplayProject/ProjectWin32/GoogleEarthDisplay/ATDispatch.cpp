/*
=================================================================
Workfile: ATDispatch.cpp
Date: 2010-02-08 09:26
Author: 陈俊
Description: 为LPDISPATCH类型的接口设定或获取某个属性值，
			 LPDISPATCH类型的接口中执行某个方法
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
功能 : 构造一个CATDispatch型对象
参数 : 
	LPDISPATCH pDisp : LPDISPATCH类型数据
返回 : NONE
*********************************************************************/
CATDispatch::CATDispatch(LPDISPATCH pDisp) : m_pDisp(pDisp)
{

}

CATDispatch::~CATDispatch()
{

}

/*********************************************************************
功能 : 把LPDISPATCH类型转换为CATDispatch型对象
参数 : 
	LPDISPATCH pDisp : LPDISPATCH类型数据
返回 : CATDispatch对象
*********************************************************************/
CATDispatch& CATDispatch::operator=(LPDISPATCH pDisp)
{
	m_pDisp = pDisp;
	return *this;
}

/*********************************************************************
功能 : LPDISPATCH类型的接口中获取某个属性值	
参数 :
	LPOLESTR pwProName	:	属性的名称 
	int iParams			:	有下标的属性，参数的类型(int型)
返回 : 返回一个变体类型的数据
*********************************************************************/
VARIANT CATDispatch::get(LPOLESTR pwProName, int iParams/* = -1 */)
{
	VARIANT vResult;				//属性返回的计算结果
	VariantInit(&vResult);

	//取得的序号，准备保存到这里 
	DISPID dispID = GetPropertyID(pwProName);

	DISPPARAMS dispParams;
	
	if (iParams == -1)				//无参数类型
	{
		VARIANT args[1];
		args[0].vt = VT_I2;			
		args[0].iVal = 0;			//参数的个数是1
		dispParams = GetDispParams(args[0]);
	}
	else							//一个参数类型（int型参数）
	{
		VARIANT args[2];
		args[0].vt = VT_I2;			
		args[0].iVal = 1;			//参数的个数是1
		args[1].vt = VT_I2;			
		args[1].iVal = (short)iParams;	//第一个参数，值
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
功能 : LPDISPATCH类型的接口中设置某个属性值	
参数 :
	LPOLESTR pwProName	:	属性的名称 
	VARIANT vParam		:	属性设定的值
	int iParams			:	有下标的属性，参数的类型(int型)
返回 : 返回一个变体类型的数据
*********************************************************************/
VARIANT CATDispatch::put(LPOLESTR pwProName, VARIANT vParam)
{
	VARIANT vResult;				//属性返回的计算结果
	VariantInit(&vResult);
	DISPID dispidNamed = DISPID_PROPERTYPUT;

	//取得的序号，准备保存到这里 
	DISPID dispID = GetPropertyID(pwProName);

	DISPPARAMS dispParams;
	
	dispParams.cArgs = 1;			//表示参数的计数
	dispParams.cNamedArgs = 1;		//表示命名参数的计数
	dispParams.rgdispidNamedArgs = &dispidNamed;//表示命名参数的调度ID
	dispParams.rgvarg = &vParam;	//表示对参数数组的引用
	
	vResult = GetInvokeValue(dispID, dispParams, DISPATCH_PROPERTYPUT);
	
	return vResult;
}

/*********************************************************************
功能 : LPDISPATCH类型的接口中执行某个方法
参数 :
	LPOLESTR pwFunName	:	函数的名称 
	VARIANT	vParamCnt	:	参数的个数(int变体)
	...					:	参数列表(变体类型)
返回 : 返回一个变体类型的数据
*********************************************************************/
VARIANT CATDispatch::Function(LPOLESTR pwFunName, const VARIANT vParamCnt, ...)
{
	VARIANT vResult;				//函数返回的计算结果
	VariantInit(&vResult);

	//取得的序号，准备保存到这里
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
功能 : 根据属性或函数的名称，取得序号
参数 :
	LPOLESTR pwName : 属性或函数的名称 
返回 : 属性或函数的序号
*********************************************************************/
DISPID CATDispatch::GetPropertyID(LPOLESTR pwName)
{
	DISPID dispID;					//取得的序号，准备保存到这里 

	//根据属性名，取得序号
	HRESULT hr = m_pDisp->GetIDsOfNames( 
		IID_NULL, 
		&pwName,					//属性名称的数组 
		1,							//属性名称数组中的元素个数 
		LOCALE_SYSTEM_DEFAULT,		//使用系统默认的语言环境 
		&dispID);					//返回值 
	
	//如果失败，说明组件根本就没有pwName属性或函数
	ASSERT(SUCCEEDED(hr));

	return dispID;
}

/*********************************************************************
功能 : 根据属性或函数的序号，参数结构体，取得属性或方法的返回值
参数 :
	DISPID dispID			: 属性或函数的序号
	DISPPARAMS dispParams	: 参数结构体
	WORD wFlags				: 标识方法还是属性
		DISPATCH_METHOD			: 标识方法
		DISPATCH_PROPERTYGET	: 标识获取属性
		DISPATCH_PROPERTYPUT	: 标识设置属性
返回 : 属性或函数的返回值
*********************************************************************/
VARIANT CATDispatch::GetInvokeValue(DISPID dispID, DISPPARAMS dispParams, WORD wFlags)
{
	VARIANT vResult;				//函数返回的计算结果
	VariantInit(&vResult);
	VariantClear(&vResult);

	EXCEPINFO excepInfo;			//异常信息
	UINT errArg;					//错误信息

	HRESULT hr = m_pDisp->Invoke(	//调用属性
		dispID,						//属性由dispID指定 
		IID_NULL, 
		LOCALE_USER_DEFAULT/*LOCALE_SYSTEM_DEFAULT*/,	//使用系统默认的语言环境 
		wFlags,						//调用的是方法或属性
		&dispParams,				//参数 
		&vResult,					//返回值 
		&excepInfo,					//异常处理 
		&errArg);					//错误处理 

	ASSERT(SUCCEEDED(hr));			//如果失败，说明参数传递错误

	return vResult;
}

/*********************************************************************
功能 : 根据变参取得参数结构体
参数 :
	VARIANT vParamCnt		: 参数的个数
	va_list& args			: 可变参数列表
返回 : 参数结构体
*********************************************************************/
DISPPARAMS CATDispatch::GetDispParams(const VARIANT vParamCnt, va_list& args)
{
	ASSERT(vParamCnt.vt == VT_I2);
	int iParamCnt = vParamCnt.iVal;
	
	DISPPARAMS dispParams;

	if (iParamCnt <= 0)				//无参数类型
	{
		dispParams.cArgs = 0;
		dispParams.cNamedArgs = 0;
		dispParams.rgdispidNamedArgs = NULL;
	}
	else							//n个参数
	{
		dispParams.cArgs = iParamCnt;//表示参数的计数
		dispParams.cNamedArgs = 0;	//表示命名参数的计数
		dispParams.rgdispidNamedArgs = NULL;//表示命名参数的调度ID
		dispParams.rgvarg = new VARIANTARG[iParamCnt];//表示对参数数组的引用

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
功能 : 根据变参取得参数结构体
参数 :
	VARIANT vParamCnt		: 参数的个数
	...						: 可变参数列表
返回 : 参数结构体
*********************************************************************/
DISPPARAMS CATDispatch::GetDispParams(const VARIANT vParamCnt, ...)
{
	ASSERT(vParamCnt.vt == VT_I2);
	int iParamCnt = vParamCnt.iVal;
	
	DISPPARAMS dispParams;

	if (iParamCnt <= 0)				//无参数类型
	{
		dispParams.cArgs = 0;
		dispParams.cNamedArgs = 0;
		dispParams.rgdispidNamedArgs = NULL;
	}
	else							//n个参数
	{
		dispParams.cArgs = iParamCnt;//表示参数的计数
		dispParams.cNamedArgs = 0;	//表示命名参数的计数
		dispParams.rgdispidNamedArgs = NULL;//表示命名参数的调度ID
		dispParams.rgvarg = new VARIANTARG[iParamCnt];//表示对参数数组的引用

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



