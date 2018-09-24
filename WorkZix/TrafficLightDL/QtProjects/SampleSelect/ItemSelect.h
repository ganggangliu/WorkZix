#ifndef ITEM_SELECT_H
#define ITEM_SELECT_H

#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include "OpenCVInc.h"

using namespace std;
using namespace boost::property_tree;

class CObjectInfo
{
public:
    string szName;
    string szPose;
    int nId;
    int nTruncated;
    int nOccluded;
    int nXmin;
    int nYmin;
    int nXmax;
    int nYmax;
    int nDifficult;
    CObjectInfo();
    ptree WriteToPtree();
};

class CImageObjectInfo
{
public:
    string szFolder;
    string szFileName;
    string szDataBase;
    string szAnnotaion;
    string szImage;
    int nWidth;
    int nHeight;
    int nDepth;
    int nSegmented;
    vector<CObjectInfo> Objects;
    CImageObjectInfo();
    ptree WriteToPtree();
    int SaveXml(string file_path);
};

class CMenuSelect
{
public:
    CMenuSelect();
    string szType;
    int nTypeSel;
    int nDrawRect;
    int nCurStep;    //0:select type  1:draw rect confirm
    void Init();
    int InputKey(char szKey);
    int GetCurStep();
    string GetTip();
};

class CDispMap
{
public:
    CDispMap();
    void NamedWindow(string szWindName, int mode = 0);
    void Show(cv::Mat img);
    int wait_key(int nMiliSecd = 0);
    void SetMouseCallBack(cv::MouseCallback pCallBack, void* pUser = 0);
    friend void MouseCallBackInner(int event, int x, int y, int flags, void* userdata);
    void SetText(string szText);
    void UpdateImgDisp();

private:
    void ResizeProc();
    void MovingProc();
    cv::MouseCallback m_pCallBack;
    void* m_pUser;
    string m_szWindName;

    cv::Mat m_OriMat;
    cv::Rect m_ptZoom;
    float m_dScale;

    bool m_bIsDragging;
    cv::Point m_ptStart;
    cv::Point m_ptEnd;
    cv::Point m_ptMoving;

    cv::Point m_ptResize;
    string m_szText;
};

class CItemSelect
{
public:
    CItemSelect(string szWindowName);
    int GetObjcts(string image_path, CImageObjectInfo& obj_info);
    void UpdateImg();

    friend void pMouseCallback(int mouseEvent,int x,int y,int flags,void* pUser);
private:
    cv::Mat m_OriMat;
    void DrawRect();
    string m_szWindowName;
    CMenuSelect m_Menu;
    cv::Rect m_rect;
    std::vector<int> m_ObjTypeList;
    std::vector<cv::Rect> m_ObjLocateList;
    cv::Rect m_ZoomIn;
    CDispMap m_MatDisp;
    cv::Point m_ptStart;
    cv::Point m_ptEnd;
    bool m_bIsTragging;
};


#endif
