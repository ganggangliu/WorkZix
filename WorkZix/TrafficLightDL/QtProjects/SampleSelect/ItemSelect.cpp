#include "ItemSelect.h"

using namespace cv;
using namespace boost;

CObjectInfo::CObjectInfo()
{
    szPose = "Frontal";
    nTruncated = 0;
    nOccluded = 0;
    nXmin = 0;
    nYmin = 0;
    nXmax = 0;
    nYmax = 0;
    nDifficult = 0;
}

ptree CObjectInfo::WriteToPtree()
{
    ptree pt_out;
    pt_out.put<string>("name", szName);
    pt_out.put<string>("pose", szPose);
    pt_out.put<int>("truncated", nTruncated);
    pt_out.put<int>("occluded", nOccluded);
    pt_out.put<int>("bndbox.xmin", nXmin);
    pt_out.put<int>("bndbox.ymin", nYmin);
    pt_out.put<int>("bndbox.xmax", nXmax);
    pt_out.put<int>("bndbox.ymax", nYmax);
    pt_out.put<int>("difficult", nDifficult);

    return pt_out;
}

CMenuSelect::CMenuSelect()
{
    Init();
}

void CMenuSelect::Init()
{
    szType = "";
    nTypeSel = -1;
    nDrawRect = -1;
    nCurStep = 0;
}

int CMenuSelect::InputKey(char szKey)
{
    if(nCurStep == 0)
    {
        if(szKey == 8)//Back-Space:delete last char
        {
            if(szType.length() > 0)
            {
                szType.pop_back();
            }
        }
        else if(szKey == 32)//Space:finish input
        {
            int nType = atoi(szType.c_str());
            nTypeSel = nType;
            nCurStep = 1;
        }
        else if(szKey >= 48 && szKey <= 57)
        {
            szType += szKey;
        }
    }
    else if(nCurStep == 1)
    {
        if(szKey == 8)
        {
            nCurStep = 0;
            szType.clear();
        }
        else if(szKey == 32)//Space:finish input
        {
            nDrawRect = 1;
            nCurStep = 0;
            szType.clear();
            return nTypeSel;
        }
    }

    return -1;
}

int CMenuSelect::GetCurStep()
{
    return nCurStep;
}

string CMenuSelect::GetTip()
{
    string szTip;
    if(nCurStep == 0)
    {
        szTip = to_string(0) + ":enter type:" + szType;
    }
    else if(nCurStep == 1)
    {
        szTip = to_string(1) + ":draw rect";
    }

    return szTip;
}

CImageObjectInfo::CImageObjectInfo()
{
    szFolder = "KOTEI";
    szDataBase = "KOTEI";
    szAnnotaion = "KOTEI";
    szImage = "flickr";
    nSegmented = 0;
    nWidth = 0;
    nHeight = 0;
    nDepth = 3;
    Objects.clear();
}
ptree CImageObjectInfo::WriteToPtree()
{
    ptree pt;
    pt.put<string>("folder", szFolder);
    pt.put<string>("filename", szFileName);
    pt.put<string>("source.database", szDataBase);
    pt.put<string>("source.annotation", szAnnotaion);
    pt.put<string>("source.image", szImage);
    pt.put<int>("size.width", nWidth);
    pt.put<int>("size.height", nHeight);
    pt.put<int>("size.depth", nDepth);
    pt.put<int>("segmented", nSegmented);
    for(unsigned int i = 0; i < Objects.size(); i++)
    {
        pt.add_child("object", Objects[i].WriteToPtree());
    }

    ptree pt_out;
    pt_out.put_child("annotation", pt);

    return pt_out;
}

int CImageObjectInfo::SaveXml(string file_path)
{
    string szFilePath(file_path);
    ptree pt = WriteToPtree();
    boost::property_tree::xml_writer_settings<string> settings('\t', 1);
    write_xml(szFilePath, pt, std::locale(), settings);
}

void pMouseCallback(int mouseEvent,int x,int y,int flags,void* pUser)
{
    CItemSelect* pItemSel = (CItemSelect*)pUser;

    if(pItemSel->m_Menu.nCurStep != 1)
        return;

    switch(mouseEvent)
    {
    case CV_EVENT_LBUTTONDOWN:
        pItemSel->m_ptStart = Point(x,y);
        pItemSel->m_ptEnd = Point(x,y);
        pItemSel->m_rect = Rect(pItemSel->m_ptStart, pItemSel->m_ptEnd);
        pItemSel->m_bIsTragging = true;
        break;
    case CV_EVENT_MOUSEMOVE:
        if(pItemSel->m_bIsTragging)
        {
            pItemSel->m_ptEnd = Point(x,y);
            pItemSel->m_rect = Rect(pItemSel->m_ptStart, pItemSel->m_ptEnd);
            pItemSel->UpdateImg();
        }
        break;
    case CV_EVENT_LBUTTONUP:
        pItemSel->m_ptEnd = Point(x,y);
        pItemSel->m_bIsTragging = false;
        pItemSel->m_rect = Rect(pItemSel->m_ptStart, pItemSel->m_ptEnd);
        pItemSel->UpdateImg();
        break;
    }
    return;
}

CItemSelect::CItemSelect(string szWindowName)
{
    m_szWindowName = szWindowName; 
    m_bIsTragging = false;
}

void CItemSelect::UpdateImg()
{
    Mat MatDisp;
    this->m_OriMat.copyTo(MatDisp);
    for(unsigned int i = 0; i < m_ObjTypeList.size(); i++)
    {
        rectangle(MatDisp, m_ObjLocateList[i], CV_RGB(255,0,0), 2);
        putText(MatDisp, to_string(m_ObjTypeList[i]), m_ObjLocateList[i].tl(),
                0, 0.5, CV_RGB(255,0,0));
    }

    rectangle(MatDisp, m_ptStart, m_ptEnd, CV_RGB(255,0,0), 2);
    putText(MatDisp, to_string(m_Menu.nTypeSel), m_ptStart,
            0, 0.5, CV_RGB(255,0,0));

    m_MatDisp.SetText(m_Menu.GetTip());
    m_MatDisp.Show(MatDisp);
}

int CItemSelect::GetObjcts(string image_path, CImageObjectInfo& obj_info)
{
    Mat img = imread(image_path.c_str());
    if (img.data == 0)
    {
        return 0;
    }
    img.copyTo(m_OriMat);
    m_MatDisp.NamedWindow(m_szWindowName.c_str(), 0);
    m_MatDisp.Show(m_OriMat);
    m_MatDisp.SetMouseCallBack(pMouseCallback, this);
    m_Menu.Init();
    UpdateImg();
    while(1)
    {
        int nRt = m_MatDisp.wait_key();
        if(nRt == 27)
        {
            break;
        }
        int nRtMenu = m_Menu.InputKey(nRt);
        if(nRtMenu == -1)
        {
            UpdateImg();
            continue;
        }
        if(m_rect.width == 0 || m_rect.height == 0)
        {
            UpdateImg();
            continue;
        }
        m_ObjTypeList.push_back(nRtMenu);
        m_ObjLocateList.push_back(m_rect);
        m_rect = Rect(0,0,0,0);
        UpdateImg();
    }
    obj_info.Objects.clear();
    obj_info.szAnnotaion = image_path;
    obj_info.szDataBase = image_path;
    obj_info.szFileName = image_path;
    obj_info.szFolder = image_path;
    obj_info.szImage = image_path;
    for(unsigned int i = 0; i < m_ObjTypeList.size(); i++)
    {
        CObjectInfo ObjT;
        ObjT.nId = m_ObjTypeList[i];
        ObjT.szName = to_string(m_ObjTypeList[i]);
        ObjT.nXmin = m_ObjLocateList[i].x;
        ObjT.nYmin = m_ObjLocateList[i].y;
        ObjT.nXmax = m_ObjLocateList[i].br().x;
        ObjT.nYmax = m_ObjLocateList[i].br().y;
        obj_info.Objects.push_back(ObjT);
    }
}

void MouseCallBackInner(int event, int x, int y, int flags, void* userdata);

CDispMap::CDispMap()
{
    m_dScale = 1.f;
    m_ptZoom = Rect(0,0,0,0);
    m_pCallBack = 0;
    m_pUser = 0;
    m_bIsDragging = false;
    m_ptResize = Point(0,0);
}

void CDispMap::NamedWindow(string szWindName, int mode)
{
    m_szWindName = szWindName;
    cvNamedWindow(m_szWindName.c_str(), mode);
    setMouseCallback(m_szWindName, MouseCallBackInner, this);
}

void CDispMap::Show(cv::Mat img)
{
    img.copyTo(m_OriMat);
    UpdateImgDisp();
}

int CDispMap::wait_key(int nMiliSecd)
{
    return waitKey(nMiliSecd);
}

void MouseCallBackInner(int event, int x, int y, int flags, void* userdata)
{
    CDispMap* pOpr = (CDispMap*)userdata;
    switch(event)
    {
    case CV_EVENT_LBUTTONDBLCLK:
        pOpr->m_ptResize = Point(x,y);
        pOpr->m_dScale -= 0.2;
        pOpr->m_dScale = pOpr->m_dScale <= 0.1 ? 0.1 : pOpr->m_dScale;
        pOpr->ResizeProc();
        break;
    case CV_EVENT_RBUTTONDBLCLK:
        pOpr->m_ptResize = Point(x,y);
        pOpr->m_dScale += 0.2;
        pOpr->m_dScale = pOpr->m_dScale >= 1.f ? 1.f : pOpr->m_dScale;
        pOpr->ResizeProc();
        break;
    case CV_EVENT_LBUTTONDOWN:
        if(flags & CV_EVENT_FLAG_CTRLKEY)
        {
            pOpr->m_ptStart = pOpr->m_ptZoom.tl() + Point(x,y);
            pOpr->m_bIsDragging = true;
        }
        break;
    case CV_EVENT_LBUTTONUP:
        pOpr->m_ptEnd = pOpr->m_ptZoom.tl() + Point(x,y);
        pOpr->m_bIsDragging = false;
        pOpr->MovingProc();
        break;
    case CV_EVENT_MOUSEMOVE:
        if(!pOpr->m_bIsDragging)
        {
            break;
        }
        pOpr->m_ptMoving = pOpr->m_ptZoom.tl() + Point(x,y);
        pOpr->MovingProc();
        break;
    }

    if(pOpr->m_pCallBack)
    {
        (*(pOpr->m_pCallBack))(event, pOpr->m_ptZoom.tl().x+x, pOpr->m_ptZoom.tl().y+y, flags, pOpr->m_pUser);
    }

    return;
}

void CDispMap::SetMouseCallBack(MouseCallback pCallBack, void* pUser)
{
    m_pCallBack = pCallBack;
    m_pUser = pUser;
}

void CDispMap::SetText(string szText)
{
    m_szText = szText;
    UpdateImgDisp();
}

void CDispMap::ResizeProc()
{
    Point ptCursor = m_ptZoom.tl() + m_ptResize;
    double dCursor2RectUpPer = (double)m_ptResize.y/(double)m_ptZoom.height;
    double dCursor2RectLeftPer = (double)m_ptResize.x/(double)m_ptZoom.width;
    int nRectWidth_ = m_OriMat.cols*m_dScale;
    int nRectHeight_ = m_OriMat.rows*m_dScale;
    int nCursor2RectUp_ = ptCursor.y - (dCursor2RectUpPer * nRectHeight_);
    int nCursor2RectLeft_ = ptCursor.x - (dCursor2RectLeftPer * nRectWidth_);
    m_ptZoom = Rect(nCursor2RectLeft_, nCursor2RectUp_, nRectWidth_, nRectHeight_);
    if(m_ptZoom.width >= m_OriMat.cols || m_ptZoom.height >= m_OriMat.rows)
    {
        m_ptZoom = Rect(0, 0, m_OriMat.cols, m_OriMat.rows);
        m_dScale = 1.f;
    }
    else
    {
        if(m_ptZoom.x < 0)
        {
            m_ptZoom.x = 0;
        }
        if(m_ptZoom.y < 0)
        {
            m_ptZoom.y = 0;
        }
        if(m_ptZoom.br().x > m_OriMat.cols)
        {
            m_ptZoom.x -= (m_ptZoom.br().x - m_OriMat.cols);
        }
        if(m_ptZoom.br().y > m_OriMat.rows)
        {
            m_ptZoom.y -= (m_ptZoom.br().y - m_OriMat.rows);
        }
    }

    UpdateImgDisp();
}

void CDispMap::MovingProc()
{
    if(!m_bIsDragging)
        return;
    Point ptDelta = m_ptMoving - m_ptStart;
    Rect new_rect = Rect(m_ptZoom.tl() - ptDelta, m_ptZoom.size());
    if(new_rect.tl().x < 0 || new_rect.tl().y < 0 ||
            new_rect.br().x >= m_OriMat.cols ||
            new_rect.br().y >= m_OriMat.rows)
    {
        return;
    }
    m_ptZoom = new_rect;
    UpdateImgDisp();
}

void CDispMap::UpdateImgDisp()
{
    if(m_ptZoom.width == 0 || m_ptZoom.height == 0)
    {
        m_dScale = 1.f;
        m_ptZoom = Rect(0, 0, m_OriMat.cols, m_OriMat.rows);
    }
    Mat MatDisp;
    if(!m_OriMat.data)
        return;
    m_OriMat(m_ptZoom).copyTo(MatDisp);
    putText(MatDisp, m_szText, Point(0,30), 0, m_dScale, CV_RGB(255,0,0), 1);
    imshow(m_szWindName, MatDisp);
}
