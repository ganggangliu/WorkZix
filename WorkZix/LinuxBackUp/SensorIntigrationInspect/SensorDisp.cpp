#include "SensorDisp.h"


CSensorDisp::CSensorDisp()
{
    m_p = new pcl::visualization::PCLVisualizer("Test");
//    m_p->createViewPort (0.0, 0.0, 1.0, 1.0, m_vp1);
    m_p->addCoordinateSystem(200.0, "Ori"/*, m_vp1*/);
//    m_p->createViewPortCamera(m_vp1);
    m_p->initCameraParameters();
    m_p->setCameraPosition(0.0, -50.0, 50.0, 0.0, 30.0, 0.0, 0.0, 0.0, 0.0/*, m_vp1*/);
    m_p->setShowFPS(false);
    m_p->removeCoordinateSystem("Ori");
    m_p->addCoordinateSystem(2.0, "Ori");

    m_nCont = 0;
}

CSensorDisp::~CSensorDisp()
{
    if(m_p)
    {
        delete m_p;
        m_p = NULL;
    }
}

int CSensorDisp::ShowPackData(LCM_SENSOR_FUSION_PACKAGE& Pack)
{
    m_nCont++;
    m_p->removeAllPointClouds();
    m_p->removeAllShapes();

    for(unsigned int i = 0; i < Pack.NaviInfo.Paths.size(); i++)
    {
        LCM_NAVI_PATH& PathT = Pack.NaviInfo.Paths[i];
        pcl::PointCloud<pcl::PointXYZ>::Ptr PathPoints(new pcl::PointCloud<pcl::PointXYZ>);
        PathPoints->reserve(2*PathT.Path.size());
        for(unsigned int j = 0; j < PathT.Path.size(); j++)
        {
            LCM_POINT2D_F& pt0 = PathT.Path[j];
            pcl::PointXYZ pt0_(pt0.x, pt0.y, 0);
            PathPoints->push_back(pt0_);
        }
        for(unsigned int j = PathT.Path.size(); j > 0; j--)
        {
            LCM_POINT2D_F& pt0 = PathT.Path[j-1];
            pcl::PointXYZ pt0_(pt0.x, pt0.y, 0);
            PathPoints->push_back(pt0_);
        }
        char szPathName[256] = {0};
        sprintf(szPathName, "Path%d", i);
        if(PathT.PathType == 0)
        {
            //m_p->addPolygon<pcl::PointXYZ>(PathPoints,0,255,0,szPathName);
            PointCloudColorHandler<pcl::PointXYZ>* handler = new PointCloudColorHandlerCustom<PointXYZ> (PathPoints, 0, 255, 0);
            m_p->addPointCloud(PathPoints,*handler,szPathName);
            delete handler;
        }
        else
        {
            //m_p->addPolygon<pcl::PointXYZ>(PathPoints,0,0,255,szPathName);
            PointCloudColorHandler<pcl::PointXYZ>* handler = new PointCloudColorHandlerCustom<PointXYZ> (PathPoints, 0, 0, 255);
            m_p->addPointCloud(PathPoints,*handler,szPathName);
            delete handler;
        }
    }

    vtkSmartPointer<vtkRendererCollection> pRendClo = m_p->getRendererCollection();
    vtkRenderer* pFirstRend = pRendClo->GetFirstRenderer();
    vtkActorCollection* ActorCol = pFirstRend->GetActors();
    vtkVolumeCollection* pVolumCol = pFirstRend->GetVolumes();
    pVolumCol->RemoveAllItems();
    ActorCol->RemoveAllItems();

//    pFirstRend->Render ();
//    printf("Obj cont: %d\n", ActorCol->GetNumberOfItems());
//    printf("Volume cont: %d\n", pVolumCol->GetNumberOfItems());
    for(unsigned int i = 0; i < m_VecCube.size(); i++)
    {
        pFirstRend->RemoveActor(m_VecCube[i]);
    }
    m_VecCube.clear();
    for(unsigned int i = 0; i < m_VecText3D.size(); i++)
    {
        pFirstRend->RemoveActor(m_VecText3D[i]);
    }
    m_VecText3D.clear();
    for(unsigned int i = 0; i < Pack.IbeoObjList.IbeoObjects.size(); i++)
    {
//        if(m_nCont != 1)
//        {
//            continue;
//        }
        LCM_IBEO_OBJECT& ObjT = Pack.IbeoObjList.IbeoObjects[i];
        Eigen::Vector3f Tran(-1*ObjT.ObjBoxCenter.y/1000.0, ObjT.ObjBoxCenter.x/1000.0, 1);
        Eigen::Matrix3f rotFromMat;
        rotFromMat = Eigen::AngleAxisf(0,  Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(ObjT.ObjOrientation/180.0*CV_PI,   Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf Rot(rotFromMat);
        char szObjName[256] = {0};
        sprintf(szObjName, "Obj%d", i);
//        m_p->addCube(Tran, Rot, ObjT.ObjBoxSize.y/1000.0, ObjT.ObjBoxSize.x/1000.0, 2,szObjName);
        vtkSmartPointer<vtkDataSet> data = createCube (Tran, Rot, ObjT.ObjBoxSize.y/1000.0, ObjT.ObjBoxSize.x/1000.0, 2);
        vtkSmartPointer<vtkLODActor> actor = vtkSmartPointer<vtkLODActor>::New ();
        vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
        mapper->SetInputData (data);
//        mapper->ImmediateModeRenderingOff ();
        actor->SetNumberOfCloudPoints (1/*int (std::max<vtkIdType> (1, data->GetNumberOfPoints () / 10))*/);
//        actor->GetProperty ()->SetInterpolationToFlat ();
        actor->SetMapper (mapper);
//        m_p->createActorFromVTKDataSet (data, actor);
//        actor->GetProperty ()->SetRepresentationToSurface ();
//        addActorToRenderer (actor, viewport);
        pFirstRend->AddActor (actor);
//        pFirstRend->RemoveActor(actor);
        m_VecCube.push_back(actor);

        pcl::PointXYZ ptLocal(-1*ObjT.ObjBoxCenter.y/1000.0, ObjT.ObjBoxCenter.x/1000.0, 1);
        char szText[256] = {0};
        if (ObjT.Classification == 0)
            strcpy(szText,"UnKn");
        else if (ObjT.Classification == 1)
            strcpy(szText,"Small");
        else if (ObjT.Classification == 2)
            strcpy(szText,"Big");
        else if (ObjT.Classification == 3)
            strcpy(szText,"Ped");
        else if (ObjT.Classification == 4)
            strcpy(szText,"Bike");
        else if (ObjT.Classification == 5)
            strcpy(szText,"Car");
        else if (ObjT.Classification == 6)
            strcpy(szText,"Truck");
        else if (ObjT.Classification == 12)
            strcpy(szText,"Under");
        else
            strcpy(szText,"UnKn");
//        sprintf(szText, "%d-%d-%d", ObjT.Id, ObjT.Classification, ObjT.Age);
//        m_p->addText3D<pcl::PointXYZ>(string(szText), ptLocal, 1.0, 1.0,
//                                      1.0, 1.0, szObjName);

        vtkSmartPointer<vtkVectorText> textSource = vtkSmartPointer<vtkVectorText>::New ();
        textSource->SetText (szText);
        textSource->Update ();
        vtkSmartPointer<vtkPolyDataMapper> textMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
        textMapper->SetInputConnection (textSource->GetOutputPort ());
        vtkSmartPointer<vtkFollower> textActor = vtkSmartPointer<vtkFollower>::New ();
        textActor->SetMapper (textMapper);
        textActor->SetPosition (ptLocal.x, ptLocal.y, ptLocal.z);
        textActor->SetScale (1.0);
        textActor->GetProperty ()->SetColor (1, 1, 1);

        textActor->SetCamera (pFirstRend->GetActiveCamera ());
        pFirstRend->AddActor (textActor);
        m_VecText3D.push_back(textActor);
    }

    char szDisp[256] = {0};
    sprintf(szDisp, "Frame:%d", Pack.IbeoObjList.FrameInd);
    m_p->addText(szDisp, 0, 0, 20, 1, 1, 1, "szDisp");

    m_p->spinOnce();
    return 1;
}
