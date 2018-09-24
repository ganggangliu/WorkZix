#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "LcmReceiver4L.h"
#include "LCM_SENSOR_FUSION_PACKAGE.hpp"

int EuclideanClusterExtraction_(pcl::PointCloud<pcl::PointXYZI>::Ptr& pt_in,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr& pt_out,
                                LCM_IBEO_OBJECT_LIST& ObjList)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_near(new pcl::PointCloud<pcl::PointXYZI>);
    for(unsigned int i = 0; i < pt_in->size(); i++)
    {
        pcl::PointXYZI& pt = pt_in->at(i);
        if (boost::math::isnan(pt.x) ||
                boost::math::isnan(pt.y) ||
                boost::math::isnan(pt.z))
        {
            continue;
        }
        double dDist = sqrt(pow(pt.x, 2) + pow(pt.y, 2) +pow(pt.z, 2));
        if(dDist >= 50.0)
        {
            continue;
        }
        cloud_near->push_back(pt);
    }
    cloud_near->is_dense = true;

    pcl::VoxelGrid<pcl::PointXYZI> vg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    vg.setInputCloud (cloud_near);
    vg.setLeafSize (0.1f, 0.1f, 0.1f);
    vg.filter (*cloud_filtered);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (0.5);
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (10000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    pt_out->clear();
    pt_out->reserve(cloud_filtered->size());
    ObjList.IbeoObjects.clear();
    ObjList.IbeoObjects.reserve(cloud_filtered->size());
    for(unsigned int i = 0; i < cluster_indices.size(); i++)
    {
        pcl::PointIndices& PtInd = cluster_indices[i];
        LCM_IBEO_OBJECT Obj;
        Obj.Age = 100;
        Obj.Classification = 1;
        Obj.Id = i;
        for(unsigned int j = 0; j < PtInd.indices.size(); j++)
        {
            pcl::PointXYZI pt_ = cloud_filtered->at(PtInd.indices[j]);
            pt_.intensity = i;
            pt_out->push_back(pt_);
            LCM_POINT2D_F LcmPt;
            LcmPt.x = pt_.y * 1000.0 - 3500;
            LcmPt.y = -1.0 * pt_.x * 1000.0;
            Obj.ContourPts.push_back(LcmPt);
        }
        Obj.ContContourPt = Obj.ContourPts.size();
//        ObjList.IbeoObjects.push_back(Obj);         //add objects to pack
    }
    ObjList.ContObjects = ObjList.IbeoObjects.size();

    return cluster_indices.size();
}
