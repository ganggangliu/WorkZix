#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <vector>
#include "OpenCVInc.h"

#include "ItemSelect.h"


using namespace std;
using namespace cv;


//int PickSamples(string image_path)
//{
//    Mat Img = imread(image_path.c_str());
//    if (Img.data == 0)
//    {
//        return 0;
//    }
//    cvNamedWindow("Pick window", 0);
//    setMouseCallback("Pick window", pMouseCallback);
//    while(1)
//    {
//        int nRt = waitKey();
//    }

//    return 1;
//}

int main()
{
    Rect aaa(0,0,10,10);
    string img_path = "/home/zix-kotei/PycharmProjects/TrafficLightSelect/temp.jpg";
    Mat img = imread(img_path);

    CItemSelect ItemSel("");
    while(1)
    {
        CImageObjectInfo obj_info;
        ItemSel.GetObjcts(img_path, obj_info);
        obj_info.SaveXml("123.xml");
    }
    
//    CDispMap MatDisp;
//    MatDisp.NamedWindow("123", 0);
//    MatDisp.Show(img);
//    while(1)
//    {
//        MatDisp.wait_key();
//    }
}

//int main1()
//{
//    boost::filesystem::path path1("/home/zix-kotei/QtProjects/SampleSelect/demo_data/testsets/images/123.jpg");
//    boost::filesystem::path::iterator pathI = --path1.end();
//    string file_name(pathI->string());

//    string image_path = "/home/zix-kotei/QtProjects/SampleSelect/demo_data/testsets/images/";
//    string label_save_path = "/home/zix-kotei/QtProjects/SampleSelect/demo_data/testsets/labels/";

//    directory_iterator item_begin(image_path);
//    directory_iterator item_end;
//    for ( ; item_begin != item_end;)
//    {
//        if (is_directory(item_begin->path()))
//        {
//            cout << item_begin->path().string() << endl;
//            item_begin++;
//            continue;
//        }
//        Mat Img = imread(item_begin->path().string());
//        if (Img.data == 0)
//        {
//            cout << item_begin->path().string() << endl;
//            item_begin++;
//            continue;
//        }
//        PickSamples(item_begin->path().string());
//    }
//}

