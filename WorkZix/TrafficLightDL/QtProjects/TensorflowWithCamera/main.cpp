/* Copyright 2017 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <vector>
#include <iostream>

#include "tensorflow/cc/ops/const_op.h"
#include "tensorflow/cc/ops/image_ops.h"
#include "tensorflow/cc/ops/standard_ops.h"
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/framework/tensor.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/lib/core/errors.h"
#include "tensorflow/core/lib/core/stringpiece.h"
#include "tensorflow/core/lib/core/threadpool.h"
#include "tensorflow/core/lib/io/path.h"
#include "tensorflow/core/lib/strings/stringprintf.h"
#include "tensorflow/core/platform/init_main.h"
#include "tensorflow/core/platform/logging.h"
#include "tensorflow/core/platform/types.h"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/util/command_line_flags.h"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
using namespace std;
using namespace boost::filesystem;


#include "OpenCVInc.h"

// These are all common classes it's handy to reference with no namespace.
using tensorflow::Flag;
using tensorflow::Tensor;
using tensorflow::Status;
using tensorflow::string;
using tensorflow::int32;
using tensorflow::uint8;

using tensorflow::Tensor;
using tensorflow::TensorShape;

// Reads a model graph definition from disk, and creates a session object you
// can use to run it.
Status LoadGraph(const string& graph_file_name,
                 std::unique_ptr<tensorflow::Session>* session) {
  tensorflow::GraphDef graph_def;
  Status load_graph_status =
      ReadBinaryProto(tensorflow::Env::Default(), graph_file_name, &graph_def);
  if (!load_graph_status.ok()) {
    return tensorflow::errors::NotFound("Failed to load compute graph at '",
                                        graph_file_name, "'");
  }
  session->reset(tensorflow::NewSession(tensorflow::SessionOptions()));
  Status session_create_status = (*session)->Create(graph_def);
  if (!session_create_status.ok()) {
    return session_create_status;
  }
  return Status::OK();
}

int GetImageTensor(Tensor& img_tensor)
{
    auto root = tensorflow::Scope::NewRootScope();
    using namespace ::tensorflow::ops;

    auto file_reader = tensorflow::ops::ReadFile(root.WithOpName("file_reader"),
                                  string("/dev/shm/tensorflow_temp.bmp"));
    auto image_reader = DecodeBmp(root.WithOpName("bmp_reader"), file_reader,
                             DecodeBmp::Channels(3));
    auto original_image = Identity(root.WithOpName("identity"), image_reader);
    auto uint8_caster = Cast(root.WithOpName("float_caster"), original_image,
                             tensorflow::DT_UINT8);
    auto dims_expander = ExpandDims(root, uint8_caster, 0);
    auto aaa = Identity(root.WithOpName("aaa"), dims_expander);

    tensorflow::GraphDef graph;
    root.ToGraphDef(&graph);

    std::vector<Tensor> out_tensors;

    std::unique_ptr<tensorflow::Session> session(
        tensorflow::NewSession(tensorflow::SessionOptions()));
//    session->reset(tensorflow::NewSession(tensorflow::SessionOptions()));
    session->Create(graph);
    session->Run({}, {"aaa", "identity"}, {}, &out_tensors);

    img_tensor = out_tensors[0];
}

void ParsePbTxt(string label_path, vector<string>& name_list)
{
    name_list.resize(10000);
    ifstream fin(label_path, std::ios::in);
    char line[1024]={0};
    int nId = 0;
    string szName;
    while(fin.getline(line, sizeof(line)))
    {
        int nPos0 = string(line).find("  id: ");
        int nPos1 = string(line).find("  display_name: \"");
        if(nPos0 >= 0)
        {
            string szTemp(&(line[nPos0+6]));
            nId = atoi(szTemp.c_str());
        }
        if(nPos1 >= 0)
        {
            string szTemp(&(line[nPos0+16]));
            szName = string(szTemp.c_str());
            name_list[nId] = szName;
        }
    }
}

int main(int argc, char* argv[])
{
//    string graph_path = "/home/zix-kotei/PycharmProjects/TrafficLightSelect/traffic_light_v2.pb";
//    string label_path = "/home/zix-kotei/zix/models/object_detection/data/traffic_light_label_map.pbtxt";
    string graph_path = "/home/zix-kotei/download/ssd_mobilenet_v1_coco_11_06_2017/frozen_inference_graph.pb";
    string label_path = "/home/zix-kotei/zix/models/object_detection/data/mscoco_label_map.pbtxt";

    vector<string> label_list;
    ParsePbTxt(label_path, label_list);

    cv::RNG rng(0);
    std::vector<cv::Scalar> color_list(10000);
    color_list[0] = CV_RGB(255,255,255);
    color_list[1] = CV_RGB(255,0,0);
    color_list[2] = CV_RGB(0,255,0);
    for(unsigned int i = 3; i < color_list.size(); i++)
    {
        color_list[i] = CV_RGB(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
    }

    std::unique_ptr<tensorflow::Session> session(
        tensorflow::NewSession(tensorflow::SessionOptions()));
    Status load_graph_status = LoadGraph(graph_path, &session);
    if (!load_graph_status.ok()) {
      LOG(ERROR) << load_graph_status;
      return -1;
    }

    cv::VideoCapture cap(0);
    if(!cap.isOpened())
    {
        printf("Camera open failed!!!!!!!!!!!!!!!!!!!!!!!!\n");
        return -1;
    }
    cv::Mat frame;
    while(1)
    {
        bool bRt = cap.read(frame);
        if(bRt == false)
        {
            break;
        }
        cv::imwrite("/dev/shm/tensorflow_temp.bmp", frame);

        Tensor out_tensors;

        GetImageTensor(out_tensors);

        std::vector<Tensor> outputs_;
        Status run_status =
            session->Run({{"image_tensor:0", out_tensors}},
                         {"detection_boxes:0", "detection_scores:0", "detection_classes:0",
                         "num_detections:0"}, {}, &outputs_);

        tensorflow::TTypes<float>::Flat num_detections = outputs_[3].flat<float>();
        tensorflow::TTypes<float>::Flat detection_classes = outputs_[2].flat<float>();
        tensorflow::TTypes<float>::Flat indices_scores = outputs_[1].flat<float>();
        auto locations_boxes = outputs_[0].flat<float>();
        int detect_cont = *num_detections.data();
        std::vector<int> vec_classes(detect_cont);
        std::vector<float> vec_scores(detect_cont);
        std::vector<cv::Vec<float,4> > vec_boxes(detect_cont);
        for(int i = 0; i < detect_cont; i++)
        {
            vec_classes[i] = detection_classes(i);
            vec_scores[i] = indices_scores(i);
            for(unsigned int j = 0; j < 4; j++)
            {
                vec_boxes[i][j] = locations_boxes(i*4+j);
            }
        }
        for(int i = 0; i < detect_cont; i++)
        {
            if (vec_scores[i] >= 0.5)
            {
                cv::Rect box(vec_boxes[i][1]*frame.cols,
                        vec_boxes[i][0]*frame.rows,
                        (vec_boxes[i][3]-vec_boxes[i][1])*frame.cols,
                        (vec_boxes[i][2]-vec_boxes[i][0])*frame.rows);
                cv::Scalar color = color_list[vec_classes[i]];
                cv::rectangle(frame, box, color, 2);
                cv::putText(frame, label_list[vec_classes[i]], box.tl(), 0, 1, color, 3);
            }
        }
        cv::namedWindow("123", 0);
        cv::imshow("123", frame);
        cv::waitKey(1);
    }



    return 1;
}
