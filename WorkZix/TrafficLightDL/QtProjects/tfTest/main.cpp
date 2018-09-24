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

//    auto input_placeholder = Placeholder(root.WithOpName("start"),
//                                                          tensorflow::DT_STRING);
//    auto file_reader =
//        tensorflow::ops::ReadFile(root.WithOpName("file_reader"),
//                                  string("/dev/shm/tensorflow_temp.bmp"));
//    Tensor aa;
//    auto init_value = Const(root, 0);
//    auto assign = Assign(root.WithOpName("assign"), var, init_value);
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

int main(int argc, char* argv[])
{

    string sample_path = "/home/zix-kotei/PycharmProjects/TrafficLightSelect/demo_data/testsets/images/";
    string graph_path = "/home/zix-kotei/PycharmProjects/TrafficLightSelect/traffic_light.pb";
    string label_path = "/home/zix-kotei/zix/models/object_detection/data/traffic_light_label_map.pbtxt";

    path full_path(sample_path);

    std::unique_ptr<tensorflow::Session> session(
        tensorflow::NewSession(tensorflow::SessionOptions()));
    Status load_graph_status = LoadGraph(graph_path, &session);
    if (!load_graph_status.ok()) {
      LOG(ERROR) << load_graph_status;
      return -1;
    }



//    std::unique_ptr<tensorflow::Session> session_(
//        tensorflow::NewSession(tensorflow::SessionOptions()));
//    GetImageTensor(&session_);






    directory_iterator item_begin(full_path);
    directory_iterator item_end;
    int nCont = 0;
    for ( ; item_begin != item_end;)
    {
        if (is_directory(item_begin->path()))
        {
            cout << item_begin->path().string() << endl;
            item_begin++;
            continue;
        }
        cv::Mat img = cv::imread(item_begin->path().string());
        if (img.data == 0)
        {
            cout << item_begin->path().string() << endl;
            item_begin++;
            continue;
        }
        cout << item_begin->path().string() << endl;
        cv::imwrite("/dev/shm/tensorflow_temp.bmp", img);
        std::vector<Tensor> out_tensors(1);
//        Tensor t(tensorflow::DT_STRING, TensorShape({1,100}));
//        auto t_matrix = t.matrix<char>();
//        memcpy(t_matrix.data(), "/dev/shm/tensorflow_temp.bmp",
//               strlen("/dev/shm/tensorflow_temp.bmp"));

//        session_->Run({/*{"start", t}*/}, {"aaa", "identity"}, {}, &out_tensors);

    //    img_tensor = out_tensors[0];

        GetImageTensor(out_tensors[0]);

        std::vector<Tensor> outputs_;
        Status run_status =
            session->Run({{"image_tensor:0", out_tensors[0]}},
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
                cv::Rect box(vec_boxes[i][1]*img.cols,
                        vec_boxes[i][0]*img.rows,
                        (vec_boxes[i][3]-vec_boxes[i][1])*img.cols,
                        (vec_boxes[i][2]-vec_boxes[i][0])*img.rows);
                cv::Scalar color(255,255,255);
                if (vec_classes[i] == 1)
                {
                    color = CV_RGB(255,0,0);
                }
                if (vec_classes[i] == 2)
                {
                    color = CV_RGB(0,255,0);
                }
                cv::rectangle(img, box, color, 3);
            }
        }
        cv::namedWindow("123", 0);
        cv::imshow("123", img);
        cv::waitKey();
        item_begin++;
    }




    /*
    tensorflow::Output image_encoder =
        EncodePng(root.WithOpName("encode"), out_tensors[1]);
    tensorflow::ops::WriteFile file_saver = tensorflow::ops::WriteFile(
        root.WithOpName("file_writer"), string("/dev/shm/tensorflow_temp_.png"), image_encoder);
    std::vector<Tensor> outputs;
    tensorflow::GraphDef graph_;
    root.ToGraphDef(&graph_);

    std::unique_ptr<tensorflow::Session> session_(
        tensorflow::NewSession(tensorflow::SessionOptions()));
    session_->Create(graph_);
    session_->Run({}, {}, {"file_writer"}, &outputs);
    */

    return 1;
}
