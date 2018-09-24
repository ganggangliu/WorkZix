import tensorflow as tf
import os
import io
import PIL.Image
import hashlib

from object_detection.utils import dataset_util

def create_tf_record(data_path, examples, record_name):
    print ('Start -----------' + record_name)
    writer = tf.python_io.TFRecordWriter(record_name)
    for line in examples:
        nPos = line.index(' ')
        if nPos <= 0:
            continue
        image_path = os.path.join(data_path, line[0:nPos])
        label_path = os.path.join(data_path, line[nPos + 1:-1])
        nPos = line.index('images/')
        if nPos < 0:
            continue
        image_id_str = line[nPos+7:nPos+12]
        image_id = int(image_id_str)

        with tf.gfile.GFile(image_path, 'rb') as fid:
            encoded_jpg = fid.read()
        encoded_jpg_io = io.BytesIO(encoded_jpg)
        image = PIL.Image.open(encoded_jpg_io)
        if image.format != 'JPEG':
            raise ValueError('Image format not JPEG')

        width = int(image.size[0])
        height = int(image.size[1])

        label_file_object = open(label_path)
        label_text_lines = label_file_object.readlines()
        for index, line_string in enumerate(label_text_lines):
            xmin = []
            ymin = []
            xmax = []
            ymax = []
            classes = []
            classes_text = []
            truncated = []
            poses = []
            difficult_obj = []

            label_split_list = line_string.split(' ')
            light_type = int(label_split_list[0])
            xmin_ = int(label_split_list[1])
            ymin_ = int(label_split_list[2])
            xmax_ = int(label_split_list[3])
            ymax_ = int(label_split_list[4])
            width_ = xmax_ - xmin_ + 1
            height_ = ymax_ - ymin_ + 1
            rect_xmin = xmin_ - width_
            if rect_xmin < 0:
                rect_xmin = 0
            xmin1 = xmin_ - rect_xmin
            rect_xmax = xmax_ + width_
            if rect_xmax >= width:
                rect_xmax = width - 1
            xmax1 = xmax_ - rect_xmin
            rect_ymin = ymin_ - height_
            if rect_ymin < 0:
                rect_ymin = 0
            ymin1 = ymin_ - rect_ymin
            rect_ymax = ymax_ + height_
            if rect_ymax >= height:
                rect_ymax = height - 1
            ymax1 = ymax_ - rect_ymin

            image_ = image.crop((rect_xmin, rect_ymin, rect_xmax, rect_ymax))
            temp_file_name = 'temp/' + str(image_id) + '_' + str(index) + '_' + str(light_type) + '.jpg'
            image_.save(temp_file_name)

            with tf.gfile.GFile(temp_file_name, 'rb') as fid_:
                encoded_jpg_ = fid_.read()
            encoded_jpg_io_ = io.BytesIO(encoded_jpg_)
            image_ = PIL.Image.open(encoded_jpg_io_)
            if image_.format != 'JPEG':
                raise ValueError('Image format not JPEG')
            key = hashlib.sha256(encoded_jpg_).hexdigest()

            width1 = int(image_.size[0])
            height1 = int(image_.size[1])

            difficult_obj.append(int(0))
            poses.append('Front'.encode('utf-8'))
            truncated.append(int(0))
            classes_text.append(str(light_type).encode('utf-8'))
            classes.append(light_type)
            xmin.append(float(xmin1)/float(width1))
            ymin.append(float(ymin1)/float(height1))
            xmax.append(float(xmax1)/float(width1))
            ymax.append(float(ymax1)/float(height1))

            example = tf.train.Example(features=tf.train.Features(feature={
                'image/height': dataset_util.int64_feature(height1),
                'image/width': dataset_util.int64_feature(width1),
                'image/filename': dataset_util.bytes_feature(
                    temp_file_name.encode('utf8')),
                'image/source_id': dataset_util.bytes_feature(
                    temp_file_name.encode('utf8')),
                'image/key/sha256': dataset_util.bytes_feature(key.encode('utf8')),
                'image/encoded': dataset_util.bytes_feature(encoded_jpg_),
                'image/format': dataset_util.bytes_feature('jpeg'.encode('utf8')),
                'image/object/bbox/xmin': dataset_util.float_list_feature(xmin),
                'image/object/bbox/xmax': dataset_util.float_list_feature(xmax),
                'image/object/bbox/ymin': dataset_util.float_list_feature(ymin),
                'image/object/bbox/ymax': dataset_util.float_list_feature(ymax),
                'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
                'image/object/class/label': dataset_util.int64_list_feature(classes),
                'image/object/difficult': dataset_util.int64_list_feature(difficult_obj),
                'image/object/truncated': dataset_util.int64_list_feature(truncated),
                'image/object/view': dataset_util.bytes_list_feature(poses),
            }))

            writer.write(example.SerializeToString())

            print ('image_id: ' + str(image_id) + '      object_cont: ' + str(index))

    writer.close()

    print ('End -----------' + record_name)



data_path = '/home/zix/PycharmProjects/TrafficLightSelect/demo_data/trainsets'
list_file_path = os.path.join(data_path, 'list')
file_object = open(list_file_path)
lines_list = file_object.readlines()
num_examples = len(lines_list)
num_train = int(1 * num_examples)
train_examples = lines_list[:num_train]
val_examples = lines_list[num_train:]
create_tf_record(data_path, train_examples, 'traffic_light_train.record')
create_tf_record(data_path, val_examples, 'traffic_light_eval.record')
