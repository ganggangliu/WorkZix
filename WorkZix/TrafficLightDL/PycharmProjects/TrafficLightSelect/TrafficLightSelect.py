import tensorflow as tf
import os
import io
import PIL.Image
import hashlib

from object_detection.utils import dataset_util

def GetSamples(nCont, writer, image, obj_line):
    label_split_list = obj_line.split(' ')
    light_type = int(label_split_list[0])
    xmin_ = int(label_split_list[1])
    ymin_ = int(label_split_list[2])
    xmax_ = int(label_split_list[3])
    ymax_ = int(label_split_list[4])

    region = image.crop((xmin_, ymin_, xmax_, ymax_))
    file_path_temp = 'temp/' + str(nCont) + '.jpg'
    region.save(file_path_temp)

    with tf.gfile.GFile(file_path_temp, 'rb') as fid_:
        encoded_jpg_ = fid_.read()
    encoded_jpg_io_ = io.BytesIO(encoded_jpg_)
    image_ = PIL.Image.open(encoded_jpg_io_)
    if image_.format != 'JPEG':
        raise ValueError('Image format not JPEG')
    key = hashlib.sha256(encoded_jpg_).hexdigest()

    width = int(image_.size[0])
    height = int(image_.size[1])

    xmin = []
    ymin = []
    xmax = []
    ymax = []
    classes = []
    classes_text = []
    truncated = []
    poses = []
    difficult_obj = []

    difficult_obj.append(int(0))
    poses.append('Front'.encode('utf-8'))
    truncated.append(int(0))
    classes_text.append(str(light_type).encode('utf-8'))
    classes.append(light_type)
    xmin.append(float(0))
    ymin.append(float(1))
    xmax.append(float(0))
    ymax.append(float(1))

    example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': dataset_util.int64_feature(height),
        'image/width': dataset_util.int64_feature(width),
        'image/filename': dataset_util.bytes_feature(
            str(nCont).encode('utf8')),
        'image/source_id': dataset_util.bytes_feature(
            str(nCont).encode('utf8')),
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



writer1 = tf.python_io.TFRecordWriter('TrafficLightRecord.record')
data_path1 = '/home/zix/PycharmProjects/TrafficLightSelect/demo_data/trainsets'
list_file_path1 = os.path.join(data_path1, 'list')
file_object1 = open(list_file_path1)
sample_path1 = os.path.join(data_path1, 'samples/')
nCont1 = 0
for line1 in file_object1:
    nPos = line1.index(' ')
    if nPos <= 0:
        continue
    image_path1 = os.path.join(data_path1, line1[0:nPos])
    label_path1 = os.path.join(data_path1, line1[nPos+1:-1])
    with tf.gfile.GFile(image_path1, 'rb') as fid1:
        encoded_jpg1 = fid1.read()
    encoded_jpg_io1 = io.BytesIO(encoded_jpg1)
    image1 = PIL.Image.open(encoded_jpg_io1)
    if image1.format != 'JPEG':
        raise ValueError('Image format not JPEG')
    label_file_object1 = open(label_path1)
    label_text_lines1 = label_file_object1.readlines()
    for obj_line1 in label_text_lines1:
        GetSamples(nCont1, writer1, image1, obj_line1)
        print('%d %s %s' % (nCont1, image_path1, obj_line1))
        nCont1 = nCont1 + 1
writer1.close()
