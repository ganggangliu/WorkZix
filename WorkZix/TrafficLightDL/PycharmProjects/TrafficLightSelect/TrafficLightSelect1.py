import tensorflow as tf
import os
import io
import PIL.Image
import hashlib

from object_detection.utils import dataset_util

def GetSamples(image_id, img_path, label_path, sample_path):
    with tf.gfile.GFile(img_path, 'rb') as fid_:
        encoded_jpg_ = fid_.read()
    encoded_jpg_io_ = io.BytesIO(encoded_jpg_)
    image_ = PIL.Image.open(encoded_jpg_io_)
    if image_.format != 'JPEG':
        raise ValueError('Image format not JPEG')
    key_ = hashlib.sha256(encoded_jpg_).hexdigest()

    width_ = int(image_.size[0])
    height_ = int(image_.size[1])

    ratio = 0.5
    width = int(width_ * ratio)
    height = int(height_ * ratio)
    image_ = image_.resize((width, height))
    image_.save('temp.jpg')
    with tf.gfile.GFile('temp.jpg', 'rb') as fid:
        encoded_jpg = fid.read()
    encoded_jpg_io = io.BytesIO(encoded_jpg)
    image = PIL.Image.open(encoded_jpg_io)
    if image.format != 'JPEG':
        raise ValueError('Image format not JPEG')
    key = hashlib.sha256(encoded_jpg).hexdigest()

    xmin = []
    ymin = []
    xmax = []
    ymax = []
    classes = []
    classes_text = []
    truncated = []
    poses = []
    difficult_obj = []

    label_file_object = open(label_path)
    label_text_lines = label_file_object.readlines()
    object_id = 0
    for i in range(len(label_text_lines)):
        label_split_list = label_text_lines[i].split(' ')
        light_type = int(label_split_list[0])
        difficult_obj.append(int(0))
        poses.append('Front'.encode('utf-8'))
        truncated.append(int(0))
        classes_text.append(str(light_type).encode('utf-8'))
        classes.append(light_type)
        xmin.append(ratio * float(label_split_list[1]) / width)
        ymin.append(ratio * float(label_split_list[2]) / height)
        xmax.append(ratio * float(label_split_list[3]) / width)
        ymax.append(ratio * float(label_split_list[4]) / height)
        object_id = object_id + 1

    bbb = len(encoded_jpg)
    aaa = dataset_util.bytes_feature(encoded_jpg)

    example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': dataset_util.int64_feature(height),
        'image/width': dataset_util.int64_feature(width),
        'image/filename': dataset_util.bytes_feature(
            str(image_id).encode('utf8')),
        'image/source_id': dataset_util.bytes_feature(
            str(image_id).encode('utf8')),
        'image/key/sha256': dataset_util.bytes_feature(key.encode('utf8')),
        'image/encoded': dataset_util.bytes_feature(encoded_jpg),
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

    print('%s %d' % (img_path, len(xmin)))

    return example


writer = tf.python_io.TFRecordWriter('traffic_light_train.record')
data_path = '/home/zix/PycharmProjects/TrafficLightSelect/demo_data/trainsets'
list_file_path = os.path.join(data_path, 'list')
file_object = open(list_file_path)
sample_path = os.path.join(data_path, 'samples/')
image_id = 0
for line in file_object:
    nPos = line.index(' ')
    if nPos <= 0:
        continue
    image_path = os.path.join(data_path, line[0:nPos])
    label_path = os.path.join(data_path, line[nPos+1:-1])
    tf_example = GetSamples(image_id, image_path, label_path, sample_path)
    writer.write(tf_example.SerializeToString())
    image_id = image_id + 1
writer.close()
