import tensorflow as tf
import io
from PIL import Image

record_path = '/home/zix/PycharmProjects/TrafficLightSelect/traffic_light_train.record'
filename_queue = tf.train.string_input_producer([record_path])
reader = tf.TFRecordReader()
_, serialized_example = reader.read(filename_queue)
features = tf.parse_single_example(serialized_example,
                                   features={
                                        'image/height': tf.FixedLenFeature([], tf.int64),
                                        'image/width': tf.FixedLenFeature([], tf.int64),
                                        'image/filename': tf.FixedLenFeature([], tf.string),
                                        'image/source_id': tf.FixedLenFeature([], tf.string),
                                        'image/key/sha256': tf.FixedLenFeature([], tf.string),
                                        'image/encoded': tf.FixedLenFeature([], tf.string),
                                        'image/format': tf.FixedLenFeature([], tf.string),
                                        'image/object/bbox/xmin': tf.VarLenFeature(tf.float32),
                                        'image/object/bbox/xmax': tf.VarLenFeature(tf.float32),
                                        'image/object/bbox/ymin': tf.VarLenFeature(tf.float32),
                                        'image/object/bbox/ymax': tf.VarLenFeature(tf.float32),
                                        'image/object/class/text': tf.VarLenFeature(tf.string),
                                        'image/object/class/label': tf.VarLenFeature(tf.int64),
                                        'image/object/difficult': tf.VarLenFeature(tf.int64),
                                        'image/object/truncated': tf.VarLenFeature(tf.int64),
                                        'image/object/view': tf.VarLenFeature(tf.string)
                                   })
height = tf.cast(features['image/height'], tf.int64)
width = tf.cast(features['image/width'], tf.int64)
filename = tf.cast(features['image/filename'], tf.string)
source_id = tf.cast(features['image/source_id'], tf.string)
sha256 = tf.cast(features['image/key/sha256'], tf.string)
encoded = tf.cast(features['image/encoded'], tf.string)
format = tf.cast(features['image/format'], tf.string)
xmin = tf.cast(features['image/object/bbox/xmin'], tf.float32)
xmax = tf.cast(features['image/object/bbox/xmax'], tf.float32)
ymin = tf.cast(features['image/object/bbox/ymin'], tf.float32)
ymax = tf.cast(features['image/object/bbox/ymax'], tf.float32)
text = tf.cast(features['image/object/class/text'], tf.string)
label = tf.cast(features['image/object/class/label'], tf.int64)
difficult = tf.cast(features['image/object/difficult'], tf.int64)
truncated = tf.cast(features['image/object/truncated'], tf.int64)
view = tf.cast(features['image/object/view'], tf.string)

with tf.Session() as sess:
    init_op = tf.initialize_all_variables()
    sess.run(init_op)
    coord = tf.train.Coordinator()
    threads = tf.train.start_queue_runners(coord=coord)
    for i in range(1000):
        filename_, example, l, xmin_, xmax_, ymin_, ymax_, width_, height_ = sess.run([filename, encoded, label, xmin, xmax, ymin, ymax, width, height])
        encoded_jpg_io = io.BytesIO(example)
        image_ = Image.open(encoded_jpg_io)
        for index, j in enumerate(l.values):
            xmin1 = int(xmin_.values[index] * width_)
            ymin1 = int(ymin_.values[index] * height_)
            xmax1 = int(xmax_.values[index] * width_)
            ymax1 = int(ymax_.values[index] * height_)
            region = image_.crop((xmin1, ymin1, xmax1, ymax1))
            image_save_name = 'check_temp/' + str(filename_) + '_' + str(index) + '_' + str(j) + '.jpg'
            print(image_save_name)
            region.save(image_save_name)
    coord.request_stop()
    coord.join(threads)
