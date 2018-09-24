import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
import time

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image

plt.ioff()

# This is needed to display the images.
#%matplotlib inline

# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")

from utils import label_map_util

from utils import visualization_utils as vis_util

# What model to download.
MODEL_NAME = 'ssd_mobilenet_v1_coco_11_06_2017'
MODEL_FILE = MODEL_NAME + '.tar.gz'
DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'

# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = '/home/zix/Downloads/ssd_mobilenet_v1_coco_11_06_2017/frozen_inference_graph.pb'
#PATH_TO_CKPT = '/home/zix/zix/tensorflow-master/tensorflow/models/object_detection/xinzi.pb'

# List of the strings that is used to add correct label for each box.
#PATH_TO_LABELS = os.path.join('data', 'mscoco_label_map.pbtxt')
PATH_TO_LABELS = '/home/zix/zix/tensorflow-master/tensorflow/models/object_detection/data/mscoco_label_map.pbtxt'
#PATH_TO_LABELS = '/home/zix/zix/tensorflow-master/tensorflow/models/object_detection/data/pascal_label_map.pbtxt'

NUM_CLASSES = 90
#NUM_CLASSES = 20

#opener = urllib.request.URLopener()
#opener.retrieve(DOWNLOAD_BASE + MODEL_FILE, MODEL_FILE)
#tar_file = tarfile.open(MODEL_FILE)
#for file in tar_file.getmembers():
#    file_name = os.path.basename(file.name)
#    if 'frozen_inference_graph.pb' in file_name:
#        tar_file.extract(file, os.getcwd())


detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)


def load_image_into_numpy_array(image):
  (im_width, im_height) = image.size
  return np.array(image.getdata()).reshape(
      (im_height, im_width, 3)).astype(np.uint8)


# For the sake of simplicity we will use only 2 images:
# image1.jpg
# image2.jpg
# If you want to test the code with your images, just add path to the images to the TEST_IMAGE_PATHS.
PATH_TO_TEST_IMAGES_DIR = 'test_images'
TEST_IMAGE_PATHS = []
TEST_IMGAES_ROOT_PATH = '/home/zix/Downloads/test_data'

for parent,dirnames,filenames in os.walk(TEST_IMGAES_ROOT_PATH):
    for filename in filenames:
        TEST_IMAGE_PATHS.append(os.path.join(parent, filename))

# Size, in inches, of the output images.
IMAGE_SIZE = (12, 8)


with detection_graph.as_default():
  with tf.Session(graph=detection_graph) as sess:
    for image_path in TEST_IMAGE_PATHS:
      print (image_path)
      image_ = Image.open(image_path)
      # the array based representation of the image will be used later in order to prepare the
      # result image with boxes and labels on it.
      image_np_ = load_image_into_numpy_array(image_)
      # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
      image_np_expanded_ = np.expand_dims(image_np_, axis=0)
      image_tensor_ = detection_graph.get_tensor_by_name('image_tensor:0')
      # Each box represents a part of the image where a particular object was detected.
      boxes_ = detection_graph.get_tensor_by_name('detection_boxes:0')
      # Each score represent how level of confidence for each of the objects.
      # Score is shown on the result image, together with the class label.
      scores_ = detection_graph.get_tensor_by_name('detection_scores:0')
      classes_ = detection_graph.get_tensor_by_name('detection_classes:0')
      num_detections_ = detection_graph.get_tensor_by_name('num_detections:0')
      # Actual detection.
      time1 = time.time()
      print('Start!')
      (boxes, scores, classes, num_detections) = sess.run(
          [boxes_, scores_, classes_, num_detections_],
          feed_dict={image_tensor_: image_np_expanded_})
      time2 = time.time()
      print time2 - time1

      image = Image.open(image_path)
      # the array based representation of the image will be used later in order to prepare the
      # result image with boxes and labels on it.
      image_np = load_image_into_numpy_array(image)
      # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
      image_np_expanded = np.expand_dims(image_np, axis=0)
      image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
      # Each box represents a part of the image where a particular object was detected.
      boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
      # Each score represent how level of confidence for each of the objects.
      # Score is shown on the result image, together with the class label.
      scores = detection_graph.get_tensor_by_name('detection_scores:0')
      classes = detection_graph.get_tensor_by_name('detection_classes:0')
      num_detections = detection_graph.get_tensor_by_name('num_detections:0')
      # Actual detection.
      time1 = time.time()
      print('Start!')
      (boxes, scores, classes, num_detections) = sess.run(
          [boxes, scores, classes, num_detections],
          feed_dict={image_tensor: image_np_expanded})
      time2 = time.time()
      print time2 - time1
      for i in range(0, 2):
          for j in categories:
              if j.get('id') == np.take(classes, i):
                  print j.get('name')
#          print(np.take(categories, np.take(classes, i)-1))
          print(np.take(scores, i))


      # Visualization of the results of a detection.
      vis_util.visualize_boxes_and_labels_on_image_array(
          image_np,
          np.squeeze(boxes),
          np.squeeze(classes).astype(np.int32),
          np.squeeze(scores),
          category_index,
          use_normalized_coordinates=True,
          line_thickness=2,
          max_boxes_to_draw=10,
          min_score_thresh=0.2)
      plt.figure(figsize=IMAGE_SIZE)
      plt.imshow(image_np)
      plt.show()