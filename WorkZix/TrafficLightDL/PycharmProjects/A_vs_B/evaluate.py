import numpy as np
import tensorflow as tf
import model
from PIL import Image
import matplotlib.pyplot as plt
import input_train_val_split
import time

def get_one_image(train):
    '''Randomly pick one image from training data
    Return: ndarray
    '''
    n = len(train)
    ind = np.random.randint(0, n)
    img_dir = train[ind]

    image = Image.open(img_dir)
    image = image.convert("RGB")
#    plt.imshow(image)
#    plt.show()
    image = image.resize([64, 64])
    image = np.array(image)
    return image

def evaluate_one_image():
    '''Test one image against the saved models and parameters
    '''

    # you need to change the directories to yours.
    train_dir = '/home/zix/PycharmProjects/A_vs_B/data_test/'
    logs_train_dir = '/home/zix/PycharmProjects/A_vs_B/log_train/'

    train, train_label, val, val_label, capacity, nem_class = input_train_val_split.get_files(train_dir, 0.5)
    image_array = get_one_image(train)
#    plt.imshow(image_array)
#    plt.show()

    with tf.Graph().as_default():
        BATCH_SIZE = 1

        image = tf.cast(image_array, tf.float32)
        image = tf.image.per_image_standardization(image)
        image = tf.reshape(image, [1, 64, 64, 3])

        logit = model.inference(image, BATCH_SIZE, nem_class)

        logit = tf.nn.softmax(logit)

        x = tf.placeholder(tf.float32, shape=[64, 64, 3])

        # you need to change the directories to yours.


        saver = tf.train.Saver()

        with tf.Session() as sess:

            print("Reading checkpoints...")
            ckpt = tf.train.get_checkpoint_state(logs_train_dir)
            if ckpt and ckpt.model_checkpoint_path:
                global_step = ckpt.model_checkpoint_path.split('/')[-1].split('-')[-1]
                saver.restore(sess, ckpt.model_checkpoint_path)
                print('Loading success, global_step is %s' % global_step)
            else:
                print('No checkpoint file found')

            time1 = time.time()
            prediction, img = sess.run([logit, image], feed_dict={x: image_array})
            time2 = time.time()
            print time2 - time1
            _,len =  np.shape(prediction)
            for i in range(len):
                print("%d: %f" %(i, prediction[:,i]))
            plt.imshow(image_array)
            plt.show()



