import os
import numpy as np
import tensorflow as tf
import input_train_val_split
import model

IMG_W = 64  # resize the image, if the input image is too large, training will be very slow.
IMG_H = 64
BATCH_SIZE = 64


# %%
def run_test_auto():
    # you need to change the directories to yours.
    test_data_dir = '/home/zix/PycharmProjects/A_vs_B/data_test/'
    ckpt_dir = '/home/zix/PycharmProjects/A_vs_B/log_train/'

    capacity = 10000
    num_class = 2
    test_data, test_label, _, _, capacity, num_class = input_train_val_split.get_files(test_data_dir, 0)

    with tf.Graph().as_default():

        test_batch, test_label_batch = input_train_val_split.get_batch(test_data,
                                                                         test_label,
                                                                         IMG_W,
                                                                         IMG_H,
                                                                         BATCH_SIZE,
                                                                         capacity)

        logits = model.inference(test_batch, BATCH_SIZE, num_class)
        acc = model.evaluation(logits, test_label_batch)

        x = tf.placeholder(tf.float32, shape=[BATCH_SIZE, IMG_W, IMG_H, 3])
        y_ = tf.placeholder(tf.int16, shape=[BATCH_SIZE])

        saver = tf.train.Saver()

        with tf.Session() as sess:
            print("Reading checkpoints...")
            ckpt = tf.train.get_checkpoint_state(ckpt_dir)
            if ckpt and ckpt.model_checkpoint_path:
                global_step = ckpt.model_checkpoint_path.split('/')[-1].split('-')[-1]
                saver.restore(sess, ckpt.model_checkpoint_path)
                print('Loading success, global_step is %s' % global_step)
            else:
                print('No checkpoint file found')

            coord = tf.train.Coordinator()
            threads = tf.train.start_queue_runners(sess=sess, coord=coord)

            try:
                val_acc_mean = 0.0
                for i in range(10):
                    if coord.should_stop():
                        break
                    val_images, val_labels = sess.run([test_batch, test_label_batch])
                    val_acc = sess.run([acc], feed_dict={x: val_images, y_: val_labels})
                    val_acc_mean = val_acc_mean + val_acc[0]
                    print('%d: %.2f' %(i, val_acc[0]))
                print('mean: %.2f' %(val_acc_mean/10))
            except tf.errors.OutOfRangeError:
                print('Done training -- epoch limit reached')
            finally:
                coord.request_stop()
            coord.join(threads)




