
import os
import math
import random
import pandas as pd
import numpy as np
import tensorflow as tf
import cv2
import rospkg 

slim = tf.contrib.slim

rospack = rospkg.RosPack()
# get the file path for rospy_tutorials
pa = rospack.get_path('object_detector_ssd_tf_ros')

ckpt_filename = pa+'/ssd/model/ssd_300_vgg.ckpt'

from ssd import ssd_vgg_300, ssd_common, np_methods, ssd_vgg_preprocessing

 
class ssdWrapper():
    def __init__(self , config, net_shape = (300, 300), data_format = 'NHWC', ckpt_filename = ckpt_filename):
        self.isess = tf.InteractiveSession(config=config)
        # Input placeholder.
        self.net_shape = net_shape
        self.img_input = tf.placeholder(tf.uint8, shape=(None, None, 3))
        # Evaluation pre-processing: resize to SSD net shape.
        image_pre, _, _, self.bbox_img = ssd_vgg_preprocessing.preprocess_for_eval(
                    self.img_input, None, None, net_shape, data_format, resize=ssd_vgg_preprocessing.Resize.WARP_RESIZE)
        self.image_4d = tf.expand_dims(image_pre, 0)

        # Define the SSD model.
        
        reuse = True if 'ssd_net' in locals() else None
        ssd_net = ssd_vgg_300.SSDNet()
        with slim.arg_scope(ssd_net.arg_scope(data_format=data_format)):
            self.predictions, self.localisations, _, _ = ssd_net.net(self.image_4d, is_training=False, reuse=reuse)

        # Restore SSD model.
        self.isess.run(tf.global_variables_initializer())
        saver = tf.train.Saver()
        print(ckpt_filename)
        saver.restore(self.isess, ckpt_filename)

        # SSD default anchor boxes.
        self.ssd_anchors = ssd_net.anchors(net_shape)




    # Main image processing routine.
    def process_image(self, img, select_threshold=0.5, nms_threshold=.45):
        # Run SSD network.
        _, rpredictions, rlocalisations, rbbox_img = self.isess.run([self.image_4d, self.predictions, self.localisations, self.bbox_img], feed_dict={self.img_input: img})
        
        # Get classes and bboxes from the net outputs.
        rclasses, rscores, rbboxes , lprobs = np_methods.ssd_bboxes_select(
                rpredictions, rlocalisations, self.ssd_anchors,
                select_threshold=select_threshold, img_shape=self.net_shape, num_classes=21, decode=True)
        #print(lprobs.shape)
        rbboxes = np_methods.bboxes_clip(rbbox_img, rbboxes)
        rclasses, rscores, rbboxes, rprobs = np_methods.bboxes_sort(rclasses, rscores, rbboxes,lprobs, top_k=400)
        rclasses, rscores, rbboxes, rprobs = np_methods.bboxes_nms(rclasses, rscores, rbboxes,rprobs, nms_threshold=nms_threshold)
        #print(rprobs)
        # Resize bboxes to original image shape. Note: useless for Resize.WARP!
        rbboxes = np_methods.bboxes_resize(rbbox_img, rbboxes)
        return rclasses, rscores, rbboxes, rprobs








