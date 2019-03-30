#!/usr/bin/env python
## Author: Rohit
## Date: July, 25, 2017
# Purpose: Ros node to detect objects using tensorflow

import os
import sys
import cv2
import numpy as np
import six.moves.urllib as urllib
import tarfile
try:
    import tensorflow as tf
except ImportError:
    print("unable to import TensorFlow. Is it installed?")
    print("  sudo apt install python-pip")
    print("  sudo pip install tensorflow")
    sys.exit(1)

# ROS related imports
import rospy
from std_msgs.msg import String , Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# Object detection module imports
import object_detection
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

# SET FRACTION OF GPU YOU WANT TO USE HERE
GPU_FRACTION = 0.4

######### Set model here ############
# MODEL_NAME =  'ssd_mobilenet_v1_coco_11_06_2017'
# DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'
# MODEL_FILE = MODEL_NAME + '.tar.gz'
# # By default models are stored in data/models/
# MODEL_PATH = os.path.join(os.path.dirname(sys.path[0]),'data','models' , MODEL_NAME)
# # Path to frozen detection graph. This is the actual model that is used for the object detection.
# PATH_TO_CKPT = MODEL_PATH + '/frozen_inference_graph.pb'
# ######### Set the label map file here ###########
# LABEL_NAME = 'mscoco_label_map.pbtxt'
# # By default label maps are stored in data/labels/
# PATH_TO_LABELS = os.path.join(os.path.dirname(sys.path[0]),'data','labels', LABEL_NAME)
# ######### Set the number of classes here #########
# NUM_CLASSES = 90
# opener = urllib.request.URLopener()
# opener.retrieve(DOWNLOAD_BASE + MODEL_FILE, MODEL_FILE)
# tar_file = tarfile.open(MODEL_FILE)
# for file in tar_file.getmembers():
#   file_name = os.path.basename(file.name)
#   if 'frozen_inference_graph.pb' in file_name:
#     tar_file.extract(file, os.path.join(os.path.dirname(sys.path[0]),'data','models'))


MODEL_PATH = os.path.join('/home','Documents','model_1')
# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_PATH + '/frozen_inference_graph.pb'
######### Set the label map file here ###########
LABEL_NAME = 'label_map.pbtxt'
# By default label maps are stored in data/labels/
PATH_TO_LABELS = os.path.join('/home','da-cam','Object-Detection-Training','data', LABEL_NAME)
######### Set the number of classes here #########
NUM_CLASSES = 1


detection_graph = tf.Graph()
with detection_graph.as_default():
  od_graph_def = tf.GraphDef()
  with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')


## Loading label map
# Label maps map indices to category names, so that when our convolution network predicts `5`,
# we know that this corresponds to `airplane`.  Here we use internal utility functions,
# but anything that returns a dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)
# print(category_index)
# Setting the GPU options to use fraction of gpu that has been set
config = tf.ConfigProto()
config.gpu_options.per_process_gpu_memory_fraction = GPU_FRACTION

sess = tf.Session(graph=detection_graph,config=config)

with detection_graph.as_default():

    class Detector:
        """
        Object detector class
        """
        def __init__(self):
            self.bound_buffer = 5
            self.bridge = CvBridge()
            self.gate_id = 76
            self.buoy_id = 34
            gate_topic = "/object_detect/gate_output"
            buoy_topic = "/object_detect/buoy_output"
            sub_topic = "/vision/output"
            self.gate_pub = rospy.Publisher(gate_topic, Image,
            queue_size=1)
            self.buoy_pub = rospy.Publisher(buoy_topic, Image,
            queue_size=1)
            self.image_sub = rospy.Subscriber(sub_topic, Image, self.image_cb,
            queue_size=1, buff_size=2**24)

        def image_cb(self, data):
            """
            Callback function

            :param data: sensor_msgs/Image message
            """
            detect_array = Detection2DArray()
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)
            image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)

            # The array based representation of the image will be used later in order
            # to prepare the result image with boxes and labels on it.
            np_image = np.asarray(image)

            # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
            np_image_expanded = np.expand_dims(np_image, axis=0)
            image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

            # Each box represents a part of the image where a particular object was detected.
            boxes   = detection_graph.get_tensor_by_name('detection_boxes:0')

            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            scores  = detection_graph.get_tensor_by_name('detection_scores:0')
            classes = detection_graph.get_tensor_by_name('detection_classes:0')
            num_detections = detection_graph.get_tensor_by_name('num_detections:0')

            (boxes, scores, classes, num_detections) = sess.run(
                [boxes, scores, classes, num_detections],
                feed_dict={image_tensor: np_image_expanded})

            objects=vis_util.visualize_boxes_and_labels_on_image_array(
                image,
                np.squeeze(boxes),
                np.squeeze(classes).astype(np.int32),
                np.squeeze(scores),
                category_index,
                use_normalized_coordinates=True,
                line_thickness=2)

            detect_array.detections = []
            detect_array.header     = data.header

            for obj in objects:
                cv2.imwrite("object_img.png", image)
                detect_array.detections.append(self.create_detect_msg(
                                                obj,data.header, cv_image))

            # Sort the detect array by object confidence score, so that the
            # first detection of an object in the array is the one most likely
            # to be said object
            sorted_detect = sorted (detect_array.detections,
                key = lambda detect: detect.results[0].score,
                reverse=True)

            # Send the cropped image of the most confident detect of each object to
            # the appropriate publisher, if such an object is detected
            found_gate = False
            found_buoy = False
            for detection in sorted_detect:
                if (detection.results[0].id == self.gate_id) and not found_gate:
                    print("found a gate")
                    self.gate_pub.publish(detection.source_img)
                    found_gate = True
                elif (detection.results[0].id == self.buoy_id) and not found_buoy:
                    print("found a buoy")
                    self.buoy_pub.publish(detection.source_img)
                    found_buoy = True

        def create_detect_msg(self, object_data, header, cv_image):
            """
            Creates a Detection2D message from the given  detected
            object and image data

            :param object_data: detected object's data
            :param header: image message header
            :param cv_image: open cv image
            """
            image_height,image_width,channels = cv_image.shape

            detection     = Detection2D()
            hypothesis    = ObjectHypothesisWithPose()
            cropped_image = Image()

            object_id     = object_data[0]
            object_score  = object_data[1]
            dimensions    = object_data[2]
            print("found object: " + str(object_id))

            crop_y  = int(dimensions[0] * image_height)
            crop_dy = int(dimensions[2] * image_height)
            crop_x  = int(dimensions[1] * image_width)
            crop_dx = int(dimensions[3] * image_width)

            cropped = cv_image[crop_y-self.bound_buffer : crop_dy+self.bound_buffer,
                               crop_x-self.bound_buffer : crop_dx+self.bound_buffer]
            try:
                cropped_image = self.bridge.cv2_to_imgmsg(cropped, "bgr8")
            except CvBridgeError as e:
                print(e)

            hypothesis.id     = object_id
            hypothesis.score  = object_score
            print("score: " + str(object_score))

            detection.bbox.size_x   = crop_dx - crop_x
            detection.bbox.size_y   = crop_dy - crop_y
            detection.bbox.center.x = (crop_dx - crop_x)/2
            detection.bbox.center.y = (crop_dy - crop_y)/2
            detection.source_img    = cropped_image
            detection.header        = header
            detection.results.append(hypothesis)

            cv2.imwrite("cropped.png", cropped)
            cv2.imwrite("uncropped.png", cv_image)

            return detection




def main(args):
    rospy.init_node("object_detect_node")
    detector = Detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    cv2.destroyAllWindows()

if __name__=='__main__':
  main(sys.argv)
