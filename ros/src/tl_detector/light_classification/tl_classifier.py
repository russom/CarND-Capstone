from styx_msgs.msg import TrafficLight
import rospy
# Remove warnings from Cuda
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
#
import tensorflow as tf
# Remove warings from tf
tf.get_logger().setLevel('ERROR')
#
import numpy as np
from helper import load_model, process_image

class TLClassifier(object):
    def __init__(self):
        self.detection_graph = None
        self.tl_map = {
            1: TrafficLight.GREEN,
            2: TrafficLight.RED,
            3: TrafficLight.YELLOW,
            4: TrafficLight.UNKNOWN
        }
        self.detection_graph = tf.Graph()
        model_path = rospy.get_param('~pb_path')
        load_model(model_path, self.detection_graph)
        # Model warmup - run a dummy classification against a random image
        rospy.loginfo("Model warmup starting")
        with tf.Session(graph=self.detection_graph) as sess:
            image_tensor = sess.graph.get_tensor_by_name('image_tensor:0')
            detection_boxes = sess.graph.get_tensor_by_name('detection_boxes:0')
            detection_scores = sess.graph.get_tensor_by_name('detection_scores:0')
            detection_classes = sess.graph.get_tensor_by_name('detection_classes:0')
            #
            gen_image = np.uint8(np.random.randn(1, 600, 800, 3))
            sess.run([detection_boxes, detection_scores, detection_classes], feed_dict={image_tensor: gen_image})
            #
        rospy.loginfo("Model warmup done")
        
    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        image = process_image(image)
        light_class = self.predict(image)
        return light_class

    def predict(self, image, min_score_thresh=0.75):
        boxes, scores, classes = self.run_inference(image, self.detection_graph)
        for i, box in enumerate(boxes):
            if scores[i] > min_score_thresh:
                light_class = self.tl_map[classes[i]]
                return light_class

        return None

    def run_inference(self, image, graph):
        with graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')
                detection_boxes = tf.get_default_graph().get_tensor_by_name('detection_boxes:0')
                detection_scores = tf.get_default_graph().get_tensor_by_name('detection_scores:0')
                detection_classes = tf.get_default_graph().get_tensor_by_name('detection_classes:0')
                #
                (boxes, scores, classes) = sess.run([detection_boxes, detection_scores, detection_classes],
                                                    feed_dict={image_tensor: np.expand_dims(image, axis=0)})
                #
                scores = np.squeeze(scores)
                classes = np.squeeze(classes)
                boxes = np.squeeze(boxes)
        return boxes, scores, classes
