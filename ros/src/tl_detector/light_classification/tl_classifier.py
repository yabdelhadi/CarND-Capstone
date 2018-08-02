from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import rospy

MODEL_PATH = 'models/sim_graph.pb'
MIN_CONFIDENCE = 0.80

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.graph = tf.Graph()
        with self.graph.as_default():
            graph_def = tf.GraphDef()
            with tf.gfile.GFile(MODEL_PATH, 'rb') as f:
                s_graph = f.read()
                graph_def.ParseFromString(s_graph)
		tf.import_graph_def(graph_def, name='')
        
        self.image = self.graph.get_tensor_by_name('image_tensor:0')
        self.detections = self.graph.get_tensor_by_name('detection_scores:0')
        self.d_classes = self.graph.get_tensor_by_name('detection_classes:0')

	    self.sess = tf.Session(graph=self.graph)

    #def get_tensor(self, name):
	#return self.graph.get_tensor_by_name(name)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        etection_count = np.zeros(3, dtype=int)

        with self.graph.as_default():
            expanded_img = np.expand_dims(image, axis=0)
            feed_dict = {self.image: expanded_img}
            (scores, classes) = self.sess.run([self.detections,
                                               self.d_classes],
                                              feed_dict=feed_dict)
            for i, score in enumerate(scores[0]):
                if score > MIN_CONFIDENCE:
                    detection_count[int(classes[0][i]) - 1] += 1

            if np.sum(detection_count) == 0:
                # No confident votes for any class
                return TrafficLight.UNKNOWN
            else:
                # Pick best class
                detected_class = np.argmax(detection_count) + 1
                if detected_class == 1:
                    return TrafficLight.GREEN
                elif detected_class == 2:
                    return TrafficLight.YELLOW
                elif detected_class == 3:
                    return TrafficLight.RED

            rospy.loginfo('Traffic Light: {}'.format(self.traffic_light_to_str(output)))
            return TrafficLight.UNKNOWN
    
    def graph_class_to_traffic_light(graph_class):
        """ Converts from a class number as defined in the TensorFlow
            model, to a class number as defined in styx_msgs/TrafficLight
        """
        if graph_class == 1:
            return TrafficLight.GREEN
        elif graph_class == 2:
            return TrafficLight.YELLOW
        elif graph_class == 3:
            return TrafficLight.RED

        return TrafficLight.UNKNOWN
