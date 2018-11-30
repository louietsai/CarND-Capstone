#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

from scipy.spatial import KDTree
import math
import numpy as np

from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None

        self.waypoints_2d = None
        self.waypoint_tree = None

        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

	sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes,self.darknet_bboxes_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.simulation = int(rospy.get_param("/simulation"))
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.darknet_bboxes = None

        # Simulation : If diagonal size of bounding box is more than 85px
        # Simulation : Bounding Box class should be 'traffic light' with probability >= 85%
        simulation_bboxes_size_threshold = 85 #px
        simulation_bboxes_probability = 0.85
        # Site : If diagonal size of bounding box is more than 80px
        # Site : Bounding Box class should be 'traffic light' with probability >= 25%
        site_bboxes_size_threshold = 40 #px
        site_bboxes_probability = 0.25

        if self.simulation == 1:
            self.prob_thresh = simulation_bboxes_probability
            self.size_thresh = simulation_bboxes_size_threshold
        else:
            self.prob_thresh = site_bboxes_probability
            self.size_thresh = site_bboxes_size_threshold

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        print "into image_cb"
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights(msg)

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def darknet_bboxes_cb(self, msg):

        print "into darknet_bboxes_cb"
        self.darknet_bboxes = []

        for bbox in msg.bounding_boxes:
            if str(bbox.Class) == 'traffic light' and bbox.probability >= self.prob_thresh:
                if math.sqrt((bbox.xmin - bbox.xmax)**2 + (bbox.ymin - bbox.ymax)**2) >= self.size_thresh:
                    self.darknet_bboxes.append(bbox)

                    # if running in site mode/ROS bag mode
                    if self.simulation == 0:
                        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                        bb_image = cv_image[bbox.ymin:bbox.ymax, bbox.xmin:bbox.xmax]
                        self.light_classifier.detect_light_state(bb_image)

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #pose.position.x
        #pose.position.y
        #TODO implement
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]
        return closest_idx

    def get_light_state(self, light,camera_image):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # For testing, just return the light state
        #return light.state

        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(camera_image, "bgr8")

        #Get classification
        print ' before classification need darknet bbox'
        return  self.light_classifier.get_classification(cv_image,self.darknet_bboxes, self.simulation)

    def process_traffic_lights(self,camera_image):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        closest_light = None
        line_wp_idx = None

        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            #TODO find the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                # Get Stop line waypoint index
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                # Find closest stop line waypoint index
                d = temp_wp_idx - car_wp_idx
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx
        if closest_light:
            state = self.get_light_state(closest_light,camera_image)
            return line_wp_idx, state

        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
