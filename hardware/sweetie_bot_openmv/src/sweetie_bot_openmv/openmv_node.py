#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""SweetieBot CV Lite JSON to ROS bridge"""

import serial
import json
import numpy as np
import rospy

from sweetie_bot_text_msgs.msg import DetectionArray as DetectionArrayMsg, Detection as DetectionMsg
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger, TriggerResponse

class Detection:
    def __init__(self, id = None, label = None, type = None, pose = None):
        #process id
        if id == None or isinstance(id, int):
            self.id = id 
        else:
            raise TypeError("Detection: id must be int.")
        # process label
        if label == None or isinstance(label, str):
            self.label = label
        else:
            raise TypeError("Detection: label must be string.")
        # process type
        if not isinstance(type, str):
            raise TypeError("Detection: type must be string.")
        self.type = type
        # process pose
        if isinstance(pose, np.ndarray) and len(pose) == 3:
            self.pose = pose
        elif isinstance(pose, Pose):
            self.pose = np.array([pose.position.x, pose.position.y, pose.position.z])
        elif isinstance(pose, Point):
            self.pose = np.array([pose.x, pose.y, pose.z])
        else:
            raise TypeError("Detection: pose must be numpy.array, geometry_msgs/Point or geometry_msgs/Pose")

    def to_msg(self):
        msg = DetectionMsg()
        msg.id = self.id
        msg.label = self.label if self.label else ''
        msg.type = self.type
        msg.pose.position = Point(x = self.pose[0], y = self.pose[1], z = self.pose[2])
        return msg

    def to_visualizaton_msg(self):
        msg = Marker()
        msg.ns = 'camera'
        msg.id = self.id
        msg.action = Marker.ADD
        if self.type == 'face':
            msg.type = Marker.SPHERE
            msg.scale = Vector3(x=0.15, y=0.15, z=0.15)
            msg.color = ColorRGBA(r=1.0, g=0.8, b=0.8, a=1.0)
        elif self.type == 'april_tag':
            msg.type=Marker.CUBE
            msg.scale =Vector3(x=0.05, y=0.05, z=0.05)
            msg.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        else:
            msg.type=Marker.CYLINDER
            msg.scale =Vector3(x=0.05, y=0.05, z=0.05)
            msg.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        msg.lifetime = rospy.Duration(1.0)
        msg.frame_locked = False
        msg.text = '(%s, %s, %s)' % (self.id, self.label, self.type)
        msg.pose.position = Point(x=self.pose[0], y=self.pose[1], z=self.pose[2])
        return msg

    def __repr__(self):
        return "(type: %s, id: %s, label: %s, pose: (%f, %f, %f)" % (self.type, self.id, self.label, self.pose[0], self.pose[1], self.pose[2])


class DetectionFilter:
    def __init__(self, length=3, threshold=2, alpha = 0.5):
        # filter configuration
        self.detection_hits = []
        self.detection_hits_len = length
        self.detection_hits_threshold = threshold
        self.alpha = alpha
        # detection
        self.id = 1024 + hash(rospy.Time.now()) % 1024 # 'unique' id
        self.label = None
        self.type = None
        self.pose = None

    def update(self, detection = None):
        if detection:
            # set type and id
            if detection.type:
                self.type = detection.type
            if detection.label:
                self.label = detection.label
            # perform filtering
            if not self.pose is None:
                self.pose = self.alpha*self.pose + (1-self.alpha)*detection.pose
            else:
                self.pose = detection.pose
            # add element to hit list
            self.detection_hits.append(True)
        else:
            # me have missed one step
            self.detection_hits.append(False)
        # clear  
        if len(self.detection_hits) > self.detection_hits_len:
            del self.detection_hits[0]

    def is_detected(self):
        # calculate hit count number
        hit_count = 0
        for hit in self.detection_hits:
            if hit:
                hit_count += 1
        # check condidtions 
        if (not self.type is None) and (not self.pose is None) and (hit_count >= self.detection_hits_threshold):
            return True

    def is_lost(self):
        if not any(self.detection_hits):
            return True

    def get_detection(self):
        # check if object is missing
        if not any(self.detection_hits):
            return None
        # generate label
        # TODO: Is it good method to generate label?
        if not self.label:
            label = 'label' + str(self.id)
        else:
            label = self.label
        # id, type, pose must always present 
        return Detection(id = self.id, label = label, type = self.type, pose = self.pose)

    def match(self, detection):
        matches = True
        # self type may be empty but detection type must always present
        if self.type and self.type != detection.type:
            matches = False
        # detection id may be missing, but filter must have it.
        if detection.label and self.label != detection.label:
            matches = False
        # calculate distance
        if not self.pose is None:
            distance = np.linalg.norm(detection.pose - self.pose)
        else:
            distance = 0.0
        # return match result
        return (matches, distance)

        
class OpenMVBridge:

    def __init__(self):
        # Initialize the node, name it, set log verbosity level
        rospy.init_node('openmv_node')
        # COMPONENT INTERFACE
        # publish topics
        self.detections_pub = rospy.Publisher('detections', DetectionArrayMsg, queue_size=5)
        self.markers_pub = rospy.Publisher('markers', MarkerArray, queue_size=5)

        # services 
        self.reconfigure_srv = rospy.Service('~reconfigure', Trigger, self.reconfigureCallback)

        # COMPONENT STATE
        self.ok = False
        self.openmv_port = None
        self.message_buffer = bytes()
        self.MESSAGE_BUFFER_LIMIT = 2048
        self.detection_filters = []
        self.detection_filter_params_cache = {}

        # Configure component
        self.configure()

    def is_ok(self):
        return self.ok

    def reconfigureCallback(self, req):
        # TODO syncronization. It is dangerous not to use syncronization.
        self.configure()
        return TriggerResponse(success = self.ok)

    def configure(self):
        # reset state
        self.ok = False
        self.openmv_port = None
        self.message_buffer = bytes()
        self.detection_filters = []
        self.detection_filter_params_cache = {}

        # connect to camera
        try:
            # PARAMETERS

            # camera frame
            self.camera_frame = rospy.get_param('~camera_frame', 'camera')
            if not isinstance(self.camera_frame, str):
                raise EnvironmentError('"camera_frame" parameter must be supplied and be a correct frame name.')
            # camera matrix
            self.camera_matrix = rospy.get_param('~camera_matrix', [539.789244, 0.000000, 320.856994, 0.000000, 540.065811, 210.732496, 0.000000, 0.000000, 1.000000])
            if not isinstance(self.camera_matrix, list) or len(self.camera_matrix) != 9 or not all(isinstance(x, (float,int)) for x in self.camera_matrix):
                raise EnvironmentError('"camera_matrix" must be array float with size 12 wich represent 3x3 matrix (row major).')
            self.camera_matrix = np.matrix(self.camera_matrix).reshape([3, 3])
            # distance
            self.distance = rospy.get_param('~distance', 1.0)
            if not isinstance(self.distance, (int,float)) or self.distance <= 0.0:
                raise EnvironmentError('"distance" must be positive float.')

            # serial port configuration
            self.openmv_port = self.create_serial_connection()
        except EnvironmentError as e:
            rospy.logerr('OpenMV node configuration failed: %s' % e)
            return False
        except serial.serialutil.SerialException as e:
            rospy.logerr('OpenMV node configuration failed: %s' % e)
            return False
           
        # node is fully configured
        self.ok = True
        rospy.loginfo('OpneMV node is configured.')

    def back_projection(self, cx, cy, z):
        # convert to image center relative position
        cx -= self.camera_matrix[0,2]
        cy -= self.camera_matrix[1,2]
        # calculate object position
        x = - z / self.camera_matrix[0,0] * cx
        y = - z / self.camera_matrix[1,1] * cy
        return np.array([x, y, z])

    def step(self):
        # check if node configured
        if not self.ok:
            return

        # read next portion of data
        try:
            # Get a byte from the serial port
            # HINT: It's a blocking routine
            self.message_buffer += self.openmv_port.read_until('\n', self.MESSAGE_BUFFER_LIMIT)
        except serial.serialutil.SerialException as e:
            # Check if it is an interrupted system call
            # TODO: That's an ugly solution... Do we have other options with PySerial?
            if str(e).find("Interrupted system call") == -1:
                rospy.logerr('Failed to read from the serial device: %s', e)
                self.ok = False
            else:
                rospy.logdebug('EINTR received.')
            return

        # check overflow
        if len(self.message_buffer) > self.MESSAGE_BUFFER_LIMIT:
            rospy.logerr('Message buffer limit is exceeded.')
            self.message_buffer = bytes()
            return

        # check if EOL received
        if not len(self.message_buffer) > 0 or self.message_buffer[-1] != '\n':
           return 

        rospy.logdebug("Message buffer %s" % self.message_buffer.strip())

        # message buffer may contain full json message so parse it
        detections = self.parse_openmv_frame(self.message_buffer)
        self.message_buffer = bytes()

        rospy.logdebug("Detections: %s" % detections)

        # apply detection filtes
        detections = self.apply_detections_filters(detections)

        rospy.logdebug("Filtered detections: %s" % detections)

        stamp = rospy.Time.now()
        # create DetectionArray message
        detections_msg = DetectionArrayMsg()
        for detection in detections:
            detection_msg = detection.to_msg()
            detection_msg.header.frame_id = self.camera_frame
            detection_msg.header.stamp = stamp
            detections_msg.detections.append(detection_msg)

        # create MarkerArray
        markers_msg = MarkerArray()
        for detection in detections:
            marker_msg = detection.to_visualizaton_msg()
            marker_msg.header.stamp = stamp
            marker_msg.header.frame_id = self.camera_frame
            markers_msg.markers.append(marker_msg)

        # publish it
        self.detections_pub.publish(detections_msg)
        if len(detections) > 0:
            self.markers_pub.publish(markers_msg)

    def parse_openmv_frame(self, message_buffer):
        # parse OpenMV serial frame
        detections = []
        try:
            openmv_frame = json.loads(message_buffer)  # Decode the JSON message

            for obj in openmv_frame:
                # create detection object
                if obj['type'] == 'face':
                    cx = obj['x'] + obj['w']/2
                    cy = obj['y'] + obj['h']/2
                    pose = self.back_projection(cx, cy, self.distance)
                    detection = Detection(type = 'face', pose = pose)

                elif obj['type'] == 'april':
                    cx = obj['x'] + obj['w']/2
                    cy = obj['y'] + obj['h']/2
                    pose = self.back_projection(cx, cy, self.distance)
                    detection = Detection(type = 'april_tag', label = str(obj['id']), pose = pose)

                else:
                    detection = None

                # add it to list
                if detection:
                    detections.append(detection)

        except (ValueError) as e:  # Incorrect JSON message
            rospy.logerr('Failed to parse JSON message "%s": %s', message_buffer, e)

        return detections

    def apply_detections_filters(self, detections):
        # match detections agnist filters
        matches = []
        for i in range(len(detections)):
            for j in range(len(self.detection_filters)):
                # compare detection with filter
                (matched, distance) = self.detection_filters[j].match(detections[i])
                if matched:
                    matches.append([i,j,distance])

        # find the best fits for each filter
        detection_filters_updated = [ False ] * len(self.detection_filters)
        detections_processed = [ False ] * len(detections)
        while (len(matches) > 0):
            # find best fit
            best_match_index = 0
            best_distance = matches[0][2]
            for i in range(len(matches)):
                if matches[i][2] < best_distance:
                    best_match_index = i
                    best_distance = matches[i][2]
            # extract fit information and remove it
            (detection_index, filter_index, distance) = matches[best_match_index]
            del matches[best_match_index]
            # apply filter
            self.detection_filters[filter_index].update( detections[detection_index] )
            detection_filters_updated[filter_index] = True
            detections_processed[detection_index] = True

        # update not matched filters
        for i in range(len(self.detection_filters)):
            if not detection_filters_updated[i]:
                self.detection_filters[i].update(None)
        # remove lost ones
        self.detection_filters = [ f for f in self.detection_filters if not f.is_lost() ]

        # create new filters for not processed detections
        for i in range(len(detections)):
            if not detections_processed[i]:
                # create filter with default settings
                params = self.get_detection_filter_params(detections[i].type)
                f = DetectionFilter(**params)
                f.update(detections[i])
                self.detection_filters.append(f)

        # return filters output
        detections = [ f.get_detection() for f in self.detection_filters if f.is_detected() ]
        return detections

    def get_detection_filter_params(self, ftype):
        params = self.detection_filter_params_cache.get(ftype)
        if not params:
            length = rospy.get_param('~filter/%s/length' % ftype, 3)
            if not isinstance(length, int) or length < 1:
                raise EnvironmentError('filter "length" must be positive int.')
            threshold = rospy.get_param('~filter/%s/threshold' % ftype, 2)
            if not isinstance(threshold, int) or threshold < 1:
                raise EnvironmentError('"filter/threshold" must be positive int.')
            alpha = rospy.get_param('~filter/%s/alpha' % ftype, 0.3)
            if not isinstance(alpha, (int, float)) or alpha < 0.0 or alpha > 1.0:
                raise EnvironmentError('"filter/alpha" must be float in [0,1].')
            # add to cache
            params = { 'length': length, 'threshold': threshold, 'alpha': alpha }
            self.detection_filter_params_cache[ftype] = params
        return params

    def create_serial_connection(self):
        # port name
        port_name = rospy.get_param('~serial/port', None)  # Retrieve a serial port path
        if not isinstance(port_name, str):
            raise EnvironmentError('Serial port parameter value must present and be a string!')

        # baudrate
        baudrate = rospy.get_param('~serial/baudrate', 115200)
        if not isinstance(baudrate, int):
            raise EnvironmentError('Serial port baud rate value has to be an integer!')

        # byte size
        bytesize = rospy.get_param('~serial/bytesize', 8)
        if not isinstance(bytesize, int):
            raise EnvironmentError('Serial port byte size value has to be an integer!')

        # parity
        parity = rospy.get_param('~serial/parity', 'N')
        if not isinstance(parity, str):
            raise EnvironmentError('Serial port parity value has to be a string!')

        # stopbits
        stopbits = rospy.get_param('~serial/stopbits', 1)
        if not isinstance(stopbits, (int, float)):
            raise EnvironmentError('Serial port stop bits number has to be an integer or a float!')

        # software flow control
        xonxoff = rospy.get_param('~serial/xonxoff', False)
        if not isinstance(xonxoff, bool):
            raise EnvironmentError('Serial port XON/XOFF value has to be a bool!')

        #hardware flow control
        rtscts = rospy.get_param('~serial/rtscts', False)
        if not isinstance(rtscts, bool):
            raise EnvironmentError('Serial port RTS/CTS value has to be a bool!')

        # read timeout
        read_timeout = rospy.get_param('~serial/read_timeout', 1.0)
        if not isinstance(read_timeout, float):
            raise EnvironmentError('Read timeout value has to be a bool!')

        rospy.logdebug('Serial port: dev %s baud %s parity %s datab %s stopb %s xonxoff %s rstcst %s timeout %s', (port_name, baudrate, bytesize, parity, stopbits, xonxoff, rtscts, read_timeout))

        # create serial connection to camera
        openmv_port = serial.Serial(
            port=port_name,
            baudrate=baudrate,
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits,
            xonxoff=xonxoff,
            rtscts=rtscts,
            # Blocking mode
            # WARNING: We have to keep a timeout for the correct node shutdown!
            timeout=read_timeout
        )
        rospy.logdebug('OpenMV serial port: %s', openmv_port)

        return openmv_port

# Main function
def main():
    node = OpenMVBridge()

    while (True):
        # check if node should be stopped
        if rospy.is_shutdown():
            break
        # check if node properly configured
        if not node.is_ok():
            rospy.sleep(rospy.Duration(1.0))
            continue

        node.step()

    rospy.logerr("OpenMV: shutdown node.")
