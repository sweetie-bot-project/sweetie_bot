#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""SweetieBot CV Lite JSON to ROS bridge"""

import serial
import json

import rospy

from sweetiebot_cv_lite.msg import Rect, AprilTag, Faces, AprilTags

JSON_MESSAGE_BUFFER_LIMIT = 1024  # JSON message buffer limit

# Main function
if __name__ == '__main__':
    # Initialize the node, name it, set log verbosity level
    rospy.init_node('sweetiebot_cv_lite_bridge', log_level=rospy.DEBUG)
    rospy.logdebug('Node initialized')

    node_prefix = rospy.get_param("~prefix", default='')  # Extract a topic prefix

    if not isinstance(node_prefix, str):
        raise EnvironmentError('Node topic prefix parameter value has to be a string!')

    rospy.logdebug('Node prefix: %s', node_prefix)

    # Create a topic publisher for the detected faces
    faces_publisher = rospy.Publisher('{0}/faces'.format(node_prefix), Faces, queue_size=10)
    rospy.logdebug('Publisher for a faces topic ready: %s', faces_publisher)

    # Create a topic publisher for the detected AprilTags
    april_tags_publisher = rospy.Publisher('{0}/april_tags'.format(node_prefix), AprilTags, queue_size=10)
    rospy.logdebug('Publisher for an AprilTags topic ready: %s', april_tags_publisher)

    try:
        serial_port = rospy.get_param('~serial_port')  # Retrieve a serial port path
    except KeyError:
        raise EnvironmentError('No serial port presented!')

    if not isinstance(serial_port, str):
        raise EnvironmentError('Serial port parameter value has to be a string!')

    rospy.logdebug('Serial port: "%s"', serial_port)

    serial_baudrate = rospy.get_param('~serial_baudrate', 115200)

    if not isinstance(serial_baudrate, int):
        raise EnvironmentError('Serial port baud rate value has to be an integer!')

    rospy.logdebug('Serial port baud rate: %s', serial_baudrate)

    serial_bytesize = rospy.get_param('~serial_bytesize', 8)

    if not isinstance(serial_bytesize, int):
        raise EnvironmentError('Serial port byte size value has to be an integer!')

    rospy.logdebug('Serial port byte size: %s', serial_bytesize)

    serial_parity = rospy.get_param('~serial_parity', 'N')

    if not isinstance(serial_parity, str):
        raise EnvironmentError('Serial port parity value has to be a string!')

    rospy.logdebug('Serial port parity: "%s"', serial_parity)

    serial_stopbits = rospy.get_param('~serial_stopbits', 1)

    if not isinstance(serial_stopbits, (int, float)):
        raise EnvironmentError('Serial port stop bits number has to be an integer or a float!')

    rospy.logdebug('Serial port stop bits: "%s"', serial_stopbits)

    serial_xonxoff = rospy.get_param('~serial_xonxoff', False)

    if not isinstance(serial_xonxoff, bool):
        raise EnvironmentError('Serial port XON/XOFF value has to be a bool!')

    rospy.logdebug('Serial port XON/XOFF: %s', serial_xonxoff)

    serial_rtscts = rospy.get_param('~serial_rtscts', False)

    if not isinstance(serial_rtscts, bool):
        raise EnvironmentError('Serial port RTS/CTS value has to be a bool!')

    rospy.logdebug('Serial port RTS/CTS: %s', serial_rtscts)

    read_timeout = rospy.get_param('~read_timeout', 1.0)

    if not isinstance(read_timeout, float):
        raise EnvironmentError('Read timeout value has to be a bool!')

    rospy.logdebug('Read timeout: "%s"', read_timeout)

    openmv_port = serial.Serial(
        port=serial_port,
        baudrate=serial_baudrate,
        bytesize=serial_bytesize,
        parity=serial_parity,
        stopbits=serial_stopbits,
        xonxoff=serial_xonxoff,
        rtscts=serial_rtscts,
        # Blocking mode
        # WARNING: We have to keep a timeout for the correct node shutdown!
        timeout=read_timeout
    )
    rospy.logdebug('OpenMV serial port ready: %s', openmv_port)

    # No need in these variables from this moment
    del serial_port
    del serial_baudrate
    del serial_bytesize
    del serial_parity
    del serial_stopbits
    del serial_xonxoff
    del serial_rtscts
    del read_timeout

    json_message_buffer = ''  # Buffer for the incoming JSON message

    serial_byte = None  # Storage for a serial port byte

    while not rospy.is_shutdown():  # While the ROS is up
        try:
            # Get a byte from the serial port
            # HINT: It's a blocking routine
            serial_byte = openmv_port.read()

            if not serial_byte:  # No data in the serial port buffer (read timeout)
                pass
            elif serial_byte == '\n':  # End of the JSON message
                if len(json_message_buffer):  # Not an empty JSON message
                    # rospy.logdebug('Got JSON message from OpenMV: "%s"',
                    #              json_message_buffer)

                    try:
                        openmv_message = json.loads(json_message_buffer)  # Decode the JSON message

                        if 'faces' in openmv_message:
                            faces_msg = Faces()

                            faces_msg.header.stamp = rospy.Time.now()

                            for face in openmv_message['faces']:
                                faces_msg.faces.append(Rect(int(face['x']), int(face['y']), int(face['w']),
                                                      int(face['h'])))

                            faces_publisher.publish(faces_msg)

                        if 'april_tags' in openmv_message:
                             april_tags_msg = AprilTags()

                             april_tags_msg.header.stamp = rospy.Time.now()

                             for tag in openmv_message['april_tags']:
                                april_tags_msg.tags.append(AprilTag(int(tag['id']), Rect(int(tag['x']), int(tag['y']),
                                                          int(tag['w']), int(tag['h']))))

                             april_tags_publisher.publish(april_tags_msg)

                    except (ValueError, KeyError, TypeError) as e:  # Incorrect JSON message
                        rospy.logerr('Failed to parse JSON message "%s": %s',
                                     json_message_buffer, e)
                else:
                    rospy.logerr('Empty JSON message!')

                json_message_buffer = ''  # Clean the JSON state message buffer
            else:  # Not an end of the JSON faces message
                # JSON massage is too big
                if len(json_message_buffer) > JSON_MESSAGE_BUFFER_LIMIT:
                    rospy.logerr('Too big JSON message: "%s"', json_message_buffer)

                    json_message_buffer = ''  # Clean the JSON faces message buffer

                # Collect a character inside the faces message buffer
                json_message_buffer += serial_byte
        except serial.serialutil.SerialException as e:
            # TODO: When the ROS is down by Ctrl-C we will get an exception
            # caused by EINTR (signal interrupt)

            # Check if it is an interrupted system call
            # TODO: That's an ugly solution... Do we have other options with PySerial?
            if str(e).find("Interrupted system call") == -1:
                rospy.logerr('Failed to read from the serial device: %s', e)
            else:
                rospy.logdebug('Interrupted read')

            break  # Stop the ROS node
