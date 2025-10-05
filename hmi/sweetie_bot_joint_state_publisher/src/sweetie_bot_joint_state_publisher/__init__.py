# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math
import random
from collections import namedtuple
from xmlrpc.client import Binary
from io import BytesIO

import rospy
from actionlib import GoalStatus, SimpleActionClient

from python_qt_binding.QtCore import Qt, pyqtSlot, pyqtSignal as Signal, QSignalBlocker, QRegularExpression
from python_qt_binding.QtGui import QFont, QRegularExpressionValidator
from python_qt_binding.QtWidgets import QApplication, QHBoxLayout, QLabel, QDoubleSpinBox, QPushButton, QSlider, QVBoxLayout, QGridLayout, QScrollArea, QSpinBox, QWidget, QCheckBox, QFrame, QComboBox

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from sweetie_bot_control_msgs.msg import SetOperationalAction, SetOperationalGoal, SetOperationalResult, SetOperationalFeedback
from sweetie_bot_kinematics_msgs.msg import SupportState

RANGE = 10000

class SetOperationalWidget(QFrame):
    controller_active_signal = Signal()
    controller_done_signal = Signal(int, SetOperationalResult)
    controller_feedback_signal = Signal(SetOperationalFeedback)

    ResourceWidgets = namedtuple('ResourceWidgets', 'selector indicator')

    def __init__(self, name, action_ns, resources, parent = None, autostart = False, desc = None):
        # call superclass constructor
        super(SetOperationalWidget, self).__init__(parent)
        self.setFrameStyle(QFrame.Box)
        self.setFrameShadow(QFrame.Plain)
        self._name = name

        # ROS interface
        self._set_opertional_ac = SimpleActionClient(action_ns, SetOperationalAction)
        self._set_opertional_ac.wait_for_server(rospy.Duration(1.0))

        # Widget creation
        layout = QGridLayout()
        # title
        title_label = QLabel(f'{name}')
        title_label.setStyleSheet("font-weight: bold")
        if desc is not None:
            title_label.setToolTip(desc)
        layout.addWidget(title_label, 0, 0)
        # on/off button
        self._set_operational_button = QPushButton("TURN ON", self)
        self._set_operational_button.setCheckable(True)
        layout.addWidget(self._set_operational_button, 0, 1)
        # status 
        self._status_label = QLabel("UNKNOWN", self)
        layout.addWidget(self._status_label, 1, 0, 1, 2)
        # desc
        layout.addWidget(QLabel('Requested resources:'), 0, 2)
        layout.addWidget(QLabel('Controlled resources:'), 1, 2)
        # resources selctors and indicators
        self._resources_widgets = {}
        for col, resource in enumerate(resources):
            selector = QCheckBox(resource, self)
            selector.setTristate(False)
            selector.setChecked(True)
            indicator = QCheckBox(resource, self)
            indicator.setTristate(False)
            indicator.setChecked(False)
            indicator.setDisabled(True)
            self._resources_widgets[resource] = self.ResourceWidgets(selector, indicator)
            layout.addWidget(selector, 0, col + 3)
            layout.addWidget(indicator, 1, col + 3)
        # add layoutA
        self.setLayout(layout)

        # connect signals: button
        self._set_operational_button.clicked.connect(self.onSetOperational)
        # connect signals: SetOperationalAction related signals
        self.controller_active_signal.connect(self.onControllerActive)
        self.controller_done_signal.connect(self.onControllerDone)
        self.controller_feedback_signal.connect(self.onControllerFeedback)

        # autostart
        if autostart:
            self.onSetOperational()

    def onSetOperational(self):
        if self._set_operational_button.isFlat():
            # check if goal is not lost
            goal_status = self._set_opertional_ac.get_state()
            if goal_status in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                # controller is assumed active so cancel current goal
                self._status_label.setText('CANCELLING...') 
                self._set_opertional_ac.cancel_goal()
                rospy.loginfo(f'{self._name} controller CANCEL request is send.')
            else:
                # goal is lost
                state_string = GoalStatus.to_string(goal_status)
                rospy.loginfo(f'{self._name} is in unexpected state {state_string}')
                # button state
                self._set_operational_button.setText("TURN ON")
                self._set_operational_button.setFlat(False)
                # status indicator
                self._status_label.setText(f"INACTIVE ({state_string})")
                self._status_label.setStyleSheet("background-color : red; color: white; font-weight: bold")
                # deselect resources
                for widget in self.resources_widgets.values():
                    widget.indicator.setCheckState(Qt.Unchecked)

        else:
            # check if contrlloeer is available
            if self._set_opertional_ac.wait_for_server(rospy.Duration(1.0)):
                # controller is assumed not active
                goal = SetOperationalGoal(operational = True, resources = [ name for name, widgets in self._resources_widgets.items() if widgets.selector.isChecked() ]) 
                self._set_opertional_ac.send_goal(goal,
                                                  active_cb = lambda: self.controller_active_signal.emit(),
                                                  done_cb = lambda state, result: self.controller_done_signal.emit(state, result),
                                                  feedback_cb = lambda feedback: self.controller_feedback_signal.emit(feedback)
                                                 )
                # status indicator
                self._status_label.setStyleSheet("background-color: transparent; color: black; font-weight: normal")
                self._status_label.setText('ACTIVATING...') 
                #logging
                rospy.loginfo(f'{self._name} controller ACTIVATION request is send.')
            else:
                # action is not vavliable
                rospy.loginfo(f'{self._name} controller is not available.')
                # button state
                self._set_operational_button.setFlat(False)
                # status indicator
                self._status_label.setText("UNAVAILABLE")
                self._status_label.setStyleSheet("background-color : black; color: white; font-weight: bold")



    @pyqtSlot()
    def onControllerActive(self):
        # button
        self._set_operational_button.setText("TURN OFF")
        self._set_operational_button.setFlat(True)
        # status indicator
        self._status_label.setText("ACTIVE")
        self._status_label.setStyleSheet("background-color: green; color: white; font-weight: bold")
        # logging
        rospy.loginfo(f'{self._name} controller is ACTIVE.')

    @pyqtSlot(int, SetOperationalResult)
    def onControllerDone(self, state, result):
        state_string = GoalStatus.to_string(state)
        # button state
        self._set_operational_button.setText("TURN ON")
        self._set_operational_button.setFlat(False)
        # status indicator
        self._status_label.setText(f"INACTIVE ({state_string})")
        self._status_label.setStyleSheet("background-color : red; color: white; font-weight: bold")
        # deselect resources
        for widget in self._resources_widgets.values():
            widget.indicator.setCheckState(Qt.Unchecked)
        # logging
        rospy.loginfo(f'{self._name} controller is {state_string}: {result.error_string} ({result.error_code})')

    @pyqtSlot(SetOperationalFeedback)
    def onControllerFeedback(self, feedback):
        # set indicators
        for name, widgets in self._resources_widgets.items():
            if name in feedback.resources:
                widgets.indicator.setCheckState(Qt.Checked)
            else:
                widgets.indicator.setCheckState(Qt.Unchecked)
        # logging
        rospy.loginfo(f'{self._name} controller resources: %s', feedback.resources)


class JointStatePublisherGui(QWidget):
    # datatypes
    JointInfo = namedtuple('JointInfo', 'joint editbox slider selector')

    # signals
    joint_state_update_signal = Signal()
    supports_update_signal = Signal(SupportState)

    def __init__(self, title, jsp):
        super(JointStatePublisherGui, self).__init__()
        # get ROS paramters
        num_rows = rospy.get_param('~num_rows', 0)
        if not isinstance(num_rows, int) or num_rows < 0:
            raise ValueError('"num_rows" parameter must non-negative integer')
        resources = rospy.get_param('~resources', ['leg1', 'leg2', 'leg3', 'leg4', 'head'])
        if not isinstance(resources, list) or any(not isinstance(r, str) for r in resources):
            raise ValueError('"resources" must be  a list of joint groups names..')
        supports = rospy.get_param('~supports', ['leg1', 'leg2', 'leg3', 'leg4'])
        if not isinstance(supports, list) or any(not isinstance(r, str) for r in supports):
            raise ValueError('"supports" must be a list of kinematic chains names')
        self.pose_ns = rospy.get_param('~pose_ns', '/saved_msgs/joint_state')
        if not isinstance(self.pose_ns, str):
            raise ValueError('"pose_ns" must be a paramter namespace name.')

        # ROS interface
        # supports publishing and subscription
        self.supports_pub = rospy.Publisher('assign_supports', SupportState, queue_size=5)
        self.supports_sub = rospy.Subscriber('supports', SupportState, lambda msg: self.supports_update_signal.emit(msg))

        # connection to joint_state_publisher
        self.joint_state_update_signal.connect(self.onJointStateUpdate)
        self.jsp = jsp
        self.jsp.set_source_update_cb(lambda : self.joint_state_update_signal.emit())

        #
        # GUI creation
        #
        self.setWindowTitle(title)
        font = QFont("Helvetica", 9, QFont.Bold)
        # overall widget layout
        vlayout = QVBoxLayout(self)

        ### Set operationl widget ###
        controller_widget = SetOperationalWidget('MainTorqueSwitch', '/motion/controller/torque_off', resources, self, desc = 'This controller turns off joints from selected joint groups.')
        vlayout.addWidget(controller_widget)
        controller_widget = SetOperationalWidget('FollowJointState', '/motion/controller/joint_state', resources, self, autostart = False, desc = 'This controller make robot joints to follow pose published by this utility.')
        vlayout.addWidget(controller_widget)

        ### Support checkboxes ###
        self._support_saved_state = None
        support_layout = QHBoxLayout()
        label = QLabel('Supports selector')
        label.setToolTip('The checkboxes mark the kinematic chains that are assumed in contact with the supporting surface.\n Click checkbox to change chain state. Note it may be immediatelly overriden by an active controller.')
        label.setStyleSheet('font-weight: bold')
        support_layout.addWidget(label)
        self.support_widgets = {}
        for support in supports:
            checkbox = QCheckBox(support, self)
            checkbox.setTristate(False)
            checkbox.stateChanged.connect(lambda state, name=support: self.onSupportChange(name, state))
            self.support_widgets[support] = checkbox
            support_layout.addWidget(checkbox)
        support_layout.addStretch()
        support_all_button = QPushButton('Set All')
        support_layout.addWidget(support_all_button)
        support_none_button = QPushButton('Set None')
        support_layout.addWidget(support_none_button)
        support_restore_button = QPushButton('Restore')
        support_layout.addWidget(support_restore_button)
        support_widget = QFrame()
        support_widget.setFrameStyle(QFrame.Box)
        support_widget.setFrameShadow(QFrame.Plain)
        support_widget.setLayout(support_layout)
        vlayout.addWidget(support_widget)
        # signals
        self.supports_update_signal.connect(self.onSupportsUpdate)
        support_all_button.clicked.connect(self.onSupportAll)
        support_none_button.clicked.connect(self.onSupportNone)
        support_restore_button.clicked.connect(self.onSupportRestore)

        ### Joint states save/load widget ###
        state_layout = QHBoxLayout()
        state_label = QLabel('Named JointState')
        state_label.setToolTip(f'Save selected joints states to ROS parameter with given name in namespace "{self.pose_ns}". Joints are marked with checkboxes.\nLoad joint states from ROS paramters. In this case selection will be overriden.')
        state_label.setStyleSheet('font-weight: bold')
        state_layout.addWidget(state_label)
        self.pose_combobox = QComboBox(self)
        self.pose_combobox.setEditable(True)
        self.pose_combobox.setValidator(QRegularExpressionValidator(QRegularExpression('[A-Za-z_][A-Za-z0-9_]*')))
        state_layout.addWidget(self.pose_combobox)
        save_button = QPushButton('Save State', self)
        state_layout.addWidget(save_button)
        load_button = QPushButton('Load State', self)
        state_layout.addWidget(load_button)
        state_layout.addStretch()
        select_all_button = QPushButton('Select All Joints', self)
        state_layout.addWidget(select_all_button)
        select_no_button = QPushButton('Select No Joints', self)
        state_layout.addWidget(select_no_button)
        state_widget = QFrame()
        state_widget.setFrameStyle(QFrame.Box)
        state_widget.setFrameShadow(QFrame.Plain)
        state_widget.setLayout(state_layout)
        vlayout.addWidget(state_widget)
        
        # load poses names
        poses = rospy.get_param(self.pose_ns, {})
        for name in poses.keys():
            self.pose_combobox.addItem(name)

        # signals
        save_button.clicked.connect(self.onSaveState)
        load_button.clicked.connect(self.onLoadState)
        select_all_button.clicked.connect(self.onSelectAllJoints)
        select_no_button.clicked.connect(self.onSelectNoJoints)

        ### Generate sliders ###
        self.joint_map = {}
        sliders = []
        for name in self.jsp.joint_list:
            # check if joint can be controlled
            if name not in self.jsp.free_joints:
                continue
            joint = self.jsp.free_joints[name]
            if joint['min'] == joint['max']:
                continue
            # joint layout
            joint_layout = QVBoxLayout()
            row_layout = QHBoxLayout()
            # name/selector
            selector = QCheckBox(name)
            selector.setFont(font)
            selector.setTristate(False)
            selector.setChecked(True)
            row_layout.addWidget(selector)
            # value
            editbox = QDoubleSpinBox(self)
            editbox.setRange(joint['min'], joint['max'])
            editbox.setSingleStep(0.05)
            editbox.setDecimals(3)
            row_layout.addWidget(editbox)
            joint_layout.addLayout(row_layout)
            # slider
            slider = QSlider(Qt.Horizontal)
            slider.setFont(font)
            slider.setRange(0, RANGE)
            slider.setValue(int(RANGE/2))
            joint_layout.addWidget(slider)
            # connect relatd signals
            editbox.valueChanged.connect(lambda event,name=name: self.onEditboxChanged(name))
            slider.valueChanged.connect(lambda event,name=name: self.onSliderChanged(name))
            # add to layout and joint map
            self.joint_map[name] = JointStatePublisherGui.JointInfo(joint, editbox, slider, selector)
            sliders.append(joint_layout)

        # Determine number of rows to be used in grid
        self.num_rows = num_rows
        # if desired num of rows wasn't set, default behaviour is a vertical layout
        if self.num_rows == 0:
            self.num_rows = len(sliders)  # equals VBoxLayout

        # Generate positions in grid and place sliders there
        self.gridlayout = QGridLayout()
        self.positions = self.generate_grid_positions(len(sliders), self.num_rows)
        for item, pos in zip(sliders, self.positions):
            self.gridlayout.addLayout(item, *pos)

        # Add sliders grid to widget as scrollable area
        scrollable = QWidget()
        scrollable.setLayout(self.gridlayout)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(scrollable)
        vlayout.addWidget(scroll)

        ### Additaonal interfaces: buttons for centering sliders and etc ###
        ctrbutton = QPushButton('Center', self)
        ctrbutton.clicked.connect(self.center_event)
        vlayout.addWidget(ctrbutton)
        maxrowsupdown = QSpinBox()
        maxrowsupdown.setMinimum(1)
        maxrowsupdown.setMaximum(len(sliders))
        maxrowsupdown.setValue(self.num_rows)
        maxrowsupdown.valueChanged.connect(self.reorggrid_event)
        vlayout.addWidget(maxrowsupdown)

        ### general layout ###
        self.setLayout(vlayout)

        # Set zero positions read from parameters
        self.center()

    def onSupportChange(self, name, state):
        # publish new support state
        msg = SupportState()
        msg.header.stamp = rospy.Time.now()
        for name, checkbox in self.support_widgets.items():
            msg.name.append(name)
            msg.support.append(1.0 if checkbox.isChecked() else 0.0)
        self.supports_pub.publish(msg)

    @pyqtSlot(SupportState)
    def onSupportsUpdate(self, msg):
        for name, support in zip(msg.name, msg.support):
            checkbox = self.support_widgets.get(name)
            if checkbox is not None:
                with QSignalBlocker(checkbox) as blocker:
                    checkbox.setChecked(support > 0.0)

    def _set_support(self, value):
        self._support_saved_state = {}
        for name, checkbox in self.support_widgets.items():
            self._support_saved_state[name] = checkbox.isChecked()
            with QSignalBlocker(checkbox) as blocker:
                checkbox.setChecked(value)
        self.onSupportChange(None, None)

    def onSupportAll(self):
        self._set_support(True)
        
    def onSupportNone(self):
        self._set_support(False)

    def onSupportRestore(self):
        if self._support_saved_state is not None:
            for name, checkbox in self.support_widgets.items():
                value = self._support_saved_state.get(name)
                if value is not None:
                    with QSignalBlocker(checkbox) as blocker:
                        checkbox.setChecked(value)
        self.onSupportChange(None, None)
        

    @pyqtSlot()
    def onJointStateUpdate(self):
        self.updateFromJointState()

    def onSliderChanged(self, name):
        print('slider')
        joint_info = self.joint_map[name]
        # get new value
        value = self.sliderToValue(joint_info.slider.value(), joint_info.joint)
        # assign value to JSP and editbox
        joint_info.joint['position'] = value
        with QSignalBlocker(joint_info.editbox) as blocker:
            joint_info.editbox.setValue(value)

    def onEditboxChanged(self, name):
        print('edit')
        joint_info = self.joint_map[name]
        # get new value
        value = joint_info.editbox.value()
        # assign value to slider and editbox
        joint_info.joint['position'] = value
        with QSignalBlocker(joint_info.slider) as blocker:
            joint_info.slider.setValue( self.valueToSlider(value, joint_info.joint) )

    def onSaveState(self):
        # form message with current pose
        msg = JointState()
        for name, joint_info in self.joint_map.items():
            if joint_info.selector.isChecked():
                msg.name.append(name)
                msg.position.append(joint_info.joint['position'])
        # save message to parameter
        buf = BytesIO()
        msg.serialize(buf)
        value = Binary(buf.getvalue())
        param = self.pose_ns + '/' + self.pose_combobox.currentText()
        rospy.set_param(param, value)
        # logging
        rospy.loginfo(f'Current pose ({len(msg.name)} joints) is saved to parameter {param}.')

    def onLoadState(self):
        # load pose from paramter
        param = self.pose_ns + '/' + self.pose_combobox.currentText()
        try:
            value = rospy.get_param(param)
        except KeyError:
            rospy.logerr(f'ROS paramter {param} does not exists.')
            return
        if not isinstance(value, Binary):
            rospy.logerr(f'ROS parameter {param} is not Binary type.')
            return
        # deserialize it
        msg = JointState()
        msg.deserialize(value.data)
        # apply pose
        self.onSelectNoJoints()
        for name, position in zip(msg.name, msg.position):
            joint_info = self.joint_map.get(name)
            if joint_info is not None:
                joint_info.selector.setChecked(True)
                joint_info.joint['position'] = position
        self.updateFromJointState()
        # logging 
        rospy.loginfo(f'Load pose ({len(msg.name)} joints) form parameter {param}.')

    def onSelectAllJoints(self):
        for joint_info in self.joint_map.values():
            joint_info.selector.setChecked(True)

    def onSelectNoJoints(self):
        for joint_info in self.joint_map.values():
            joint_info.selector.setChecked(False)

    def updateFromJointState(self):
        for joint_info in self.joint_map.values():
            # get value
            value = joint_info.joint['position']
            # update slider and editbox
            with QSignalBlocker(joint_info.slider) as blocker:
                joint_info.slider.setValue( self.valueToSlider(value, joint_info.joint) )
            with QSignalBlocker(joint_info.editbox) as blocker:
                joint_info.editbox.setValue(value)

    def center_event(self, event):
        self.center()

    def center(self):
        rospy.loginfo("Centering")
        for joint_info in self.joint_map.values():
            joint_info.joint['position'] = joint_info.joint['zero']
        self.updateFromJointState()

    def reorggrid_event(self, event):
        self.reorganize_grid(event)

    def reorganize_grid(self, number_of_rows):
        self.num_rows = number_of_rows

        # Remove items from layout (won't destroy them!)
        items = []
        for pos in self.positions:
            item = self.gridlayout.itemAtPosition(*pos)
            items.append(item)
            self.gridlayout.removeItem(item)

        # Generate new positions for sliders and place them in their new spots
        self.positions = self.generate_grid_positions(len(items), self.num_rows)
        for item, pos in zip(items, self.positions):
            self.gridlayout.addLayout(item, *pos)

    def generate_grid_positions(self, num_items, num_rows):
        if num_rows == 0:
            return []
        positions = [(y, x) for x in range(int((math.ceil(float(num_items) / num_rows)))) for y in range(num_rows)]
        positions = positions[:num_items]
        return positions

    @staticmethod
    def valueToSlider(value, joint):
        return int((value - joint['min']) * float(RANGE) / (joint['max'] - joint['min']))

    @staticmethod
    def sliderToValue(slider, joint):
        pctvalue = slider / float(RANGE)
        return joint['min'] + (joint['max']-joint['min']) * pctvalue
