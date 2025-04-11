#!/usr/bin/env python3
import os
from pathlib import Path
import sys

try:
    from python_qt_binding import loadUi
    from python_qt_binding.QtGui import *
    from python_qt_binding.QtCore import *
    from python_qt_binding.QtWidgets import *
except ImportError:
        pass
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import UInt16
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseWithCovarianceStamped, Twist
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int16, Int16MultiArray, Byte
from tf2_ros import TransformException, LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import math
import copy
import shlex
from psutil import Popen
from Ui_NavigationWB import Ui_Navigation
from os import path as py_path
import csv
import time

from PoseListTableModel import PoseListTableModel

class MainWindow(QWidget, Ui_Navigation, Node):
    forwardSignal = pyqtSignal(PoseStamped)
    inputSignal = pyqtSignal(int)
    closeSignal = pyqtSignal()

    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        Node.__init__(self, "QtMainWindow")
        self.setupUi(self)
        self.vel_percent = 0.5
        self.ang_percent = 0.5
        self.max_lin_vel = 0.6
        self.max_ang_vel = 0.3
        self.lin_vel_step = 0.02
        self.ang_vel_step = 0.02
        self.manual_lin_vel = 0.
        self.manual_ang_vel = 0.
        self.target_manual_lin_vel = 0.
        self.target_manual_ang_vel = 0.
        self.jog_timer = QTimer(self)
        self.jog_timer.timeout.connect(self.simple_move_profile_timeout)
        self.pose_table_model = PoseListTableModel(self)
        self.pose_table_model.dataChanged.connect(self.pose_changed)
        self.basic_navigator = BasicNavigator()
        #time.sleep(5)
        self.basic_navigator.waitUntilNav2Active(navigator='bt_navigator', localizer='bt_navigator')
        #self.basic_navigator.waitUntilNav2Active(navigator='bt_navigator', localizer='amcl')
        print("Navigation2 Active!")
        self.navigation_check_timer = QTimer(self)
        self.navigation_check_timer.timeout.connect(self.check_navigation_status_timeout)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_goal_pose = self.create_subscription(PoseStamped, "/goal_pose", self.got_2d_goal, 10)
        self.current_pose = Pose()
        self.pose_timer = QTimer(self)
        self.pose_timer.timeout.connect(self.update_pose)
       #self.sub_current_pose = self.create_subscription(PoseWithCovarianceStamped, "/current_pose", self.update_pose, 10) 
       #self.battery_timer_counter = 0
        self.cmd_pub = self.create_publisher(Twist,"cmd_vel", 10)
        self.sub_input_pins = self.create_subscription(UInt16, "input_pins", self.input_pins_callback, 10)
       #Battery voltage subscriber
        self.sub_battery = self.create_subscription(BatteryState, "/battery", self.update_battery_voltage, 10)
        self.forwardSignal.connect(self.get_forward_signal)
        self.closeSignal.connect(self.close_program)
        self.inputSignal.connect(self.get_input_signal) 
        self.qt_pose_table_view.setModel(self.pose_table_model)
        self.qt_pose_table_view.verticalHeader().setVisible(True)
        self.qt_pose_table_view.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch) 
        self.qt_pose_table_view.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.qt_pose_table_view.setSelectionMode(QAbstractItemView.SingleSelection)
        self.qt_pose_table_view.setSortingEnabled(True)
        self.qt_pose_table_view_selection_model = self.qt_pose_table_view.selectionModel()

        self.row_togo = 0
        self.raw_pose = 0
        self.no_retry = 0

        self.run_active = False
        self.run_next_timer = QTimer(self)
        self.run_next_timer.setSingleShot(True)
        self.run_next_timer.timeout.connect(self.run_next_timeout)

        filename = 'target_poses.csv'
        if py_path.isfile(filename):
            self.load_target_from_filename(filename)
        self.marker_pub = self.create_publisher(MarkerArray,"target_pose_array", 10)
        self.marker_array = MarkerArray()
        self.gen_marker_array()

        if self.pose_table_model.contents:
            self.qt_pose_table_view.selectRow(0)

        self.pose_timer.start(10)
        self.pressed_key = -1

        self.input_pins = 0

    def update_battery_voltage(self, msg):
        battery_voltage = msg.percentage
        self.qt_bat_vol_lcdnumber.display(battery_voltage)

# Callback method for the input_pins topic
    def input_pins_callback(self, msg):
        if self.input_pins != msg.data :
            self.input_pins = msg.data
            self.inputSignal.emit(self.input_pins)

    @pyqtSlot(bool)
    def on_qt_run_button_clicked(self, checked):
        if checked:
            self.run_active = True
            indices = self.qt_pose_table_view_selection_model.selectedRows()
            if indices:
                row_togo = indices[0].row() + 1
                if row_togo >= len(self.pose_table_model.contents):
                    row_togo = 0
                one_loop = True
                row_togo_list = []
                while row_togo < len(self.pose_table_model.contents):
                    row_togo_list.append(row_togo) 
                    if self.pose_table_model.contents[row_togo][4]: # Active 
                        self.qt_pose_table_view.selectRow(row_togo)
                        break
                    row_togo += 1
                    if row_togo == len(self.pose_table_model.contents) and one_loop:
                        row_togo = 0
                        one_loop = False
                        row_togo_list.append(row_togo)
                        if self.pose_table_model.contents[row_togo][4]:
                            self.qt_pose_table_view.selectRow(row_togo)
                            break
                        row_togo += 1
                self.goto_poses(row_togo_list)
                self.navigation_check_timer.start(10)
        else:
            self.run_active = False 

    @pyqtSlot()
    def on_qt_clone_button_clicked(self):
        indices = self.qt_pose_table_view_selection_model.selectedRows()
        if indices:
            idx = indices[0].row()
            content = copy.deepcopy(self.pose_table_model.contents[idx])
            self.pose_table_model.insert_row(idx, content)
            self.gen_marker_array()

    @pyqtSlot()
    def on_qt_delete_button_clicked(self):
        indices = self.qt_pose_table_view_selection_model.selectedRows()
        if indices:
            for idx in sorted(indices, reverse=True):
                self.pose_table_model.remove_row(idx.row())
            self.gen_marker_array()

    @pyqtSlot()
    def on_qt_delete_all_button_clicked(self):
        ret = QMessageBox.question(self, "Delete ALL Target Poses!", "Do you really want to DELETE ALL the target poses!?",
                                   QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if ret == QMessageBox.Yes:
            self.pose_table_model.clear()
            self.gen_marker_array()

    @pyqtSlot()
    def on_qt_move_up_button_clicked(self):
        indices = self.qt_pose_table_view_selection_model.selectedRows()
        if indices:
            indices = sorted(indices)
            if indices[0].row() != 0:
                selected_rows = QItemSelection()
                for idx in indices:
                    if self.pose_table_model.move_up(idx.row()):
                        self.qt_pose_table_view.selectRow(idx.row()-1)
                    else:
                        self.qt_pose_table_view.selectRow(idx.row())
                    selected_rows.merge(self.qt_pose_table_view_selection_model.selection(), QItemSelectionModel.Select)
                self.qt_pose_table_view_selection_model.clearSelection()
                self.qt_pose_table_view_selection_model.select(selected_rows, QItemSelectionModel.Select)

    @pyqtSlot()
    def on_qt_move_down_button_clicked(self):
        indices = self.qt_pose_table_view_selection_model.selectedRows()
        indices = sorted(indices, reverse=True)
        if indices:
            if indices[0].row() < self.pose_table_model.rowCount(self)-1:
                selected_rows = QItemSelection()
                for idx in indices:
                    if self.pose_table_model.move_down(idx.row()):
                        self.qt_pose_table_view.selectRow(idx.row()+1)
                    else:
                        self.qt_pose_table_view.selectRow(idx.row())
                    selected_rows.merge(self.qt_pose_table_view_selection_model.selection(), QItemSelectionModel.Select)
                self.qt_pose_table_view_selection_model.clearSelection()
                self.qt_pose_table_view_selection_model.select(selected_rows, QItemSelectionModel.Select)

    @pyqtSlot()
    def on_qt_cancel_button_clicked(self):
        self.basic_navigator.cancelTask()
        self.qt_next_button.setEnabled(True)

    @pyqtSlot()
    def on_qt_rename_button_clicked(self):
        ret = QMessageBox.question(self, "Rename All Poses",
                                   "Do you really want to RENAME ALL the target poses to numbers in order!?",
                                   QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if ret == QMessageBox.Yes:
            self.pose_table_model.auto_rename()
            self.gen_marker_array()

    @pyqtSlot()
    def on_qt_load_button_clicked(self):
        filename, selectedFilter = QFileDialog.getOpenFileName(self, "Open CSV", "", "CSV Files (*.csv *.txt)")
        if filename:
            self.load_target_from_filename(filename)
        #cmd = Byte()
        #cmd.data = 8
        #self.io_pub.publish(cmd)

    @pyqtSlot()
    def on_qt_save_button_clicked(self):
        filename, selectedFilter = QFileDialog.getSaveFileName(self, "Save CSV", "", "CSV Files (*csv *.txt)")
        if filename:
            if filename.endswith(".csv") or filename.endswith(".txt"):
                self.save_target_poses_to_filename(filename)
            else:
                self.save_target_poses_to_filename(filename + ".csv")

    @pyqtSlot()
    def on_qt_go_button_clicked(self):
        indices = self.qt_pose_table_view_selection_model.selectedRows()
        if indices:
            self.goto_pose(indices[0].row())

    @pyqtSlot()
    def on_qt_next_button_clicked(self):
        indices = self.qt_pose_table_view_selection_model.selectedRows()
        if indices:
            row_togo = indices[0].row() + 1
            if row_togo >= len(self.pose_table_model.contents):
                row_togo = 0
            one_loop = True
            row_togo_list = []
            while row_togo < len(self.pose_table_model.contents):
                row_togo_list.append(row_togo) 
                if self.pose_table_model.contents[row_togo][4]: # Active 
                    self.qt_pose_table_view.selectRow(row_togo)
                    break
                row_togo += 1
                if row_togo == len(self.pose_table_model.contents) and one_loop:
                    row_togo = 0
                    one_loop = False
                    row_togo_list.append(row_togo)
                    if self.pose_table_model.contents[row_togo][4]:
                        self.qt_pose_table_view.selectRow(row_togo)
                        break
                    row_togo += 1
            self.goto_poses(row_togo_list)
            self.qt_next_button.setEnabled(False)
            self.navigation_check_timer.start(10)

    @pyqtSlot()
    def on_qt_add_current_pose_button_clicked(self):
        self.add_goal(self.current_pose) 

    @pyqtSlot()
    def on_qt_forward_button_pressed(self):
        #self.target_manual_lin_vel = self.minmax_constrain(
        #    self.target_manual_lin_vel + self.lin_vel_step, -self.max_lin_vel, self.max_lin_vel)
        #self.qt_vel_lcdnumber.display(self.target_manual_lin_vel)
        self.target_manual_lin_vel = self.vel_percent * self.max_lin_vel
        if not self.jog_timer.isActive():
            self.jog_timer.start(10)

    @pyqtSlot()
    def on_qt_forward_button_released(self):
        self.target_manual_lin_vel = 0.

    @pyqtSlot()
    def on_qt_backward_button_pressed(self):
        #self.target_manual_lin_vel = self.minmax_constrain(
        #    self.target_manual_lin_vel - self.lin_vel_step, -self.max_lin_vel,  self.max_lin_vel)
        #self.qt_vel_lcdnumber.display(self.target_manual_lin_vel)
        self.target_manual_lin_vel = -1.0 * self.vel_percent * self.max_lin_vel
        if not self.jog_timer.isActive():
            self.jog_timer.start(10)

    @pyqtSlot()
    def on_qt_backward_button_released(self):
        self.target_manual_lin_vel = 0.

    @pyqtSlot()
    def on_qt_stop_button_pressed(self):
        self.target_manual_lin_vel = 0.
        self.target_manual_ang_vel = 0.
        #self.qt_vel_lcdnumber.display(self.target_manual_lin_vel)
        #self.qt_ang_lcdnumber.display(self.target_manual_ang_vel)
        if not self.jog_timer.isActive():
            self.jog_timer.start(10)

    @pyqtSlot()
    def on_qt_rotate_left_button_pressed(self):
        #self.target_manual_ang_vel = self.minmax_constrain(
        #    self.target_manual_ang_vel + self.ang_vel_step, -self.max_ang_vel, self.max_ang_vel)
        #self.qt_ang_lcdnumber.display(self.target_manual_ang_vel)
        self.target_manual_ang_vel = self.ang_percent * self.max_ang_vel
        if not self.jog_timer.isActive():
            self.jog_timer.start(10)

    @pyqtSlot()
    def on_qt_rotate_left_button_released(self):
        self.target_manual_ang_vel = 0.

    @pyqtSlot()
    def on_qt_rotate_right_button_pressed(self):
        #self.target_manual_ang_vel = self.minmax_constrain(
        #    self.target_manual_ang_vel - self.ang_vel_step, -self.max_ang_vel, self.max_ang_vel)
        #self.qt_ang_lcdnumber.display(self.target_manual_ang_vel)
        self.target_manual_ang_vel = -1.0 * self.ang_percent * self.max_ang_vel
        if not self.jog_timer.isActive():
            self.jog_timer.start(10)

    @pyqtSlot()
    def on_qt_rotate_right_button_released(self):
        self.target_manual_ang_vel = 0.

    @pyqtSlot(int)
    def on_qt_vel_slider_valueChanged(self, value):
        self.vel_percent = float(value)/100.0
        self.qt_vel_lcdnumber.display(value)

    @pyqtSlot(int)
    def on_qt_ang_slider_valueChanged(self, value):
        self.ang_percent = float(value)/100.0
        self.qt_ang_lcdnumber.display(value)

    def update_pose_from_amcl_pose(self, data):
        self.current_pose = data.pose.pose
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        quant = self.current_pose.orientation
        orie_list = [quant.x,quant.y,quant.z,quant.w]
        (roll, pitch, yaw) = euler_from_quaternion(orie_list)
        angle = yaw * 180 / math.pi
        self.qt_x_lcdnumber.display(x)
        self.qt_y_lcdnumber.display(y)
        self.qt_angle_lcdnumber.display(angle)

    def update_pose(self):
        try:
            transf_stamped = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            t = transf_stamped.transform.translation
            r = transf_stamped.transform.rotation
            
            self.current_pose.position.x = t.x
            self.current_pose.position.y = t.y
            self.current_pose.position.z = t.z
            self.current_pose.orientation = r
            
            quant = self.current_pose.orientation
            orie_list = [quant.x,quant.y,quant.z,quant.w]
            (roll, pitch, yaw) = euler_from_quaternion(orie_list)
            angle = yaw * 180 / math.pi
            self.qt_x_lcdnumber.display(t.x)
            self.qt_y_lcdnumber.display(t.y)
            self.qt_angle_lcdnumber.display(angle)
        except TransformException as ex:
            self.get_logger().info(f'Cound not transform base_link to map!')

    def got_2d_goal(self, data):
        if self.qt_auto_add_goal_checkbox.isChecked():
            self.forwardSignal.emit(data)

    def get_forward_signal(self, data):
        self.add_goal(data.pose)

    def get_input_signal(self, data):
        if not self.qt_next_button.isEnabled():
            return
        elif data == 1536:
            self.on_qt_next_button_clicked()
        elif data == 1537:
            # Handle the case when data is 1541
            pass
        else:
            # Handle other data values if necessary
            pass


    def add_goal(self, pose):
        name = str(len(self.pose_table_model.contents))
        posi = pose.position
        orie = pose.orientation
        orie_list = [orie.x, orie.y, orie.z, orie.w]
        (roll, pitch, yaw) = euler_from_quaternion(orie_list)

        indices = self.qt_pose_table_view_selection_model.selectedRows()
        row = -1
        if indices:
            row = indices[0].row()+1
        self.pose_table_model.add_row(name, posi.x, posi.y, yaw*180/math.pi, row)
        self.gen_marker_array()

    def pose_changed(self, top_left, bottom_right):
        '''if len(self.marker_array.markers) > top_left.row():
            marker = self.marker_array.markers[top_left.row()]
            marker.action = marker.MODIFY
            marker.pose.position =
            quat_array = quaternion_from_euler(0, 0, goal[3] * math.pi / 180)
            a_marker.pose.orientation = Quaternion(0, 0, quat_array[2], quat_array[3])
            self.marker_pub.publish(self.marker_array)
        '''
        self.gen_marker_array()

    def gen_marker_array(self):
        if self.marker_array.markers:
            for marker in self.marker_array.markers:
                marker.action = marker.DELETE
            self.marker_pub.publish(self.marker_array)
            while self.marker_array.markers:
                self.marker_array.markers.pop(0)
        arrow_id = 0
        text_id = 10000
        if self.pose_table_model.contents:
            for idx in range(len(self.pose_table_model.contents)):
                goal = self.pose_table_model.contents[idx]
                a_marker = Marker()
                a_marker.header.frame_id = "map"
                a_marker.type = a_marker.ARROW
                a_marker.action = a_marker.ADD
                a_marker.scale.x = 0.21
                a_marker.scale.y = 0.03
                a_marker.scale.z = 0.03
                a_marker.color.a = 1.0
                a_marker.color.r = 0.0
                a_marker.color.g = 1.0
                a_marker.color.b = 0.0
                a_marker.pose.position = Point(x=goal[1], y=goal[2], z=0.2)
                heading_angle = goal[3]*math.pi/180.
                quat_array = quaternion_from_euler(0, 0, heading_angle )
                a_marker.pose.orientation = Quaternion(x=0., y=0., z=quat_array[2], w=quat_array[3])
                a_marker.id = arrow_id
                arrow_id += 1
                self.marker_array.markers.append(a_marker)

                text = goal[0]
                t_marker = Marker()
                t_marker.header.frame_id = "map"
                t_marker.type = t_marker.TEXT_VIEW_FACING
                t_marker.action = t_marker.ADD
                t_marker.scale.x = 0.1
                t_marker.scale.y = 0.1
                t_marker.scale.z = 0.1
                t_marker.color.a = 1.0
                t_marker.color.r = 0.0
                t_marker.color.g = 0.0
                t_marker.color.b = 1.0
                t_marker.text = text
                t_marker.pose = copy.deepcopy(a_marker.pose)
                t_marker.pose.position.z = 0.
                t_marker.pose.position.x += 0.1*(math.cos(heading_angle) + math.sin(heading_angle))
                t_marker.pose.position.y += 0.1*(math.cos(heading_angle) + math.sin(heading_angle))
                t_marker.id = text_id
                text_id += 1
                self.marker_array.markers.append(t_marker)

            self.marker_pub.publish(self.marker_array)

    def load_target_from_filename(self, filename):
        with open(filename) as csvfile:
            reader = csv.reader(csvfile)
            self.pose_table_model.set_poses_from_csv_reader(reader)

    def load_target_file(self):
        filename = 'target_poses.csv'
        self.load_target_from_filename(filename)

    def save_target_poses_to_filename(self, filename):
        with open(filename, 'w') as csvfile:
            # using csv.writer method from CSV package
            writer = csv.writer(csvfile)
            writer.writerows(self.pose_table_model.contents)

    def save_target_poses(self):
        filename = 'target_poses.csv'
        self.save_target_poses_to_filename(filename)

    def got_next_pose_to_go(self, data):
        self.on_qt_next_button_clicked()

    def raw_pose_to_pose(self, raw_pose):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position = Point(x=raw_pose[1], y=raw_pose[2], z=0.)
        quat_array = quaternion_from_euler(0, 0, raw_pose[3] * math.pi / 180)
        goal_pose.pose.orientation = Quaternion(x=0., y=0., z=quat_array[2], w=quat_array[3])
        return goal_pose 

    def goto_pose(self, row_togo):
        self.row_togo = row_togo
        self.raw_pose = self.pose_table_model.contents[self.row_togo]
        goal_pose = self.raw_pose_to_pose(self.raw_pose)
        self.basic_navigator.goToPose(goal_pose)    
   
    def goto_poses(self, row_togo_list):
        if len(row_togo_list) == 1:
            self.row_togo = row_togo_list[0]
            self.raw_pose = self.pose_table_model.contents[self.row_togo]
            goal_pose = self.raw_pose_to_pose(self.raw_pose)
            self.basic_navigator.goToPose(goal_pose)
        elif len(row_togo_list) > 1:
            goal_pose_list = []
            for row_togo in row_togo_list:
                self.raw_pose = self.pose_table_model.contents[row_togo]
                goal_pose_list.append(self.raw_pose_to_pose(self.raw_pose))
            self.basic_navigator.goThroughPoses(goal_pose_list)
            #self.basic_navigator.followWaypoints(goal_pose_list)

    def close_program(self):
        print("Close!")
        rclpy.shutdown()
        self.close()

    def closeEvent(self, ev):
        print("Close Event")
        self.pose_timer.stop()
        self.save_target_poses()
        rclpy.shutdown()
        #self.close()

    def simple_move_profile_gen(self, target, current, step):
        if target > current:
            current = min(target, current + step)
        elif target < current:
            current = max(target, current - step)
        else:
            current = target
        return current

    def minmax_constrain(self, value, minimum, maximum):
        if value < minimum:
            value = minimum
        elif value > maximum:
            value = maximum
        return value

    def simple_move_profile_timeout(self):
        twist = Twist()
        self.manual_lin_vel = self.simple_move_profile_gen(self.target_manual_lin_vel, self.manual_lin_vel, self.lin_vel_step/2)
        twist.linear.x = self.manual_lin_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        self.manual_ang_vel = self.simple_move_profile_gen(self.target_manual_ang_vel, self.manual_ang_vel, self.ang_vel_step/2)
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.manual_ang_vel
        self.cmd_pub.publish(twist)

        #if self.manual_lin_vel == self.target_manual_lin_vel and self.manual_ang_vel == self.target_manual_ang_vel:
        if self.manual_lin_vel == 0. and self.manual_ang_vel == 0. and not self.qt_stop_button.isDown():
            self.jog_timer.stop()

    def keyPressEvent(self, event):
        super(MainWindow, self).keyPressEvent(event)
        if self.pressed_key == -1:
            if event.key() == Qt.Key_I:
                self.pressed_key = Qt.Key_I
                self.on_qt_forward_button_pressed()
            elif event.key() == Qt.Key_Comma:
                self.pressed_key = Qt.Key_Comma
                self.on_qt_backward_button_pressed()
            elif event.key() == Qt.Key_L:
                self.pressed_key = Qt.Key_L
                self.on_qt_rotate_right_button_pressed()
            elif event.key() == Qt.Key_J:
                self.pressed_key = Qt.Key_J
                self.on_qt_rotate_left_button_pressed()
            elif event.key() == Qt.Key_K:
                self.pressed_key = Qt.Key_K
                self.on_qt_stop_button_pressed()
    
    def keyReleaseEvent(self, event):
        super(MainWindow, self).keyReleaseEvent(event)
        if self.pressed_key != -1:
            if self.pressed_key == Qt.Key_I:
                self.on_qt_forward_button_released()
            elif self.pressed_key == Qt.Key_Comma:
                self.on_qt_backward_button_released()
            elif self.pressed_key == Qt.Key_L:
                self.on_qt_rotate_right_button_released()
            elif self.pressed_key == Qt.Key_J:
                self.on_qt_rotate_left_button_released()
            self.pressed_key = -1

    def check_navigation_status_timeout(self):
        if self.basic_navigator.isTaskComplete():
            if self.run_active:
                # start(time), time is in millisecond unit.
                self.run_next_timer.start(10000)
            else:
                self.qt_next_button.setEnabled(True)
            self.navigation_check_timer.stop()

    def run_next_timeout(self):
        if self.run_active:
            self.on_qt_run_button_clicked(checked=True)

def main():
    app = QApplication(sys.argv)
    rclpy.init()
    agvNavigation = MainWindow()
    app.processEvents()
    agvNavigation.show()

    exec_ = MultiThreadedExecutor()
    exec_.add_node(agvNavigation)
    while rclpy.ok():
        try:
            exec_.wait_for_ready_callbacks(0)
            exec_.spin_once()
        except:
            pass
        app.processEvents()
    app.quit()
    exec_.remove_node(agvNavigation)

if __name__ == "__main__":
    main()
