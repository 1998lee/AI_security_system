import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sys
import math
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap, QImage, QFont
from PyQt5.QtCore import pyqtSignal, Qt
from threading import Thread
from rclpy.executors import MultiThreadedExecutor

class test_gui(Node):
    def __init__(self):
        super().__init__('test_gui')
        
        if not hasattr(self, 'gui_status'):
            self.gui = None
            self.gui_status = 'nothing'
            self.gui_result = 'nothing'
            
        self.test_st_pub = self.create_publisher(String, '/gui_status', 10)
        
        self.test_re_pub = self.create_publisher(String, '/gui_result', 10)
        # self.test_re_pub_timer = self.create_timer(0.1, self.test_re_pub_callback)
    
    def set_status(self, stauts_msg):
        self.status_msg = String()
        self.status_msg.data = stauts_msg
        self.test_st_pub.publish(self.status_msg)
        
    def set_result(self, result_msg):
        self.result_msg = String()
        self.result_msg.data = result_msg
        self.test_re_pub.publish(self.result_msg)
        
    # def test_re_pub_callback(self):
    #     self.result_msg = String()
    #     self.result_msg.data = self.gui_result
    #     self.test_re_pub.publish(self.result_msg)
        
    def set_gui(self, gui):
        self.gui = gui
    
class gui(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.create_gui()
        
    def create_gui(self):
        self.setWindowTitle("gui") 
        self.main_layer = QVBoxLayout()
        
        self.button1_layer = QHBoxLayout()
        self.pos_button1 = QPushButton('close')
        self.pos_button1.clicked.connect(self.send_close)
        self.button1_layer.addWidget(self.pos_button1)
        
        self.pos_button2 = QPushButton('open')
        self.pos_button2.clicked.connect(self.send_open)
        self.button1_layer.addWidget(self.pos_button2)
        self.main_layer.addLayout(self.button1_layer)
        
        self.button2_layer = QHBoxLayout()
        self.pos_button3 = QPushButton('detect')
        self.pos_button3.clicked.connect(self.send_detect)
        self.button2_layer.addWidget(self.pos_button3)
        
        self.pos_button4 = QPushButton('not detect')
        self.pos_button4.clicked.connect(self.send_not_detect)
        self.button2_layer.addWidget(self.pos_button4)
        self.main_layer.addLayout(self.button2_layer)
        
        self.setLayout(self.main_layer)
        self.node.set_gui(self)
        
    def send_close(self):
        self.node.set_status('close')
        
    def send_open(self):
        self.node.set_status('open')
    
    def send_detect(self):
        self.node.set_result('detect')
    
    def send_not_detect(self):
        self.node.set_result('not detect')

def main(args=None):
    rclpy.init(args=args)
    robot_node = test_gui()

    # PyQt 앱 준비
    app = QApplication(sys.argv)
    window = gui(robot_node)
    window.show()

    executor = MultiThreadedExecutor()
    executor.add_node(robot_node)

    # ROS spin을 별도의 스레드에서 실행
    def ros_spin():
        executor.spin()

    # ROS spin을 별도의 스레드에서 실행
    ros_thread = Thread(target=ros_spin, daemon=True)  # 데몬 스레드로 실행하여 메인 스레드 종료 시 함께 종료
    ros_thread.start()

    # PyQt 이벤트 루프 실행 (메인 스레드)
    sys.exit(app.exec_())
    
if __name__ == '__main__':
    main()