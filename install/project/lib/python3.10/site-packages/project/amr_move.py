import rclpy 
from rclpy.node import Node

from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist

class amr_move_node(Node):
    def __init__(self):
        super().__init__('amr_move_node')
        
        # 카메라 노드에서 욜로 결과를 받아오는 코드 
        self.yolo_result_sub = self.create_subscription(Float64MultiArray, '/yolo_result', self.yolo_result_sub_callback, 10)
        
        # 카메라 노드로 detection 결과를 보내주는 코드
        self.yolo_reslut_pub = self.create_publisher(String, '/amr_result', 10)
        self.yolo_reslut_pub_timer = self.create_timer(0.1, self.yolo_result_pub_to_control)
        
        # 로봇을 직접 움직이는 코드 
        self.robot_control_with_twist = self.create_publisher(Twist, '/cmd_vel', 10)
        self.robot_control_with_twist_timer = self.create_timer(0.1, self.robot_control_with_twist_callback)
        
        self.move_twist_status = False
        
    def yolo_result_sub_callback(self, msg):
        self.yolo_information_list = msg.data
        
    def yolo_result_pub_to_control(self):
        if self.yolo_information_list:
            send_message = String()
            send_message.data = 'detected'
            self.yolo_reslut_pub.publish(send_message)
            self.move_twist_status = True
            
    def robot_control_with_twist_callback(self):
        if self.move_twist_status is True and self.yolo_information_list :
            move_message = Twist()
            target_x_cencter = self.yolo_information_list[1]
            target_box_height = self.yolo_information_list[2]
            
            # 회전속도 결정
            if target_x_cencter < 305: # 목표가 왼쪽에 있는 경우
                move_message.angular.z = -0.1
            elif target_x_cencter > 335: # 목표가 오른쪽에 있는 경우
                move_message.angular.z = 0.1
            else:
                move_message.angular.z = 0.0
            
            # 선속도 결정
            if target_box_height < 260:
                move_message.linear.x = 0.13
            else:
                move_message.linear.x = 0.0    
            self.robot_control_with_twist.publish(move_message)
            
def main(args =None):
    rclpy.init(args=args)
    AMR_node = amr_move_node()
    
    rclpy.spin(AMR_node)
    
    AMR_node.destroy_node()
    rclpy.shutdown()         
        
if __name__ == '__main__':
    main()