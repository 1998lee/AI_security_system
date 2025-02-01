import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Float64MultiArray
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion

# 010 4375 8838
# ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
# ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/home/inyoung/map.yaml

# 처음 : 0.031, -0.009, -0.00, 1.00
# 0.278 -0.083 -0.772 0.636
# 0.256, -0.622 -1.0 0.135
# -0.746 -0.682 0.773 0.635
# -0.697 -0.128 0.99 0.142
# -1.496 -0.107 -0.805 0.593
class AMR_move_node(Node):
    def __init__(self):
        super().__init__('AMR_move_node') 
        
        # nav2의 초기 위치를 알려주는 퍼블리셔
        self.nav2_initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # gui의 상태를 받아오는 서브스크라이버 
        self.gui_status = self.create_subscription(String, '/gui_status', self.gui_status_callback, 10)
        
        # gui에서 detection 결과를 받아오는 서브스크라이버 
        self.gui_detection_result = self.create_subscription(Float64MultiArray, '/web_car_position', self.gui_detection_result_callback, 10)
        
        # AMR에서 detection 결과를 받아오는 서브스크라이버
        self.AMR_detection_result = self.create_subscription(String, '/AMR_result', self.AMR_detection_result_callback, 10)
        
        # Nav2 pose 클라이언트
        self.nav2_pose_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        
        # Nav2 Waypoint 클라이언트
        self.nav2_waypoint_client = ActionClient(self, FollowWaypoints, "/follow_waypoints")
        
        # 처음 실행시 시작 인덱스와 좌표 리스트를 받아옴
        if not hasattr(self, 'patrol_position_list'):
            self.patrol_position_list = [
                self.set_waypoints(0.278, -0.083, ori_z=-0.772, ori_w=0.636),
                self.set_waypoints(0.256, -0.622, ori_z=-1.0, ori_w=0.135),
                self.set_waypoints(-0.746, -0.682, ori_z=0.773, ori_w=0.635),
                self.set_waypoints(-0.697, -0.128, ori_z=0.99, ori_w=0.142),
                self.set_waypoints(-1.496, -0.107, ori_z=-0.805, ori_w=0.593),
                self.set_waypoints(-0.697, -0.128, ori_z=0.99, ori_w=0.142),
                self.set_waypoints(-0.746, -0.682, ori_z=0.773, ori_w=0.635),
                self.set_waypoints(0.256, -0.622, ori_z=-1.0, ori_w=0.135),
                self.set_waypoints(0.278, -0.083, ori_z=-0.772, ori_w=0.636)
            ]
            self.gui_detect_result = False
            self.set_nav2_initial_pose()
        
    def set_nav2_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        # 초기위치 설정
        initial_pose.pose.pose.position.x = 0.031059306665468626
        initial_pose.pose.pose.position.y = -0.008556537922417859
        initial_pose.pose.pose.position.z = 0.0  # Z should be 0 for 2D navigation

        # 회전 각도 설정
        initial_pose.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=-0.000791402461009825,
            w=0.9999996868410234  # Corresponding quaternion w component
        )

        # 불확실성 행렬 설정
        initial_pose.pose.covariance = [
            0.2055095149684755, -0.015166996680769075, 0.0, 0.0, 0.0, 0.0,
            -0.015166996680769079, 0.16294735127121404, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.07398853372793512
        ]
        
        # 퍼블리시
        self.nav2_initial_pose_pub.publish(initial_pose)
        self.get_logger().info('Initial pose published.')

    def set_waypoints(self, x, y, z=0.0, ori_x = 0.0, ori_y = 0.0, ori_z = -0.00, ori_w = 1.0):
        way_point = PoseStamped()
        way_point.header.frame_id = 'map'
        way_point.pose.position.x = x
        way_point.pose.position.y = y
        way_point.pose.position.z = z
        way_point.pose.orientation.x = ori_x
        way_point.pose.orientation.y = ori_y
        way_point.pose.orientation.z = ori_z
        way_point.pose.orientation.w = ori_w
        return way_point
 
    def gui_status_callback(self, gui_status_msg):
        self.gui_status = gui_status_msg.data
        
        if self.gui_status == 'close':
            print('start patrol')
            self.patrol_status = True
            self.way_point_send_goal(self.patrol_position_list)
            
        elif self.gui_status == 'open':
            print('go to home')
            if hasattr(self, 'current_waypoint') and type(self.current_waypoint) is int:
                self.nav2_waypoint_client._cancel_goal_async(self.goal_handle)
                self.patrol_status = False
                print(self.current_waypoint)
                
                if self.current_waypoint <= 4:
                    start_index = self.current_waypoint
                elif self.current_waypoint > 4:
                    start_index = 8 - self.current_waypoint
                
                way_points = self.patrol_position_list[0:start_index+1][::-1]
                way_points.append(self.set_waypoints(0.031, -0.009, ori_z=0.0, ori_w=1.0))
                self.way_point_send_goal(way_points)  
                self.current_waypoint = None
       
    def gui_detection_result_callback(self, gui_result_msg):
        detection_result = gui_result_msg.data
        print(detection_result)
        if detection_result:
            if self.action_run_state is True:
                self.nav2_waypoint_client._cancel_goal_async(self.goal_handle)
            
            if self.current_waypoint is None or self.current_waypoint == 0 or self.current_waypoint == 8:
                way_points = self.patrol_position_list[0:1+1][::-1]
                way_points.append(self.set_waypoints(-1.496, -0.107, ori_z=-0.805, ori_w=0.593))
                self.way_point_send_goal(way_points)
            else:
                target_pose = [-1.496, -0.107, -0.805, 0.593]
                self.target_send_goal(target_pose)
            self.current_waypoint = 3
            
    def AMR_detection_result_callback(self):
        pass
            
    def way_point_send_goal(self, way_point_list):
        self.action_run_state = True
        print('waypoint_start')
        
        # 액션 서버와 연결이 되는지 확인
        wait_count = 1
        while not self.nav2_waypoint_client.wait_for_server(timeout_sec=0.1):
            if wait_count > 3:
                self.get_logger().info('Navigate action server is not available.')
                self.action_run_state = False
                return 
            wait_count += 1
        
        # 액션 서버에 보내는 메세지 형식 작성
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = way_point_list
        
        # nav2 액션 서버에 골을 비동기 적으로 전송, 완료되면 다음 함수 실행
        self.send_goal_future = self.nav2_waypoint_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigate_to_pose_action_feedback)
        self.send_goal_future.add_done_callback(self.navigate_to_pose_action_goal)
            
    def target_send_goal(self, pose_list):
        self.action_run_state = True
        self.patrol_status = False
        print('pose_start')
        
        # 액션 서버와 연결이 되는지 확인
        wait_count = 1
        while not self.nav2_waypoint_client.wait_for_server(timeout_sec=0.1):
            if wait_count > 3:
                self.get_logger().info('Navigate action server is not available.')
                self.action_run_state = False
                return 
            wait_count += 1
        
        # 액션 서버에 보내는 메세지 형식 작성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = pose_list[0]
        goal_msg.pose.pose.position.y = pose_list[1]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = pose_list[2]
        goal_msg.pose.pose.orientation.w = pose_list[3]
        
        # nav2 액션 서버에 골을 비동기 적으로 전송, 완료되면 다음 함수 실행
        self.send_goal_future = self.nav2_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigate_to_pose_action_feedback)
        self.send_goal_future.add_done_callback(self.navigate_to_pose_action_goal)
        
    def navigate_to_pose_action_goal(self, future):
        self.goal_handle = future.result()
        
        # 골이 접수가 안되면 실행 
        if not self.goal_handle.accepted:
            self.get_logger().info("Action goal rejected.")
            self.action_run_state = False
            return

        # 비동기적으로 결과를 받아오고 기능이 끝나면 다음 함수 실행
        self.action_result_future = self.goal_handle.get_result_async()
        self.action_result_future.add_done_callback(self.navigate_to_pose_action_result)
        
    def navigate_to_pose_action_feedback(self, feedback_msg):
        if self.patrol_status is True:
            self.current_waypoint = feedback_msg.feedback.current_waypoint
        # feedback log 필요할 때 주석해제
        # self.get_logger().info("Action feedback: {0}".format(action_feedback))
        
    def navigate_to_pose_action_result(self, future):
        action_status = future.result().status
        
        # 액션 결과 출력 및 인덱스 갱신
        if action_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Action succeeded!.")
        else:
            self.get_logger().info(f"Action failed with status: {action_status}")   
            
        self.action_run_state = False               
        if self.patrol_status is True:
            self.way_point_send_goal(self.patrol_position_list)
        
def main(args =None):
    rclpy.init(args=args)
    AMR_node = AMR_move_node()
    
    executor = MultiThreadedExecutor()
    executor.add_node(AMR_node)
    
    executor.spin()
    
    AMR_node.destroy_node()
    rclpy.shutdown()         
        
if __name__ == '__main__':
    main()