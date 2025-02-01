import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Float64MultiArray
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Point

#import pygame

# # 010 4375 8838
# # ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
# # ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/home/inyoung/map.yaml
# # 0.027 0.003 0.0 1.0 0.197 0.164 0.069
# # bool값 = 액션상태, 순찰상태, gui detection 상태, amr detection 상태
class amr_control_node(Node):
    def __init__(self):
        super().__init__('AMR_move_node') 
        
        # nav2의 초기 위치를 알려주는 퍼블리셔
        self.nav2_initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # gui의 상태를 받아오는 서브스크라이버 
        self.gui_status = self.create_subscription(String, '/gui_status', self.gui_status_callback, 10)
        
        # gui에서 detection 결과를 받아오는 서브스크라이버 
        self.gui_detection_result = self.create_subscription(Float64MultiArray, '/web_car_position', self.gui_detection_result_callback, 10)
        
        # AMR에서 detection 결과를 받아오는 서브스크라이버
        self.AMR_detection_result = self.create_subscription(String, '/amr_result', self.AMR_detection_result_callback, 10)
        
        # Nav2 pose 클라이언트
        self.nav2_pose_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        
        # Nav2 Waypoint 클라이언트
        self.nav2_waypoint_client = ActionClient(self, FollowWaypoints, "/follow_waypoints")
        
        # waypoint 좌표 설정, 좌표를 다시 따야함 구석부분 말고 좀 넓은 통로를 찍자
        self.home_position = [0.065, 0.068, 0.0, 1.0] #[1.0, 1.5, 0.0, 1.0] [0.027, 0.003, 0.0, 1.0]
        self.way_point1 = [0.27, -0.67, -1.0, 0.0]
        self.way_point2 = [-0.5, -0.67, -1.0, 0.0]
        self.way_point3 = [-0.7, -0.3, 0.809, 0.588]
        self.way_point4 = [-1.0, -0.13, 1.0, 0.107]
        self.way_point5 = [-1.4, -0.274, -0.71, 0.703]
        self.way_point6 = [-0.67, -0.15, 0.1, 1.0]        
        self.way_point7 = [-0.67, -0.67, -0.652, 0.758]
        self.way_point8 = [-0.32, -0.67, -0.108, 1.0]
        self.way_point9 = [0.27, -0.67, -0.681, 0.732]

        self.patrol_position_list = [
                self.set_waypoints(self.way_point1),
                self.set_waypoints(self.way_point2),
                self.set_waypoints(self.way_point3),
                self.set_waypoints(self.way_point4),
                self.set_waypoints(self.way_point5),
                self.set_waypoints(self.way_point6),
                self.set_waypoints(self.way_point7),
                self.set_waypoints(self.way_point8),
                self.set_waypoints(self.way_point9)
            ]
        
        # 상태를 나타내는 bool값, 
        self.action_run_state = False
        self.patrol_status = False
        self.AMR_detection_result = False
        self.gui_detection_result = False
        self.set_nav2_initial_pose(self.home_position)
        
    def set_nav2_initial_pose(self, pose_list):

        # header:
        #     stamp:
        #         sec: 1735614811
        #         nanosec: 951823634
        #     frame_id: map
        #         pose:
        #         pose:
        #             position:
        #                 x: 1.1817682981491089
        #                 y: 1.4633934497833252
        #                 z: 0.0
        #                 orientation:
        #                 x: 0.0
        #                 y: 0.0
        #                 z: 0.0049979179212718845
        #                 w: 0.9999875103302301
        #             covariance:
        #                 - 0.25
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.25
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.0
        #                 - 0.06853891909122467

        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        # 초기위치 설정
        initial_pose.pose.pose.position = Point(
            x=pose_list[0],
            y=pose_list[1],
            z=0.0
        )

        # 회전 각도 설정
        initial_pose.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=pose_list[2],
            w=pose_list[3]
            )

        # 불확실성 행렬 설정
        initial_pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
        ]
        
        # 퍼블리시
        self.nav2_initial_pose_pub.publish(initial_pose)
        self.get_logger().info('Initial pose published.')

    def set_waypoints(self, pose_list):
        way_point = PoseStamped()
        way_point.header.frame_id = 'map'
        way_point.pose.position.x = pose_list[0]
        way_point.pose.position.y = pose_list[1]
        way_point.pose.position.z = 0.0
        way_point.pose.orientation.x = 0.0
        way_point.pose.orientation.y = 0.0
        way_point.pose.orientation.z = pose_list[2]
        way_point.pose.orientation.w = pose_list[3]
        return way_point
 
    def gui_status_callback(self, gui_status_msg):
        self.gui_status = gui_status_msg.data
        if self.AMR_detection_result is False:
            if self.gui_status == 'close' and self.patrol_status is False:
                if self.action_run_state is True:
                    self.nav2_waypoint_client._cancel_goal_async(self.goal_handle)
                    
                print('start patrol')
                self.patrol_status = True
                self.way_point_send_goal(self.patrol_position_list)
            
            elif self.gui_status == 'open':
                print('go to home')
                if hasattr(self, 'current_waypoint') and type(self.current_waypoint) is int:
                    self.nav2_waypoint_client._cancel_goal_async(self.goal_handle)
                    self.patrol_status = False
                    
                    way_points = self.patrol_position_list[self.current_waypoint:-1]
                    way_points.append(self.set_waypoints(self.home_position))
                    self.way_point_send_goal(way_points)  
                    self.current_waypoint = None
       
    def gui_detection_result_callback(self, gui_result_msg):
        detection_result = gui_result_msg.data
        if detection_result and self.gui_detection_result is False:
            self.gui_detection_result = True
            self.patrol_status = False
            
            if self.action_run_state is True:
                self.nav2_waypoint_client._cancel_goal_async(self.goal_handle)
            
            target_pose = self.way_point5    
            self.target_send_goal(target_pose)
            self.current_waypoint = 3
            
    def AMR_detection_result_callback(self, msg):
        amr_detecting_result = msg.data
        if amr_detecting_result == 'detected':
            # AMR이 침입자를 발견하지 못한 상태라면
            if self.AMR_detection_result is False:
                # 액션이 돌아가는 상태라면
                if self.action_run_state is True:
                    self.nav2_waypoint_client._cancel_goal_async(self.goal_handle)

                # AMR에서 침입자 감지상태로 변경
                self.AMR_detection_result = True
                # 순찰 종료
                self.patrol_status = False
                #self.warning_sound_timer = self.create_timer(3, self.warning_sound_callback)
            
    #def warning_sound_callback(self):
        #pygame.mixer.init()
        #pygame.mixer.music.load('/home/inyoung/wlsmd2/src/project/alarm.mp3')  
        #pygame.mixer.music.play()
        
    def way_point_send_goal(self, way_point_list):
        # 액션 시작 
        self.action_run_state = True
        print('waypoint_start')
        
        # 액션 서버와 연결이 되는지 확인
        wait_count = 1
        while not self.nav2_waypoint_client.wait_for_server(timeout_sec=0.1):
            if wait_count > 3:
                self.get_logger().info('Navigate action server is not available.')
                # 액션 종료
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
        # 액션 시작
        self.action_run_state = True
        # 순찰 종료
        self.patrol_status = False
        print('pose_start')
        
        # 액션 서버와 연결이 되는지 확인
        wait_count = 1
        while not self.nav2_waypoint_client.wait_for_server(timeout_sec=0.1):
            if wait_count > 3:
                self.get_logger().info('Navigate action server is not available.')
                # 액션 종료
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
            # 액션 종료
            self.action_run_state = False
            return

        # 비동기적으로 결과를 받아오고 기능이 끝나면 다음 함수 실행
        self.action_result_future = self.goal_handle.get_result_async()
        self.action_result_future.add_done_callback(self.navigate_to_pose_action_result)
        
    def navigate_to_pose_action_feedback(self, feedback_msg):
        # 순찰 중인 상태라면
        if self.patrol_status is True:
            self.current_waypoint = feedback_msg.feedback.current_waypoint
            print(self.current_waypoint)
        # feedback log 필요할 때 주석해제
        # self.get_logger().info("Action feedback: {0}".format(action_feedback))
        
    def navigate_to_pose_action_result(self, future):
        action_status = future.result().status
        
        # 액션 결과 출력 및 인덱스 갱신
        if action_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Action succeeded!.")
        else:
            self.get_logger().info(f"Action failed with status: {action_status}")   
          
        # 액션 종료  
        self.action_run_state = False
        # 순찰 중인 상태라면               
        if self.patrol_status is True:
            self.way_point_send_goal(self.patrol_position_list)
        
def main(args =None):
    rclpy.init(args=args)
    AMR_node = amr_control_node()
    
    executor = MultiThreadedExecutor()
    executor.add_node(AMR_node)
    executor.spin()
    
    AMR_node.destroy_node()
    rclpy.shutdown()         
        
if __name__ == '__main__':
    main()
