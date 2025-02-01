import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from rclpy.executors import MultiThreadedExecutor

class test_amr(Node):
    def __init__(self):
        super().__init__('test_amr')
        
        self.pub = self.create_publisher(String, '/amr_result', 10)
        self.pub_timer = self.create_timer(0.1, self.pub_callback)
        
    def pub_callback(self):
        msg = String()
        msg.data = 'detected'
        self.pub.publish(msg)
        
def main(args =None):
    rclpy.init(args=args)
    AMR_node = test_amr()
    
    executor = MultiThreadedExecutor()
    executor.add_node(AMR_node)
    
    executor.spin()
    
    AMR_node.destroy_node()
    rclpy.shutdown()         
        
if __name__ == '__main__':
    main()
    