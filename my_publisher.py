import rclpy as rp
from rclpy.node import Node

from geometry_msgs.msg import Twist # 'geometry_msgs' 패키지에서 'Twist' 메시지 타입을 불러옴. 'Twist' 메시지는 선형 속도와 각속도를 포함하는 데이터 구조.

class TurtlesimPublisher(Node): # 'Node' 클래스를 상속받아 'TurtlesimPublisher'라는 새로운 클래스를 정의.
    
    def __init__(self): # 생성자 메서드로써
        super().__init__('turtlesim_publisher') # 부모 클래스(Node)의 생성자를 호출. 'Node' 클래스의 생성자는 노드의 이름을 인자로 받음. 여기서 노드의 이름을 'turtlesim_publisher'로 설정.
        
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) 
        # 노드에서 'create_publisher' 메서드를 사용하여, 토픽에 'Twist' 메시지 타입을 발행하는 퍼블리셔를 생성.
        # '/turtle1/cmd_vel'이름의 토픽에 발행할 거임. 이 토픽은 터틀심패키지에 사전 정의된 토픽으로써,  ros2 topic list -t로 검색가능.
        # 10은 큐 사이즈
            
        timer_period = 0.5 # 주기 0.5초
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        # 'create_timer' 메서드를 사용하여 주기가 'timer_period'인 타이머를 생성합니다. ROS2에서는 타이머를 생성하기만 하면 호출하지 않아도 자동으로 작동한다.
        # 주기마다 콜백함수 호출.
        
    def timer_callback(self): # 콜백 함수
        msg = Twist() # 'Twist' 메시지 타입의 새로운 객체를 생성.
        msg.linear.x = 2.0
        msg.angular.z = 2.0
        self.publisher.publish(msg) # 수정한 'msg' 메시지를 퍼블리셔를 통해 발행.


if __name__ == '__main__': # 이 파일을 직접실행하면
    main() # main 함수를 호출.

def main(args=None): # main함수가 호출될 때 인자로 받을 수 있는 옵션인데, 지금은 None을 값으로 받았기 때문에 특별한 설정 없이 기본적인 ROS2 초기화가 진행됨.
    rp.init(args=args)  # rp.init()은 rp클래스를 초기화. 그리고 rp.init(args=args)는 앞서 전달받은 args값에 따라서 rp클래스를 초기화. 예를 들어서 args값으로 DEBUG를 받았다면, ROS2가 디버그 모드로 설정됨.
    
    turtlesim_publisher = TurtlesimPublisher() # 'TurtlesimPublisher' 클래스의 인스턴스를 생성.
    rp.spin(turtlesim_publisher) # 무한스핀
    
    turtlesim_publisher.destroy_node() # 인스턴스의 작업이 멈추고 노드를 파괴
    rp.shutdown() # 셧다운
