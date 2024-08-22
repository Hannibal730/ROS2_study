## 참고사항: turtlesim 노드는 우리의 설정과 무관하게 실행과 동시에 pose토픽을 발행하고, cmd_vel토픽을 구독한다.
## 참고사항: my_publiser노드가 cmd_vel토픽을 구독하도록 우리가 만들어둠.
## 목표1: turtlesim 노드가 발행하는 pose토픽을 구독하되, 우리가 원하는 데이터타입으로 받는다.
## 목표2: my_publisher 노드가 발행하는 cmd_vel토픽을 구독하되, 우리가 원하는 데이터타입으로 받는다.


import rclpy as rp
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from my_first_package_msgs.msg import CmdAndPoseVel #우리가 임의로 정의했던 메시지타입인 CmdAndPoseVel.msg 불러오기.

# 1.
# 우리의 목표1은 {터틀심 노드가 발행하는 Pose 토픽의 메시지타입}을 받아서, {우리가 정의한 메시지타입}의 변수에 저장하기. 
# 우리의 목표2는 {my_publisher 노드가 발행하는 cmd_vel 토픽의 메시지타입}을 받아서, {우리가 정의한 메시지타입}의 변수에 저장하기.

# 2.
# 우리가 정의한 메시지타입인 CmdAndPoseVel.msg이다.
# float32 pose_x
# float32 pose_y
# float32 linear_vel
# float32 angular_vel
# float32 cmd_vel_linear
# float32 cmd_vel_angular


# 3.
# 터틀심 노드가 발행하는 turtle1/pose 토픽의 메시지 타입 이름은 turtlesim/msg/Pose이고, 그 내용은 
# x
# y
# theta
# linear_velocity
# angular_velocity 이다.

# x -> pose_x
# y -> pose_y
# theta
# linear_velocity -> linear_vel
# angular_velocity -> angular_vel 으로 받을 거다.

# 4.
# my_publisher노드가 발행하는 turtle1/cmd_vel 토픽의 메시지타입 이름은 turtlesim/msg/Twist이고, 그 내용은
# linear벡터 [x,y,z]
# angular 벡터 [x,y,z] 이다.

# linear벡터 x -> cmd_vel_linear
# angular 벡터 z -> cmd_vel_angular


class CmdAndPose(Node):
    def __init__(self):
        super().__init__('turtle_cmd_pose') # 부모 클래스(Node)의 생성자를 호출. 'Node' 클래스의 생성자는 노드의 이름을 인자로 받음. 여기서 노드의 이름을 'turtlesim_publisher'로 설정.

        
        self.cmd_pose = CmdAndPoseVel()
        # CmdAndPoseVel 타입의 변수 cmd_pose를 생성.
        # my_publisher에서의 msg = Twist()처럼 내가 원하는 타입의 변수 생성한 거임.
        
        
        self.publisher = self.create_publisher(CmdAndPoseVel, '/cmd_and_pose',10)
        # CmdAndPoseVel 타입의 /cmd_ and_pose토픽을 발행 가능하게 함.
        # 이제 self.publisher.publish('CmdAndPoseVel타입의 변수') 해당 변수내용을 토픽으로 발행가능하다.
        
        
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.callback_pose, 10)
        # 구독할 토픽의 데이터 타입은 pose
        # 구독할 토픽의 이름은 /turtle1/pose
        # 콜백함수로 callback_pose를 지정함.
        
        self.sub_cmdvel = self.create_subscription(Twist, '/turtle1/cmd_vel',self.callback_cmd, 10)
        # 구독할 토픽의 데이터 타입은 Twist
        # 구독할 토픽의 이름은 /turtle1/cmd_vel
        # 콜백함수로 callback_cmd를 지정함.
        
        
        
        
        self.timer_period = 1.
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        # 'create_timer' 메서드를 사용하여 주기가 'timer_period'인 타이머를 생성합니다. ROS2에서는 타이머를 생성하기만 하면 호출하지 않아도 자동으로 작동한다.
        # 주기마다 콜백함수 호출.


## 전체 과정 설명
# CmdAndPose클래스의 던더로 CmdAndPseVel타입의 변수 cmd_pose를 생성함.
# CmdAndPose클래스의 던더로 CmdAndPoseVel타입의 변수를 /cmd_and_pose 이름의 토픽으로 발행가능해짐.
# CmdAndPose클래스의 던더로 pose토픽과 cmd_vel토픽을 구독함.
# 구독 과정에서의 콜백함수 때문에 cmd_pose의 변수들이 pose토픽의 데이터값과 cmd_vel토픽의 데이터값을 받게 됨.
# CmdAndPose클래스의 던더로 timer가 생성됐는데, ROS2에서 타이머는 생성과 동시에 작동함.
# 타이머의 콜백함수 때문에 CmdAndPoseVel타입의 변수 cmd_pose를 /cmd_and_pose 이름의 토픽으로 발행.


    def callback_pose(self, msg):
    # pose토픽을 구독할 때의 콜백함수를 정의.
    # 우리는 위에서 구독할 토픽의 데이터타입을 pose로 지정했다.
    # 이럴 경우, 콜백함수의 인자로 받는 msg는 turtlesim/msg/Pose타입의 객체이며 pose토픽이 보내주는 데이터이다.
#   참고로 이때의 msg는 그냥 우리가 마음대로 이름 붙인 것으로, turtlesim패키지의 msg와 다르다.
        self.cmd_pose.pose_x = msg.x
        # 이때의 msg는 콜백함수의 인자로 받은 msg.
        # 따라서 메시지타입이 pose인 데이터msg의 x, y, linear_velocity, angular_velocity 값 중 x를 받아온 것을 의미.
        self.cmd_pose.pose_y = msg.y
        self.cmd_pose.linear_vel = msg.linear_velocity
        self.cmd_pose.angular_vel = msg.angular_velocity
        print(self.cmd_pose)
    def callback_cmd(self, msg):
    # cmd_vel 토픽을 구독할 때의 콜백함수를 정의.
    # 이번에는 콜백함수의 인자로 받는 msg는 turtlesim/msg/Twist타입의 객체이겠지? 직전의 설명과 비슷.
        self.cmd_pose.cmd_vel_linear = msg.linear.x
        self.cmd_pose.cmd_vel_angular = msg.angular.z
        print(self.cmd_pose)


    def timer_callback(self):
        self.publisher.publish(self.cmd_pose)
        
        
def main(args=None):
    rp.init(args=args)
    
    turtle_cmd_pose_node = CmdAndPose()
    rp.spin(turtle_cmd_pose_node)
    
    turtle_cmd_pose_node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()