import rclpy as rp
from rclpy.node import Node  # ROS2에서 노드를 정의할 때 자주 사용. 원하는 클래스가 이 노드를 상속받아 사용함.
from turtlesim.msg import Pose  # 터틀심 패키지의 Pose라는 메시지 타입 가져오기.

class Turtlesimsubscriber(Node):  # 'Node' 클래스를 상속받아 'Turtlesimsubscriber'라는 클래스를 정의
    def __init__(self):  # 생성자 메서드로써
        super().__init__('turtlesim_subscriber')  # 부모 클래스(Node)의 생성자를 호출, Node클래스의 생성자의 첫 변수는 노드의 이름을 받는다. Node클래스의 인스턴스는 노드의 참조변수이고, 생성자의 변수로 받아진 이름이 노드의 이름이다. 이때 노드 이름을 'turtlesim_subscriber'로 설정
        self.subscription = self.create_subscription(
            Pose,  # 구독할 토픽의 메시지의 데이터타입
            '/turtle1/pose',  # 구독할 토픽 이름. 참고로 해당 토픽은 거북이의 x,y축 위치 좌표를 발행하는 토픽이다.
            self.callback,  # 메시지를 받았을 때 호출될 콜백 함수
            10)  # 큐 사이즈
        self.subscription  # 만약 self.subscription이라는 변수가 전체 코드 실행시 사용되지 않을 경우에는 코드 편집기가 "사용되지 않은 변수 있음"이라며 경고를 보낸다. 이 경고를 방지하고자 그냥 쓴 거임.

    def callback(self, msg):  # 콜백함수 정의. 토픽에서 전달되는 데이터는 자동으로 콜백함수의 인자로 전달된다. 따라서 이때의 msg는 정해진 단어가 아니다. msg는 Pose타입인 임의의 객체일 뿐이다.
        print("X: ", msg.x, "Y: ", msg.y)

def main(args=None):  # main함수가 호출될 때 인자로 받을 수 있는 옵션인데, 지금은 None을 값으로 받았기 때문에 특별한 설정 없이 기본적인 ROS2 초기화가 진행됨.
    rp.init(args=args)  # rp.init()은 rp클래스를 초기화. 그리고 rp.init(args=args)는 앞서 전달받은 args값에 따라서 rp클래스를 초기화. 예를 들어서 args값으로 DEBUG를 받았다면, ROS2가 디버그 모드로 설정됨.
    turtlesim_subscriber = Turtlesimsubscriber()  # 위에서 정의한 Turtlesimsubscriber클래스의 인스턴스를 생성
    rp.spin(turtlesim_subscriber)  # 그 인스턴스를 무한 스핀
    turtlesim_subscriber.destroy_node()  # 인스턴스의 작업이 멈추고 노드를 파괴
    rp.shutdown()  # ROS2 셧다운

if __name__ == '__main__':  # 전체 코드를 직접 실행하면
    main()  # 메인함수가 호출됨
