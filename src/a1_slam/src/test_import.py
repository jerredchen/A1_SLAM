import rospy
from std_msgs.msg import String

class Foo:
    def __init__(self) -> None:
        self.pub = rospy.Publisher('test', String, queue_size=5)