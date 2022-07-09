import rospy
# import a1_slam
# from test_import import Foo
from registration.Optimizer import Optimizer
def foo():
    rospy.init_node('test', anonymous=True)
    optim = Optimizer()
    print("optim imported")


if __name__ == "__main__":
    try:
        foo()
    except rospy.ROSInterruptException:
        pass
