import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__("simple_publisher") # name for the node "simple_publisher"
        self.pub_=self.create_publisher(String, "chatter", 10) # type of msg, topic name, queue size
        self.counter_=0  #it will count the msg will send
        self.frequency_=1.0 #frequency at which will publish msg that is 1Hz or 1ms
        self.get_logger().info("Publishing at %d Hz" %self.frequency_) #it will publ info() on terminal
        self.timer_=self.create_timer(self.frequency_, self.timerCallback) # it will call the timerCallback function at provided frequency countinously  

    def timerCallback(self):  # this function goal is to print msg everytime it called
        msg = String()  # object of String
        msg.data="hello ROS2 - counter: %d" % self.counter_  #msg we want to print/publish on each call
        self.pub_.publish(msg)  # publishing the msg

        self.counter_+=1    # incrementing the counter

def main():
    rclpy.init()
    simple_pub = SimplePublisher()  #creating the object of the class

    #to keep the timer & node/publisher active
    rclpy.spin(simple_pub)

    #to terminate the node if we press ctrl+c in the terminal
    simple_pub.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()