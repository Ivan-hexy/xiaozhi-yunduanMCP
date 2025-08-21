#!/usr/bin/python3
# coding=UTF-8
import rospy
from tf2_msgs.msg import TFMessage

class TF1ToTFForwarder:
    def __init__(self):
        # 初始化节点
        rospy.init_node('tf1_to_tf_forwarder', anonymous=True)
        
        # 创建发布者，发布到/tf话题
        self.tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=10)
        
        # 订阅/tf1话题
        self.tf1_sub = rospy.Subscriber('/tf1', TFMessage, self.tf1_callback)
        
        rospy.loginfo("TF1 to TF forwarder node initialized. Forwarding /tf1 to /tf.")

    def tf1_callback(self, msg):
        """回调函数：将接收到的/tf1消息转发到/tf"""
        # 直接发布接收到的消息到/tf话题
        self.tf_pub.publish(msg)

if __name__ == '__main__':
    try:
        # 创建转发器实例并保持运行
        forwarder = TF1ToTFForwarder()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TF1 to TF forwarder node terminated.")
