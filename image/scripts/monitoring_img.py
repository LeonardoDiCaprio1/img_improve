import rospy
from std_msgs.msg import Empty
from time import time
from sensor_msgs.msg import Image,CompressedImage

class ImageTransferRate:
    def __init__(self):
        self.image_count = 0
        self.start_time = time()

        self.publisher = rospy.Publisher('/trigger_image_publish', Empty, queue_size=1)
        # 每秒钟发布一次消息
        rospy.Timer(rospy.Duration(1.0), self.publish_image) 

    def publish_image(self, event):
        self.publisher.publish(Empty())

    def image_callback(self, msg):
        self.image_count += 1

    def compute_transfer_rate(self):
        elapsed_time = time() - self.start_time
        # 传输速率 = 图像计数 / 经过的时间
        transfer_rate = self.image_count / elapsed_time 
        rospy.loginfo(f"Image transfer rate: {transfer_rate} images/s")

def main():
        rospy.init_node('image_transfer_rate', anonymous=True)
        image_rate = ImageTransferRate()
        #rospy.Subscriber('/camera/color/image_raw', Image, image_rate.image_callback)
        rospy.Subscriber('/compressed_camera_topic', CompressedImage, image_rate.image_callback)
         # 每5秒钟计算一次传输速率
        rospy.Timer(rospy.Duration(5.0), lambda event: image_rate.compute_transfer_rate())
        rospy.spin()

if __name__ == '__main__':
    main()
