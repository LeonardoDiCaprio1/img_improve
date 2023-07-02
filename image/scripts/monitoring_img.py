import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import CompressedImage,Image

class ImageTransferRate:
    def __init__(self):
        self.image_count = 0
        self.start_time = rospy.Time.now()

        self.publisher = rospy.Publisher('/trigger_image_publish', Empty, queue_size=1)
        self.rate = rospy.Rate(0.2)  # 每5秒计算一次传输速率

        rospy.Timer(rospy.Duration(1.0), self.publish_image) 

    def publish_image(self, event):
        if self.image_count > 0:
            self.publisher.publish(Empty())

    def image_callback(self, msg):
        self.image_count += 1

    def compute_transfer_rate(self):
        elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
        transfer_rate = self.image_count / elapsed_time 
        rospy.loginfo(f"Image transfer rate: {transfer_rate} images/s at {rospy.get_time()}")
        self.image_count = 0  # 重置计数器
        self.start_time = rospy.Time.now()  # 重置起始时间

    def run(self):
        while not rospy.is_shutdown():
            self.compute_transfer_rate()
            self.rate.sleep()

def main():
    rospy.init_node('image_transfer_rate', anonymous=True)
    image_rate = ImageTransferRate()
    rospy.Subscriber('/camera/rgb/image_raw', Image, image_rate.image_callback)
    # rospy.Subscriber('/compressed_camera_topic', CompressedImage, image_rate.image_callback)
    image_rate.run()
    rospy.spin()

if __name__ == '__main__':
    main()
