#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
import cv2
import threading
import numpy as np

def compress_image(image):
    # 压缩图像为JPEG格式
    _, compressed_image = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return compressed_image

class ImageProcessor:
    def __init__(self):
        self.lock = threading.Lock()
        self.cv_image = None
        self.running = True

    def process_image(self, msg):
        try:
        # 将图像消息转换为numpy数组
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
            with self.lock:
                self.cv_image = cv_image
        except Exception as e:
             rospy.logerr(e)

    def compress_and_publish(self):
        rate = rospy.Rate(30)
        publisher = rospy.Publisher('/compressed_camera_topic', CompressedImage, queue_size=1)

        while self.running and not rospy.is_shutdown():
            with self.lock:
                if self.cv_image is not None:
                # 压缩图像
                    compressed_image = compress_image(self.cv_image)
                    msg = CompressedImage()
                    msg.format = 'jpeg'
                    msg.data = compressed_image.tobytes()
                    publisher.publish(msg)
                rate.sleep()

    def shutdown(self):
        self.running = False

def image_publisher():
    # 初始化ROS节点
    rospy.init_node('image_publisher', anonymous=True)

    processor = ImageProcessor()
    # 订阅原始相机图像话题
    rospy.Subscriber('/camera/rgb/image_raw', Image, processor.process_image)
    # 创建并启动压缩和发布线程
    compress_thread = threading.Thread(target=processor.compress_and_publish)
    compress_thread.start()

    rospy.on_shutdown(processor.shutdown)
    rospy.spin()

    compress_thread.join()

if __name__ == '__main__':
    image_publisher()
