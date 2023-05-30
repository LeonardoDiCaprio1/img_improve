#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import threading

import SophonModel
def compress_image(image):
    
    _, compressed_image = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 50])
    return compressed_image

class ImageProcessor:
    def __init__(self, bmodel_path, labels_path):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.cv_image = None
        self.running = True
        # init infer engine
        # self.engine = SophonModel.BmodelEngine(bmodel_path, labels_path)
        # self.avg = None
        
    def process_image(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
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
                    compressed_image = compress_image(self.cv_image)
                    msg = CompressedImage()
                    msg.format = 'jpeg'
                    msg.data = compressed_image.tobytes()  
                    publisher.publish(msg)
            rate.sleep()

    def shutdown(self):
        self.running = False

def image_publisher():
    rospy.init_node('image_publisher', anonymous=True)

    processor = ImageProcessor()

    rospy.Subscriber('/camera/color/image_raw', Image, processor.process_image)

    compress_thread = threading.Thread(target=processor.compress_and_publish)
    compress_thread.start()

    rospy.on_shutdown(processor.shutdown)
    rospy.spin()

    compress_thread.join()
    # ic = ImageProcessor(bmodel_path="/home/linaro/test_ws/src/SogoBot/data/ssd-vgg-300x300-int8-1b.bmodel",
    #                     labels_path = "/home/linaro/test_ws/src/SogoBot/data/voc_labels.txt")
    
if __name__ == '__main__':
    image_publisher()
