#!/usr/bin/env python3

import rospy
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray, Bool
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

class ObjectDetectionNode:
    """
    ROS node for object detection using a YOLOv8 model.
    Subscribes to ZED camera feed, detects objects, and publishes detection results,
    bounding boxes, and annotated images.
    """

    def __init__(self):
        # Topics
        self.image_topic = "/zed_node/rgb/image_rect_color"
        self.info_topic = "/zed_node/rgb/camera_info"

        # ROS Setup
        rospy.Subscriber(self.image_topic, Image, self.image_callback)
        rospy.Subscriber(self.info_topic, CameraInfo, self.info_callback)

        self.mallet_pub = rospy.Publisher('mallet_detected', Bool, queue_size=1)
        self.waterbottle_pub = rospy.Publisher('waterbottle_detected', Bool, queue_size=1)
        self.bbox_pub = rospy.Publisher('object/bbox', Float64MultiArray, queue_size=10)
        self.vis_pub = rospy.Publisher('vis/object_detections', Image, queue_size=10)

        self.bridge = CvBridge()
        self.K = None
        self.D = None

        # Load YOLO model
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(script_dir, 'best.pt')
        self.model = YOLO(model_path)
        self.model.conf = 0.5  # Confidence threshold

        # Class map (update based on model training)
        self.class_map = {0: "mallet", 1: "waterbottle"}

        rospy.loginfo("YOLO Object Detection Node Initialized.")

    def info_callback(self, info_msg):
        """Camera intrinsic and distortion parameters callback."""
        self.D = np.array(info_msg.D)
        self.K = np.array(info_msg.K).reshape(3, 3)

    def image_callback(self, ros_image):
        """Convert ROS image to OpenCV and run detection."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return

        self.detect_objects(cv_image)

    def detect_objects(self, img):
        """Run YOLO detection on the image, visualize, and publish results."""
        if self.model is None:
            rospy.logerr("YOLO model not loaded.")
            return

        results = self.model(img)

        mallet_found = False
        waterbottle_found = False

        for result in results:
            detections = result.boxes

            for box in detections:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = box.conf[0]
                cls = int(box.cls[0])

                if cls not in self.class_map:
                    continue

                obj_name = self.class_map[cls]
                area = (x2 - x1) * (y2 - y1)

                # Publish bounding box data
                bbox_msg = Float64MultiArray(data=[x1, y1, x2, y2, area, cls])
                self.bbox_pub.publish(bbox_msg)

                # Draw bounding box and label
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{obj_name} {conf:.2f}"
                cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Set detection flags
                if obj_name == "mallet":
                    mallet_found = True
                elif obj_name == "waterbottle":
                    waterbottle_found = True

        # Publish detection status
        self.mallet_pub.publish(Bool(data=mallet_found))
        self.waterbottle_pub.publish(Bool(data=waterbottle_found))

        # Publish visualization image
        vis_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.vis_pub.publish(vis_msg)

def main():
    rospy.init_node('object_detector')
    node = ObjectDetectionNode()
    rospy.spin()

if __name__ == "__main__":
    main()