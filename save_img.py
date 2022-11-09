#!/usr/bin/env python

"""Extract images from a rosbag.
"""

import os
import argparse
import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def rostime2floatSecs(rostime):
    return rostime.secs + (rostime.nsecs / 1000000000.0)

def main():    
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("--bag_file", help="Input ROS bag.", default="2022-11-01-17-43-56.bag")
    parser.add_argument("--out_dir", help="Output directory.", default="imgs/")
    parser.add_argument("--sampling", type=int, help="downsampling image per this value", default=10)

    args = parser.parse_args()
    bag_file = args.bag_file #"_2019-03-27-22-30-28.bag" 
    output_dir = args.out_dir #"extractedImages/"
    # compressed = args.compressed #true false
    
    image_topic = "/teli_camera/image_raw/compressed"
    # image_topic = "/camera/image_color"
    
    jpg_quality = 100
    compr = cv2.IMWRITE_JPEG_QUALITY
    quality = jpg_quality  # jpg quality is in [0,100] range, png [0,9]
    params = [compr, quality]
    
    bag = rosbag.Bag(bag_file, "r")
    bridge = CvBridge()

    print("Save images...")
    count = 0
    denom = args.sampling
    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        if count % denom == 0:
            # cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_img = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            if cv_img is None:
                print("nnonon")
                continue
            print(topic, t, cv_img)
            picSaveName = "{}_{}.jpg".format(msg.header.stamp.secs, msg.header.stamp.nsecs)
            print(picSaveName)
            picPath = os.path.join(output_dir, picSaveName)
            cv2.imwrite(picPath, cv_img, params)        
        count += 1
    bag.close()
    
    return

if __name__ == '__main__':
    main()
