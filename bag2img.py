#!/usr/bin/env python

"""Extract images from a rosbag.
"""

import os
import argparse
import cv2

import rosbag
from sensor_msgs.msg import Image, NavSatFix, TimeReference
from cv_bridge import CvBridge
#for testing
#import std_msgs
#import rospy

import piexif
from fractions import Fraction
from datetime import datetime

def to_deg(value, loc):
    """convert decimal coordinates into degrees, munutes and seconds tuple
    Keyword arguments: value is float gps-value, loc is direction list ["S", "N"] or ["W", "E"]
    return: tuple like (25, 13, 48.343 ,'N')
    """
    if value < 0:
        loc_value = loc[0]
    elif value > 0:
        loc_value = loc[1]
    else:
        loc_value = ""
    abs_value = abs(value)
    deg =  int(abs_value)
    t1 = (abs_value-deg)*60
    min = int(t1)
    sec = round((t1 - min)* 60, 5)
    return (deg, min, sec, loc_value)


def change_to_rational(number):
    """convert a number to rantional
    Keyword arguments: number
    return: tuple like (1, 2), (numerator, denominator)
    """
    f = Fraction(str(number))
    return (f.numerator, f.denominator)


def set_gps_location(file_name, lat, lng, altitude, gpsTime):
    """Adds GPS position as EXIF metadata
    Keyword arguments:
    file_name -- image file
    lat -- latitude (as float)
    lng -- longitude (as float)
    altitude -- altitude (as float)
    """
    lat_deg = to_deg(lat, ["S", "N"])
    lng_deg = to_deg(lng, ["W", "E"])

    exiv_lat = (change_to_rational(lat_deg[0]), change_to_rational(lat_deg[1]), change_to_rational(lat_deg[2]))
    exiv_lng = (change_to_rational(lng_deg[0]), change_to_rational(lng_deg[1]), change_to_rational(lng_deg[2]))

    gps_ifd = {
        piexif.GPSIFD.GPSVersionID: (2, 0, 0, 0),
        piexif.GPSIFD.GPSAltitudeRef: 0,
        piexif.GPSIFD.GPSAltitude: change_to_rational(round(altitude + 50)),
        piexif.GPSIFD.GPSLatitudeRef: lat_deg[3],
        piexif.GPSIFD.GPSLatitude: exiv_lat,
        piexif.GPSIFD.GPSLongitudeRef: lng_deg[3],
        piexif.GPSIFD.GPSLongitude: exiv_lng,
        piexif.GPSIFD.GPSDateStamp: gpsTime
    }

    exif_dict = {"GPS": gps_ifd}
    exif_bytes = piexif.dump(exif_dict)
    piexif.insert(exif_bytes, file_name)



def getNearestData(data,stamp):
    """Get nearest datapoint for query stamp from a dataarray with stamps
    Keyword arguments:
    data -- a dataarray to query from
    stamp -- query stamp
    """
    i = 0
    for oneData in data:       
        if stamp.to_time() < oneData.header.stamp.to_time() and i > 0:
            deltaPrev = abs(data[i-1].header.stamp - stamp)
            deltaNext= abs(data[i].header.stamp - stamp)
            if deltaPrev < deltaNext:
                return data[i-1]
            else:
                return data[i]              
        i=i+1
    return data[-1]

def rostime2floatSecs(rostime):
    return rostime.secs + (rostime.nsecs / 1000000000.0)

def main():
    """Extract a folder of images with metadata from a rosbag.
    """
    
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("compressed", help="image compressed \"1\" or not \"0\"")

    args = parser.parse_args()
    bag_file = args.bag_file #"_2019-03-27-22-30-28.bag" 
    output_dir = args.output_dir #"extractedImages/"
    compressed = args.compressed #true false
    
    if compressed=="1":
        image_topic = "/teli_camera/image_raw/compressed"
        # "/toshiba/usb/image_raw/compressed"
    else:
        image_topic = "/camera/image_color"
        
    gpsPos_topic = "/dji_osdk_ros/gps_position"
# "/mavros/global_position/global"	# /mavros/global_position/raw/fix
    gpsTime_topic = "/mavros/time_reference"
    useGpsTime = False
    
    jpg_quality = 100
    compr = cv2.IMWRITE_JPEG_QUALITY
    quality = jpg_quality  # jpg quality is in [0,100] range, png [0,9]
    params = [compr, quality]
    
    bag = rosbag.Bag(bag_file, "r")
    bridge = CvBridge()
    
    print("Parse GPS Data...")
    gpsPosData=[]
    for topic, msg, t in bag.read_messages(topics=[gpsPos_topic]):
        gpsPosData.append(msg)
    if len(gpsPosData) == 0:
        print("No GPS position data found -> dont save pos to exif")
    print(len(gpsPosData))      


    gpsTimeData=[]
    if useGpsTime:
        for topic, msg, t in bag.read_messages(topics=[gpsTime_topic]):
            gpsTimeData.append(msg)
    if useGpsTime == True and len(gpsTimeData)== 0:
        print("No GPS Time data found -> use unix recording timestamps instead")
        useGpsTime = False
    print(len(gpsTimeData))

    print("Save images...")
    count = 0
    denom = 3
    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        if count % denom == 0:
            if compressed=="1":
                cv_img = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            else:
                cv_img = bridge.imgmsg_to_cv2(msg, "bgr8") # desired_encoding="passthrough"
            if cv_img is None:
                print("nnonon")
                continue
            print(topic, t, cv_img)
            if useGpsTime:
                nearestPicGpsTimeData = getNearestData(gpsTimeData,msg.header.stamp)
                picGpsTimeFlt = rostime2floatSecs(nearestPicGpsTimeData.time_ref) - (rostime2floatSecs(nearestPicGpsTimeData.header.stamp) - rostime2floatSecs(msg.header.stamp)) 
            else:
                picGpsTimeFlt = rostime2floatSecs(msg.header.stamp)
            
            picNameTimeString = datetime.utcfromtimestamp(picGpsTimeFlt).strftime('%Y_%m_%d_%H_%M_%S_%.1f')
            picSaveName=""
            picSaveName = picNameTimeString + ".jpg"
            print("save image " + picSaveName)
            picPath = os.path.join(output_dir, picSaveName)
            cv2.imwrite(picPath, cv_img, params)
            
            if len(gpsPosData) > 0:
                picGpsPos = getNearestData(gpsPosData, msg.header.stamp)
                gpsTimeString = datetime.utcfromtimestamp(picGpsTimeFlt).strftime('%Y:%m:%d %H:%M:%S')
                set_gps_location(picPath,lat=picGpsPos.latitude, lng=picGpsPos.longitude,altitude=picGpsPos.altitude, gpsTime=gpsTimeString )
        
        count += 1
    bag.close()
    
    return

if __name__ == '__main__':
    main()


"""    
#Test gps data assignment to pic based on stamp
headerMsg = NavSatFix()
headerMsg.header.stamp = rospy.rostime.Time.from_seconds(1553698847.23123)
picGpsPos = getNearestData(gpsPosData,headerMsg.header.stamp )
picGpsTime = getNearestData(gpsTimeData,headerMsg.header.stamp )
picGpsTimeFlt = picGpsTime.time_ref.secs + (picGpsTime.time_ref.nsecs / 1000000000.0)
picGpsTimeString = datetime.utcfromtimestamp(picGpsTimeFlt).strftime('%Y-%m-%dT%H:%M:%S.%f')
print(picGpsTime.time_ref)
print(GpsTimeString)
"""

"""
#EXIF Reader test
exif_dict = piexif.load("")
for ifd in ("0th", "Exif", "GPS", "1st"):
    for tag in exif_dict[ifd]:
        print(piexif.TAGS[ifd][tag]["name"], exif_dict[ifd][tag])
"""