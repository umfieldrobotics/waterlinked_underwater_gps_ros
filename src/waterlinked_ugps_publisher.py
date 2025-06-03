#!/usr/bin/env python
import rospy
import requests
import json
import utm
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus, FluidPressure, Temperature
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point
import message_filters
import tf2_ros
import geometry_msgs.msg

class WaterlinkedUGPSNode:
    def __init__(self):
        rospy.init_node("waterlinked_ugps_publisher")

        self.base_url = rospy.get_param("~base_url", "http://192.168.2.94")
        rate_hz = rospy.get_param("~rate", 5.0)

        barometer_sub = message_filters.Subscriber("/BlueROV/pressure2_fluid", FluidPressure)
        imu_temp_sub = message_filters.Subscriber("/vectornav/Temp", Temperature)

        ts = message_filters.ApproximateTimeSynchronizer([barometer_sub, imu_temp_sub], queue_size=50, slop=0.1)
        ts.registerCallback(self.synchronized_callback)

        self.navsat_pub = rospy.Publisher("/waterlinked/fix", NavSatFix, queue_size=10)
        self.odom_pub = rospy.Publisher("/waterlinked/odom", Odometry, queue_size=10) # relative odometry
        self.raw_pub = rospy.Publisher("/waterlinked/string_raw", String, queue_size=10)
        self.utm_pub = rospy.Publisher("/waterlinked/utm", Point, queue_size=10)

        self.initial_position = None

        self.rate = rospy.Rate(rate_hz)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def get_data(self, url):
        try:
            response = requests.get(url, timeout=1.0)
            if response.status_code == 200:
                return response.json()
            else:
                rospy.logerr("Waterlinked GPS HTTP error {}: {}".format(response.status_code, response.text))
        except requests.exceptions.RequestException as e:
            rospy.logerr("Waterlinked GPS Request exception: {}".format(e))

    def set_depth(self, url, depth, temp):
        payload = dict(depth=depth, temp=temp)
        try:
            r = requests.put(url, json=payload, timeout=10)
            if r.status_code != 200:
                rospy.logerr("Waterlinked GPS error setting depth: {} {}".format(r.status_code, r.text))
        except requests.exceptions.RequestException as e:
            rospy.logerr("Waterlinked GPS Request exception: {}".format(e))

    def synchronized_callback(self, baro_data, imu_temp_data):
        metric_depth = (baro_data.fluid_pressure - 993.399) * 100 / 9.81 / 997
        temp = imu_temp_data.temperature
        self.set_depth(self.base_url + "/api/v1/external/depth", metric_depth, temp)

    def publish_navsat_fix(self, global_position, depth):
        msg = NavSatFix()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "ugps_link"

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = global_position.get('lat', 0.0)
        msg.longitude = global_position.get('lon', 0.0)
        msg.altitude = -depth if depth is not None else 0.0

        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.navsat_pub.publish(msg)

    def publish_odometry(self, acoustic_position):
        now = rospy.Time.now()

        # Initialize the reference position
        if self.initial_position is None:
            self.initial_position = {
                'x': acoustic_position.get('x', 0.0),
                'y': acoustic_position.get('y', 0.0),
                'z': acoustic_position.get('z', 0.0)
            }

        # Compute relative position
        rel_x = acoustic_position.get('x', 0.0) - self.initial_position['x']
        rel_y = acoustic_position.get('y', 0.0) - self.initial_position['y']
        rel_z = acoustic_position.get('z', 0.0) - self.initial_position['z']

        # Publish Odometry with relative position
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "ugps_link"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = rel_x
        odom.pose.pose.position.y = rel_y
        odom.pose.pose.position.z = rel_z
        odom.pose.pose.orientation = Quaternion(0, 0, 0, 1)

        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36
        self.odom_pub.publish(odom)

        # Publish TF with relative position
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "ugps_link"
        t.child_frame_id = "base_link"
        t.transform.translation.x = rel_x
        t.transform.translation.y = rel_y
        t.transform.translation.z = rel_z
        t.transform.rotation = Quaternion(0, 0, 0, 1)

        self.tf_broadcaster.sendTransform(t)

    def publish_utm(self, global_position, depth):
        try:
            lat = global_position.get('lat', 0.0)
            lon = global_position.get('lon', 0.0)
            utm_x, utm_y, _, _ = utm.from_latlon(lat, lon)

            point = Point()
            point.x = utm_x
            point.y = utm_y
            point.z = -depth if depth is not None else 0.0  # Z is depth
            self.utm_pub.publish(point)
        except Exception as e:
            rospy.logwarn("Waterlinked GPS UTM conversion failed: {}".format(e))


def main():
    waterlinked_ugps_node = WaterlinkedUGPSNode()
    rospy.loginfo("Starting Water Linked UGPS publisher at {}".format(waterlinked_ugps_node.base_url))

    while not rospy.is_shutdown():
        acoustic = waterlinked_ugps_node.get_data(waterlinked_ugps_node.base_url + "/api/v1/position/acoustic/filtered")
        global_pos = waterlinked_ugps_node.get_data(waterlinked_ugps_node.base_url + "/api/v1/position/global")

        if acoustic and global_pos:
            depth = acoustic.get("z", 0.0)
            waterlinked_ugps_node.publish_navsat_fix(global_pos, depth)
            waterlinked_ugps_node.publish_odometry(acoustic)
            waterlinked_ugps_node.publish_utm(global_pos, depth)

            raw_msg = {
                "acoustic": acoustic,
                "global": global_pos
            }
            waterlinked_ugps_node.raw_pub.publish(json.dumps(raw_msg))

        waterlinked_ugps_node.rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
