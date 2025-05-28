import rospy
import requests
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header

def get_data(url):
    try:
        r = requests.get(url)
        r.raise_for_status()
        return r.json()
    except requests.exceptions.RequestException as e:
        rospy.logwarn("Request failed: {}".format(e))
        return None

def get_position(url):
    return get_data("{}/api/v1/position/global".format(url))

def get_depth(url):
    acoustic_data = get_data("{}/api/v1/position/acoustic/filtered".format(url))
    if acoustic_data and "z" in acoustic_data:
        return -acoustic_data["z"]  # altitude = -depth
    return 0.0

def ugps_publisher(base_url):
    pub = rospy.Publisher("/waterlinked/navsat_fix", NavSatFix, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        global_pos = get_position(base_url)
        if not global_pos:
            rospy.logwarn("No global position data received")
            rate.sleep()
            continue

        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "ugps"

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = global_pos.get("lat", 0.0)
        msg.longitude = global_pos.get("lon", 0.0)
        msg.altitude = get_depth(base_url)

        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("waterlinked_underwater_gps_node")
    base_url = rospy.get_param("~base_url", "https://demo.waterlinked.com")
    try:
        ugps_publisher(base_url)
    except rospy.ROSInterruptException:
        pass
