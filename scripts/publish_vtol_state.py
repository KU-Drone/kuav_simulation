import time
import rospy
import argparse
from pymavlink import mavutil
from pymavlink.dialects.v20.common import MAVLink
from std_msgs.msg import Byte
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform

EXTENDED_SYS_STATE = 245

MAV_VTOL_STATE_UNDEFINED = 0	
MAV_VTOL_STATE_TRANSITION_TO_FW	= 1 
MAV_VTOL_STATE_TRANSITION_TO_MC	= 2 
MAV_VTOL_STATE_MC = 3
MAV_VTOL_STATE_FW = 4

def main():
    parser = argparse.ArgumentParser(description="Connects to a mavlink object, starts the EXTENDED_SYS_STATE message sending and publishes the vtol state to ROS.")
    parser.add_argument("connection_string", help="Connection string as used bu mavlink. [protocol:]address[:port]")
    parser.add_argument("-r", "--rate", help="Rate(hz) at which to request EXTENDED_STATE messages.", default=4, type=float)
    parser.add_argument("--publish-tf", help="If true, will publish orientation corrections to the tf system", action="store_true")
    parser.add_argument("--child-frame", help="the name of the corrected frame EX: base link of device", type=str)
    parser.add_argument("--frame", help="the name of the frame published by the fcu EX: fcu_frame", type=str)
    args = parser.parse_args(rospy.myargv()[1:])
    if args.publish_tf and (args.child_frame is None or args.frame is None):
        parser.error("--publish-tf requires --frame and --child-frame")

    rospy.init_node("publish_vtol_state")

    connection = mavutil.mavlink_connection(args.connection_string) #"udpin:127.0.0.1:14550"
    connection.wait_heartbeat()
    connection.mav.command_long_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, EXTENDED_SYS_STATE, int((1/args.rate)*1e+6), 0, 0, 0, 0, 0)
    
    last_time = time.perf_counter()
    pub = rospy.Publisher("~/vtol_state", Byte, queue_size=10)
    transform_broadcaster = tf2_ros.TransformBroadcaster()

    identity_transform = Transform()
    identity_transform.rotation.w = 1
    identity_transform.rotation.x = 0
    identity_transform.rotation.y = 0
    identity_transform.rotation.z = 0

    vtol_transform = Transform()
    vtol_transform.rotation.w = 0.7071068
    vtol_transform.rotation.x = 0
    vtol_transform.rotation.y = -0.7071068
    vtol_transform.rotation.z = 0

    while not rospy.is_shutdown():
        try:
            msg = connection.recv_match(type='EXTENDED_SYS_STATE', blocking=True)
            data = Byte()
            data.data = msg.vtol_state
            pub.publish(data)
            if args.publish_tf:
                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = args.frame
                transform.child_frame_id = args.child_frame
                if data.data == MAV_VTOL_STATE_FW:
                    transform.transform = identity_transform
                elif data.data == MAV_VTOL_STATE_MC:
                    transform.transform = vtol_transform
                elif data.data == MAV_VTOL_STATE_TRANSITION_TO_MC:
                    transform.transform = vtol_transform
                elif data.data == MAV_VTOL_STATE_TRANSITION_TO_FW:
                    transform.transform = identity_transform
                elif data.data == MAV_VTOL_STATE_UNDEFINED:
                    transform.transform = identity_transform
                else:
                    print("Invalid vtol state")
                    transform.transform = identity_transform
                transform_broadcaster.sendTransform(transform)
            # t = time.perf_counter()
            # print(1/(t-last_time))
            # last_time = t
        except KeyError:
            pass

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("exiting.")