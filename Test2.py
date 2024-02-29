#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_srvs.srv import Empty, EmptyResponse

# Global mode variable to track the current operation
MODE = "NONE"
# Placeholder for the current local position of the drone
LOCAL_POSE = PoseStamped()
# Placeholder for the current state of the drone (connected, armed, etc.)
CURR_STATE = State()

# Callback to update the current state of the drone
def state_cb(msg):
    global CURR_STATE
    CURR_STATE = msg

# Callback to update the current local position of the drone
def pose_cb(msg):
    global LOCAL_POSE
    LOCAL_POSE = msg.pose

# Handlers for each operation
def handle_launch():
    global MODE
    MODE = "LAUNCH"
    print('Launch Requested. Your drone should take off.')

def handle_test():
    global MODE
    MODE = "TEST"
    print('Test Requested. Your drone should perform the required tasks. Recording starts now.')

def handle_land():
    global MODE
    MODE = "LAND"
    print('Land Requested. Your drone should land.')

def handle_abort():
    global MODE
    MODE = "ABORT"
    print('Abort Requested. Your drone should land immediately due to safety considerations')

# Service callbacks
def callback_launch(request):
    handle_launch()
    return EmptyResponse()

def callback_test(request):
    handle_test()
    return EmptyResponse()

def callback_land(request):
    handle_land()
    return EmptyResponse()

def callback_abort(request):
    handle_abort()
    return EmptyResponse()

# Main communication node for ground control
def comm_node():

    global LOCAL_POSE, MODE, CURR_STATE

    node_name = "rob498_drone_07"

    rospy.init_node(node_name)

    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=pose_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    srv_launch = rospy.Service(node_name + '/comm/launch', Empty, callback_launch)
    srv_test = rospy.Service(node_name + '/comm/test', Empty, callback_test)
    srv_land = rospy.Service(node_name + '/comm/land', Empty, callback_land)
    srv_abort = rospy.Service(node_name + '/comm/abort', Empty, callback_abort)

    last_req = rospy.Time.now()

    while not rospy.is_shutdown():
        rate = rospy.Rate(20)  # Setpoint publishing MUST be faster than 2Hz

        if MODE == "NONE" and not rospy.is_shutdown():
            print("MODE is NONE.")
            while MODE == "NONE" and not rospy.is_shutdown():
                rate.sleep()
            
        elif (MODE == "LAUNCH" or MODE == "TEST") and not rospy.is_shutdown():
            print("MODE is LAUNCH or TEST.")
            pose = PoseStamped()
            empty_pose = PoseStamped()

            while LOCAL_POSE == empty_pose:
                print("ERROR: current_local_pose not initialized.")
                rate.sleep()

            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = 1.1

            pose.pose.orientation.x = LOCAL_POSE.orientation.x
            pose.pose.orientation.y = LOCAL_POSE.orientation.y
            pose.pose.orientation.z = LOCAL_POSE.orientation.z
            pose.pose.orientation.w = LOCAL_POSE.orientation.w

            print(pose)
            
            while (MODE == "LAUNCH" or MODE == "TEST") and not rospy.is_shutdown():
                local_pos_pub.publish(pose)
                rate.sleep()

        elif MODE == "LAND" and not rospy.is_shutdown():
            print("MODE is LAND.")
            pose = PoseStamped()
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = 1.1

            pose.pose.orientation.x = LOCAL_POSE.orientation.x
            pose.pose.orientation.y = LOCAL_POSE.orientation.y
            pose.pose.orientation.z = LOCAL_POSE.orientation.z
            pose.pose.orientation.w = LOCAL_POSE.orientation.w
            
            while MODE == "LAND" and not rospy.is_shutdown():
                pose.pose.position.z = max(pose.pose.position.z / 1.5, 0.2)
                local_pos_pub.publish(pose)
                rate.sleep()

        elif MODE == "ABORT" and not rospy.is_shutdown():
            print("MODE is ABORT.")
            pose = PoseStamped()
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = 0

            pose.pose.orientation.x = LOCAL_POSE.orientation.x
            pose.pose.orientation.y = LOCAL_POSE.orientation.y
            pose.pose.orientation.z = LOCAL_POSE.orientation.z
            pose.pose.orientation.w = LOCAL_POSE.orientation.w
                     
            arm_cmd = CommandBoolRequest()
            arm_cmd.value = False

            while not rospy.is_shutdown() and MODE == "ABORT":
                local_pos_pub.publish(pose)
                
                if CURR_STATE.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if arming_client.call(arm_cmd).success:
                        rospy.loginfo("Vehicle disarmed")
            
                last_req = rospy.Time.now()
        else:
            rospy.loginfo("Error: Unrecognised MODE.")

if __name__ == "__main__":
    comm_node()