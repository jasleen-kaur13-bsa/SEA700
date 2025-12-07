#!/usr/bin/env python
import rospy
import random
import actionlib
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Trigger

# -----------------------------
# CONFIG
# -----------------------------
USE_RANDOM_WANDER = False # exploration did not work well so keep to False
SPIN_TIME = 30
LOCALIZATION_TIMEOUT = 60
LOCALIZATION_TIMEOUT_AFTER_PICK = 15
PICK_TIMEOUT = 45
RELOCALIZE_SPIN_TIME = 8.0

WAYPOINT_PICK1 = (1.0075341, 2.6080185, 0.7617231, 0.64790261)
WAYPOINT_DROP1 = (0.51078, -0.04668795, 0.079764, 0.996814)
# -----------------------------
# GLOBAL VARIABLES
# -----------------------------
map_data = None
amcl_cov_ok = False

# -----------------------------
# CALLBACKS
# -----------------------------
def map_callback(msg):
    global map_data
    map_data = msg

def amcl_callback(msg):
    """Check AMCL covariance to determine if localization is good."""
    global amcl_cov_ok
    cov = msg.pose.covariance
    pos_var = cov[0] + cov[7]
    if pos_var < 0.5:
        amcl_cov_ok = True

# -----------------------------
# LOCALIZATION ROUTINES
# -----------------------------

def wait_for_localization(timeout_sec):
    start = rospy.Time.now().to_sec()
    rate = rospy.Rate(10)
    while rospy.Time.now().to_sec() - start < timeout_sec:
        if amcl_cov_ok:
            rospy.loginfo("AMCL localized successfully.")
            return True
        rate.sleep()
    return False


def publish_initial_pose(x = -0.6216127,y = 0.02544182, z = -0.0395059, w = 0.9921933301):
    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    pose.pose.pose.position.x = x
    pose.pose.pose.position.y = y
    pose.pose.pose.orientation.z = z
    pose.pose.pose.orientation.w = w
    rospy.sleep(1)
    pub.publish(pose)
    rospy.logwarn("Fallback initial pose published.")

def spin_robot():
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    tw = Twist()
    tw.angular.z = 0.75
    rate = rospy.Rate(10)

    rospy.loginfo("Spinning robot {} seconds...".format(SPIN_TIME))
    t0 = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - t0 < SPIN_TIME:
        pub.publish(tw)
        rate.sleep()

    tw.angular.z = 0
    pub.publish(tw)
    rospy.loginfo("Spin complete.")

def clear_costmaps():
    """Clear stale obstacles before sending a new goal."""
    try:
        rospy.wait_for_service("/move_base/clear_costmaps", timeout=5)
        clear_srv = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        clear_srv()
        rospy.loginfo("Cleared move_base costmaps.")
    except Exception as e:
        rospy.logwarn("Failed to clear costmaps: {}".format(e))

def wait_for_pick_finish(timeout_sec):
    """Poll automatic_pick status parameter until pick completes or times out."""
    start = rospy.Time.now().to_sec()
    rate = rospy.Rate(5)
    while rospy.Time.now().to_sec() - start < timeout_sec:
        try:
            status = rospy.get_param("/automatic_pick/status", "")
            if status == "pick_finish":
                return True
        except Exception:
            pass
        rate.sleep()
    return False

# -----------------------------
# ACTIONLIB NAVIGATION
# -----------------------------
def send_actionlib_goal(x, y, z, w):
    client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w

    rospy.loginfo("Sending actionlib goal: ({}, {})".format(x, y))
    client.send_goal(goal)
    client.wait_for_result()

    state = client.get_state()
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached.")
    else:
        rospy.logwarn("Failed to reach goal. State: {}".format(state))

# -----------------------------
# RANDOM NAVIGATION
# -----------------------------
# def pick_random_free_cell():
#     global map_data
#     w = map_data.info.width
#     h = map_data.info.height
#     res = map_data.info.resolution
#     origin = map_data.info.origin

#     while True:
#         x = random.randint(0, w - 1)
#         y = random.randint(0, h - 1)
#         idx = x + y * w
#         if map_data.data[idx] == 0:
#             wx = x * res + origin.position.x
#             wy = y * res + origin.position.y
#             return wx, wy

# def random_walk():
#     rospy.loginfo("Starting random exploration using actionlib...")
#     while not rospy.is_shutdown():
#         x, y = pick_random_free_cell()
#         send_actionlib_goal(x, y)
#         rospy.sleep(2)

# -----------------------------
# MAIN
# -----------------------------
if __name__ == "__main__":
    rospy.init_node("autonomous_nav_actionlib")

    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_callback)

    rospy.loginfo("Waiting for map...")
    while map_data is None:
        rospy.sleep(0.2)

    # Step 1: Publish intiial pose
    publish_initial_pose()

    # Step 2: Spin to help AMCL converge
    spin_robot()

    # Step 3: Wait for localization
    # Require a fresh AMCL covariance check
    amcl_cov_ok = False
    if not wait_for_localization(LOCALIZATION_TIMEOUT):
        rospy.logwarn("Localization timeout. Could not localise...")
        rospy.sleep(1)

    # Step 4: Navigate
    # if USE_RANDOM_WANDER:
    #     random_walk()
    # else:

    rospy.loginfo("Starting waypoint navigation with actionlib...")
    # Step 1: go to PICK1
    send_actionlib_goal(*WAYPOINT_PICK1)
    rospy.loginfo("At PICK1 calling pick service")
    pick_srv = rospy.ServiceProxy("/automatic_pick/pick", Trigger)
    pick_srv()
    # Wait for pick to actually finish before proceeding
    wait_for_pick_finish(PICK_TIMEOUT)

    # Brief spin to help AMCL converge if odom slipped during pick
    spin_robot()
    amcl_cov_ok = False
    wait_for_localization(LOCALIZATION_TIMEOUT_AFTER_PICK)
    clear_costmaps()

    # Step 2: go to DROP1
    send_actionlib_goal(*WAYPOINT_DROP1)
    rospy.loginfo("At DROP1 calling place service")
    place_srv = rospy.ServiceProxy("/automatic_pick/place", Trigger)
    place_srv()

    rospy.spin()
