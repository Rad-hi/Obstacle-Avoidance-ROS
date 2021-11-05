#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist #ros msg that deals with moving the robot
from sensor_msgs.msg import LaserScan #ros msg that gets the laser scans
# import time

# obstacle threshhold, objects a this distance or below it
#are considered obstacles
OBSTACLE_DIST = 0.5
#the angle in which each region extends
REGIONAL_ANGLE = 30
PI = 3.141592653

#when there's no obstacles, the robot will move with this linear velocity
NORMAL_LIN_VEL = 0.50 #meters/second
#after detecting an obstacle, the robot shall back up a bit (negative) while
# rotating to help in case it can't perform a stationary rotation
TRANS_LIN_VEL = -0.08
#the robot always rotates with the same value of angular velocity
TRANS_ANG_VEL = 1.75

#this list keeps track of the order in which the regions' readings are obtained
REGIONS = [
             "front_C", "front_L", "left_R",
             "left_C", "left_L", "back_R",
             "back_C", "back_L", "right_R",
             "right_C", "right_L", "front_R",
          ]
#this is a global variable that keeps handles the orders for the robot to follow
#if there's a detected object, "act" is turned to True
#and the angular_vel and sleep values are calculated appropriately
Urgency_Report = {
                    "act": False, "angular_vel": 0.0, "sleep": 0
                 }
#this dict keeps track of the distance measures for each region
Regions_Report = {
                     "front_C":[], "front_L":[], "left_R":[],
                     "left_C":[], "left_L":[], "back_R":[],
                     "back_C":[], "back_L":[], "right_R":[],
                     "right_C":[], "right_L":[], "front_R":[],
                 }
#These are the costs to deviate from each region to the goal region (front_C)
Regions_Distances = {
                     "front_C": 0, "front_L": 1, "left_R": 2,
                     "left_C": 3, "left_L": 4, "back_R": 5,
                     "back_C": 6, "back_L": -5, "right_R": -4,
                     "right_C": -3, "right_L": -2, "front_R": -1,
                 }

#in this function the clearest paths are calculated and the appropriate
#values for the angular_vel and the execution times are assigned
def ClearanceTest():
    global Urgency_Report

    goal = "front_C"
    closest = 10e6
    regional_dist = 0
    maxima = {"destination": "back_C", "distance": 10e-6}
    for region in Regions_Report.items():
        regional_dist = abs(Regions_Distances[region[0]]-Regions_Distances[goal])
        #if there're no obstacles in that region
        if not len(region[1]):
            #check if it's the cheapest option
            if (regional_dist < closest):
                closest = regional_dist
                maxima["distance"] = OBSTACLE_DIST
                maxima["destination"] = region[0]
        #check if it's the clearest option
        elif(max(region[1]) > maxima["distance"]):
            maxima["distance"] = max(region[1])
            maxima["destination"] = region[0]

    #calculate the cost to the chosen orientation
    regional_dist = Regions_Distances[maxima["destination"]]-Regions_Distances[goal]

    #we act whenever the clearest path is not the front_C (front center)
    Urgency_Report["act"] = (closest != 0)
    Urgency_Report["angular_vel"] = ((regional_dist/max(1, abs(regional_dist)))
                                    *TRANS_ANG_VEL)
    Urgency_Report["sleep"] = ((abs(regional_dist)*REGIONAL_ANGLE*PI)
                              /(180*TRANS_ANG_VEL))

def IdentifyRegions(scan):
    global Regions_Report
    for i, region in enumerate(REGIONS):
        Regions_Report[region] = [
                x for x in scan.ranges[REGIONAL_ANGLE*i : REGIONAL_ANGLE*(i+1)]
                        if x <= OBSTACLE_DIST and x != 'inf']

def Steer(velocity):
    global Urgency_Report

    #since we're moving only on the plane, all we need is move in the x axis,
    #and rotate in the z (zeta) axis.
    velocity.linear.x = TRANS_LIN_VEL
    velocity.linear.y = 0
    velocity.linear.z = 0
    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = Urgency_Report["angular_vel"]

    return velocity

def main():
    #Initialize our node
    rospy.init_node("Laser_Obs_Avoid_node")
    #Subscribe to the "/scan" topic in order to read laser scans data from it
    rospy.Subscriber("/scan", LaserScan, IdentifyRegions)
    #create our publisher that'll publish to the "/cmd_vel" topic
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    vel = Twist()
    #ros will try to run this code 10 times/second
    rate = rospy.Rate(10) #10Hz
    #keep running while the ros-master isn't isn't shutdown
    while not rospy.is_shutdown():

        # Need a do{ ... }while(); here (C is awesome)
        # Since I need to check at least once the clearance 
        done = False
        while not done:
            ClearanceTest()
            if(Urgency_Report["act"]):
                vel = Steer(vel)
                pub.publish(vel)
            else:
                done = True
        # This else belongs to the while(), and the code below it could be cleaned furthermore
        else: 
            vel.linear.x = NORMAL_LIN_VEL
            vel.linear.y = 0
            vel.linear.z = 0
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = 0
            pub.publish(vel)

        ### This is stupid and shouldn't be done (sleep()) !
        # After publishing our action, we give it some time to execute the
        # needed actions before reading the data again.
        # time.sleep(Urgency_Report["sleep"])
        
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
