#draw_tbot_design.py
#draws turtlebot design using lines and circles
#can also draw circle to user input specifications
#run with gazebo turtlebot3_empty_world or turtlebot

#max ang vel of turtlebot burger is 2.84 radians
#max lin vel of turtlebot burger is 0.22 m/s

import rospy 
import math
from geometry_msgs.msg import Twist
pub = None

#VEL_LIN is a constant, so the linear velocity of the turtlebot is pretty consistent
#MAX_ANG_VEL is just precautionary
VEL_LIN = 0.16 #we want it traveling about this fast at all times
MAX_ANG_VEL = 2.80
min_radius = VEL_LIN / MAX_ANG_VEL

stop = Twist()
stop.linear.x = 0
stop.angular.z = 0

def pause(sec):
 #pause the robot for one second
    start_time =rospy.Time.now().to_sec()
    stop_time = start_time + sec
    while (rospy.Time.now().to_sec() < stop_time):
        pub.publish(stop)

#LINE FUNCTION#
def line(length):
    twist = Twist()
    twist.linear.x = VEL_LIN
    twist.angular.z = 0
    
    start_time = rospy.Time.now().to_sec()
    runtime = length / VEL_LIN
    stop_time = start_time + runtime

    while (rospy.Time.now().to_sec() < stop_time):
        pub.publish(twist)

    pause(1)

##CIRCLE_FUNCTIONS##
#draws a circle 
#specify radius and radians (how much of the circle you want to cover):
def circle(radius, radians):
    print("Drawing circle")

    #1 Calculate angular velocity given VEL_LIN, radius, and radians
    ang_vel = VEL_LIN / radius
    ang_vel = ang_vel * (radians/abs(radians)) #deals with whether or not radians negative

    # if ang_vel > MAX_ANG_VEL:
    #     print("ERROR! Calculated angular velocity is too big:", ang_vel)
    #     print("Radius must be greater than:", min_radius)
    #     return

    #2 Calculate runtime in seconds from radians and angular velocity
    runtime = abs(radians) / abs(ang_vel)
    print("runtime: {}".format(runtime))

    #set the fields of the twist message
    twist = Twist()
    twist.linear.x = VEL_LIN
    twist.angular.z = ang_vel

    #start the clock and specify the stop time
    start_time = rospy.Time.now().to_sec()
    stop_time = start_time + runtime

    #run for specified amount of time
    while (rospy.Time.now().to_sec() < stop_time):
        pub.publish(twist)

    pause(1)
    response = input("Continue? (Y for Yes, Q for Quit): ").strip()
    if response == "Y" or response == "y":
        return
    else:
        exit()
    

#Code for prompting user to input radius and radians, can uncomment in main to test
def prompt_circle():
    pub.publish(stop)
    print("Please write all answers as a decimal or whole number, e.g. 3 or 3.1415")

    print("Radius must be greater than", min_radius)
    radius = float(input("What radius? (m): ").strip())
    radians = float(input("How much of the circle do you want to traverse (radians)?: ").strip())
    circle(radius, radians)
    print("Circle done!")
    pub.publish(stop)


#MAIN, DO NOT CHANGE
if __name__ == '__main__':
    try: 
        
        rospy.init_node('draw_tbot_design')

        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1000)

        #Uncomment prompt_circle() if you want program to prompt user for radius/radians
        #prompt_circle()

        #PUT COMMANDS FOR DRAWING DESIGN HERE#
        circle(0.3, 6.28)
        circle(0.2, 6.28)
        circle(0.1, 6.28)
        circle(0.1, -6.28)
        circle(0.2, -6.28)
        circle(0.3, -6.28)
        
            
    except rospy.ROSInterruptException:
        pass