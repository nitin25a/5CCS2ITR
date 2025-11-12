#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D
from delivery_bot.srv import GetLocation, GetLocationResponse

def handle_get_location(req):
    # Coordinates chosen from the visible grid in simple_house.world
    # (matching the Stage screenshot, range x:-9->10, y:-6->5)
    locations = {
        # UPDATED: x=10.7, y=4.3, theta=0.0
        "kitchen": Pose2D(x=10.7, y=4.3, theta=0.0),
        # UPDATED: x=10.4, y=8.9, theta=1.57
        "living_room": Pose2D(x=10.4, y=8.9, theta=1.57),
        "person": Pose2D(x=1.0, y=0.0, theta=0.0),
        #"bedroom": Pose2D(x=5.5, y=3.5, theta=3.14),       # top-right room
        #"bathroom": Pose2D(x=-3.0, y=3.5, theta=-1.57)     # top-left room
    }

    if req.location_name in locations:
        rospy.loginfo(f"[Service] Returning coordinates for {req.location_name}")
        return GetLocationResponse(locations[req.location_name])
    else:
        rospy.logwarn(f"[Service] Unknown location: {req.location_name}")
        return GetLocationResponse(Pose2D())

def main():
    rospy.init_node('location_server')
    rospy.Service('get_location', GetLocation, handle_get_location)
    rospy.loginfo("Location Service Server is ready with corrected map coordinates.")
    rospy.spin()

if __name__ == '__main__':
    main()

