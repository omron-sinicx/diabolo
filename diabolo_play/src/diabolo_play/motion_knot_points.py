#!/usr/bin/env python

import rospy

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
import copy
class KnotPointsServer:
    '''
    This class allows setting points with interactive markers
    This allows setting an arbitrary number of knot points for complex motion creation
    Designed for the diabolo setup, and assumes two sets of knot points
    '''
    def processFeedback(self, feedback):
        p = copy.deepcopy(feedback.pose.position)
        self.server.marker_contexts[feedback.marker_name].int_marker.pose = copy.deepcopy(feedback.pose)
        m = copy.deepcopy(feedback.marker_name)
        #the initial position of the knot point group this point is in
        initial_pos = self.initial_points[int(m[0])][0] 
        p.x = p.x - initial_pos.x
        p.y = p.y - initial_pos.y
        p.z = p.z - initial_pos.z
        print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z) 
        

    def ignoreFeedback(self, feedback):
        p = feedback.pose.position
        print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z) 

    def __init__(self, number_of_knot_points, initial_points):
        '''
        Parameters: 
           number_of_knot_points: Number of knot points for a single hand
           initial_points: Expects a list of geometry_msgs::Point messages of length equal to the number of knot points to set 
                           This is the initial position of the spline path. This point will be a fixed marker
                           The positions of the markers will be returned relative to this marker
        '''
        self.initial_points = initial_points
        self.number_of_knot_points = number_of_knot_points
        self.server  = InteractiveMarkerServer("motion_knot_points")
       
        self.initialize_markers()

    def __del__(self):
        self.server.clear()
        self.server.applyChanges()

    def initialize_markers(self):
        
        # Go through each set of initial points
        for p in range(len(self.initial_points)):
            
            initial_marker = self.get_interactive_marker(self.initial_points[p][0], (1.0, 1.0, 1.0))
            # format = <knot_point_group>_knot_point_<marker_number>
            initial_marker.name = str(p) + "_knot_point_" + "0" 
            initial_marker.controls[0].interaction_mode = InteractiveMarkerControl.NONE
            initial_marker.controls[0].name = "none"
            
            self.server.insert(copy.deepcopy(initial_marker), self.ignoreFeedback)
            initial_pos = self.initial_points[p][0]

            for i in range(1, self.number_of_knot_points+1):
                position = self.initial_points[p][i]
                # The initial seed positions are sent relative to the initial marker, but they need to be in the world frame for display
                position.x += initial_pos.x
                position.y += initial_pos.y
                position.z += initial_pos.z
                #The marker color changes from red to blue from "earlier" marker to "later" marker
                col = (1.0-(float(i)/float(self.number_of_knot_points+1)), 0, float(i)/float(self.number_of_knot_points+1))
                int_marker = self.get_interactive_marker(copy.deepcopy(position), col)
                int_marker.name = str(p) + "_knot_point_" + str(i)
                self.server.insert(int_marker, self.processFeedback)


        self.server.applyChanges()

    def reset_initial_points(self, initial_points):
        self.initial_points = initial_points
    
    def get_marker_positions(self, relative=True):
        '''
        If relative is true, the returned positions will be relative to the initial position of that group
        If false, they will be the marker position in the world frame
        '''
        #self.server.applyChanges()
        #Get the current marker positions from the server marker context as a list of points
        # print("Number of knot points = " + str(self.number_of_knot_points))
        # print("Number of knot groups = " + str(len(self.initial_points)))
        position_list = []
        for p in range(len(self.initial_points)):
            base_position = copy.deepcopy(self.server.marker_contexts[str(p) + "_knot_point_0"].int_marker.pose.position)
            position_list.append([])
            #Ignore the first marker since that is the provided initial position
            for i in range(1, self.number_of_knot_points+1): 
                marker_name = str(p) + "_knot_point_" + str(i)
                
                position = copy.deepcopy(self.server.marker_contexts[marker_name].int_marker.pose.position)
                # print(marker_name + "'s position is " + str(position))
                if relative:
                    position.x -= base_position.x
                    position.y -= base_position.y
                    position.z -= base_position.z
                position_list[p].append(position)

        return position_list

    def get_interactive_marker(self, position, color):
        '''
        Return a control marker capable of motion along the y z plane
        '''
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "world"
        int_marker.pose.position = position
        int_marker.scale = 1

        int_marker.name = "_knot_point_"
        int_marker.description = "yz control"

        visual_marker = Marker()
        visual_marker.type = Marker.SPHERE
        visual_marker.scale.x = 0.05
        visual_marker.scale.y = 0.05
        visual_marker.scale.z = 0.05
        visual_marker.color.r = color[0]
        visual_marker.color.g = color[1]
        visual_marker.color.b = color[2]
        visual_marker.color.a = 1.0

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.markers.append(visual_marker)
        control.always_visible = True
        int_marker.controls.append(copy.deepcopy(control))
        
        return int_marker        

 