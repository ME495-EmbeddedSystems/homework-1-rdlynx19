"""
Publishes twist for a turtle to follow a set of loaded waypoints. 
Additionally computes the deviation from the shortest path for one loop 
through the given waypoints.

PUBLISHERS:
 + /turtle1/cmd_vel (geometry_mgs/Twist)            - The velocity for the turtle to move to a given waypoint
 + /loop_metrics    (turtle_interfaces/ErrorMetric) - The stats after 1 complete loop around the waypoints

SUBSCRIBERS:
 + /turtle1/pose (turtlesim/Pose) - To calculate the velocity with respect to current pose and next waypoint

SERVICES:
+ /toggle (std_srvs/Empty)              - To switch between MOVING and STOPPED states for the turtle
+ /load   (turtle_interfaces/Waypoints) - To load waypoints and compute minimum distance between them

CLIENTS:
+ /reset                     (std_srvs/Empty)             - To reset the turtle to start configuration
+ /turtle1/teleport_absolute (turtlesim/TeleportAbsolute) - To teleport the turtle to the waypoints and mark them with X's
+ /turtle1/set_pen           (turtlesim/SetPen)           - To control the pen color, width and set it off and on to plot the X's 

PARAMETERS:
+ /frequency    (integer/double) - To set the frequency of DEBUG log message
+ /turtle_state (string)         - To store the turtle's current state i.e. either MOVING or STOPPED
+ /tolerance    (double)         - To set the distance tolerance value to mark a waypoint as visited
"""

import rclpy
from rclpy.node import Node

import math,copy

import rclpy.parameter
from std_srvs.srv import Empty

from geometry_msgs.msg import Twist, Vector3

from turtle_interfaces.srv import Waypoints

from turtle_interfaces.msg import ErrorMetric

from turtlesim.msg import Pose

from turtlesim.srv import TeleportAbsolute, SetPen

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


def turtle_twist(linear_vel, angular_vel):
    """Create a twist velocity suitable for a turtle

        Args:
            linear_vel (list of floats): the linear velocities
            angular_vel (list of floats): the angular velocities

        Returns:
            Twist - a 2D twist object corresponding to linear/angular velocity
    """
    return Twist(linear = Vector3(x = linear_vel[0], y = linear_vel[1], z = linear_vel[2]),
                    angular = Vector3(x = angular_vel[0], y = angular_vel[1], z = angular_vel[2]))

def minimum_distance(waypoints):
    """Calculate the minimum distance between a set of waypoints

        Args:
            waypoints (list of geometry_msgs/Point): the loaded waypoints

        Returns:
            min_dist - a double value corresponding to the minimum distance calculated
    """
    min_dist = 0.0
    for i in range(0,len(waypoints)-1):
        min_dist += math.sqrt((waypoints[i].x - waypoints[i+1].x)**2 + (waypoints[i].y - waypoints[i+1].y)**2)
    
    return min_dist

def calculate_tolerance(current_pos, current_waypoint):
    """Calculate the distance between the current position and currently tracked waypoint

        Args:
            current_pos (turtlesim/Pose): the current position of the turtle
            current_waypoint (geometry_msgs/Point): the currently tracked waypoint (the waypoint to be visited)

        Returns:
            tol - a double value corresponding to the distance between the current pose and current waypoint

    """
    tol = math.sqrt((current_pos.x - current_waypoint.x)**2 + (current_pos.y - current_waypoint.y)**2)

    return tol

def actual_distance(stored_poses):
    """Calculate the actual distance covered by the turtle while traversing the waypoints

        Args:
            stored_poses (list of turtlesim/Pose): the turtle's poses while travelling through the waypoints

        Returns:
            actual_dist - a double value corresponding to actual distance travelled by the turtle

    """
    actual_dist = 0.0
    for i in range(0, len(stored_poses)-1):
        actual_dist += math.sqrt((stored_poses[i].x - stored_poses[i+1].x)**2 + (stored_poses[i].y - stored_poses[i+1].y)**2)

    return actual_dist



class Waypoint(Node):
    """The waypoint Node"""

    def __init__(self):
        super().__init__('waypoint')
        self.get_logger().info('waypoint_node')

        # Publishers
        self._vel_pub = self.create_publisher(Twist, "cmd_vel", 1)
        self._metric_pub = self.create_publisher(ErrorMetric, "loop_metrics", 1)

        # Subscribers
        self._pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

        # Services
        self._toggle_srv = self.create_service(Empty,'toggle', self.toggle_callback)
        self._load_srv = self.create_service(Waypoints, 'load', self.waypoints_callback)

        self.cbgroup = MutuallyExclusiveCallbackGroup()
        # Clients
        self._reset_client = self.create_client(Empty, "reset", callback_group=self.cbgroup)
        self._teleport_client = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute", callback_group=self.cbgroup)
        self._pen_client = self.create_client(SetPen, "/turtle1/set_pen", callback_group=self.cbgroup)

        # Parameters
        self.declare_parameter('frequency', 90) 
        self.declare_parameter('turtle_state','STOPPED')
        self.declare_parameter('tolerance', 0.05)

        # Can change the parameter value during runtime but change is not effective
        self.freq = self.get_parameter('frequency').value

        # Timers
        self.timer = self.create_timer((1/self.freq),self.timer_callback)
        self._control_timer = self.create_timer((1), self.control_timer_callback)

        # Helper Variables
        self.current_pose = Pose()
        self.loop_stats = ErrorMetric()

        self.waypoints = []
        self.waypoints_count = 0
        self.actual_path = []

        self.straight_line_distance = 0.0
        self.i = 0
        self.pos_count = 0
    
        
    def timer_callback(self):
        """The timer to log a debug command at a fixed frequency if the turtle is MOVING
        """
        tim_turt_state = self.get_parameter('turtle_state').get_parameter_value().string_value
        if(tim_turt_state == 'MOVING'):
            self.get_logger().debug("Issuing Command!") 
          
    
    def pose_callback(self, msg):
        """Callback function for the /turtle1/pose subscription

            Args:
                msg (turtlesim/Pose) : the turtle's current pose

        """
        self.current_pose = msg
        self.actual_path.append(msg)
        

    def control_timer_callback(self):
        """The control loop timer, issues velocity to the turtle based on it's current 
        position and the current waypoint to be visited
        """

        turt_state = self.get_parameter('turtle_state').get_parameter_value().string_value
        tol_val = self.get_parameter('tolerance').get_parameter_value().double_value
        if(turt_state == "MOVING"):
            # Checking if one loop has been completed
            if (self.i < len(self.waypoints) and calculate_tolerance(self.current_pose, self.waypoints[self.i]) < tol_val):
                if(self.i == 0):
                    self.pos_count += 1
                self.i += 1
                if(self.i == len(self.waypoints)):
                    self.i = 0
                if(self.pos_count == 2):
                    self.pos_count = 1
                    self.get_logger().info("Loop Complete!")
                    # self.toggle_callback(Empty.Request(),Empty.Response())
                    

                    # Update and publish stats to /loop_metric
                    self.loop_stats.complete_loops += 1
                    path = copy.deepcopy(self.actual_path)
                    act_distance = actual_distance(path)
                    self.loop_stats.actual_distance = act_distance
                    self.loop_stats.error = act_distance - self.straight_line_distance

                    self._metric_pub.publish(self.loop_stats)

                    self.actual_path = []
                    # return 

            # Computing and publishing the linear and angular velocity for the turtle
            yaw_vel =  (math.atan2(self.waypoints[self.i].y - self.current_pose.y, self.waypoints[self.i].x - self.current_pose.x) - self.current_pose.theta)
            x_vel = 0.6 * calculate_tolerance(self.current_pose, self.waypoints[self.i])
            robot_velocity = turtle_twist([x_vel, 0.0, 0.0], [0.0, 0.0, yaw_vel])
            self._vel_pub.publish(robot_velocity)


    def toggle_callback(self,request,response):
        """Callback function for /toggle service which modifies the turtle_state parameter

            Args:
                request (EmptyRequest) : empty request object

                response (EmptyResponse): empty response object
            
            Returns:
                An empty response object
        """
        turt_state = self.get_parameter('turtle_state').get_parameter_value().string_value
        if(turt_state == 'MOVING'):
            self.get_logger().info('Stopping!')
            
            move_2_stop = rclpy.parameter.Parameter('turtle_state', rclpy.Parameter.Type.STRING, 'STOPPED')
            mv_switch = [move_2_stop]
            self.set_parameters(mv_switch)
            return response
        elif(turt_state == 'STOPPED'):
            stop_2_move = rclpy.parameter.Parameter('turtle_state', rclpy.Parameter.Type.STRING, 'MOVING')
            stp_switch = [stop_2_move]
            self.set_parameters(stp_switch)

            if(len(self.waypoints) == 0):
                self.get_logger().error("No waypoints loaded. Load them with the \"load\" service ")
                return response 

            self.waypoints_count = len(self.waypoints)
            return response
            
    async def waypoints_callback(self, request, response):
        """Async Callback function for the /load service

            Args:
                request (WaypointsObject): contains a list of geometry_msgs/Point
                    which are the waypoints to be loaded
                
                response (WaypointsResponse): the response object
            
            Returns:
                A WaypointsResponse, containing the minimum distance for 
                    traversing through all the waypoints
        """
        # calling the turtlesim /reset service to reset the turtle
        while not self._reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Reset service not available, waiting again")
        await self._reset_client.call_async(Empty.Request()) # Resetting the turtle position
        
        while not self._teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Teleport service not available, waiting again")
        
        while not self._pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Set Pen service not available, waiting again")

        
        self.waypoints = request.waypoints # Storing the loaded waypoints for computations in other functions

        self.loop_stats = ErrorMetric() # Resetting all the /loop_metric stats when the /load service is called

        # Initialising pen to white to draw the X's
        pen_status = SetPen.Request()
        pen_status.r = 255
        pen_status.g = 255
        pen_status.b = 255
        pen_status.width = 4

        tel_pos = TeleportAbsolute.Request() # Initialising a teleport request object 

        for i in range(0,len(request.waypoints)):
            pen_status.off = 255
            await self._pen_client.call_async(pen_status)
            tel_pos.x = request.waypoints[i].x
            tel_pos.y = request.waypoints[i].y      
            await self._teleport_client.call_async(tel_pos)
            
            # Making the turtle draw X at each waypoint
            pen_status.off = 0
            await self._pen_client.call_async(pen_status)
            tel_pos.x = request.waypoints[i].x + 0.2
            tel_pos.y = request.waypoints[i].y - 0.2
            await self._teleport_client.call_async(tel_pos)
            tel_pos.x = request.waypoints[i].x - 0.2 
            tel_pos.y = request.waypoints[i].y + 0.2
            await self._teleport_client.call_async(tel_pos)
            pen_status.off = 255
            await self._pen_client.call_async(pen_status)
            tel_pos.x = request.waypoints[i].x - 0.2
            tel_pos.y = request.waypoints[i].y - 0.2
            await self._teleport_client.call_async(tel_pos)
            pen_status.off = 0
            await self._pen_client.call_async(pen_status)
            tel_pos.x = request.waypoints[i].x + 0.2
            tel_pos.y = request.waypoints[i].y + 0.2
            await self._teleport_client.call_async(tel_pos)

        pen_status.off = 255
        await self._pen_client.call_async(pen_status)
        # Teleporting the turtle back to the first waypoint
        tel_pos.x = request.waypoints[0].x
        tel_pos.y = request.waypoints[0].y
        await self._teleport_client.call_async(tel_pos)

        # Setting the pen red to plot path while traversing the waypoints        
        pen_status.r = 255
        pen_status.g = 0
        pen_status.b = 0
        pen_status.off = 0
        await self._pen_client.call_async(pen_status)

        response.distance = minimum_distance(request.waypoints)
        self.straight_line_distance = response.distance

        self.get_logger().info("Waypoint Callback Done!")
        return response

def main(args=None):
    rclpy.init(args=args)

    waypoint = Waypoint()

    rclpy.spin(waypoint)

    waypoint.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()