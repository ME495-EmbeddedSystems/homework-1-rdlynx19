import rclpy
from rclpy.node import Node

import math

import rclpy.parameter
from std_srvs.srv import Empty

from geometry_msgs.msg import Twist, Vector3, Point

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

def compute_distance(waypoints):
    total_dist = 0.0
    for i in range(0,len(waypoints)-1):
        total_dist += math.sqrt((waypoints[i].x - waypoints[i+1].x)**2 + (waypoints[i].y - waypoints[i+1].y)**2)
    
    return total_dist

def calculate_tolerance(current_pos, current_waypoint):
    tol = math.sqrt((current_pos.x - current_waypoint.x)**2 + (current_pos.y - current_waypoint.y)**2)

    return tol



class Waypoint(Node):
    """The waypoint Node"""

    def __init__(self):
        super().__init__('waypoint')
        self.get_logger().info('waypoint_node')

        self.pub = self.create_publisher(Twist, "cmd_vel", 1)

        self._metric_pub = self.create_publisher(ErrorMetric, "loop_metrics", 1)
        
        self.sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

        self.current_pose = Pose()

        self.waypoints = []
        self.actual_path = []

        self.waypoints_count = 0
        self.i = 0
    
        self.declare_parameter('frequency',90)
        # Can change the parameter value during runtime but change is not effective
        self.freq = self.get_parameter('frequency').value

        self.declare_parameter('turtle_state','STOPPED')

        self.cbgroup = MutuallyExclusiveCallbackGroup()
        
        self.timer = self.create_timer((1/self.freq),self.timer_callback)

        self._vel_timer = self.create_timer((1), self.vel_timer_callback)

        self._srv = self.create_service(Empty,'toggle', self.toggle_callback)

        self._interface_srv = self.create_service(Waypoints, 'load', self.waypoints_callback)

        self._reset_client = self.create_client(Empty, "reset", callback_group=self.cbgroup)
        
        self._teleport_client = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute", callback_group=self.cbgroup)
        
        self._pen_client = self.create_client(SetPen, "/turtle1/set_pen", callback_group=self.cbgroup)

    def timer_callback(self):
        tim_turt_state = self.get_parameter('turtle_state').get_parameter_value().string_value
        if(tim_turt_state == 'MOVING'):

            self.get_logger().debug("Issuing Command!") 
            # twist = turtle_twist([4.5, 0.0, 0.0], [0.0, 0.0, -2.0])
            # self.pub.publish(twist)
    
    def pose_callback(self, msg):
        # self.get_logger().info("Theta value is: '%s'" %msg.theta)
        # Write a way to store the received message data
        self.current_pose = msg
        self.actual_path.append(msg)
        


    def vel_timer_callback(self):
        turt_state = self.get_parameter('turtle_state').get_parameter_value().string_value
        if(turt_state == "MOVING"):
            if (self.i < len(self.waypoints) and calculate_tolerance(self.current_pose, self.waypoints[self.i]) < 0.05):
                self.i += 1
                if(self.i == len(self.waypoints)):
                    self.get_logger().info("Loop Complete!")
                    # self.toggle_callback(Empty.Request(),Empty.Response())
                    

                    # Update Error Metric Values
                    loop_stats = ErrorMetric()
                    loop_stats.complete_loops += 1
                    self._metric_pub.publish(loop_stats)

                    self.i = 0
                    # return 

            
            yaw_vel =  (math.atan2(self.waypoints[self.i].y - self.current_pose.y, self.waypoints[self.i].x - self.current_pose.x) - self.current_pose.theta)
            x_vel = 0.6 * calculate_tolerance(self.current_pose, self.waypoints[self.i])
            robot_velocity = turtle_twist([x_vel, 0.0, 0.0], [0.0, 0.0, yaw_vel])
            self.pub.publish(robot_velocity)
        # self.toggle_callback(Empty.Request(), Empty.Response())


    def toggle_callback(self,request,response):
        turt_state = self.get_parameter('turtle_state').get_parameter_value().string_value
        if(turt_state == 'MOVING'):
            self.get_logger().info('Stopping!')
            
            move_2_stop = rclpy.parameter.Parameter('turtle_state', rclpy.Parameter.Type.STRING, 'STOPPED')
            mv_switch = [move_2_stop]
            self.set_parameters(mv_switch)
            return response
        elif(turt_state == 'STOPPED'):
            # Add the part of leaving a visual trail behind while moving !!!!
            stop_2_move = rclpy.parameter.Parameter('turtle_state', rclpy.Parameter.Type.STRING, 'MOVING')
            stp_switch = [stop_2_move]
            self.set_parameters(stp_switch)

            if(len(self.waypoints) == 0):
                self.get_logger().error("No waypoints loaded. Load them with the \"load\" service ")
                return response 

            self.waypoints_count = len(self.waypoints)
            return response
            
    async def waypoints_callback(self, request, response):
        # calling the turtlesim /reset service to reset the turtle
        while not self._reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Reset service not available, waiting again")
        await self._reset_client.call_async(Empty.Request())
        
        # Think about passing geometry_msgs/Point as service call arguments -- cite Joe for the syntax
        while not self._teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Teleport service not available, waiting again")
        
        tel_pos = TeleportAbsolute.Request()

        while not self._pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Set Pen service not available, waiting again")

        self.waypoints = request.waypoints

        pen_status = SetPen.Request()
        pen_status.r = 255
        pen_status.g = 255
        pen_status.b = 255
        pen_status.width = 4

 
        for i in range(0,len(request.waypoints)):
            pen_status.off = 255
            await self._pen_client.call_async(pen_status)
            tel_pos.x = request.waypoints[i].x
            tel_pos.y = request.waypoints[i].y      
            await self._teleport_client.call_async(tel_pos)
            # write code to make turtlesim draw X
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
        tel_pos.x = request.waypoints[0].x
        tel_pos.y = request.waypoints[0].y
        await self._teleport_client.call_async(tel_pos)

        move_2_stop = rclpy.parameter.Parameter('turtle_state', rclpy.Parameter.Type.STRING, 'MOVING')
        mv_switch = [move_2_stop]
        self.set_parameters(mv_switch) 
        self.toggle_callback(Empty.Request(),Empty.Response())
        
        pen_status.r = 255
        pen_status.g = 0
        pen_status.b = 0
        pen_status.off = 0
        await self._pen_client.call_async(pen_status)

        response.distance = compute_distance(request.waypoints)

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