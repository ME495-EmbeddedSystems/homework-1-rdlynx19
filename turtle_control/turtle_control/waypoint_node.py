import rclpy
from rclpy.node import Node

import rclpy.parameter
from std_srvs.srv import Empty

from geometry_msgs.msg import Twist, Vector3

from turtle_interfaces.srv import Waypoints

from turtlesim.srv import TeleportAbsolute

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




class Waypoint(Node):
    """The waypoint Node"""

    def __init__(self):
        super().__init__('waypoint')
        self.get_logger().info('waypoint_node')

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)


        self.declare_parameter('frequency',90)
        # Can change the parameter value during runtime but change is not effective
        self.freq = self.get_parameter('frequency').value

        self.declare_parameter('turtle_state','STOPPED')

        self.cbgroup = MutuallyExclusiveCallbackGroup()
        
        self.timer = self.create_timer((1/self.freq),self.timer_callback)
        self._srv = self.create_service(Empty,'toggle', self.toggle_callback)

        self._interface_srv = self.create_service(Waypoints, 'load', self.waypoints_callback)

        self._reset_client = self.create_client(Empty, "reset", callback_group=self.cbgroup)
        

        self._teleport_client = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute", callback_group=self.cbgroup)
        

    def timer_callback(self):
        tim_turt_state = self.get_parameter('turtle_state').get_parameter_value().string_value
        if(tim_turt_state == 'MOVING'):

            self.get_logger().debug("Issuing Command!") 
            twist = turtle_twist([4.5, 0.0, 0.0], [0.0, 0.0, -2.0])
            self.pub.publish(twist)
            # self._pose_pub.publish()
    
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

        for i in range(0,len(request.waypoints)):
            tel_pos.x = request.waypoints[i].x
            tel_pos.y = request.waypoints[i].y      
            await self._teleport_client.call_async(tel_pos)
            # write code to make turtlesim sleep
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