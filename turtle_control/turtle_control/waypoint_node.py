import rclpy
from rclpy.node import Node

import rclpy.parameter
from std_srvs.srv import Empty

class Waypoint(Node):
    """The waypoint Node"""

    def __init__(self):
        super().__init__('waypoint')
        self.get_logger().info('way_node')

        self.declare_parameter('frequency',90)
        # Can change the parameter value during runtime but change is not effective
        self.freq = self.get_parameter('frequency').value

        self.declare_parameter('turtle_state','STOPPED')
        
        self.timer = self.create_timer((1/self.freq),self.timer_callback)
        self._srv = self.create_service(Empty,'toggle', self.toggle_callback)

    def timer_callback(self):
        tim_turt_state = self.get_parameter('turtle_state').get_parameter_value().string_value
        if(tim_turt_state == 'MOVING'):

            self.get_logger().debug("Issuing Command!") 
    
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
            
   

def main(args=None):
    rclpy.init(args=args)

    waypoint = Waypoint()

    rclpy.spin(waypoint)

    waypoint.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()