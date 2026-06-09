import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
import time

class DroneMission(Node):
    def __init__(self):
        super().__init__('drone_mission')
        
        # 1. Create ALL required clients at once
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        # 2. Wait for MAVROS just once
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MAVROS services...')

        self.get_logger().info('Connected! Starting automated mission.')
        self.execute_mission()

    def execute_mission(self):
        # --- PHASE 1: PREPARATION ---
        self.get_logger().info('Setting mode to GUIDED...')
        mode_req = SetMode.Request()
        mode_req.custom_mode = 'GUIDED'
        self.mode_client.call_async(mode_req)
        time.sleep(0) # Wait 2 seconds for the drone to process the mode change

        self.get_logger().info('Arming motors...')
        arm_req = CommandBool.Request()
        arm_req.value = True
        self.arm_client.call_async(arm_req)
        time.sleep(2) # Wait 2 seconds for propellers to spin up

        # --- PHASE 2: TAKEOFF ---
        self.get_logger().info('Taking off to 10 meters...')
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = 10.0
        self.takeoff_client.call_async(takeoff_req)
        
        # --- PHASE 3: THE MISSION WORK ---
        self.get_logger().info('Hovering for 15 seconds to simulate taking photos...')
        time.sleep(15) # Pause the script while the drone hovers

        # --- PHASE 4: RETURN TO EARTH ---
        self.get_logger().info('Mission complete. Landing...')
        mode_req.custom_mode = 'LAND'
        self.mode_client.call_async(mode_req)

def main(args=None):
    rclpy.init(args=args)
    node = DroneMission()
    
    # Keep the node alive long enough for the final landing message to send
    rclpy.spin_once(node, timeout_sec=0)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()