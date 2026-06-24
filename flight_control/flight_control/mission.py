#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        
        self.current_state = State()
        
        # 1. Subscriber to constantly monitor the drone's brain
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        
        # 2. Service Clients to send commands
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        
        # 3. Main Mission Loop (runs every 2 seconds)
        self.timer = self.create_timer(2.0, self.mission_loop)
        self.phase = 0
        
    def state_cb(self, msg):
        # Update our internal state whenever MAVROS sends new data
        self.current_state = msg
        
    def mission_loop(self):
        # PHASE 0: Wait for connection
        if self.phase == 0:
            if self.current_state.connected:
                self.get_logger().info("🔗 Connected to Flight Controller!")
                self.phase = 1
            else:
                self.get_logger().info("Waiting for FCU connection...")
                
        # PHASE 1: Switch to GUIDED mode
        # Note: ArduPilot will reject this until it has a GPS lock!
        elif self.phase == 1:
            if self.current_state.mode != "GUIDED":
                self.get_logger().info("🔄 Requesting GUIDED mode (Waiting for GPS lock)...")
                self.set_mode("GUIDED")
            else:
                self.get_logger().info("✅ GUIDED mode confirmed.")
                self.phase = 2
                
        # PHASE 2: Arm the motors
        elif self.phase == 2:
            if not self.current_state.armed:
                self.get_logger().info("⚙️ Arming motors...")
                self.arm_motors(True)
            else:
                self.get_logger().info("✅ Motors armed.")
                self.phase = 3
                
        # PHASE 3: Takeoff
        elif self.phase == 3:
            self.get_logger().info("🚀 Sending Takeoff command (Altitude: 10m)...")
            self.takeoff(20.0)
            self.hover_ticks = 0  # Start a counter for our hover time
            self.phase = 4
            
        # PHASE 4: Hovering (Simulate doing work)
        elif self.phase == 4:
            self.hover_ticks += 1
            # Since the timer runs every 2 seconds, 5 ticks = 10 seconds
            self.get_logger().info(f"Hovering and scanning... ({self.hover_ticks * 2}/10 seconds)")
            
            if self.hover_ticks >= 5: 
                self.phase = 5
                
        # PHASE 5: Return to Earth
        elif self.phase == 5:
            self.get_logger().info("🛬 Mission complete. Commanding LAND mode...")
            self.set_mode("LAND")
            self.phase = 6
            
        # PHASE 6: Shutdown
        elif self.phase == 6:
            # Check if ArduPilot accepted the LAND command
            if self.current_state.mode == "LAND":
                self.get_logger().info("✅ Land mode confirmed. Shutting down node.")
                self.timer.cancel()  # Stop the loop, we are done!

    # --- Helper Functions to call ROS 2 Services ---
    def set_mode(self, custom_mode):
        req = SetMode.Request()
        req.custom_mode = custom_mode
        self.mode_client.call_async(req)
        
    def arm_motors(self, state):
        req = CommandBool.Request()
        req.value = state
        self.arm_client.call_async(req)
        
    def takeoff(self, altitude):
        req = CommandTOL.Request()
        req.altitude = altitude
        self.takeoff_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
