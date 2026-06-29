import rclpy
from rclpy.node import Node
import numpy as np
import math

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped,PointStamped
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray

class Geotagger(Node):
    def __init__(self):
        super().__init__('geotagger')
        
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            mavros_qos
        )
        self.yolo_sub=self.create_subscription(
            Detection2DArray,
            '/detections',
            self.yolo_callback,
            10
        )
        self.info_sub=self.create_subscription(
        CameraInfo,
        '/camera/camera_info',
        self.camera_info_callback,
        10
        )
        
        self.raw_tag_pub = self.create_publisher(PointStamped, '/sar/raw_tags', 10)
        
        self.drone_gps_position=[0.0,0.0,0.0]
        self.pose_received=False
        self.yaw=0.0
        self.pitch=0.0
        self.roll=0.0
        self.quaternion_received=False
        
        self.f_x=0.0
        self.f_y=0.0
        self.o_x=0.0
        self.o_y=0.0
        self.z_ground=0.0
        self.intrinsics_ready= False
        
        
        self.get_logger().info('Geotagger initialised. Waiting for YOLOv8, MAVROS and CameraInfo data...')
    def camera_info_callback(self, msg):
        if self.intrinsics_ready:
            return
        self.f_x=msg.k[0]
        self.o_x=msg.k[2]
        self.f_y=msg.k[4]
        self.o_y=msg.k[5]
        
        if self.f_x==0.0 or self.f_y==0.0:
            self.get_logger().error("Received invalid K matrix from camera_info!")
            return
        self.intrinsics_ready=True
        
    def pose_callback(self, msg):
        self.drone_gps_position=[
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        
        qx=msg.pose.orientation.x
        qy=msg.pose.orientation.y
        qz=msg.pose.orientation.z
        qw=msg.pose.orientation.w
        self.yaw, self.pitch, self.roll=quaternion_to_euler(qx,qy,qz,qw)
        self.pose_received=True
        self.quaternion_received=True
    def yolo_callback(self, msg):
        

        if not self.pose_received:
            self.get_logger().warn("Waiting for drone position...")
            return
        if not self.intrinsics_ready:
            self.get_logger().warn("Waitting for camera info...")
            return
        
        if not self.quaternion_received:
            self.get_logger().warn("Waiting for drone orientation...")
            return
        
        
        for detection in msg.detections:    
            u=detection.bbox.center.position.x
            v=detection.bbox.center.position.y
            track_id=detection.id
            ground_point=self.geotagging(u,v)
            
            if ground_point is None:
                continue
            
            raw_tag=PointStamped()
            raw_tag.header.stamp = self.get_clock().now().to_msg()
            raw_tag.header.frame_id = str(track_id)
            raw_tag.point.x = ground_point[0]
            raw_tag.point.y = ground_point[1]
            raw_tag.point.z = ground_point[2]
            self.raw_tag_pub.publish(raw_tag)
            self.get_logger().info(f'Detection id={track_id} at pixel ({u:.0f},{v:.0f}) corresopndingto the point on the ground ({ground_point[0]:.3f},{ground_point[1]:.3f})')
        
    def geotagging(self,u,v):
    
        v_camera=self.pixel_to_camera(u,v)
        R_drone_to_camera=np.array([[0,0,1],
                                    [0,1,0],
                                    [-1,0,0]])
        R_world_to_camera=self.get_world_to_camera_rotation_matrix(R_drone_to_camera)
        v_world=self.camera_ray_to_world_ray(v_camera,R_world_to_camera)
    
        if np.isclose(v_world[2],0.0):
            return None
        
        point_location=self.get_ground_point_from_world_ray(v_world)
    
        return point_location
        
    def pixel_to_camera (self,u,v):
        x_optical=(u-self.o_x)/self.f_x
        y_optical=(v-self.o_y)/self.f_y
        z_optical=1
        v_optical=np.array([x_optical,y_optical,z_optical])
        R_camera_to_optical=np.array([[0,0,1],
                                      [-1,0,0],
                                      [0,-1,0]])  
        v_camera=np.dot(R_camera_to_optical,v_optical) 
    
        return v_camera
        
    def get_world_to_camera_rotation_matrix(self,R_drone_to_camera):
        psi=self.yaw
        theta=self.pitch
        phi=self.roll
    
    
        # R_world_to_drone= R_z * R_y * R_x   
 
        R_z=np.array([[np.cos(psi), -np.sin(psi), 0],[np.sin(psi), np.cos(psi), 0],[0,0,1]])
        R_y=np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
        R_x=np.array([[1,0,0],[0,np.cos(phi),-np.sin(phi)],[0,np.sin(phi),np.cos(phi)]])
        R_world_to_drone=np.dot(R_z,np.dot(R_y,R_x))

        # R_world_to_camera= R_world_to_drone * R_drone_to_camera
    
        R_world_to_camera=np.dot(R_world_to_drone,R_drone_to_camera)
    
        return R_world_to_camera

    def camera_ray_to_world_ray(self,v_camera,R_world_to_camera):
    
        
        v_world=np.dot(R_world_to_camera,v_camera)
    
        return v_world
        
    def get_ground_point_from_world_ray(self,v_world):
        x_drone,y_drone,z_drone=self.drone_gps_position  #these are the x,y,z coordintes of the dorne's position in world coordinates- basically the components of t vector
        if abs(v_world[2])<1e-6:
            return None
        
        s=(z_drone-self.z_ground)/-v_world[2]     # - since v_world[2] is going to be negative as the vector's z component(it is a world coordinate) points along negative world z axis
        if s<0:
            return None
            
        # here z_ground is the world z coordinate of the ground level
        x_point=x_drone+s*v_world[0]
        y_point=y_drone+s*v_world[1]
        z_point=self.z_ground
    
        return np.array([x_point,y_point,z_point])


def quaternion_to_euler(x,y,z,w):
    sinr_cosp= 2*(w*x+y*z)
    cosr_cosp= 1-2*(x*x+y*y)
    roll= math.atan2(sinr_cosp, cosr_cosp)
    
    sinp= 2*(w*y-z*x)
    if abs(sinp) >= 1:
        pitch=math.copysign(math.pi/2,sinp)
    else:
        pitch=math.asin(sinp)
    
    siny_cosp=2*(w*z+x*y)
    cosy_cosp=1-2*(y*y+z*z)
    yaw=math.atan2(siny_cosp, cosy_cosp)
    
    return yaw,pitch,roll   

def main(args=None):
    rclpy.init(args=args)
    node=Geotagger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__=='__main__':
    main()
