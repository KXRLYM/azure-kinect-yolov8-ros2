import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PointStamped
from tf2_ros import StaticTransformBroadcaster, TransformListener, Buffer
from tf_transformations import quaternion_from_matrix, quaternion_matrix, translation_from_matrix
import tf2_py
import numpy as np
from numpy.linalg import inv

def unpack(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    return [x, y, z, w]

# Define a function to create a transformation matrix from DH parameters
def dh_transform(theta, d, a, alpha):
    return np.array([[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                     [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                     [0, np.sin(alpha), np.cos(alpha), d],
                     [0, 0, 0, 1]])

# Function to compute the forward kinematics for the UR5 robot
def forward_kinematics(joint_angles, dh_parameters):
    T = np.eye(4)
    for i in range(len(joint_angles)):
        theta, d, a, alpha = dh_parameters[i]
        T_i = dh_transform(joint_angles[i] + theta, d, a, alpha)
        T = np.dot(T, T_i)
    return T

def make_transform(frame_id, T_final):
    rot3 = quaternion_from_matrix(T_final)

    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rclpy.time.Time().to_msg()
    transform_stamped.header.frame_id = 'world'
    transform_stamped.child_frame_id = frame_id
    transform_stamped.transform.translation.x = T_final[0,3]
    transform_stamped.transform.translation.y = T_final[1,3]
    transform_stamped.transform.translation.z = T_final[2,3]
    transform_stamped.transform.rotation.x = rot3[0]
    transform_stamped.transform.rotation.y = rot3[1]
    transform_stamped.transform.rotation.z = rot3[2]
    transform_stamped.transform.rotation.w = rot3[3]
    return transform_stamped


# We are going to broadcast four different hard-coded static transforms. 
class TestBroadcasterNode(Node):
    def __init__(self, t_path):
        super().__init__('test_broadcaster_node')
        
        loaded_data = np.load(t_path)
        transform_matrix_ee2rgb = loaded_data['arr_0']

        # DH parameters for UR5 from the provided image
        self.dh_params = [
            # theta, d, a, alpha
            (0, 0.267, 0, -np.pi/2), 
            (0, 0, 0.28949, 0),
            (0, 0, 0.0775, -np.pi/2),
            (0, 0.3425, 0, np.pi/2),
            (0, 0, 0.076, -np.pi/2),
            (0, 0.097, 0, 0),
        ]

        offset = 1.384
        # Joint angles for UR5 in degrees (to be converted to radians)
        # self.joint_positions_deg = [106.3, 30.1, -61.7, -27.2, 72.5, 259.1]
        self.joint_positions_centre = [0.887, -0.293-offset, -0.4518+offset, 0.002293, 0.8198, 2.4375]
        self.joint_positions_left = [1.846, 0.5252-offset, -1.07614+offset, -0.47445, 1.264992, 4.52253]
        self.joint_positions_right = [0.7404, 0.54756-offset, -1.68805+offset, 0.03938, 1.6763, 1.7867]
        self.joint_positions_back = [-0.1166, 0.83417-offset, -1.6146+offset, 0.48515, 1.98912, 1.39253]

        self.T_world2slider = np.array([
            [1,0,0,0],
            [0,1,0,0],
            [0,0,1,0.135/2],
            [0,0,0,1]
        ])
        
        self.T_ee2base = np.array([
            [1,0,0,0],
            [0,1,0,0],
            [0,0,1,0.013],
            [0,0,0,1]
        ])

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        
        try:
            transform_depth = self.tf_buffer.lookup_transform(
                target_frame = 'camera_base',
                source_frame = 'depth_camera_link',
                time = rclpy.time.Time(),
                timeout = rclpy.duration.Duration(seconds=5.0)
            )
            transform_matrix_depth = np.identity(4)
            q_depth = unpack(transform_depth.transform.rotation)
            transform_matrix_depth[:3, :3] = quaternion_matrix(q_depth)[:3,:3]
            transform_matrix_depth[0,3] = transform_depth.transform.translation.x
            transform_matrix_depth[1,3] = transform_depth.transform.translation.y
            transform_matrix_depth[2,3] = transform_depth.transform.translation.z
            print(transform_matrix_depth)
        except Exception as e:
            self.get_logger().error(f'Error looking up transform: {str(e)}')

        
        angle_x = np.pi
        angle_y = np.pi/2
        #angle_z = -np.pi/2
        rx = [[1,0,0],[0, np.cos(angle_x), -np.sin(angle_x)],[0, np.sin(angle_x), np.cos(angle_x)]]
        ry = [[np.cos(angle_y), 0, np.sin(angle_y)],[0, 1, 0],[-np.sin(angle_y), 0, np.cos(angle_y)]]
        #rz = [[np.cos(angle_z), np.sin(angle_z), 0],[np.sin(angle_z), np.cos(angle_z), 0],[0,0,1]]
        x_rot = np.identity(4)
        x_rot[:3, :3] = rx
        y_rot = np.identity(4)
        y_rot[:3,:3] = ry
        self.T_base2cam = np.dot(x_rot,y_rot)
        #self.T_base2cam = np.identity(4)
        #self.T_base2cam[:3,:3] = ry
        self.T_base2cam[0,3] = 0.08
        self.T_base2cam[1,3] = 0.0
        self.T_base2cam[2,3] = 0.0
        #rot[:3,:3] = np.dot(ry,rx)
        #transform_matrix_ee2rgb = np.identity(4)
        
        # use this if testing camera calibration
        T_centre = np.dot(np.dot(self.T_world2slider, forward_kinematics(self.joint_positions_centre, self.dh_params)), transform_matrix_ee2rgb)
        T_left = np.dot(np.dot(self.T_world2slider, forward_kinematics(self.joint_positions_left, self.dh_params)), transform_matrix_ee2rgb)
        T_right = np.dot(np.dot(self.T_world2slider, forward_kinematics(self.joint_positions_right, self.dh_params)), transform_matrix_ee2rgb)
        T_back = np.dot(np.dot(self.T_world2slider, forward_kinematics(self.joint_positions_back, self.dh_params)), transform_matrix_ee2rgb)
        
        # default camera transformation
        #T_centre = np.dot(np.dot(np.dot(np.dot(self.T_world2slider, forward_kinematics(self.joint_positions_centre, self.dh_params)), self.T_ee2base), self.T_base2cam), transform_matrix_depth)
        #T_left = np.dot(np.dot(np.dot(np.dot(self.T_world2slider, forward_kinematics(self.joint_positions_left, self.dh_params)), self.T_ee2base), self.T_base2cam), transform_matrix_depth)
        #T_right = np.dot(np.dot(np.dot(np.dot(self.T_world2slider, forward_kinematics(self.joint_positions_right, self.dh_params)), self.T_ee2base), self.T_base2cam), transform_matrix_depth)
        #T_back = np.dot(np.dot(np.dot(np.dot(self.T_world2slider, forward_kinematics(self.joint_positions_back, self.dh_params)), self.T_ee2base), self.T_base2cam), transform_matrix_depth)

        transforms = [make_transform("centre", T_centre), make_transform("left",T_left), make_transform("right",T_right), make_transform("back",T_back)]
        # Broadcast the static transform
        self.static_broadcaster.sendTransform(transforms)


        ############ testing transformation matrix ################
        self.test_matrix = np.array([[ 0.0101185,  -0.99837769, -0.05603213,  0.21688149],
                                    [-0.9990645,  -0.00773759, -0.042547 ,   0.28435518],
                                    [ 0.04204442,  0.05641022, -0.99752201,  0.24986517],
                                    [ 0.  ,        0.      ,    0.      ,    1.        ]])

        test_transform = np.dot(self.T_world2slider, self.test_matrix)
        final_test = make_transform("test", test_transform)
        self.static_broadcaster.sendTransform(transforms)


def main():
    rclpy.init()

    # Replace 'your_npz_file_path.npz' with the actual path to your .npz file
    t_path = '/home/nam017/ws_calibration/src/HandEyeCalibration-using-OpenCV/FinalTransforms/T_gripper2cam_Method_1.npz'

    node = TestBroadcasterNode(t_path)
    rclpy.spin(node)
    node.destory_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
