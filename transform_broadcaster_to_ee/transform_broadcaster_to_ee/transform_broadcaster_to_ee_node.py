import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformListener, Buffer
from tf_transformations import quaternion_from_matrix, quaternion_matrix, translation_from_matrix
import tf2_py
import numpy as np

def unpack(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    return [x, y, z, w]

class StaticTransformBroadcasterNode(Node):
    def __init__(self, t_path):
        super().__init__('static_transform_broadcaster_node')
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer_rgbtodepth = Buffer()
        self.listener_rgbtodepth = TransformListener(self.tf_buffer_rgbtodepth, self, spin_thread=True)
        self.tf_buffer_depthtobase = Buffer()
        self.listener_depthtobase = TransformListener(self.tf_buffer_depthtobase, self, spin_thread=True)

        # Load the transformation matrix from the npz file
        loaded_data = np.load(t_path)
        transform_matrix_basetorgb = loaded_data['arr_0']
        
        #q_basetorgb = quaternion_from_matrix(transform_matrix_basetorgb)
        #t_basetorgb = transform_matrix_basetorgb[:,3]
        #print("quarternion:", q_basetorgb)
        #print("Translation:", t_basetorgb)



        try:
            transform_rgbtodepth = self.tf_buffer_rgbtodepth.lookup_transform(
                target_frame='depth_camera_link',
                source_frame='rgb_camera_link',
                time=rclpy.time.Time(),
                timeout = rclpy.duration.Duration(seconds=5.0)
            )
            transform_matrix_rgbtodepth = np.identity(4)
            q_rgbtodepth = unpack(transform_rgbtodepth.transform.rotation)
            transform_matrix_rgbtodepth[:3, :3] = quaternion_matrix(q_rgbtodepth)[:3,:3]
            transform_matrix_rgbtodepth[0,3] = transform_rgbtodepth.transform.translation.x
            transform_matrix_rgbtodepth[1,3] = transform_rgbtodepth.transform.translation.y
            transform_matrix_rgbtodepth[2,3] = transform_rgbtodepth.transform.translation.z
            print(transform_matrix_rgbtodepth)
        except Exception as e:
            self.get_logger().error(f'Error looking up transform: {str(e)}')

        try:
            transform_depthtobase = self.tf_buffer_depthtobase.lookup_transform(
                target_frame='camera_base',
                source_frame='depth_camera_link',
                time=rclpy.time.Time(),
                timeout = rclpy.duration.Duration(seconds=5.0)
            )
            transform_matrix_depthtobase = np.identity(4)
            q_depthtobase = unpack(transform_depthtobase.transform.rotation)
            transform_matrix_depthtobase[:3, :3] = quaternion_matrix(q_depthtobase)[:3,:3]
            transform_matrix_depthtobase[0,3] = transform_depthtobase.transform.translation.x
            transform_matrix_depthtobase[1,3] = transform_depthtobase.transform.translation.y
            transform_matrix_depthtobase[2,3] = transform_depthtobase.transform.translation.z
            print(transform_matrix_depthtobase)
        except Exception as e:
            self.get_logger().error(f'Error looking up transform: {str(e)}')

        # q_eetobase * q_basetodepth * q_depthtorgb = q_eetorgb
        # q_eetobase = q_eetorgb * (q_basetodepth)-1 * (q_depthtorgb)-1
        # q_eetobase = (q_eetorgb * q_depthtobase) * q_rgbtodepth

        transformation_matrix = np.dot(np.dot(transform_matrix_basetorgb, transform_matrix_depthtobase), transform_matrix_rgbtodepth)
        translation = translation_from_matrix(transformation_matrix)
        rotation = quaternion_from_matrix(transformation_matrix)
        
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rclpy.time.Time().to_msg()
        transform_stamped.header.frame_id = 'xarm_gripper_base_link'
        transform_stamped.child_frame_id = 'camera_base'

        # Set the transformation matrix elements
        transform_stamped.transform.translation.x = translation[0]
        transform_stamped.transform.translation.y = translation[1]
        transform_stamped.transform.translation.z = translation[2]
        transform_stamped.transform.rotation.x = rotation[0]
        transform_stamped.transform.rotation.y = rotation[1]
        transform_stamped.transform.rotation.z = rotation[2]
        transform_stamped.transform.rotation.w = rotation[3]

        rot2 = quaternion_from_matrix(np.identity(4))

        transform_stamped2 = TransformStamped()
        transform_stamped2.header.stamp = rclpy.time.Time().to_msg()
        transform_stamped2.header.frame_id = 'linear_direct_motor_base'
        transform_stamped2.child_frame_id = 'link_base'
        transform_stamped2.transform.translation.x = 0.0
        transform_stamped2.transform.translation.y = 0.0
        transform_stamped2.transform.translation.z = 0.135/2
        transform_stamped2.transform.rotation.x = rot2[0]
        transform_stamped2.transform.rotation.y = rot2[1]
        transform_stamped2.transform.rotation.z = rot2[2]
        transform_stamped2.transform.rotation.w = rot2[3]


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
        rot = np.dot(x_rot,y_rot)

        rot3 = quaternion_from_matrix(rot)

        transform_stamped3 = TransformStamped()
        transform_stamped3.header.stamp = rclpy.time.Time().to_msg()
        transform_stamped3.header.frame_id = 'xarm_gripper_base_link'
        transform_stamped3.child_frame_id = 'camera_base'
        transform_stamped3.transform.translation.x = 0.08 # reposition camera_base 
        transform_stamped3.transform.translation.y = 0.0
        transform_stamped3.transform.translation.z = 0.0
        transform_stamped3.transform.rotation.x = rot3[0]
        transform_stamped3.transform.rotation.y = rot3[1]
        transform_stamped3.transform.rotation.z = rot3[2]
        transform_stamped3.transform.rotation.w = rot3[3]

        # Broadcast the static transform
        self.static_broadcaster.sendTransform([transform_stamped3, transform_stamped2])


def main():
    rclpy.init()

    # Replace 'your_npz_file_path.npz' with the actual path to your .npz file
    t_path = '/home/nam017/ws_calibration/src/HandEyeCalibration-using-OpenCV/FinalTransforms/T_gripper2cam_Method_3.npz'

    node = StaticTransformBroadcasterNode(t_path)
    rclpy.spin(node)
    node.destory_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
