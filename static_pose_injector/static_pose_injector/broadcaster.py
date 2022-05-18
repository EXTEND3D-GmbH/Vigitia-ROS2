import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.parameter import Parameter
from rclpy.serialization import deserialize_message

from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import ParameterType, ParameterDescriptor

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from ros2bag import command
from geometry_msgs.msg import TransformStamped

from lxml import etree
import numpy as np
import quaternion


class FramePublisher(Node):
    def __init__(self):
        super().__init__("static_pose_injector")
        self._tf_publisher = StaticTransformBroadcaster(self)
        self._timer_period = 0.5  # seconds

        # Parameters
        werklicht_extrinsic_path = ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                                       description='Path to the werklicht extrinsic xml')
        self.declare_parameter('werklicht_extrinsic_path',
                               'extrinsics.xml',
                               werklicht_extrinsic_path)

        self.werklicht_transform = TransformStamped()

        self.parse_werklicht_extrinsics(self.get_parameter(
            "werklicht_extrinsic_path").get_parameter_value().string_value)

        self.add_on_set_parameters_callback(self.parameter_change_callback)

        self._timer = self.create_timer(
            self._timer_period, self.timer_callback)

    def timer_callback(self):
        self._tf_publisher.sendTransform(self.werklicht_transform)

    def parameter_change_callback(self, data):
        for parameter in data:
            if parameter.name == "werklicht_extrinsic_path":
                if parameter.type_ == Parameter.Type.STRING:
                    self.parse_werklicht_extrinsics(parameter.value)

        return SetParametersResult(successful=True)

    def parse_werklicht_extrinsics(self, path):
        self.get_logger().info(f"werklicht_extrinsic_path: {path}")

        success, tf = self._parse_transform(path)
        if success:
            self.werklicht_transform = tf
            # Inverting transform to not change kinect camera system
            self.werklicht_transform.header.frame_id = 'rgb_camera_link'
            self.werklicht_transform.header.stamp = self.get_clock().now().to_msg()
            self.werklicht_transform.child_frame_id = 'projector_link'

            rot = self.werklicht_transform.transform.rotation
            translation = self.werklicht_transform.transform.translation
            
            q = np.quaternion(rot.w, rot.x, rot.y, rot.z)
            q_t = q.conjugate()
            
            t = np.array([translation.x, translation.y, translation.z])
            t = quaternion.rotate_vectors(q_t, t)
            t = -t
            
            rot.w = q_t.w
            rot.x = q_t.x
            rot.y = q_t.y
            rot.z = q_t.z
            
            translation.x = t[0]
            translation.y = t[1]
            translation.z = t[2]

    def _parse_transform(self, path):
        try:
            root = etree.parse(path)

            quat = root.find("Quaternion")
            if quat is None or 'w' not in quat.attrib or 'x' not in quat.attrib or 'y' not in quat.attrib or 'z' not in quat.attrib:
                self.get_logger().error("Parsing failed. Could not find 'Quaternion'")
                return

            translation = root.find("Translation")
            if translation is None or 'x' not in translation.attrib or 'y' not in translation.attrib or 'z' not in translation.attrib:
                self.get_logger().error("Parsing failed. Could not find 'Translation'")
                return

            result = TransformStamped()
            result.transform.translation.x = float(translation.attrib['x']) 
            result.transform.translation.y = float(translation.attrib['y']) 
            result.transform.translation.z = float(translation.attrib['z']) 
            result.transform.rotation.w = float(quat.attrib['w'])
            result.transform.rotation.x = float(quat.attrib['x'])
            result.transform.rotation.y = float(quat.attrib['y'])
            result.transform.rotation.z = float(quat.attrib['z'])

            self.get_logger().warning(
                f"Parsed extrinsics:\n{result.transform.translation.x}, {result.transform.translation.y}, {result.transform.translation.z}")
            return True, result
        except OSError as e:
            self.get_logger().error("Failed to load extrinsics")

        return False, TransformStamped()


def main(args=None):
    rclpy.init(args=args)

    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
