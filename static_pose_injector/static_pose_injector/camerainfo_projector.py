import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.parameter import Parameter

from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import ParameterType, ParameterDescriptor

from sensor_msgs.msg import CameraInfo

from lxml import etree


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__("camerainfo_projector")
        self._ci_publisher = self.create_publisher(
            CameraInfo, "/projector/camera_info", 10)
        self._has_intrinsics = False
        self._timer_period = 0.5  # seconds

        # Parameters
        werklicht_intrinsic_path = ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                                       description='Path to the werklicht intrinsic xml')
        self.declare_parameter('werklicht_intrinsic_path',
                               'intrinsics.xml',
                               werklicht_intrinsic_path)

        self.werklicht_intrinsics = CameraInfo()

        self.parse_werklicht_intrinsics(self.get_parameter(
            "werklicht_intrinsic_path").get_parameter_value().string_value)

        self.add_on_set_parameters_callback(self.parameter_change_callback)

        self._timer = self.create_timer(
            self._timer_period, self.timer_callback)

    def timer_callback(self):
        if self._has_intrinsics:
            self._ci_publisher.publish(self.werklicht_intrinsics)

    def parameter_change_callback(self, data):
        for parameter in data:
            if parameter.name == "werklicht_intrinsic_path":
                if parameter.type_ == Parameter.Type.STRING:
                    self.parse_werklicht_intrinsics(parameter.value)

        return SetParametersResult(successful=True)

    def parse_werklicht_intrinsics(self, path):
        self.get_logger().info(f"werklicht_intrinsics: {path}")

        self.werklicht_intrinsics.header.frame_id = 'Projector'
        self.werklicht_intrinsics.header.stamp = self.get_clock().now().to_msg()
        try:
            root = etree.parse(path)

            intrinsics = root.find("Intrinsics")
            if intrinsics is None \
                    or 'm11' not in intrinsics.attrib or 'm12' not in intrinsics.attrib or 'm13' not in intrinsics.attrib \
                    or 'm21' not in intrinsics.attrib or 'm22' not in intrinsics.attrib or 'm23' not in intrinsics.attrib \
                    or 'm31' not in intrinsics.attrib or 'm32' not in intrinsics.attrib or 'm33' not in intrinsics.attrib:
                self.get_logger().error("Parsing failed. Could not find 'Intrinsics'")
                return

            distortion = root.find("Distortion")
            if distortion is None or 'a' not in distortion.attrib or 'b' not in distortion.attrib \
                    or 'c' not in distortion.attrib or 'd' not in distortion.attrib or 'e' not in distortion.attrib \
                    or 'f' not in distortion.attrib or 'g' not in distortion.attrib or 'h' not in distortion.attrib:
                self.get_logger().error("Parsing failed. Could not find 'Distortion'")
                return

            image = root.find("Image")
            if image is None or 'Width' not in image.attrib or 'Height' not in image.attrib:
                self.get_logger().error("Parsing failed. Could not find 'Image'")
                return

            self.werklicht_intrinsics.width = int(image.attrib["Width"])
            self.werklicht_intrinsics.height = int(image.attrib["Height"])

            self.werklicht_intrinsics.distortion_model = "rational_polynomial"
            self.werklicht_intrinsics.d = [float(distortion.attrib["a"]),
                                           float(distortion.attrib["b"]),
                                           float(distortion.attrib["c"]),
                                           float(distortion.attrib["d"]),
                                           float(distortion.attrib["e"]),
                                           float(distortion.attrib["f"]),
                                           float(distortion.attrib["g"]),
                                           float(distortion.attrib["h"])]

            self.werklicht_intrinsics.k = [float(intrinsics.attrib["m11"]),
                                           float(intrinsics.attrib["m12"]),
                                           float(intrinsics.attrib["m13"]),
                                           float(intrinsics.attrib["m21"]),
                                           float(intrinsics.attrib["m22"]),
                                           float(intrinsics.attrib["m23"]),
                                           float(intrinsics.attrib["m31"]),
                                           float(intrinsics.attrib["m32"]),
                                           float(intrinsics.attrib["m33"])]

            self.werklicht_intrinsics.p = [float(intrinsics.attrib["m11"]),
                                           float(intrinsics.attrib["m12"]),
                                           float(intrinsics.attrib["m13"]),
                                           0.0,
                                           float(intrinsics.attrib["m21"]),
                                           float(intrinsics.attrib["m22"]),
                                           float(intrinsics.attrib["m23"]),
                                           0.0,
                                           float(intrinsics.attrib["m31"]),
                                           float(intrinsics.attrib["m32"]),
                                           float(intrinsics.attrib["m33"]),
                                           0.0]

            self.werklicht_intrinsics.r = [1.0,
                                           0.0,
                                           0.0,
                                           1.0,
                                           0.0,
                                           0.0,
                                           1.0,
                                           0.0,
                                           0.0]
            self.get_logger().warn("Parsed Werklicht intrinsics")
            self._has_intrinsics = True
            self._ci_publisher.publish(self.werklicht_intrinsics)
        except OSError as e:
            self.get_logger().error("Failed to load Werklicht intrinsics")


def main(args=None):
    rclpy.init(args=args)

    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
