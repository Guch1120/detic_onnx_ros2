from setuptools import setup
import os
from glob import glob

package_name = "detic_onnx_ros2"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("./launch/*.launch.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yusei Nagao",
    maintainer_email="nagaoaustineo@gmail.com",
    description="Run detic inference with ROS 2 image.",
    license="Apache 2.0",
    # tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detic_onnx_ros2_node = detic_onnx_ros2.detic_onnx_ros2:main",
            "detic_onnx_ros2_realsense = detic_onnx_ros2.detic_onnx_ros2_realsense:main",
            "detic_onnx_clip_ros2_node = detic_onnx_ros2.detic_onnx_clip_ros2:main",
            "detic_onnx_obj = detic_onnx_ros2.detic_onnx_obj:main",
            "detic_onnx_obj_srv = detic_onnx_ros2.detic_onnx_obj_srv:main",
            "srv_test = detic_onnx_ros2.srv_test:main"
        ],
    },
)
