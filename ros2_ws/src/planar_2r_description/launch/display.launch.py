from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    return LaunchDescription([

        IncludeLaunchDescription(

            PathJoinSubstitution([
                FindPackageShare("urdf_launch"),
                "launch",
                "display.launch.py"
            ]),

            launch_arguments={

                "urdf_package": "planar_2r_description",

                "urdf_package_path": "urdf/planar_2r.urdf.xacro",

                "jsp_gui": "true",

            }.items()

        )

    ])