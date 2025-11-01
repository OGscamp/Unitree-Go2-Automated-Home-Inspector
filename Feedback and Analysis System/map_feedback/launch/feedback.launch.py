from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    python_exe = os.environ.get("PYTHON_EXECUTABLE", "python3")

    return LaunchDescription([
        ExecuteProcess(
            cmd=[python_exe, "-m", "map_feedback.map_feedback_node"],
            output="screen"
        )
    ])