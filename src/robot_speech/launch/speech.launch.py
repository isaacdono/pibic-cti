# speech.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

# Caminho do Python do venv
PYTHON_VENV = os.path.expanduser('~/Documents/pibic-cti/projeto-raia/.venv/bin/python3.12')

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[PYTHON_VENV, '-m', 'robot_speech.stt_node'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[PYTHON_VENV, '-m', 'robot_speech.tts_node'],
            output='screen'
        ),
    ])
