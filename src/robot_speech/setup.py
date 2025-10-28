from setuptools import setup
from glob import glob
import os

package_name = 'robot_speech'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='isaac',
    maintainer_email='isaac052503@gmail.com',
    description='Nós de STT e TTS para o robô',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'stt_node = robot_speech.stt_node:main',
            'tts_node = robot_speech.tts_node:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
    ],
)
