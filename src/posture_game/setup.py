from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'posture_game'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # Archivos de voz TTS
        (os.path.join('share', package_name, 'models/TTS'), glob('resource/models/TTS/*')),

        # Modelo de emociones
        (os.path.join('share', package_name, 'models/emotions'), glob('resource/models/emotions/*.h5')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erickgonza',
    maintainer_email='erickgonza@todo.todo',
    description='Sistema IA para acompañamiento terapéutico de niños con TDAH',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'game_manage = posture_game.game_manage:main',
            'checker_node = posture_game.checker_node:main',
            'camera_listener = posture_game.camera_listener_node:main',
            'dummy_subscriber = posture_game.dummy_subscriber:main',
            'speaker_node = posture_game.speaker_node:main',
            'detect_emotions_node = posture_game.detect_emotions_node:main',
        ],
    },
)
