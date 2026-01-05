from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nurse_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['resource/.env']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.pt')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.npy')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.task')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mock_voice = nurse_bot.mock_voice_server:main',
            'mock_vision = nurse_bot.mock_vision_server:main',
            'nurse_ctrl = nurse_bot.nurse_controller:main_logic',
            'real_vision = nurse_bot.real_vision_server:main',
            'real_voice = nurse_bot.real_voice_server:main',
        ],
    },
)
