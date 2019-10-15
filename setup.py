# NOTE:
# This file has been configured to package the core inference functionality.
# Asynchronous/ROS support is available, but currently unpackaged.
# If we want to operate DOPE online, we need to reconfigure to support the original packaging.

requirements = [
    'pyrr==0.9.2',
    'torch',
    'numpy',
    'scipy',
    'opencv_python',
    'Pillow',
    'torchvision',
    'transforms3d',
    'PyYAML'
]

setuptools.setup(
    name="dope",
    version="0.0.1",
    install_requires=requirements,
    author="Jonathan Tremblay and Thang To and Balakumar Sundaralingam and Yu Xiang and Dieter Fox and Stan Birchfield",
    description="Deep Object Pose Estimation for Semantic Robotic Grasping of Household Objects",
    packages = ['dope', 'dope.inference'],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
)
