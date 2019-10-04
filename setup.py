# ## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
# 
# from distutils.core import setup
# from catkin_pkg.python_setup import generate_distutils_setup
# 
# # fetch values from package.xml
# setup_args = generate_distutils_setup(
#     packages=['dope'],
#     package_dir={'': 'src'})
# 
# setup(**setup_args)
# 
# import setuptools

# NOTE:
# This file has been configured to package the core inference functionality.
# Asynchronous/ROS support is available, but currently unpackaged.
# If we want to operate DOPE online, we need to reconfigure to support the original packaging.

with open("README.md", "r") as fh:
    long_description = fh.read()

with open('requirements.txt') as f:
    requirements = f.read().splitlines()

setuptools.setup(
    name="dope",
    version="0.0.1",
    install_requires=requirements,
    author="Jonathan Tremblay and Thang To and Balakumar Sundaralingam and Yu Xiang and Dieter Fox and Stan Birchfield",
    description="Deep Object Pose Estimation for Semantic Robotic Grasping of Household Objects",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages = ['dope'],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
)
