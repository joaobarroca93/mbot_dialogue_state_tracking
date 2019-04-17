#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['mbot_dst_rule_based', 'mbot_dst_rule_based_ros'],
 package_dir={'mbot_dst_rule_based': 'common/src/mbot_dst_rule_based', 'mbot_dst_rule_based_ros': 'ros/src/mbot_dst_rule_based_ros'}
)

setup(**d)
