# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/Applications/MATLAB_R2022a.app/sys/ros1/maci64/ros1/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/Applications/MATLAB_R2022a.app/sys/ros1/maci64/ros1/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/Applications/MATLAB_R2022a.app/sys/ros1/maci64/ros1;/Applications/MATLAB_R2022a.app/toolbox/ros/mlroscpp/custom_messages'.split(';'):
        python_path = os.path.join(workspace, 'lib/python3.9/site-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/Users/flyinglightspec/src/FLS-Multimedia2022/ROSbag/ROSpackages/matlab_msg_gen_ros1/maci64/devel/env.sh')

output_filename = '/Users/flyinglightspec/src/FLS-Multimedia2022/ROSbag/ROSpackages/matlab_msg_gen_ros1/maci64/build/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
