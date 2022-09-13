Installation
============

If you haven't already, make sure that you have ROS already installed using the instructions `here <http://wiki.ros.org/ROS/Installation/>`_.

GTSAM is the only dependency, which can be installed from `source <https://github.com/borglab/gtsam>`_ or with::

  pip install --user gtsam

The build process consists of::

  git clone https://github.com/jerredchen/A1_SLAM.git
  cd A1_SLAM && mkdir A1_SLAM
  catkin_make
