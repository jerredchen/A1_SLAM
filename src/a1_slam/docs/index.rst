A1 SLAM: Quadruped SLAM using the A1's onboard sensors
======================================================

.. image:: /misc/a1_slam_trajectory.gif

A1 SLAM is a rospackage that brings real-time SLAM capabilities utilizing factor graph optimization to Unitree's A1 quadruped.

.. note::

   This project is under active development.

Why use A1 SLAM?
----------------

Performance
^^^^^^^^^^^

A1 SLAM yields SLAM results competitive to the most prominent SLAM algorithms. Below are benchmarked Absolute Pose Error (APE) results compared to 
out-of-the-box `Cartographer <https://github.com/cartographer-project/cartographer/>`_, one of the most widely-used 2D SLAM libraries available.
The baseline was obtained using motion capture data.

=============  =============  =============  =============  =============
Algorithm      Trajectory 1   Trajectory 2*  Trajectory 3*  Trajectory 4
=============  =============  =============  =============  =============
Cartographer   **0.04049**    0.96218        1.39692        0.13435
A1 SLAM        0.04681        **0.11836**    **0.14518**    **0.04347**
=============  =============  =============  =============  =============
*Trajectories were recorded with noticeably more aggressive rotation and/or translation.

Convenience
^^^^^^^^^^^

A1 SLAM is intentionally designed around using the sensors that are already onboard the A1 quadruped. This means:
 * Static transforms between sensor frames and the base link are already established
 * Algorithms tested on the onboard sensors for obtaining good performance
 * Documentation is provided for obtaining onboard sensor data (that may not be clearly documented elsewhere)

Contents
--------

.. toctree::

   installation
   usage
