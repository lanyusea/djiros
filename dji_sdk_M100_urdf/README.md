##DJI Matrice100 URDF


### how to use

1. Install any possible ros packages at first

    ros-<>-rviz, ros-<>-rviz-visual-tools, ros-<>-urdf_tutorial

2. `roslaunch dji_sdk M100_urdf display_basic.launch` for one link only, with no propeller

    or

    `roslaunch dji_sdk M100_urdf display_fixed.launch` for one link only, with four fixed propellers

    or

    `roslaunch dji_sdk M100_urdf display_all.launch` for five links, with rotatable propeller

    currently, no paramters added, any contribution are welcome.


---

contributor: unknown

license: MIT

