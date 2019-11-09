# ROS Robot Status Indicator
[![Build Status](https://www.travis-ci.org/LucidOne/robot_indicator.svg?branch=master)](https://www.travis-ci.org/LucidOne/robot_indicator)
[![Build Status](http://build.ros.org/buildStatus/icon?subject=Kinetic&job=Kbin_uX64__robot_indicator__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__robot_indicator__ubuntu_xenial_amd64__binary/)

[This](https://github.com/LucidOne/robot_indicator) package works with
[robot_systemd](https://github.com/LucidOne/robot_systemd) to provide a GUI for
Systemd based control of roslaunch units.

![Indicator](https://raw.githubusercontent.com/LucidOne/robot_indicator/master/doc/indicator.png)

![Launch Package Selection](https://raw.githubusercontent.com/LucidOne/robot_indicator/master/doc/launch_package.png)
![Launch UI](https://raw.githubusercontent.com/LucidOne/robot_indicator/master/doc/launch_ui.png)

## TL;DR
```bash
mkdir -p ~/.config/autostart
cp `rospack find robot_indicator`/robot_indicator.desktop ~/.config/autostart/robot_indicator.desktop
rosrun robot_indicator robot_indicator
# or log out and log back in
```
