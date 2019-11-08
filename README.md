# ROS Robot Status Indicator
![Indicator](https://raw.githubusercontent.com/LucidOne/robot_indicator/master/doc/indicator.png)

[This](https://github.com/LucidOne/robot_indicator) package works with
[robot_systemd](https://github.com/LucidOne/robot_systemd) to provide a GUI for
Systemd based control of roslaunch units.

## TL;DR
```bash
mkdir -p ~/.config/autostart
cp `rospack find robot_indicator`/robot_indicator.desktop ~/.config/autostart/robot_indicator.desktop
rosrun robot_indicator robot_indicator
# or log out and log back in
```
