# rosdep
Add gnome packages to rosdep

# autostart
Provide a means of auto installing the desktop file
~/.config/autostart/robot_indicator.desktop

# Startup selection
Enable selection of
- start at boot
- start at login

## Start at boot
```bash
pkexec loginctl enable-linger $USER
```

## Start at login
```bash
pkexec loginctl disable-linger $USER
```

Document an alternative setup
```bash
touch /var/lib/systemd/linger/$USER
```

# Diagnostic Aggregator
Modify indicator icon based on dignostic status and add sub-systems to
indicator menu.

This approach seems more useful than adding topics, nodes, etc.

# roscorelaunch@.service
Need user feedback to better understand how to handle this use-case.
Perhaps the program should just throw an error if it encounters an active
roscorelaunch@.service unit.

# external updates
If a user runs `systemctl --user start roslaunch@pkg:file.launch` in a terminal
it may take up to 30 seconds for the indicator to detect the change.

This should probably be replaced with a 'Launch Units'>'Update' submenu item.
It may be worth considering how the systemd unit can notify the indicator
instead of polling.

# Default units
Support for a list of default units in a config file so they don't disappear
when a launch unit is stopped and disabled.

# Launch interface
GUI interface to start new roslaunch@.service units
