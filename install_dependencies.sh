#!/bin/bash

#######################################################################
# STOP: If the dependency you want to add is required for the project #
#       to build, it should be added as a rosdep. This script should  #
#       only contain other dependecies, like those required for gazebo#
#######################################################################

######################################################################
# This script will download and install dependencies for the project #
######################################################################

echo "================================================================"
echo "Installing ROS dependencies..."
echo "================================================================"

# Update Rosdeps
rosdep update

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Clone any ros dependencies of this repo
rosws update

# Install all required dependencies to build this repo
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
catkin_make install

echo "================================================================"
echo "Finished installing ROS dependencies."
echo "================================================================"
echo ""
echo "================================================================"
echo "Installing Misc. Utilities"
echo "================================================================"

sudo apt-get install -y\
    clang-format\
    python-rosinstall


echo "================================================================"
echo "Installing Project Dependent ROS packages."
echo "================================================================"

# Setup rosinstall
mkdir -p external_pkg
rosinstall external_pkg /opt/ros/kinetic .rosinstall
rosinstall .

echo "================================================================"
echo "Setup .bashrc"
echo "================================================================"
# Shell config files that various shells source when they run.
# This is where we want to add aliases, source ROS environment
# variables, etc.
SHELL_CONFIG_FILES=(
    "$HOME/.bashrc"\
            "$HOME/.zshrc"
    )

# All lines listed here will be added to the shell config files
# listed above, if they are not present already
declare -a new_shell_config_lines=(
    #"source $CURR_DIR/setup.sh"
    "source /opt/ros/kinetic/setup.sh"
    )

# Add all of our new shell config options to all the shell
# config files, but only if they don't already have them
for file_name in "${SHELL_CONFIG_FILES[@]}";
do
    echo "Setting up $file_name"
    for line in "${new_shell_config_lines[@]}";
        do
            if ! grep -Fq "$line" $file_name
            then
                echo "$line" >> $file_name
            fi
        done
done

echo "================================================================"
echo "Installing Udev rules for phidgets"
echo "================================================================"

# Setup udev rules
# The reason the script that comes with the phidget package isn't used is
# because stupid bash/sh doesn't know which directory it's in. The actual
# script that comes with phidget is under external_pkg/phidgets_api/share/setup-udev.sh
# and has more or less the same content as below:
sudo cp external_pkg/phidgets_api/share/udev/99-phidgets.rules /etc/udev/rules.d
echo "Phidgets udev rules have been copied to /etc/udev/rules.d"

echo "================================================================"
echo "Finished Installing Utilities"
echo "================================================================"

