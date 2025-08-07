set shell := ["zsh", "-c"]

setup: install-git-filters rosdep build

install-git-filters:
	# Install git filters
	# The vscode settings file gets updated by the ros extension and contains the full path to the current user's home directory.
	# We don't want to commit this path, so we use a git filter to remove it when git adds the file to the staging area.
	# This does not affect the file on disk, so vscode will still work as expected.
	git config filter.removeFullHomePath.clean "sed '/\/\(home\|root\).*\(install\|build\)/d'"

rosdep:
	# Initialize rosdep if not already done
	[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
	# Update rosdep and install dependencies from meta directory
	rosdep update
	rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

clean:
	# Clean the workspace
	# This removes all build, install, and log directories
	rm -rf build/ install/ log/

clean-robot ip:
	# Clean the workspace on the robot with the given IP address
	ssh ubuntu@{{ip}} "cd /home/ubuntu/turtlebot_workspace && rm -rf build/ install/ log/"

build:
	# Build the workspace
	. /opt/ros/humble/setup.zsh
	colcon build --symlink-install --continue-on-error

_build-robot ip:
	# Run the build command on the robot
	ssh ubuntu@{{ip}} "cd /home/ubuntu/turtlebot_workspace && . /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install --continue-on-error"

build-package package:
	# Build a specific package in the workspace
	. /opt/ros/humble/setup.zsh
	colcon build --symlink-install --packages-select {{package}}

sync ip:
	# Sync the workspace to a robot with the given IP address
	# Copy this repository to the robot
	rsync --delete -av --exclude='.git' --exclude='install' --exclude='build' --exclude='log' . ubuntu@{{ip}}:/home/ubuntu/turtlebot_workspace/

deploy ip: (sync ip) (_build-robot ip)

deploy-clean ip: (clean-robot ip) (deploy ip)

connect ip:
	# Start a zenoh connection to the specified IP address
	zenoh-bridge-ros2dds -e tcp/{{ip}}:7447
