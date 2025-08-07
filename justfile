set shell := ["zsh", "-c"]

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
	rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

clean:
	# Clean the workspace
	# This removes all build, install, and log directories
	rm -rf build/ install/ log/

build:
	# Build the workspace
	. /opt/ros/humble/setup.zsh
	colcon build --symlink-install --continue-on-error

build-package package:
	# Build a specific package in the workspace
	. /opt/ros/humble/setup.zsh
	colcon build --symlink-install --packages-select {{package}}

deploy:
	# Deploy the robots
	echo "Deploying robots..."
	# This is a placeholder for the actual deployment command
