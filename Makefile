.PHONY : install-git-filters rosdep status clean build deploy

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

status:
	# Show status of all repositories
	vcs status . --nested

clean:
	# Clean the workspace
	# This removes all build, install, and log directories
	rm -rf build/ install/ log/

build:
	# Build the workspace
	cba

build_%:
	# Build a specific package
	# Usage: make build_<package_name>
	# This will build the specified package and its dependencies
	cbs $*

deploy:
	# Deploy the robots
	echo "Deploying robots..."
	# This is a placeholder for the actual deployment command
