#!/usr/bin/env -S just --justfile

dir := justfile_directory()
pkg_prefix := "wa_"

# Start a new shell with ROS2 setup
@repl:
    bash --init-file <(echo "source \"$HOME/.bashrc\"; eval $(just setup)")

# Setup ROS2 in the current shell (must be evaluated)
setup:
    #!/usr/bin/env -S cat -s
    # Must be executed in the main shell
    # in bash, run it with `eval $(just setup)`

    # Setup the project
    export PROJECT="warehouse_automation"
    export PROJECT_ROOT="{{ dir }}"

    # Setup ROS2
    ros_path="/opt/ros/humble"
    [[ -d "$ros_path" ]] && source "$ros_path/setup.bash"

    # Disable the system notification handler extension
    export COLCON_EXTENSION_BLOCKLIST="colcon_core.event_handler.desktop_notification"

    # Default to x11 on QT apps (many break on wayland)
    export QT_QPA_PLATFORM="xcb"

# Build the ROS2 warehouse_automation workspace
build:
    #!/usr/bin/env bash
    eval "$(just setup)"
    set -euo pipefail
    colcon build --symlink-install

# Install the ROS2 warehouse_automation workspace (must be evaluated)
install:
    #!/usr/bin/env -S cat -s
    # Must be executed in the main shell
    # in bash, run it with `eval $(just install)`

    # Run the setup recipe
    eval $(just setup)

    # Source the workspace setup.bash to install the local ROS2 packages
    source {{ dir }}/install/setup.bash 2>/dev/null

# Remove the build artifacts from the warehouse_automation workspace
[confirm("Would you like to clean the workspace? [y/N]")]
@clean:
    @rm -rf {{ dir }}/build {{ dir }}/install {{ dir }}/log

# Rebuild the warehouse_automation workspace from scratch
clean-build: clean build

# List the warehouse_automation packages available in the environment
list-packages:
    #!/usr/bin/env bash
    eval "$(just install)"
    set -euo pipefail
    ros2 pkg list | grep "^{{ pkg_prefix }}" || echo "No packages found"

# Start a Docker container with the ROS2 environment for development
docker-repl:
    @# Allow X11 forwarding to root
    xhost +SI:localuser:root
    -docker run --rm -it \
        -v {{ dir }}:/workspace \
        -v /dev/dri:/dev/dri \
        -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --ipc host \
        osrf/ros:humble-desktop-full bash
