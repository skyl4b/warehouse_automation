#!/usr/bin/env -S just --justfile

dir := justfile_directory()

# Package prefix for the warehouse_automation packages

pkg_prefix := "wa_"

# Worlds directory

world_dir := dir / "src/wa_environment/worlds"

# Start a new shell with ROS2 set up
repl:
    #!/usr/bin/env bash
    init_file=$(cat <<-END
        source "$HOME/.bashrc"
        source <(just install)
    END
    )
    bash --init-file <(echo "$init_file")

# Setup ROS2 in the current shell (must be sourced)
setup:
    #!/usr/bin/env -S cat -s
    # Must be sourced to the main shell
    # in bash, run it with `source <(just setup)`

    # Setup the project
    export PROJECT="warehouse_automation"
    export PROJECT_ROOT="{{ dir }}"

    # Setup ROS2
    export ROS_DISTRO="humble"
    if [[ -n "$WA_ROS_INSTALL_OVERRIDE" ]]; then
        $WA_ROS_INSTALL_OVERRIDE
    else
        source "/opt/ros/$ROS_DISTRO/setup.bash"
    fi

    # Disable the system notification handler extension
    export COLCON_EXTENSION_BLOCKLIST="colcon_core.event_handler.desktop_notification"

    # Default to x11 on QT apps (many break on wayland)
    export QT_QPA_PLATFORM="xcb"

    # Use CycloneDDS implementation (faster and more stable networking),
    # fixes time issues with nav2.
    # See: https://github.com/ros-navigation/navigation2/issues/3352#issuecomment-1374005889
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Build the ROS2 warehouse_automation workspace
build packages="":
    #!/usr/bin/env bash
    source <(just setup)
    set -euo pipefail

    # Filter out the warning about missing local_setup files.
    # Since this warning is emitted with logger, it can't be
    # filtered with the COLCON_WARNINGS environment variable.
    warning_filter="WARNING:colcon\.colcon_ros\.prefix_path\.ament:\
    The path .* in the environment variable AMENT_PREFIX_PATH \
    doesn't contain any 'local_setup\..*' files"

    # Wrap build command
    function _build() {
        colcon build "$@" --symlink-install 2>&1 | grep -v "$warning_filter"
    }

    # Build the workspace with all or selected packages
    if [[ -n "{{ packages }}" ]]; then
        _build --packages-select "{{ packages }}"
    else
        _build
    fi

# Install the ROS2 warehouse_automation workspace (must be sourced)
install:
    #!/usr/bin/env -S cat -s
    # Must be sourced to the main shell
    # in bash, run it with `source <(just install)`

    # Run the setup recipe
    source <(just setup)

    # Source the workspace setup.bash to install the local ROS2 packages
    source "{{ dir }}/install/setup.bash" 2>/dev/null

# Remove the build artifacts from the warehouse_automation workspace
[confirm("Would you like to clean the workspace? [y/N]")]
@clean:
    @rm -rf "{{ dir }}/build" "{{ dir }}/install" "{{ dir }}/log"

# Rebuild the warehouse_automation workspace from scratch
clean-build: clean build

# List the warehouse_automation packages available in the environment
list-packages:
    #!/usr/bin/env bash
    source <(just install)
    set -euo pipefail
    ros2 pkg list | grep "^{{ pkg_prefix }}" || echo "No packages found"

# Open Gazebo with the specified world file
gazebo world="empty":
    #!/usr/bin/env bash
    source <(just install)
    set -euo pipefail

    # Set Gazebo model lookup paths
    custom_models_path="{{ dir }}/src/wa_environment/models"
    if [ -z "${GAZEBO_MODEL_PATH:-}" ]; then
        export GAZEBO_MODEL_PATH="$custom_models_path"
    else
        export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$custom_models_path"
    fi

    # Check if the world provided matches the stem of a base world
    world="{{ world }}"
    for file in $(ls {{ world_dir }}); do
        if [[ "${file%.*}" == "{{ world }}" ]]; then
            world="{{ world_dir }}/$file"
            break
        fi
    done

    # Check if the file exists
    if [[ ! -f "$world" ]]; then
        echo "Error: '$world' does not exist."
        exit 1
    fi

    # Open the Gazebo simulator with the ROS plugins
    gazebo --verbose \
        -s libgazebo_ros_init.so \
        -s libgazebo_ros_factory.so \
        -s libgazebo_ros_force_system.so \
        "$world"

# Record a Ros2 bag of provided topics
bag-record name *topics="--all":
    ros2 bag record --use-sim-time -o experiments/bag_recordings/{{name}} {{topics}}

# Record the warehouse automaton simulation
record name="simulation": (bag-record name "/wa/task/started" "/wa/task/completed" "/wa/demand_generator/demand" "/wa/demand_generator/unbounded_demand" "/wa/task_transmitter/map")

# Launch the warehouse automation simulation
launch *args:
    ros2 launch wa_bringup simulation.launch.py {{args}}

# Display launch arguments for the simulation launch
launch-help: (launch "-s")

# Start a Nix shell with the ROS2 environment for development
nix:
    nix develop --command just repl

# Start a Docker container with the ROS2 environment for development
docker-repl:
    @# Allow X11 forwarding to root
    xhost +SI:localuser:root
    -docker run --rm -it \
        --name warehouse_automation \
        -v {{ dir }}:/workspace \
        -v /dev/dri:/dev/dri \
        -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --ipc host \
        -e QT_X11_NO_MITSHM=1 \
        osrf/ros:humble-desktop-full bash
