{
  description = "A development environment";

  inputs = {
    nixpkgs.url = "github:lopsided98/nixpkgs/nix-ros";
    flake-utils.url = "github:numtide/flake-utils";
    nix-ros = {
      url = "github:lopsided98/nix-ros-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    husky = {
      url = "github:husky/husky/humble-devel";
      flake = false;
    };
  };

  # Flake outputs
  outputs = inputs@{ self, nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            inputs.nix-ros.overlays.default

            # HACK: Fix navigation2 build
            # 1. Use patched ompl until this PR is merged
            # 2. Add flag "-Wno-error=maybe-uninitialized"
            # See https://github.com/lopsided98/nix-ros-overlay/issues/311
            # and https://github.com/lopsided98/nix-ros-overlay/issues/286
            # for more information.
            (final: prev: {
              rosPackages = prev.rosPackages // {
                humble = prev.rosPackages.humble // rec {
                  # Fix ompl
                  # NOTE: The entire dependency stack must be overridden
                  ompl = prev.rosPackages.humble.ompl.overrideAttrs ({ patches ? [ ], ... }: {
                    version = "Fix full paths patch";
                    patches = patches ++ [
                      # Use full install paths for pkg-config
                      (final.fetchpatch {
                        url = "https://github.com/hacker1024/ompl/commit/1ddecbad87b454ac0d8e1821030e4cf7eeff2db2.patch";
                        hash = "sha256-sAQLrWHoR/DhHk8TtUEy8E8VXqrvtXl2BGS5UvElJl8=";
                      })
                    ];
                  });
                  nav2-smac-planner = prev.rosPackages.humble.nav2-smac-planner.override { ompl = ompl; };

                  # Fix build flags
                  dwb-critics = prev.rosPackages.humble.dwb-critics.overrideAttrs (oldAttrs: {
                    CXXFLAGS = oldAttrs.CXXFLAGS or "" + " -Wno-error=maybe-uninitialized";
                  });
                  nav2-behaviors = prev.rosPackages.humble.nav2-behaviors.overrideAttrs (oldAttrs: {
                    CXXFLAGS = oldAttrs.CXXFLAGS or "" + " -Wno-error=maybe-uninitialized";
                  });
                  dwb-plugins = prev.rosPackages.humble.dwb-plugins.overrideAttrs (oldAttrs: {
                    CXXFLAGS = oldAttrs.CXXFLAGS or "" + " -Wno-error=maybe-uninitialized";
                  });
                  nav2-dwb-controller = prev.rosPackages.humble.nav2-dwb-controller.override {
                    dwb-critics = dwb-critics;
                    dwb-plugins = dwb-plugins;
                  };
                  nav2-constrained-smoother = prev.rosPackages.humble.nav2-constrained-smoother.overrideAttrs (oldAttrs: {
                    CXXFLAGS = oldAttrs.CXXFLAGS or "" + " -Wno-error=maybe-uninitialized";
                  });
                  nav2-planner = prev.rosPackages.humble.nav2-planner.overrideAttrs (oldAttrs: {
                    CXXFLAGS = oldAttrs.CXXFLAGS or "" + " -Wno-error=maybe-uninitialized";
                  });
                  nav2-smoother = prev.rosPackages.humble.nav2-smoother.overrideAttrs (oldAttrs: {
                    CXXFLAGS = oldAttrs.CXXFLAGS or "" + " -Wno-error=maybe-uninitialized";
                  });
                  nav2-waypoint-follower = prev.rosPackages.humble.nav2-waypoint-follower.overrideAttrs (oldAttrs: {
                    CXXFLAGS = oldAttrs.CXXFLAGS or "" + " -Wno-error=maybe-uninitialized";
                  });

                  # Apply fixes
                  navigation2 = prev.rosPackages.humble.navigation2.override {
                    nav2-smac-planner = nav2-smac-planner;
                    nav2-dwb-controller = nav2-dwb-controller;
                    nav2-behaviors = nav2-behaviors;
                    nav2-constrained-smoother = nav2-constrained-smoother;
                    nav2-planner = nav2-planner;
                    nav2-smoother = nav2-smoother;
                    nav2-waypoint-follower = nav2-waypoint-follower;
                  };
                  nav2-bringup = prev.rosPackages.humble.nav2-bringup.override {
                    navigation2 = navigation2;
                  };
                };
              };
            })

            # NOTE: Add humble husky packages, not released for humble yet (probably won't be)
            (final: prev: {
              rosPackages = prev.rosPackages // {
                humble = prev.rosPackages.humble // rec {
                  husky-control = final.rosPackages.humble.buildRosPackage {
                    pname = "ros2-humble-husky-control";
                    version = "latest";
                    src = inputs.husky + "/husky_control";
                    buildType = "ament_cmake";
                    buildInputs = with final.rosPackages.humble; [ ament-cmake ];
                    propagatedBuildInputs = with final.rosPackages.humble; [
                      controller-manager
                      diff-drive-controller
                      husky-description
                      interactive-marker-twist-server
                      joint-state-broadcaster
                      joint-trajectory-controller
                      joy
                      robot-localization
                      robot-state-publisher
                      teleop-twist-joy
                      twist-mux
                    ];
                    nativeBuildInputs = with final.rosPackages.humble; [ ament-cmake ];
                    meta = {
                      description = ''Clearpath Husky controller configurations'';
                      license = with final.lib.licenses; [ bsdOriginal ];
                    };
                  };
                  husky-description = final.rosPackages.humble.buildRosPackage {
                    pname = "ros2-humble-husky-description";
                    version = "latest";
                    src = inputs.husky + "/husky_description";
                    buildType = "ament_cmake";
                    buildInputs = with final.rosPackages.humble; [ ament-cmake ];
                    propagatedBuildInputs = with final.rosPackages.humble; [ realsense2-description urdf ];
                    nativeBuildInputs = with final.rosPackages.humble; [ ament-cmake ];
                    meta = {
                      description = ''Clearpath Husky URDF description'';
                      license = with final.lib.licenses; [ bsdOriginal ];
                    };
                  };
                  husky-gazebo = final.rosPackages.humble.buildRosPackage {
                    pname = "ros2-humble-husky-gazebo";
                    version = "latest";
                    src = inputs.husky + "/husky_gazebo";
                    buildType = "ament_cmake";
                    buildInputs = with final.rosPackages.humble; [ ament-cmake ];
                    checkInputs = with final.rosPackages.humble; [ ament-lint-auto huament-lint-common ];
                    propagatedBuildInputs = with final.rosPackages.humble; [
                      gazebo-plugins
                      gazebo-ros
                      gazebo-ros2-control
                      husky-control
                      husky-description
                      ros2launch
                      urdf
                      xacro
                    ];
                    nativeBuildInputs = with final.rosPackages.humble; [ ament-cmake ];
                    meta = {
                      description = ''Clearpath Husky Simulator bringup'';
                      license = with final.lib.licenses; [ bsdOriginal ];
                    };
                  };
                  husky-simulator = final.rosPackages.humble.buildRosPackage {
                    pname = "ros2-humble-husky-simulator";
                    version = "latest";
                    src = inputs.husky + "/husky_simulator";
                    buildType = "ament_cmake";
                    buildInputs = with final.rosPackages.humble; [ ament-cmake ];
                    propagatedBuildInputs = [ husky-gazebo ];
                    nativeBuildInputs = with final.rosPackages.humble; [ ament-cmake ];
                    meta = {
                      description = ''Metapackage for Clearpath Husky simulation software'';
                      license = with final.lib.licenses; [ bsdOriginal ];
                    };
                  };
                  husky-msgs = final.rosPackages.humble.buildRosPackage {
                    pname = "ros2-humble-husky-msgs";
                    version = "latest";
                    src = inputs.husky + "/husky_msgs";
                    buildType = "ament_cmake";
                    buildInputs = with final.rosPackages.humble; [ ament-cmake rosidl-default-generators ];
                    propagatedBuildInputs = with final.rosPackages.humble; [ builtin-interfaces rosidl-default-runtime std-msgs ];
                    nativeBuildInputs = with final.rosPackages.humble; [ ament-cmake ];
                    meta = {
                      description = ''Messages for Clearpath Husky'';
                      license = with final.lib.licenses; [ bsdOriginal ];
                    };
                  };
                  husky-viz = final.rosPackages.humble.buildRosPackage {
                    pname = "ros2-humble-husky-viz";
                    version = "latest";
                    src = inputs.husky + "/husky_viz";
                    buildType = "ament_cmake";
                    buildInputs = with final.rosPackages.humble; [ ament-cmake ];
                    checkInputs = with final.rosPackages.humble; [ ament-lint-auto ament-lint-common ];
                    propagatedBuildInputs = with final.rosPackages.humble; [
                      husky-description
                      joint-state-publisher
                      joint-state-publisher-gui
                      launch-ros
                      robot-state-publisher
                      rviz2
                    ];
                    nativeBuildInputs = with final.rosPackages.humble; [ ament-cmake ];
                    meta = {
                      description = ''Visualization configuration for Clearpath Husky'';
                      license = with final.lib.licenses; [ bsdOriginal ];
                    };
                  };
                  husky-desktop = final.rosPackages.humble.buildRosPackage {
                    pname = "ros2-humble-husky-desktop";
                    version = "latest";
                    src = inputs.husky + "/husky_desktop";
                    buildType = "ament_cmake";
                    buildInputs = with final.rosPackages.humble; [ ament-cmake ];
                    propagatedBuildInputs = [ husky-msgs husky-viz ];
                    nativeBuildInputs = with final.rosPackages.humble; [ ament-cmake ];
                    meta = {
                      description = ''Metapackage for Clearpath Husky visualization software'';
                      license = with final.lib.licenses; [ bsdOriginal ];
                    };
                  };
                };
              };
            })
          ];
        };
      in
      {
        # Development environment output
        devShell = pkgs.mkShell {
          # The Nix packages provided in the environment
          packages = with pkgs; [
            # Python packages
            (python3.withPackages (ps: with ps; [ mypy jedi ]))

            # Setup environment hook for ROS2
            (writeShellScriptBin "nix-env-hook" ''
              # Setup bash autocompletions
              if ! (return 0 2>/dev/null); then
                echo "Nix-ros-setup must be sourced, skipping."
                echo "source nix-ros-setup"
                exit 1
              fi

              # Completions
              echo "Adding ROS2 completions to the environment."
              eval "$(${python3Packages.argcomplete}/bin/register-python-argcomplete ros2)"
              eval "$(${python3Packages.argcomplete}/bin/register-python-argcomplete colcon)"
              eval "$(${python3Packages.argcomplete}/bin/register-python-argcomplete rosidl)"
              source "${pkgs.gazebo}/share/gazebo/setup.sh"

              # Environment variables for turtlebot3 and nav2-bringup
              export TURTLEBOT3_MODEL=waffle
              export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:`ros2 pkg prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/"

              # Build tools
              function colcon() {
                # Filter no setup files warning
                command colcon "$@" 2>&1 | grep -v "WARNING:colcon\.colcon_ros\.prefix_path\.ament:The path .* in the environment variable AMENT_PREFIX_PATH doesn't contain any 'local_setup\..*' files"
              }
              function clean() {
                rm -r $FLAKE_ROOT/build $FLAKE_ROOT/install $FLAKE_ROOT/log
              }
              alias install="source $FLAKE_ROOT/install/setup.bash"
              alias build="colcon build --symlink-install && install"
              alias clean-build="clean && build"

              # Apps wrappers
              alias gazebo="gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so -s libgazebo_ros_force_system.so"
            '')
          ];
          nativeBuildInputs = [
            # ROS2 packages built with a custom environment
            (pkgs.rosPackages.humble.buildEnv {
              paths = with pkgs; [
                # Base ROS2 packages
                rosPackages.humble.desktop
                colcon

                # Gazebo ROS2 interface
                rosPackages.humble.gazebo-ros-pkgs
                rosPackages.humble.gazebo-ros2-control

                # Navigation packages
                rosPackages.humble.navigation2
                rosPackages.humble.turtlebot3-simulations
                rosPackages.humble.turtlebot3-teleop
                rosPackages.humble.nav2-bringup

                # Husky overlaid packages
                rosPackages.humble.husky-desktop
                rosPackages.humble.husky-simulator
              ];
            })
          ];

          # Environment variables provided in the environment
          PROJECT = "warehouse_automation";

          # Disable the system notification handler extension
          COLCON_EXTENSION_BLOCKLIST = "colcon_core.event_handler.desktop_notification";

          # Hook commands to run in the environment
          shellHook = ''
            # Set flake root
            export FLAKE_ROOT="$(git rev-parse --show-toplevel)"
          '';
        };
      });
}
