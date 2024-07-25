{
  description = "A development environment";

  inputs = {
    nixpkgs.url = "github:lopsided98/nixpkgs/nix-ros";
    flake-utils.url = "github:numtide/flake-utils";
    nix-ros = {
      url = "github:lopsided98/nix-ros-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
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
                  nav2-smac-planner = prev.rosPackages.humble.nav2-smac-planner.override { inherit ompl; };

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
                    inherit dwb-critics;
                    inherit dwb-plugins;
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
                    inherit nav2-smac-planner;
                    inherit nav2-dwb-controller;
                    inherit nav2-behaviors;
                    inherit nav2-constrained-smoother;
                    inherit nav2-planner;
                    inherit nav2-smoother;
                    inherit nav2-waypoint-follower;
                  };
                  nav2-bringup = prev.rosPackages.humble.nav2-bringup.override {
                    inherit navigation2;
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
            # Bash interactive to solve some issues with subshells breaking
            bashInteractive

            # Python packages
            (python3.withPackages (ps: with ps; [ numpy ]))

            # Setup environment install for ROS2
            (writeShellScriptBin "nix-ros-install" ''
              # Setup bash autocompletions
              if ! (return 0 2>/dev/null); then
                echo "Error: $0 must be sourced, exiting."
                echo "source $0"
                exit 1
              fi

              # Completions
              eval "$(${python3Packages.argcomplete}/bin/register-python-argcomplete ros2)"
              eval "$(${python3Packages.argcomplete}/bin/register-python-argcomplete colcon)"
              eval "$(${python3Packages.argcomplete}/bin/register-python-argcomplete rosidl)"
              source "${pkgs.gazebo}/share/gazebo/setup.sh"
            '')
          ];
          nativeBuildInputs = [
            # ROS2 packages built with a custom environment
            (pkgs.rosPackages.humble.buildEnv {
              paths = with pkgs; [
                # Base ROS2 packages
                rosPackages.humble.desktop
                rosPackages.humble.ament-cmake-core
                rosPackages.humble.python-cmake-module
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

          # Override the default ROS2 installation command
          WA_ROS_INSTALL_OVERRIDE = "source nix-ros-install";

          # Hook commands to run in the environment
          shellHook = ''
            # Set flake root
            export FLAKE_ROOT="$(git rev-parse --show-toplevel)"
          '';
        };
      });
}
