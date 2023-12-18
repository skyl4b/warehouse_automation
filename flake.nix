{
  description = "A development environment";

  inputs = {
    nixpkgs.url = "github:lopsided98/nixpkgs/nix-ros";
    flake-utils.url = "github:numtide/flake-utils";
    nix-ros = {
      url = "github:lopsided98/nix-ros-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    nixgl = {
      url = "github:guibou/nixGL";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
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
            inputs.nixgl.overlay

            # HACK: Fix navigation2 build
            # 1. Use patched ompl until this PR is merged
            # 2. Add flag "-Wno-error=maybe-uninitialized"
            # See https://github.com/lopsided98/nix-ros-overlay/issues/311
            # and https://github.com/lopsided98/nix-ros-overlay/issues/286
            # for more information.
            (self: super: {
              rosPackages = super.rosPackages // {
                humble = super.rosPackages.humble // rec {
                  # Fix ompl
                  # NOTE: The entire dependency stack must be overridden
                  ompl = super.rosPackages.humble.ompl.overrideAttrs ({ patches ? [ ], ... }: {
                    version = "Fix full paths patch";
                    patches = patches ++ [
                      # Use full install paths for pkg-config
                      (self.fetchpatch {
                        url = "https://github.com/hacker1024/ompl/commit/1ddecbad87b454ac0d8e1821030e4cf7eeff2db2.patch";
                        hash = "sha256-sAQLrWHoR/DhHk8TtUEy8E8VXqrvtXl2BGS5UvElJl8=";
                      })
                    ];
                  });
                  nav2-smac-planner = super.rosPackages.humble.nav2-smac-planner.override { ompl = ompl; };

                  # Fix build flags
                  dwb-critics = super.rosPackages.humble.dwb-critics.overrideAttrs (oldAttrs: {
                    CXXFLAGS = oldAttrs.CXXFLAGS or "" + " -Wno-error=maybe-uninitialized";
                  });
                  nav2-behaviors = super.rosPackages.humble.nav2-behaviors.overrideAttrs (oldAttrs: {
                    CXXFLAGS = oldAttrs.CXXFLAGS or "" + " -Wno-error=maybe-uninitialized";
                  });
                  dwb-plugins = super.rosPackages.humble.dwb-plugins.overrideAttrs (oldAttrs: {
                    CXXFLAGS = oldAttrs.CXXFLAGS or "" + " -Wno-error=maybe-uninitialized";
                  });
                  nav2-dwb-controller = super.rosPackages.humble.nav2-dwb-controller.override {
                    dwb-critics = dwb-critics;
                    dwb-plugins = dwb-plugins;
                  };
                  nav2-constrained-smoother = super.rosPackages.humble.nav2-constrained-smoother.overrideAttrs (oldAttrs: {
                    CXXFLAGS = oldAttrs.CXXFLAGS or "" + " -Wno-error=maybe-uninitialized";
                  });
                  nav2-planner = super.rosPackages.humble.nav2-planner.overrideAttrs (oldAttrs: {
                    CXXFLAGS = oldAttrs.CXXFLAGS or "" + " -Wno-error=maybe-uninitialized";
                  });
                  nav2-smoother = super.rosPackages.humble.nav2-smoother.overrideAttrs (oldAttrs: {
                    CXXFLAGS = oldAttrs.CXXFLAGS or "" + " -Wno-error=maybe-uninitialized";
                  });
                  nav2-waypoint-follower = super.rosPackages.humble.nav2-waypoint-follower.overrideAttrs (oldAttrs: {
                    CXXFLAGS = oldAttrs.CXXFLAGS or "" + " -Wno-error=maybe-uninitialized";
                  });

                  # Apply fixes
                  navigation2 = super.rosPackages.humble.navigation2.override {
                    nav2-smac-planner = nav2-smac-planner;
                    nav2-dwb-controller = nav2-dwb-controller;
                    nav2-behaviors = nav2-behaviors;
                    nav2-constrained-smoother = nav2-constrained-smoother;
                    nav2-planner = nav2-planner;
                    nav2-smoother = nav2-smoother;
                    nav2-waypoint-follower = nav2-waypoint-follower;
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
            (python3.withPackages (ps: with ps; [ mypy jedi ]))
            # Base ROS2 packages
            rosPackages.humble.desktop
            colcon

            # Gazebo ROS2 interface
            rosPackages.humble.gazebo-ros-pkgs
            rosPackages.humble.gazebo-ros2-control

            # Navigation packages
            rosPackages.humble.navigation2

            # Setup nixgl for gpu applications
            nixgl.nixGLIntel

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

              # Set nixGL wrapper for gazebo
              if command -v nixGLIntel &>/dev/null; then
                alias gazebo="nixGLIntel gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so -s libgazebo_ros_force_system.so"
                alias rviz2="nixGLIntel rviz2"
              fi
            '')
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
