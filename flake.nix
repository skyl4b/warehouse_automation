{
  description = "A development environment";

  inputs = {
    nixpkgs.url = "github:lopsided98/nixpkgs/nix-ros";
    flake-utils.url = "github:numtide/flake-utils";
    nix-ros = {
      url = "github:lopsided98/nix-ros-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  # Flake outputs
  outputs = { self, nixpkgs, flake-utils, nix-ros }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros.overlays.default ];
        };
      in
      {
        # Development environment output
        devShell = pkgs.mkShell {
          # The Nix packages provided in the environment
          packages = with pkgs; [
            (python3.withPackages (ps: with ps; [ mypy jedi ]))
            rosPackages.humble.desktop
            rosPackages.humble.gazebo-ros-pkgs
            rosPackages.humble.gazebo-ros2-control
            colcon

            # Setup environment for ROS
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
          COLCON_EXTENSION_BLOCKLIST="colcon_core.event_handler.desktop_notification";

          # Hook commands to run in the environment
          shellHook = ''
            # Set flake root
            export FLAKE_ROOT="$(git rev-parse --show-toplevel)"
          '';
        };
      });
}
