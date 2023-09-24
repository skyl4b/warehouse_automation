{
  description = "A development environment";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    nix-ros.url = "github:lopsided98/nix-ros-overlay";
  };

  # Flake outputs
  outputs = { self, nixpkgs, flake-utils, nix-ros }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros.overlays.default ];
        };
      in {
        # Development environment output
        devShell = pkgs.mkShell {
          # The Nix packages provided in the environment
          packages = with pkgs; [
            rosPackages.humble.desktop
            colcon
            
            # Setup bash autocompletions
            (writeShellScriptBin "completions" ''
            if (return 0 2>/dev/null); then
              echo "Adding ROS2 completions to the environment."
              eval "$(${python3Packages.argcomplete}/bin/register-python-argcomplete ros2)"
              eval "$(${python3Packages.argcomplete}/bin/register-python-argcomplete colcon)"
              eval "$(${python3Packages.argcomplete}/bin/register-python-argcomplete rosidl)"
            else
              echo "Completions must be sourced, skipping."
              echo "source completions"
            fi
            '')
          ];

          # Environment variables provided in the environment
          PROJECT = "warehouse_automation";

          # Hook commands to run in the environment
          shellHook = ''
            # Set flake root
            export FLAKE_ROOT="$(git rev-parse --show-toplevel)"
          '';
        };
      });
}
