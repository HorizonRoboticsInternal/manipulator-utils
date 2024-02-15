{
  description = "Utilities and drivers for manipulators (robotic arms)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/23.11";
    utils.url = "github:numtide/flake-utils";
    
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay";
    nix-ros-overlay.inputs.nixpkgs.follows = "nixpkgs";
    nix-ros-overlay.inputs.flake-utils.follows = "utils";
  };

  outputs = { self, nixpkgs, utils, ... }@inputs: {
    overlays.dynamixel = final: prev: {
      DynamixelSDK = final.callPackage ./nix/pkgs/DynamixelSDK {};
      dynamixel-workbench = final.callPackage ./nix/pkgs/dynamixel-workbench {};
    };
    overlays.dev = nixpkgs.lib.composeManyExtensions [
      self.overlays.dynamixel
      inputs.nix-ros-overlay.overlays.default
    ];
    overlays.default = nixpkgs.lib.composeManyExtensions [
      self.overlays.dev
    ];
  } // utils.lib.eachSystem [
    "x86_64-linux" "aarch64-linux"
  ] (system:
    let pkgs-dev = import nixpkgs {
          inherit system;
          overlays = [ self.overlays.dev ];
        };

        pkgs = import nixpkgs {
          inherit system;
          overlays = [ self.overlays.default ];
        };

    in {
      devShells.diagnostics = pkgs-dev.callPackage ./nix/pkgs/ros-diagnostics-devshell {};

      packages = {
        inherit (pkgs) DynamixelSDK dynamixel-workbench;
      };
    });
}
