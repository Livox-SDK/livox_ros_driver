{ system ? builtins.currentSystem
, pkgs ? import nix/nix-infra { inherit system; }
, rosDistro ? "noetic"
, rosPkgs ? pkgs.rosPackages.${rosDistro}
, Livox-SDK-src ? pkgs.airtonomy.nix-thunk.thunkSource nix/Livox-SDK
, Livox-SDK ? import nix/Livox-SDK.nix {
    inherit pkgs;
    src = Livox-SDK-src;
  }
}:
rec {
  inherit Livox-SDK;

  livox_ros_driver = rosPkgs.buildRosPackage {
    pname = "ros-${rosDistro}-livox_ros_driver";
    version = "1.0";

    src = ./livox_ros_driver;

    buildType = "catkin";
    buildInputs = [
      Livox-SDK
      rosPkgs.message-generation
    ];
    propagatedBuildInputs = with rosPkgs; [
      message-runtime
      pcl-ros
      rosbag
      roscpp
      rospy
      sensor-msgs
      std-msgs
    ];
    nativeBuildInputs = [ rosPkgs.catkin ];

    meta = {
      description = ''Livox ROS driver'';
    };
  };
}
