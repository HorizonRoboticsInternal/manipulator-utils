{ mkShell
, rosPackages
}:

mkShell rec {
  name = "ros-diagnostics";

  packages = with rosPackages.humble; [
    ros-core
    geometry-msgs
    xacro
    rviz2
  ];

  shellHook = ''
    export PS1="$(echo -e '\uf1c0') {\[$(tput sgr0)\]\[\033[38;5;228m\]\w\[$(tput sgr0)\]\[\033[38;5;15m\]} (${name}) \\$ \[$(tput sgr0)\]"
  '';
}
