# Isaac Sim on Windows

1. [Download Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html)
- Extract contents of zip file to C:\Users\<USER>\isaacsim
- Run post_install.bat
- Run isaac-sim.bat (First open takes a looong time and freezes a lot due to shaders. Just wait)
2. [Install Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/binaries_installation.html)
- We need to add environmental variables to Windows.
- Search "Environemental Variables" in Windows Search
- Under User Variables, click new and add:
    - Variable name: `ISAACSIM_PATH`
    - Variable value `C:\Users\<USER>\isaacsim\isaac-sim-standalone-4.5.0-windows-x86_64`
- And add:
    - Variable name: `ISAACSIM_PYTHON_EXE`
    - Variable value: `C:\Users\<USER>\isaacsim\isaac-sim-standalone-4.5.0-windows-x86_64\python.bat`
