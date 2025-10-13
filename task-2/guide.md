# Isaac Sim on Windows

1. [Download Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html)
- Extract contents of zip file to C:\Users\<USER>\isaacsim
- Run post_install.bat
- Run isaac-sim.bat (First open takes a long time and freezes a lot due to shaders. Just wait)
2. [Install Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/binaries_installation.html)
- We need to add environmental variables to Windows.
- Search "Environemental Variables" in Windows Search
- Under User Variables, click new and add:
    - Variable name: `ISAACSIM_PATH`
    - Variable value `C:\Users\<USER>\isaacsim\isaac-sim-standalone-4.5.0-windows-x86_64`
- And add:
    - Variable name: `ISAACSIM_PYTHON_EXE`
    - Variable value: `C:\Users\<USER>\isaacsim\isaac-sim-standalone-4.5.0-windows-x86_64\python.bat`
- [Install Miniconda in Windows](https://www.anaconda.com/docs/getting-started/miniconda/install#windows-powershell)
- Enable Windows long paths or this will fail
- Run the following commands in Conda Powershell terminal in \IsaacLab dir:
```powershell
# Make Conda env
conda create -n env_isaaclab python=3.10
conda activate env_isaaclab

# Update pip
python -m pip install --upgrade pip

# Install CUDA PyTorch for CUDA 12.8
pip install torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128

# Install IsaacLab via pip
pip install isaaclab[isaacsim,all]==2.1.0 --extra-index-url https://pypi.nvidia.com

# Test
isaacsim --help

# Launch Sample
isaacsim isaacsim.exp.full.kit
```
2. Installing ROS 2 Humble
- We are going to install this on WSL (Ubuntu 22.04) 24 doesn't work
- Everything else so far is on Windows, but ROS 2 will be WSL. They communicate via network.


