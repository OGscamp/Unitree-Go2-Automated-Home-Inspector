# Unitree SDK Setup Notes

## Installing SDK Python Module

Follow the instructions in the README.md file: 
https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/README.md

### Make a venv and install the module

```bash
python3 -m venv venv
source venv/bin/activate
pip install -e ~/path/to/....../unitree_sdk2_python
```

### Export CycloneDDS variables

```bash
export CMAKE_PREFIX_PATH=/home/<USER>/cyclonedds/install:$CMAKE_PREFIX_PATH
CYCLONEDDS_HOME=/home/<USER>/cyclonedds/install
```

## Adding Module to Pylance

To allow IntelliSense in VSCode, add the following to your `settings.json` file:

```json
"python.analysis.extraPaths": [
    "/your/path/......../unitree_sdk2_python"
]
```