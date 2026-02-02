# For UFACTORY data collection usage

## 1. Install
Please first install additional dependencies into your environment by running:
- #### Install librealsense
    ```bash
    # https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md
    # Register the server's public key
    sudo mkdir -p /etc/apt/keyrings
    curl -sSf https://librealsense.realsenseai.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

    # Make sure apt HTTPS support is installed: 
    sudo apt-get install apt-transport-https

    # Add the server to the list of repositories
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.realsenseai.com/Debian/apt-repo `lsb_release -cs` main" | sudo tee /etc/apt/sources.list.d/librealsense.list
    sudo apt-get update

    # Install the libraries (see section below if upgrading packages):
    sudo apt-get install librealsense2-dkms
    sudo apt-get install librealsense2-utils
    ```
- #### Install additional python dependencies
    ```bash
    # This would install gello and space mouse related modules.
    $ pip install -r requirements_extra.txt
    $ pip install -e src/gello/third_party/DynamixelSDK/python
    # install pyrealsense2
    $ pip install pyrealsense2
    # install xarm-python-sdk
    $ pip install git+https://github.com/xArm-Developer/xArm-Python-SDK.git
    ```

## 2. Dataset Recording
Please use provided `uf_robot_record.py` script in this directory instead of lerobot_record.py in scripts folder. We have made modifications for space mouse data collection (delta command to absolute command) and enabled using external yaml file as flexible configuration. Check the two `*_record_config.yaml` files in config and specify correct filename executing `uf_robot_record.py`.

**Keyboard control** for data recording:  

“`->`” Exit early: Finish current episode, save and enter reset process for next episode preparation；  
"`<-`" rerecord_episode + Exit early: Terminate current recording, reset and then re-record this episode；  
"`Esc`" stop_recording + Exit early: Exit recording process；

**Please follow the on-screen instructions**

- #### Gello
    ```bash
    python uf_robot_record.py --config config/xarm7_gello_record_config.yaml

    # Use the --resume parameter to continue collecting data on an existing dataset.
    ```

- #### Space Mouse
    *The space mouse currently is configured as **2D control** (in XY plane) for Push T block task, if you need more freedom, check and edit the interface in `teleoperators/space_mouse/space_mouse.py`.*  

    ```bash
    python uf_robot_record.py --config config/xarm7_spacemouse_record_config.yaml

    # Use the --resume parameter to continue collecting data on an existing dataset.
    ```

## 3. Training
- #### Gello
    Use official script for training, here use `act` model and resume training option as example, edit or remove arguments based on your case:
    ```bash
    python -m lerobot.scripts.lerobot_train  --dataset.repo_id=ufactory/xarm7_record_datas --policy.type=act --output_dir=outputs/train/xarm7_record_datas --job_name=xarm7_record_datas --policy.device=cuda --policy.repo_id=ufactory/xarm7_record_datas

    # Use the --resume parameter to continue training
    # --resume=true --config_path=<YOUR_LOCAL_PATH>/train/xarm7_pushT/checkpoints/last/pretrained_model/train_config.json
    ```
- #### Space Mouse
    Use official script for training, here use `diffusion` model and resume training option as example, edit or remove arguments based on your case:
    ```bash
    python -m lerobot.scripts.lerobot_train  --dataset.repo_id=ufactory/xarm7_pushT --policy.type=diffusion --output_dir=outputs/train/xarm7_pushT --job_name=xarm7_pushT --policy.device=cuda --policy.repo_id=ufactory/xarm7_pushT

    # Use the --resume parameter to continue training
    # --resume=true --config_path=<YOUR_LOCAL_PATH>/train/xarm7_pushT/checkpoints/last/pretrained_model/train_config.json
    ```

## 4. Evaluation
Use official script for trained policy evaluation, for example:
- #### Gello
    ```bash
    python uf_robot_eval.py --config config/xarm7_gello_record_config.yaml --policy.path=<YOUR_LOCAL_PATH>/outputs/train/xarm7_record_datas/checkpoints/last/pretrained_model/
    ```
- #### Space Mouse
    ```bash
    python uf_robot_eval.py --config config/xarm7_spacemouse_record_config.yaml --policy.path=<YOUR_LOCAL_PATH>/outputs/train/xarm7_record_datas/checkpoints/last/pretrained_model/
    ```


## 5. Others
For other LeRobot supported features like Dataset Visualization or Editting, please go through LeRobot Documentation for instructions.

## 6. Special Notices:
Users need to **study the whole codebase thoroughly**, and understand about relevant configuration parameters, since the configurations written in the code are not for all use cases and set-ups, it is the users job to study the code or theories to get good knowledge about them and modify/tune on their own. Especially for diffusion policy, default parameters from LeRobot may be targeted for simulation and not optimized for real robot scenarios.