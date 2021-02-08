# Custom Message Support

Starling uses ROS2 and MAVROS to communicate with the vehicles. MAVROS includes
some custom message types that need to be built for MATLAB before it can use
them. The main MATLAB documentation is [here](https://uk.mathworks.com/help/ros/ref/ros2genmsg.html).
The specific steps for Starling are outlined below.

1. Check that your MATLAB installation is setup to be able to work with ROS2.
   The MATLAB documentation is [here](https://uk.mathworks.com/help/ros/gs/ros-system-requirements.html).
   Most importantly for ROS2, you need to ensure that you are using Python 3.
   Use `pyenv` in the MATLAB prompt to check the current setup. If it is not
   setup to use Python 3, restart MATLAB and use `pyenv('Version','3.7')` to
   set the version on Windows. On Linux, use `pyenv('Version','/path/to/python')`
   Additionally, you also need a C++ compiler and CMake.

1. Download the MAVROS repo with ROS2 message support from Github. Note that
   the official repo does not yet include ROS2 support. A modified version is
   available here: https://github.com/rob-clarke/mavros/tree/mavros2
   You can download the code as a [ZIP](https://github.com/rob-clarke/mavros/archive/mavros2.zip)
   from that site or use `git` to clone it:
   ```sh
   git clone -b mavros2 https://github.com/rob-clarke/mavros
   ```

1. Download the geographic_info repo. Again make sure you have the ROS2 branch:
    ```sh
    git clone -b ros2 https://github.com/ros-geographic-info/geographic_info
    ```

1. Copy `mavros_msgs` from `mavros` and `geographic_msgs` from
   `geographic_info` to a new folder called `matlab_msgs`:
   ```bash
   mkdir matlab_msgs
   cp -r mavros/mavros_msgs matlab_msgs/mavros_msgs
   cp -r geographic_info/geographic_msgs matlab_msgs/geographic_msgs
   ```

1. Clone the `unique_identifier_msgs` repo into your `matlab_msgs` folder:
   ```
   git clone https://github.com/ros2/unique_identifier_msgs matlab_msgs/unique_identifier_msgs
   ```

1. Use MATLAB's `ros2genmsg` on the `matlab_msgs` folder
   source:
   ```matlab
   ros2genmsg('matlab_msgs')
   ```

1. MATLAB's thing failed for me and I had to modify the venv it generates:
   ```bash
   echo "export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libexpat.so" >> ~/.matlab/R2019b/ros2/python_venv/bin/activate
   ```
   It then worked the second time round...

# Python Installation Notes

## On Ubuntu 18

See [here](https://uk.mathworks.com/matlabcentral/answers/581567-how-do-i-configure-ros2-for-matlab-r2020a-on-ubuntu-18-04)
for an answer from MATLAB support.

```bash
sudo apt install python3.7 libpython3.7 python3.7-venv
sudo ln -s /usr/lib/x86_64-linux-gnu/libpython3.7m.so.1 /usr/lib/libpython3.7m.so.1
sudo ln -s /usr/lib/x86_64-linux-gnu/libpython3.7m.so.1.0 /usr/lib/libpython3.7m.so.1.0
```