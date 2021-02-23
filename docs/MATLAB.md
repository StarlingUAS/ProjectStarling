# Custom Message Support

[**MATLAB HAS NO CUSTOM ROS2 SERVICE SUPPORT**](https://uk.mathworks.com/matlabcentral/answers/741977-has-matlab-ros2-support-for-actions-and-services-not-been-implemented-yet/?s_tid=ans_lp_feed_leaf)


Starling uses ROS2 and MAVROS to communicate with the vehicles. MAVROS includes
some custom message types that need to be built for MATLAB before it can use
them. The main MATLAB documentation is [here](https://uk.mathworks.com/help/ros/ref/ros2genmsg.html).
The specific steps for Starling are outlined below.

1. Check that your MATLAB installation is setup to be able to work with ROS2.
   The MATLAB documentation is [here](https://uk.mathworks.com/help/ros/gs/ros-system-requirements.html).
   Firstly you need MATLAB's ROS Toolbox. For ROS2, you need to ensure that
   you are using Python 3. Use `pyenv` in the MATLAB prompt to check the
   current setup. If it is not setup to use Python 3, restart MATLAB and use
   `pyenv('Version','3.7')` to set the version on Windows. On Linux, use
   `pyenv('Version','/path/to/python')` Additionally, you also need a C+
   compiler and CMake.

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

## On Ubuntu 18

`ros2genmsg` failed for me and I had to modify the venv it generates:
```bash
echo "export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libexpat.so" >> ~/.matlab/R2019b/ros2/python_venv/bin/activate
```
It then worked the second time round...

See [here](https://uk.mathworks.com/matlabcentral/answers/581567-how-do-i-configure-ros2-for-matlab-r2020a-on-ubuntu-18-04)
for an answer from MATLAB support.

```bash
sudo apt install python3.7 libpython3.7 python3.7-venv
sudo ln -s /usr/lib/x86_64-linux-gnu/libpython3.7m.so.1 /usr/lib/libpython3.7m.so.1
sudo ln -s /usr/lib/x86_64-linux-gnu/libpython3.7m.so.1.0 /usr/lib/libpython3.7m.so.1.0
```

## Windows

`ros2genmsg` apparently needs a very specfic version of Python (namely 3.7). The last available
Windows installer for this is Python 3.7.9. See the [Python downloads page](https://www.python.org/downloads/windows/)

The ROS Toolbox needs a very specific C++ compiler too: Microsoft Visual C++ 2017.
MATLAB documentation on compiler support is [here](https://uk.mathworks.com/support/requirements/supported-compilers.html).

Visual Studio 2017 can be downloaded from [here](https://docs.microsoft.com/en-us/visualstudio/releasenotes/vs2017-relnotes).
You should only need the community edition.

Once Visual Studio is installed, run `mex -setup cpp` in the MATLAB prompt. You should see something like:
`MEX configured to use 'Microsoft Visual C++ 2017' for C++ language compilation.`

You also need to have CMake, installed. A binary installer can be found here:
[cmake.org/download/](https://cmake.org/download/)

*I'm considering giving up on Windows at this point*

File paths might be too long! There may be a registry fix, but we'll just move things to a shorter path!

Also MATLAB installs outdated versions of things which break some of the mavros_msgs files. Need to edit
`mavros_msgs/msg/BatteryStatus.msg` to add any random characters next to the unit comments for `voltage`
and `current`.

## macOS

You will need xcode tools installed. Press Cmd-space and search for "Terminal". Then type the command
below and press enter.
```bash
xcode-select --install
```

# Deployment

Once built, the Matlab script calls `ros.internal.custommsgs.updatePreferences(msgFullName,{},{},{},'ros2',genDir)`,
where `genDir = fullfile(folderPath, 'matlab_msg_gen', computer('arch'));` (e.g. `matlab_msgs/matlab_msg_gen/win64`),
and `msgFullName` is a cell array containing `${PKG_NAME}/${MSG_NAME}` strings. How to do this without the build
process? I don't know...
