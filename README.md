# Fetch Teleop

## Git Submodules
As there are submodules in this repository, make sure to clone it using the `--recursive` tag:

    git clone https://github.com/kerry-he/fetch-teleop.git --recursive

If the repository has already been cloned, then the submodules can be fetched using:

    git submodule update --init
   
Further installation instructions can be found in corresponding folders. 

## Python3 Environment

To configure workspace using a Python3 environment:

    catkin build --cmake-args \ 
        -DCMAKE_BUILD_TYPE=Release \ 
        -DPYTHON_EXECUTABLE=/usr/bin/python3.8 \ 
        -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 \ 
        -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so

Check which version of Python to use by checking which files exist in the relevant directories. 

