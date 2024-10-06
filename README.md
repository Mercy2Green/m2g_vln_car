# m2g_vln_car
 For the car


If you are installing on multiple robots, you may find it convenient to build the source code using the backend method. This requires an internet connection but is not dependent on Linux versions or kernel versions and does not require patching. Once you have extracted a source-code zip file, you can build the SDK from source with the simple method below:

    Go to the Librealsense root directory of the extracted folder and create a Build folder, and then change to that created folder using the command below:

mkdir build && cd build

    Whilst in the Build folder, run the CMake command below:

cmake ../ -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true

    Whilst still in the Build folder, run the CMake command below:

sudo make uninstall && make clean && make -j8 && sudo make install


dpkg -l | grep "realsense" | cut -d " " -f 3 | xargs sudo dpkg --purge