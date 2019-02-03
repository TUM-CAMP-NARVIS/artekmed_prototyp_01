ARTEKMED Prototype I
---------------------

Prototype to explore 3D Pointcloud fusion and rendering.

by ulrich eck <ulrich.eck@tum.de>



Installation of dependencies on Ubuntu Linux 18.04
++++++++++++++++++++++++++++++++++++++++++++++++++


- Install Desktop Version X64

- Install system packages
 ```
  $ sudo apt-get update
  $ sudo apt-get upgrade
  $ sudo apt-get install build-essential

  $ sudo apt-get install libv4l-dev qv4l2
  $ sudo apt-get install opencl-headers
  $ sudo apt-get install ocl-icd-opencl-dev
  $ sudo apt-get install libusb-1.0-0-dev
  $ sudo apt-get install libgtk2.0-dev pkg-config
  $ sudo apt-get install libomp-dev
 ```


- Install Graphics Drivers (Nvidia Howto: https://linuxconfig.org/how-to-install-the-nvidia-drivers-on-ubuntu-18-04-bionic-beaver-linux)
 ```
  $ sudo add-apt-repository ppa:graphics-drivers/ppa
  $ sudo apt update
  $ ubuntu-drivers devices
  $ sudo ubuntu-drivers autoinstall
 ```

- Install CUDA
 ```
  $ wget https://developer.nvidia.com/compute/cuda/10.0/Prod/local_installers/cuda-repo-ubuntu1804-10-0-local-10.0.130-410.48_1.0-1_amd64
  $ mv cuda-repo-ubuntu1804-10-0-local-10.0.130-410.48_1.0-1_amd64 cuda-repo-ubuntu1804-10-0-local-10.0.130-410.48_1.0-1_amd64.deb

  $ sudo dpkg -i cuda-repo-ubuntu1804-10-0-local-10.0.130-410.48_1.0-1_amd64.deb
  $ sudo apt-key add /var/cuda-repo-10-0-local-10.0.130-410.48/7fa2af80.pub
  $ sudo apt-get update
  $ sudo apt-get install cuda
 ```

  carefully read the "UEFI" related dialogs and register the MOK in bios if asked
  make sure your installation works (glxinfo / nvidia-settings, cuda examples)
  maybe you need to blacklist some nvidia related modules (https://linuxconfig.org/how-to-disable-nouveau-nvidia-driver-on-ubuntu-18-04-bionic-beaver-linux)

 ```
  $ cd /usr/local/cuda/samples/1_Utilities/deviceQuery
  $ sudo make
  $ ./deviceQuery
 ```

- ZED Stereo Camera SDK [optional]
 ```
  $ wget https://download.stereolabs.com/zedsdk/2.7/ubuntu18
  $ mv ubuntu18 ZED_SDK_Ubuntu18_v2.7.1.run
  $ chmod +x ./ZED_SDK_Ubuntu18_v2.7.1.run
  $ ./ZED_SDK_Ubuntu18_v2.7.1.run
 ```
  

- Install developer Tools
 ```
  $ sudo apt-get install git cmake
  $ sudo apt-get install python3-pip python3-dev
 ```

- Install conan package manager
 ```
  $ sudo pip3 install --upgrade conan
  $ conan remote add bincrafters "https://api.bintray.com/conan/bincrafters/public-conan"
  $ conan remote add camposs "https://conan.campar.in.tum.de/api/conan/conan-camposs"
  $ conan remote add ubitrack "https://conan.campar.in.tum.de/api/conan/conan-ubitrack"
  $ conan remote add vendor "https://conan.campar.in.tum.de/api/conan/conan-vendor"
 ```

  change c++ default for stdlib in ~/.conan/profiles/default:
 ```
  $ conan profile update settings.compiler.libcxx=libstdc++11
 ```

  maybe needs fix: only python3 executable is installed but python is missing
  should be corrected in python_dev-config package; temporary fix:
 ```
  $ sudo ln -s /usr/bin/python3 /usr/local/bin/python
 ```


- Build an Ubitrack 1.3 release from scratch [optional]
 ```
  $ sudo pip3 install doit
  $ git clone https://github.com/ubitrack/ubitrack_release_tools.git
  $ cd ubitrack_release_tools
  $ cp custom_options_example.yml local_options.yml
 ```
  now adjust your configuration in local_options.yml
 ```
  $ doit custom_options=local_options.yml
 ```


- Build Artekmed Prototype (Example assumes CLion or cmd line):
 ```
  $ git clone https://github.com/TUM-CAMP-NARVIS/artekmed_prototyp_01.git
  $ cd artekmed_prototyp_01
  $ mkdir cmake-build-release && cd cmake-build-release
  $ conan install .. --build missing --build outdated -s build_type=Release
  $ cmake .. -DCMAKE_BUILD_TYPE=Release
  $ make -j 8
 ```

  Or launch Clion and use CMake / Build / Run to develop.

- Run Artekmed Protoype (assumes download of sample data and adjustments of paths in artekmed_zed_only_player.dfg)
 ```
  $ cd artekmed_prototyp_01/cmake-build-release
  $ source activate_run.sh
  $ ./bin/artekmed_p1 ../config/dfgs/artekmed_zed_only_player.dfg
 ```




Notes:
- NVIDIDA Docker Installation: https://github.com/NVIDIA/nvidia-docker