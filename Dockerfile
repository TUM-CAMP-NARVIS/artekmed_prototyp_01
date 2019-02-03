FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04

ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

RUN apt-get update --fix-missing && \
    apt-get install -y wget bzip2 ca-certificates curl apt-utils sudo

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
                       build-essential libv4l-dev qv4l2 opencl-headers systemtap-sdt-dev\
                       ocl-icd-opencl-dev libusb-1.0-0-dev libgtk2.0-dev \
                       pkg-config libomp-dev git cmake python3-pip python3-dev python3-setuptools

RUN pip3 install --upgrade conan && pip3 install --upgrade doit

RUN conan user && \
    conan remote add bincrafters "https://api.bintray.com/conan/bincrafters/public-conan" && \
    conan remote add camposs "https://conan.campar.in.tum.de/api/conan/conan-camposs" && \
    conan remote add ubitrack "https://conan.campar.in.tum.de/api/conan/conan-ubitrack" && \
    conan remote add vendor "https://conan.campar.in.tum.de/api/conan/conan-vendor" && \
    conan profile new default --detect && \
    conan profile update settings.compiler.libcxx=libstdc++11 default

# fix to find python ..
RUN ln -s /usr/bin/python3 /usr/local/bin/python

ENV DISPLAY=:1

WORKDIR /root

RUN git clone https://github.com/ubitrack/ubitrack_release_tools.git && \
    cd ubitrack_release_tools && \
    doit

ENTRYPOINT ["/bin/bash"]

