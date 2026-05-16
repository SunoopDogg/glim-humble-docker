cd ~

apt update

apt install -y --no-install-recommends \
    libomp-dev \
    libboost-all-dev \
    libmetis-dev \
    libfmt-dev \
    libspdlog-dev \
    libglm-dev \
    libglfw3-dev \
    libpng-dev \
    libjpeg-dev

apt install -y --no-install-recommends \
    libopencv-dev \
    ros-humble-image-transport \
    ros-humble-cv-bridge

# ouster_ros (real Ouster driver) build/runtime deps — pulled by `rosdep install
# --from-paths src/ouster-ros` but listed here so the real_mapping path builds one-shot.
apt install -y --no-install-recommends \
    ros-humble-pcl-conversions \
    libpcl-dev \
    libtins-dev \
    libpcap-dev

# Go2 EDU DDS bridge needs the CycloneDDS RMW (the image only ships cyclonedds-tools /
# libcycloneddsidl — the DDS lib, NOT the ROS RMW). Without this the Go2 path silently
# can't reach the robot (go2_sport_bridge publishes on fastrtps, Go2 listens on cyclonedds).
apt install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp

git clone https://github.com/borglab/gtsam
cd gtsam && git checkout 4.3a0
mkdir build && cd build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_WITH_TBB=OFF \
         -DGTSAM_USE_SYSTEM_EIGEN=ON \
         -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j$(nproc)
make install

git clone https://github.com/koide3/iridescence --recursive
mkdir iridescence/build && cd iridescence/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
make install

git clone https://github.com/koide3/gtsam_points
mkdir gtsam_points/build && cd gtsam_points/build
cmake .. -DBUILD_WITH_CUDA=ON    # GPU 없으면 OFF로 변경
make -j$(nproc)
make install

ldconfig

apt clean \
    && rm -rf /var/lib/apt/lists/*

cd ~/glim-humble-docker
