export PKG_CONFIG_PATH=/home/victor/developpement/aravis/install/lib/pkgconfig/
cmake .. -DCMAKE_CUDA_HOST_COMPILER=/home/victor/developpement/gcc/install/bin/gcc -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc
cmake . -DOpenCV_DIR=/home/victor/developpement/opencv/install/share/OpenCV/
cmake . -Dnanoflann_DIR=/home/victor/developpement/nanoflann/install/lib/cmake/nanoflann/
cmake . -Dg2o_DIR=/home/victor/developpement/g2o/install/lib/cmake/g2o/
cmake . -DCMAKE_INSTALL_PREFIX=/home/victor/developpement/slam/install
cmake . -DCMAKE_BUILD_TYPE=Debug

