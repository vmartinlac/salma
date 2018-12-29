export PKG_CONFIG_PATH=/home/victor/developpement/aravis/install/lib/pkgconfig/
cmake .. -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER=clang -DCMAKE_CUDA_HOST_COMPILER=/home/victor/developpement/gcc/install/bin/gcc -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc
cmake . -DOpenCV_DIR=/home/victor/developpement/opencv/install/share/OpenCV/
cmake . -Dnanoflann_DIR=/home/victor/developpement/nanoflann/install/lib/cmake/nanoflann/
cmake . -DCMAKE_INSTALL_PREFIX=/home/victor/developpement/slam/install
cmake . -DCMAKE_BUILD_TYPE=Debug

