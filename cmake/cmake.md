Collection of cmake commands for different build configurations.

# python 3 module with minimum build but third party enabled; multiple installed CUDA versions that cause OpenCV conflict

cmake -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.5m.so -DPYTHON_INCLUDE_DIR=/usr/include/python3.5 -DPY_VER=3.5 -DMTF_PY_INSTALL_DIR=/usr/local/lib/python3.5/dist-packages -DWITH_TEMPLATED=OFF -DWITH_FLANN=OFF -DWITH_FEAT=OFF -DWITH_MEX=OFF  -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-8.0 ..

## feature tracker enabled

cmake -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.5m.so -DPYTHON_INCLUDE_DIR=/usr/include/python3.5 -DPY_VER=3.5 -DMTF_PY_INSTALL_DIR=/usr/local/lib/python3.5/dist-packages -DWITH_TEMPLATED=OFF -DWITH_FLANN=OFF -DWITH_MEX=OFF  -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-8.0 ..

### FLANN enabled too

cmake -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.5m.so -DPYTHON_INCLUDE_DIR=/usr/include/python3.5 -DPY_VER=3.5 -DMTF_PY_INSTALL_DIR=/usr/local/lib/python3.5/dist-packages -DWITH_TEMPLATED=OFF -DWITH_MEX=OFF  -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-8.0 ..

# python 2 module

cmake -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython2.7.so -DPYTHON_INCLUDE_DIR=/usr/include/python2.7 -DPY_VER=2.7 -DMTF_PY_INSTALL_DIR=/usr/local/lib/python2.7/dist-packages -DWITH_TEMPLATED=OFF -DWITH_FLANN=OFF -DWITH_FEAT=OFF -DWITH_MEX=OFF  -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-8.0 ..

