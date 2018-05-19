# python 3 module with minimum build but third party enabled
# multiple installed CUDA versions that cause OpenCV conflict

cmake -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.5m.so -DPYTHON_INCLUDE_DIR=/usr/include/python3.5 -DPY_VER=3.5 -DMTF_PY_INSTALL_DIR=/usr/local/lib/python3.5/dist-packages -DWITH_TEMPLATED=OFF -DWITH_FLANN=OFF -DWITH_FEAT=OFF -DWITH_MEX=OFF  -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-8.0 ..
