# script to install sai-model

cd rbdl
mkdir -p build && cd build && cmake .. && make -j8

cd ../..
mkdir -p build && cd build && cmake .. && make -j8

cd ..
