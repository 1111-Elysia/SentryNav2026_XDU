# 1、第三方插件 Pangolin 编译安装
cd ./src/lightning-lm/thirdparty
sudo apt install unzip
unzip Pangolin-0.9.3.zip 
cd Pangolin-0.9.3/
mkdir build
cd build
cmake ..
make 
sudo make install
# 2、依赖库安装
cd ../../../../..
bash depend_install.sh
# 3、lightning-lm安装，注意，不建议把lightning-lm与其他包同时编译
colcon build --packages-select lightning --cmake-args -DCMAKE_BUILD_TYPE=Release
# 4、导航系统编译安装
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release