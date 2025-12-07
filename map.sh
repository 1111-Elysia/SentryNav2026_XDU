cd ./src/pcd_to_nav_map
rm -rf build
mkdir build
cd build
cmake .. ..
make -j8
cd ../../../
./src/pcd_to_nav_map/build/pcd_to_nav2_map ./src/bringup/map/global.pcd ./src/pcd_to_nav_map/config/config.json