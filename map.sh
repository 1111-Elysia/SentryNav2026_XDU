source ./install/setup.bash
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: 'new_map'}"
rm -rf ./data/new_map/*.yaml ./data/new_map/*.pgm
cp ./data/new_map/* ./src/bringup/map/
cd ./src/pcd_to_nav_map
rm -rf build
mkdir build
cd build
cmake .. ..
make -j8
cd ../../../
./src/pcd_to_nav_map/build/pcd_to_nav2_map ./src/bringup/map/global.pcd ./src/pcd_to_nav_map/config/config.json
