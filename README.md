*This branch is about simulation modelling. Trained model is in model branch.*

# 2D-lidar-RCnn-Ros
	sudo apt install ros-noetic-* 
# Required ros packages 
	sudo apt install ros-noetic-fath-pivot-mount-description 
	sudo apt install ros-noetic-flir-camera-description
	sudo apt install ros-noetic-lms1xx
	sudo apt install ros-noetic-velodyne-description
# Usage
	cd src/map_creator_script/src
	sudo apt install libopencv-dev
	g++ circle.cpp main.cpp map.cpp obstacle.cpp ramp.cpp rmg.cpp square.cpp -o random-map `pkg-config --cflags --libs opencv4`
        cd src/map_creator_script/
	python3 random_map_generator.py

# Python Libaries
	pip3 install shapely 	
	pip3 install pcg_gazebo
## upgde python pip and downgrade softunicode
# Future Work
1. *Qt gui for simulation modelling. Now it is only work with python scripts*
2. *docker and bash installation file*
