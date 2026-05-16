cmake_minimum_required(VERSION 3.8)

project(robot_urdf)

find_package(ament_cmake REQUIRED)
foreach(dir config launch meshes urdf worlds)
	install(DIRECTORY ${dir}/
		DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

install(PROGRAMS
  scripts/zed_noise_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
