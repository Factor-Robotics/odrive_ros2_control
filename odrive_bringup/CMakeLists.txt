cmake_minimum_required(VERSION 3.5)
project(odrive_bringup)

find_package(ament_cmake_auto REQUIRED)

ament_auto_package(INSTALL_TO_SHARE config launch)
