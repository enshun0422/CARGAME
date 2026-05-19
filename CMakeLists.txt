cmake_minimum_required(VERSION 3.10)
project(CarSimulator)

# 設定 C++ 標準
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 包含當前目錄，這樣程式才找得到標頭檔 (.hpp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 告訴 CMake 哪些檔案要編譯成執行檔
add_executable(CarGame main.cpp Vehicle.cpp Tire.cpp AeroDynamics.cpp TireDynamics.cpp)