cmake_minimum_required(VERSION 3.10)
project(UR_RTDE_OpenCV_RealSense_Project)

# ==========================================================
# 🧠 Compiler configuration
# ==========================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# ==========================================================
# 🧩 Use your locally built OpenCV 4.12.0 (with contrib)
# ==========================================================
# ⚠️ IMPORTANT: Use ABSOLUTE PATHS (no $ENV{HOME}) to avoid confusion
set(OpenCV_DIR "/home/lunet/ttpp/opencv_build/install/lib/cmake/opencv4")

# Include and lib paths from your successful install
include_directories(BEFORE /home/lunet/ttpp/opencv_build/install/include/opencv4)
link_directories(BEFORE /home/lunet/ttpp/opencv_build/install/lib)

# ==========================================================
# 📸 Include Intel RealSense SDK (adjust if local or system install)
# ==========================================================
include_directories(/home/lunet/ttpp/realsense_local/include)
link_directories(/home/lunet/ttpp/realsense_local/lib)

# ==========================================================
# 🔍 Find OpenCV and show configuration
# ==========================================================
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

message(STATUS "")
message(STATUS "-------------------------------------------")
message(STATUS "✅ OpenCV version: ${OpenCV_VERSION}")
message(STATUS "✅ OpenCV include dir: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "✅ OpenCV libraries: ${OpenCV_LIBS}")
message(STATUS "-------------------------------------------")

# ==========================================================
# 📦 Include your custom project headers
# ==========================================================
include_directories(${CMAKE_SOURCE_DIR}/../include)

# ==========================================================
# ⚙️ Library directories (UR RTDE, etc.)
# ==========================================================
link_directories(${CMAKE_SOURCE_DIR}/../build)

# ==========================================================
# 🔎 Collect all .cpp files in this directory
# ==========================================================
file(GLOB SRC_FILES "${CMAKE_SOURCE_DIR}/*.cpp")

# ==========================================================
# 🏗️ Output directory for built executables
# ==========================================================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# ==========================================================
# ⚙️ Build each .cpp file into a separate executable
# ==========================================================
foreach(SRC_FILE ${SRC_FILES})
    get_filename_component(EXEC_NAME ${SRC_FILE} NAME_WE)

    add_executable(${EXEC_NAME} ${SRC_FILE})

    # ✅ Link dependencies — OpenCV handles aruco automatically
    target_link_libraries(${EXEC_NAME}
        rtde
        boost_system
        boost_thread
        pthread
        realsense2
        ${OpenCV_LIBS}
    )
endforeach()

# ==========================================================
# ✅ Summary message
# ==========================================================
message(STATUS "")
message(STATUS "==========================================================")
message(STATUS " ✅  Project: ${PROJECT_NAME}")
message(STATUS " ✅  OpenCV version: ${OpenCV_VERSION}")
message(STATUS " ✅  Output directory: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "==========================================================")
