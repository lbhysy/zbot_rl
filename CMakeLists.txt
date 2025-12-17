cmake_minimum_required(VERSION 3.5)
project(can_test)

# ========================
# 基本配置
# ========================
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# ========================
# 头文件路径
# ========================
include_directories(
    include
    can
)

# ========================
# LibTorch 选项
# ========================
option(USE_LIBTORCH "Enable linking with LibTorch (TorchScript)" ON)

if (USE_LIBTORCH)
    if (NOT DEFINED CMAKE_PREFIX_PATH OR CMAKE_PREFIX_PATH STREQUAL "")
        set(CMAKE_PREFIX_PATH "/home/rain/libtorch" CACHE PATH "Path to LibTorch")
        message(STATUS "CMAKE_PREFIX_PATH not provided, using /home/rain/libtorch")
    endif()
endif()

# ========================
# Motor_test 选项（默认 OFF）
# ========================
option(USE_MOTOR_TEST "Use Motor_test instead of Zbot_RL" OFF)

# ========================
# 源文件
# ========================
set(SOURCES
    src/Hipnuc_IMU.cpp
    src/RS_motor.cpp
    src/Zbot_RL.cpp
    src/can_node.cpp
    src/Motor_test.cpp
)

# ========================
# 可执行文件
# ========================
add_executable(can_node ${SOURCES})

if (USE_MOTOR_TEST)
    target_compile_definitions(can_node PRIVATE USE_MOTOR_TEST)
endif()

# ★ 强制使用 C++17（std::clamp 必需）
target_compile_features(can_node PRIVATE cxx_std_17)

# ========================
# 链接基础库
# ========================
target_link_libraries(can_node
    pthread
    usb_can
    ${CMAKE_THREAD_LIBS_INIT}
    ${CMAKE_DL_LIBS}
)

# ========================
# LibTorch
# ========================
if (USE_LIBTORCH)
    find_package(Torch REQUIRED)
    target_compile_definitions(can_node PRIVATE USE_LIBTORCH=1)
    target_link_libraries(can_node "${TORCH_LIBRARIES}")
    message(STATUS "LibTorch linked")
endif()

# ========================
# 输出路径
# ========================
set_target_properties(can_node PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build
)

# # ========================
# # 安装（可选）
# # ========================
# install(TARGETS can_node DESTINATION bin)