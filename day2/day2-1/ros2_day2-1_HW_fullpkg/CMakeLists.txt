cmake_minimum_required(VERSION 3.5)
project(control_turtle_pkg VERSION 0.1 LANGUAGES CXX)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# CMake의 자동 UIC, MOC, RCC 기능 비활성화
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTOMOC OFF)
set(CMAKE_AUTORCC ON)

# 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost 1.71.0 REQUIRED COMPONENTS thread)
find_package(Qt5 COMPONENTS Widgets Core Gui Network REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(turtlesim REQUIRED)

# 디렉터리 경로 설정
set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})
set(UI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ui)
set(RESOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/resources)

# 소스, 헤더, UI, 리소스 파일 설정
set(SOURCES
    ${SOURCE_DIR}/main.cpp
    ${SOURCE_DIR}/main_window.cpp
    ${SOURCE_DIR}/qnode.cpp
)
set(UI_FILES
    ${UI_DIR}/mainwindow.ui
)
set(RESOURCES
    ${RESOURCE_DIR}/images.qrc
)

# Qt 파일 처리
qt5_wrap_ui(QT_UI_HPP ${UI_FILES})
qt5_wrap_cpp(QT_MOC_HPP ${INCLUDE_DIR}/main_window.hpp ${INCLUDE_DIR}/qnode.hpp)
qt5_add_resources(QT_RESOURCE_HPP ${RESOURCES})

# include 경로 지정
include_directories(
    ${INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# 실행 파일 생성
add_executable(${PROJECT_NAME}
    ${SOURCES}
    ${QT_UI_HPP}
    ${QT_MOC_HPP}
    ${QT_RESOURCE_HPP}
    ${RESOURCES}
)

# 의존성 및 라이브러리 설정
ament_target_dependencies(${PROJECT_NAME} rclcpp Boost geometry_msgs std_srvs turtlesim)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Core Qt5::Gui Qt5::Network)

# 설치 경로 지정
install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

# 테스트 빌드 설정
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
