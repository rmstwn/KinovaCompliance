cmake_minimum_required(VERSION 3.0.0)
project(KinovaCompliance VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()


# Activate C++ 11
# set (CMAKE_CXX_STANDARD 11)

# Find pkg
find_package(Eigen3 REQUIRED)
find_package(casadi REQUIRED)
find_package(pinocchio REQUIRED)

# set casadi include directory
set(CASADI_INCLUDE_DIR /usr/local/include/casadi)

# find casadi library
find_library(CASADI_LIBRARY
    NAMES casadi
    HINTS ${CASADI_INCLUDE_DIR}/../lib $ENV{CASADI_PREFIX}/lib)
if(CASADI_LIBRARY)
    set(CASADI_LIBRARIES ${CASADI_LIBRARIES} ${CASADI_LIBRARY})
endif()

# include header directories
include_directories(
    SYSTEM ${CASADI_INCLUDE_DIR}
    include/
    src/
    ${EIGEN3_INCLUDE_DIR}
    ${PINOCCHIO_INCLUDE_DIRS}
    )

# Set the path to the Kinova Kortex API include directory
set(KORTEX_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/kortex/include")

# Set the path to the Kinova Kortex API library directory
set(KORTEX_LIBRARY_DIR "${CMAKE_SOURCE_DIR}/kortex/lib")

# Add the Kinova Mobile library directory to the library path
include_directories(include/)   

# Add the Kinova Kortex API include directory to the include path
include_directories(${KORTEX_INCLUDE_DIR})
include_directories("${KORTEX_INCLUDE_DIR}/client")
include_directories("${KORTEX_INCLUDE_DIR}/client_stubs")
include_directories("${KORTEX_INCLUDE_DIR}/common")
include_directories("${KORTEX_INCLUDE_DIR}/google")
include_directories("${KORTEX_INCLUDE_DIR}/messages")

include_directories("${PROJECT_SOURCE_DIR}/thirdParty/cxxopts")

add_definitions(-D_OS_UNIX)

# Add the Kinova Mobile library directory to the library path
link_directories(include/)

# Add the Kinova Kortex API library directory to the library path
link_directories(${KORTEX_LIBRARY_DIR})

# Add your source files
set(SOURCES 
    main.cpp 
    src/KinovaClient/utilities.cpp
    # src/KinovaMobile/kinova_mobile.cpp
    # src/KinovaClient/kinova_client.cpp
    # src/KinovaClient/specifications.cpp
    # src/KinovaClient/utilities.cpp
    # src/KinovaControl/controller.cpp
    # src/KinovaControl/rate_counter.cpp
    # src/KinovaControl/state.cpp
)

# Create the executable
add_executable(KinovaCompliance ${SOURCES})

# Link against the Kinova Kortex API library
target_link_libraries(KinovaCompliance ${KORTEX_LIBRARY_DIR})
target_link_libraries(KinovaCompliance ${KORTEX_LIBRARY_DIR}/release/libKortexApiCpp.a )

# Link against the Casadi library
target_link_libraries(KinovaCompliance ${CASADI_LIBRARIES})


# Include directories for your project
target_include_directories(KinovaCompliance PRIVATE include/)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)


# cmake_minimum_required(VERSION 3.0.0)
# project(KinovaCompliance VERSION 0.1.0 LANGUAGES C CXX)

# set (CMAKE_CXX_STANDARD 11)

# include(CTest)
# enable_testing()

# include_directories(include/)
# link_directories(include/)

# add_executable(mainet main.cpp src/kinova_mobile.cpp)
# target_include_directories(mainet PRIVATE include/)


# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)
