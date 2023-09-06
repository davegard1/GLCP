cmake_minimum_required (VERSION 3.10)

# set the project name
project(GLCP VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)


# Boost
# find_package(Boost 1.78.0) 
# if(Boost_FOUND)
#     include_directories(${Boost_INCLUDE_DIRS}) 
# endif()


find_package(Eigen3 3.4 REQUIRED NO_MODULE ) 



# include_directories(include)


# add_subdirectory(src)
# # add_subdirectory(drivers)


# add_executable(demo main.cpp)

# add_subdirectory(src)
set(SOURCES 
    src/riccati_solver.cpp
    src/controllers.cpp
    src/observers.cpp
    src/eom.cpp
    src/integration.cpp
    src/system.cpp
    src/steppers.cpp
    )


add_library(GLCP STATIC ${SOURCES})

target_include_directories(GLCP
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include 
)

target_link_libraries(GLCP 
    PUBLIC
    Eigen3::Eigen 
)

# target_include_directories(GCP PRIVATE include)
# target_include_directories(GCP PRIVATE src)

add_executable( demo cartpole_demo.cpp)

target_link_libraries( demo 
    PRIVATE
        GLCP
)

target_link_libraries(demo 
    PUBLIC
    Eigen3::Eigen 
)

