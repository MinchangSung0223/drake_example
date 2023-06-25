cmake_minimum_required(VERSION 3.8)
project(indy7 LANGUAGES CXX)

set(CMAKE_DEBUG_POSTFIX "d")
file(GLOB SOURCE_FILES "*.cc" "*.c" "*.h" "*.hpp")

add_library(${PROJECT_NAME} SHARED STATIC
            ${SOURCE_FILES}
            )

add_library(sub::indy7 ALIAS ${PROJECT_NAME} )

target_include_directories( ${PROJECT_NAME}
        PUBLIC ${PROJECT_SOURCE_DIR})

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen drake::drake)

#find_package(Eigen 3.0.0)
#IF( Eigen_FOUND )
#    add_definitions( ${Eigen_DEFINITIONS} )
#    target_include_directories (${PROJECT_NAME} PUBLIC ${Eigen_INCLUDE_DIRS} )
#ELSE()
#    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN_INCLUDE_DIR to the include directory of your Eigen installation.")
#ENDIF()