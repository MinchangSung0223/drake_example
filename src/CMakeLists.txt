# SPDX-License-Identifier: MIT-0
find_package(gflags REQUIRED)


add_library(particle particle.cc particle.h)
target_link_libraries(particle drake::drake)

add_executable(particle_test particle_test.cc)
add_executable(main main.cc)
add_executable(double_pendulum double_pendulum.cc)
add_executable(indy7_test indy7_test.cc)

target_link_libraries(particle_test particle drake::drake gtest gflags)
target_link_libraries(main drake::drake gtest gflags)
target_link_libraries(double_pendulum drake::drake gtest gflags)
target_link_libraries(indy7_test drake::drake gtest gflags sub::indy7)

add_test(NAME cc_particle_test COMMAND particle_test)
set_tests_properties(cc_particle_test PROPERTIES LABELS small TIMEOUT 60)

add_test(NAME python_particle_test
  COMMAND Python::Interpreter -B -m unittest particle_test
)
set_tests_properties(python_particle_test PROPERTIES
  ENVIRONMENT "PYTHONPATH=${DRAKE_PYTHONPATH}"
  LABELS small
  REQUIRED_FILES "${CMAKE_CURRENT_SOURCE_DIR}/particle_test.py"
  TIMEOUT 60
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
