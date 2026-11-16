# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Sadie Forbes

add_executable(nlaser_tests
  test_main.cpp
  test_ruida_codec.cpp
  test_ruida_decode.cpp
  test_ruida_encode.cpp
  test_job_plan.cpp
  test_transport.cpp
  test_device_config.cpp
)
target_compile_definitions(nlaser_tests PRIVATE
  NLASER_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
target_link_libraries(nlaser_tests PRIVATE ncad_laser_core nlaser_flags)
add_test(NAME nlaser_tests COMMAND nlaser_tests)

if(NLASER_WITH_NET)
  find_package(Threads REQUIRED)
  add_executable(nlaser_net_tests
    test_main.cpp
    test_udp.cpp
    test_serial.cpp
  )
  target_link_libraries(nlaser_net_tests PRIVATE ncad_laser_net Threads::Threads nlaser_flags)
  add_test(NAME nlaser_net_tests COMMAND nlaser_net_tests)
endif()

if(NLASER_WITH_NET)
  add_executable(nlaser_fakectl fakectl.cpp)
  target_link_libraries(nlaser_fakectl PRIVATE ncad_laser_core nlaser_flags)
endif()
