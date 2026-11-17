# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Sadie Forbes

add_executable(nlaser_tests
  test_main.cpp
  test_ruida_codec.cpp
  test_ruida_decode.cpp
  test_ruida_encode.cpp
  test_lightburn_fixtures.cpp
  test_job_plan.cpp
  test_transport.cpp
  test_device_config.cpp
  test_raster.cpp
  test_svg_import.cpp
  test_lbrn2.cpp
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

# The NotoCAD bridge, in its own executable: it links NotoCAD, and the core
# tests must not.
if(NLASER_WITH_NOTOCAD)
  add_executable(nlaser_ncad_tests
    test_main.cpp
    test_laser_renderer.cpp
    test_laser_command.cpp
    test_svgin.cpp
  )
  target_include_directories(nlaser_ncad_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(nlaser_ncad_tests PRIVATE ncad_laser_ncad ncad_lisp nlaser_flags)
  add_test(NAME nlaser_ncad_tests COMMAND nlaser_ncad_tests)
endif()
