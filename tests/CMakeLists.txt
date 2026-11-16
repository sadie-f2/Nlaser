# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Sadie Forbes

add_executable(nlaser_tests
  test_main.cpp
  test_ruida_codec.cpp
  test_ruida_decode.cpp
  test_ruida_encode.cpp
  test_job_plan.cpp
)
target_compile_definitions(nlaser_tests PRIVATE
  NLASER_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
target_link_libraries(nlaser_tests PRIVATE ncad_laser_core nlaser_flags)
add_test(NAME nlaser_tests COMMAND nlaser_tests)
