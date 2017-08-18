include (${project_cmake_dir}/Utils.cmake)

file(GLOB sources "*.cc")
list(SORT sources)

file(GLOB gtest_sources "*_TEST.cc")
list(SORT gtest_sources)

foreach(test ${gtest_sources})
  list(REMOVE_ITEM sources ${test})
endforeach(test)

ign_build_tests(${gtest_sources})
ign_add_library(${PROJECT_NAME_LOWER} ${sources})
ign_install_library(${PROJECT_NAME_LOWER})
