set(TEST_TYPE "INTEGRATION")

file(GLOB test_files "*.cc")
list(SORT test_files)

set(tests "")

foreach(test_file ${test_files})

  get_filename_component(test ${test_file} NAME)
  list(APPEND tests ${test})

endforeach(test_file)

link_directories(${PROJECT_BINARY_DIR}/test)

ign_build_tests(${tests})
