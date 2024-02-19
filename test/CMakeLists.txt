find_package(GTest REQUIRED)

get_filename_component(TEST_DIR_SRC ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(${PROJECT_NAME}_test_path_matching_diagnostic test_path_matching_diagnostic.cpp)
target_link_libraries(${PROJECT_NAME}_test_path_matching_diagnostic ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_path_matching_diagnostic PRIVATE -std=c++17)
add_test(test_path_matching_diagnostic ${PROJECT_NAME}_test_path_matching_diagnostic)

add_executable(${PROJECT_NAME}_test_on_the_fly_path_matching_diagnostic test_on_the_fly_path_matching_diagnostic.cpp)
target_link_libraries(${PROJECT_NAME}_test_on_the_fly_path_matching_diagnostic ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_on_the_fly_path_matching_diagnostic PRIVATE -std=c++17)
add_test(test_on_the_fly_path_matching_diagnostic ${PROJECT_NAME}_test_on_the_fly_path_matching_diagnostic)

add_executable(${PROJECT_NAME}_test_on_the_fly_path_matching test_on_the_fly_path_matching.cpp)
target_link_libraries(${PROJECT_NAME}_test_on_the_fly_path_matching ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_on_the_fly_path_matching PRIVATE -std=c++17)
add_test(test_on_the_fly_path_matching ${PROJECT_NAME}_test_on_the_fly_path_matching)

add_executable(${PROJECT_NAME}_test_path_matching test_path_matching.cpp)
target_link_libraries(${PROJECT_NAME}_test_path_matching ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_path_matching PRIVATE -std=c++17)
add_test(test_path_matching ${PROJECT_NAME}_test_path_matching)
