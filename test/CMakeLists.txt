find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME}_test_path_matching_diagnostic test_path_matching_diagnostic.cpp)
target_link_libraries(${PROJECT_NAME}_test_path_matching_diagnostic ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_path_matching_diagnostic PRIVATE -std=c++17)
add_test(test_path_matching_diagnostic ${PROJECT_NAME}_test_path_matching_diagnostic)

add_executable(${PROJECT_NAME}_test_on_the_fly_path_matching_diagnostic test_on_the_fly_path_matching_diagnostic.cpp)
target_link_libraries(${PROJECT_NAME}_test_on_the_fly_path_matching_diagnostic ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_on_the_fly_path_matching_diagnostic PRIVATE -std=c++17)
add_test(test_on_the_fly_path_matching_diagnostic ${PROJECT_NAME}_test_on_the_fly_path_matching_diagnostic)
