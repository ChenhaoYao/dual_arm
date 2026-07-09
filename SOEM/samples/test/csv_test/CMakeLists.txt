add_executable(csv_test csv_test.c)
target_link_libraries(csv_test soem)
install(TARGETS csv_test DESTINATION bin)
