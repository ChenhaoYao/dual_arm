add_executable(cst_test cst_test.c)
target_link_libraries(cst_test soem)
install(TARGETS cst_test DESTINATION bin)
