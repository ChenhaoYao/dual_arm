add_executable(csp_test csp_test.c)
target_link_libraries(csp_test soem)
install(TARGETS csp_test DESTINATION bin)
