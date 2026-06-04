add_subdirectory(csp_test)
add_subdirectory(cst_test)

if (${CMAKE_SYSTEM_NAME} STREQUAL Linux)
  add_subdirectory(eoe_test)
endif()

find_package (Python3 QUIET)
if (Python3_FOUND)
  add_subdirectory(eni_test)
endif()
