# set(DIAG_BASE_CLASSES DiagBase)
set(DIAG_TOOLS Diagnostics)

addPrefixAndSuffix("${DIAG_TOOLS}" "Test/src/" ".cc" DIAG_SRC)
# addPrefixAndSuffix("${DIAG_BASE_CLASSES}" "Test/" ".h" DIAG_BASE_HEADERS)
# addPrefixAndSuffix("${DIAG_TOOLS}" "Test/" ".h" DIAG_HEADERS)
# set(TEST_HEADERS ${DIAG_HEADERS} ${DIAG_BASE_HEADERS} Test/mtf_test.h PARENT_SCOPE)
set(MTF_INCLUDE_DIRS ${MTF_INCLUDE_DIRS} Test/include)
if (WIN32)
add_library (mtf_test STATIC ${DIAG_SRC})
else()
add_library (mtf_test SHARED ${DIAG_SRC})
endif()
target_compile_definitions(mtf_test PUBLIC ${MTF_DEFINITIONS})
target_compile_options(mtf_test PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
target_link_libraries(mtf_test ${MTF_LIBS})
target_include_directories(mtf_test PUBLIC ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
if(WIN32)
	install(TARGETS mtf_test ARCHIVE DESTINATION ${MTF_LIB_INSTALL_DIR} COMPONENT test_lib)
else()
	install(TARGETS mtf_test LIBRARY DESTINATION ${MTF_LIB_INSTALL_DIR} COMPONENT test_lib)
endif()  
add_custom_target(test_lib DEPENDS mtf_test)
if(NOT WIN32)
add_custom_target(install_test_lib
  ${CMAKE_COMMAND}
  -D "CMAKE_INSTALL_COMPONENT=test_lib"
  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
  DEPENDS mtf_test
  )
endif()
#add_custom_target(mtft DEPENDS install_lib install_test_lib install_header install_test_exe)
