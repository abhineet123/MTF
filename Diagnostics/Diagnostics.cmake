# set(DIAG_BASE_CLASSES DiagBase)
set(DIAG_TOOLS Diagnostics)

addPrefixAndSuffix("${DIAG_TOOLS}" "Diagnostics/src/" ".cc" DIAG_SRC)
# addPrefixAndSuffix("${DIAG_BASE_CLASSES}" "Diagnostics/" ".h" DIAG_BASE_HEADERS)
# addPrefixAndSuffix("${DIAG_TOOLS}" "Diagnostics/" ".h" DIAG_HEADERS)
# set(DIAG_HEADERS ${DIAG_HEADERS} ${DIAG_BASE_HEADERS} Diagnostics/mtf_diag.h PARENT_SCOPE)
set(MTF_INCLUDE_DIRS ${MTF_INCLUDE_DIRS} Diagnostics/include)
if (WIN32)
	add_library (mtf_diag STATIC ${DIAG_SRC})
else()
	add_library (mtf_diag SHARED ${DIAG_SRC})
endif()
target_compile_definitions(mtf_diag PUBLIC ${MTF_DEFINITIONS})
target_compile_options(mtf_diag PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
target_link_libraries(mtf_diag ${MTF_LIBS})
target_include_directories(mtf_diag PUBLIC ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
if(WIN32)
	install(TARGETS mtf_diag ARCHIVE DESTINATION ${MTF_INSTALL_DIR}/lib COMPONENT diag_lib)
else()
	install(TARGETS mtf_diag LIBRARY DESTINATION ${MTF_INSTALL_DIR}/lib COMPONENT diag_lib)
endif()  
add_custom_target(diag_lib DEPENDS mtf_diag)
if(NOT WIN32)
	add_custom_target(install_diag_lib
	  ${CMAKE_COMMAND}
	  -D "CMAKE_INSTALL_COMPONENT=diag_lib"
	  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
	  DEPENDS mtf_diag
	  )
endif()
#add_custom_target(mtft DEPENDS install_lib install_diag_lib install_header install_diag_exe)
