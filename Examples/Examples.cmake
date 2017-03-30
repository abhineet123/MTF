set(MTF_EXEC_INSTALL_DIR /usr/local/bin CACHE PATH "Directory to install the executable")
set(MTF_PY_INSTALL_DIR /usr/local/lib/python2.7/dist-packages CACHE PATH "Directory to install the Python interface module (normally the CModules sub directory of PTF)") 
set(MTF_PY_LIB_NAME pyMTF.so)


# set(WARNING_FLAGS -Wfatal-errors -Wno-write-strings -Wno-unused-result)
# set(CT_FLAGS -std=c++11)
# set(MTF_TOOLS inputCV inputBase cvUtils PreProc)
# addPrefixAndSuffix("${MTF_TOOLS}" "Tools/" ".h" MTF_TOOLS_HEADERS)
# message(STATUS "MTF_TOOLS_HEADERS: ${MTF_TOOLS_HEADERS}")

# set(MTF_INCLUDE_DIRS ${MTF_INCLUDE_DIRS} Examples/include PARENT_SCOPE)

find_package(Boost REQUIRED COMPONENTS filesystem system)
message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")
# message(STATUS "Examples: MTF_RUNTIME_FLAGS: ${MTF_RUNTIME_FLAGS}")

add_executable(runMTF Examples/src/runMTF.cc)
target_compile_options(runMTF PUBLIC ${MTF_RUNTIME_FLAGS})
target_include_directories(runMTF PUBLIC  Examples/include ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(runMTF mtf ${MTF_LIBS} ${Boost_LIBRARIES})
install(TARGETS runMTF RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT exe)
add_custom_target(install_exe
  ${CMAKE_COMMAND}
  -D "CMAKE_INSTALL_COMPONENT=exe"
  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
   DEPENDS runMTF
  )
add_custom_target(exe DEPENDS runMTF)
add_custom_target(mtfe DEPENDS runMTF install_exe)

add_executable(trackUAVTrajectory Examples/src/trackUAVTrajectory.cc)
target_compile_options(trackUAVTrajectory PUBLIC ${MTF_RUNTIME_FLAGS})
target_include_directories(trackUAVTrajectory PUBLIC  Examples/include ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(trackUAVTrajectory mtf ${MTF_LIBS} ${Boost_LIBRARIES})
install(TARGETS trackUAVTrajectory RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT uav)
add_custom_target(install_uav
  ${CMAKE_COMMAND}
  -D "CMAKE_INSTALL_COMPONENT=uav"
  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
   DEPENDS trackUAVTrajectory
  )
add_custom_target(uav DEPENDS trackUAVTrajectory)
add_custom_target(mtfu DEPENDS trackUAVTrajectory install_uav)

add_executable(trackMarkers Examples/src/trackMarkers.cc)
target_compile_options(trackMarkers PUBLIC ${MTF_RUNTIME_FLAGS})
target_include_directories(trackMarkers PUBLIC  Examples/include ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(trackMarkers mtf ${MTF_LIBS} ${Boost_LIBRARIES})
install(TARGETS trackMarkers RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT qr)
add_custom_target(install_qr
  ${CMAKE_COMMAND}
  -D "CMAKE_INSTALL_COMPONENT=qr"
  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
   DEPENDS trackMarkers
  )
add_custom_target(qr DEPENDS trackMarkers)
add_custom_target(mtfq DEPENDS trackMarkers install_qr)

add_executable(extractPatch Examples/src/extractPatch.cc)
target_compile_options(extractPatch PUBLIC ${MTF_RUNTIME_FLAGS})
target_include_directories(extractPatch PUBLIC  Examples/include ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(extractPatch mtf ${MTF_LIBS} ${Boost_LIBRARIES})
install(TARGETS extractPatch RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT patch)
add_custom_target(install_patch
  ${CMAKE_COMMAND}
  -D "CMAKE_INSTALL_COMPONENT=patch"
  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
   DEPENDS extractPatch
  )
add_custom_target(mtfpa DEPENDS extractPatch install_patch)

add_executable(generateSyntheticSeq Examples/src/generateSyntheticSeq.cc)
target_compile_options(generateSyntheticSeq PUBLIC ${MTF_RUNTIME_FLAGS})
target_include_directories(generateSyntheticSeq PUBLIC  Examples/include ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(generateSyntheticSeq mtf ${MTF_LIBS} ${Boost_LIBRARIES})
install(TARGETS generateSyntheticSeq RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT syn)
add_custom_target(install_syn
  ${CMAKE_COMMAND}
  -D "CMAKE_INSTALL_COMPONENT=syn"
  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
   DEPENDS generateSyntheticSeq
  )
add_custom_target(syn DEPENDS generateSyntheticSeq)
add_custom_target(mtfs DEPENDS generateSyntheticSeq install_syn)

add_executable(createMosaic Examples/src/createMosaic.cc)
target_compile_options(createMosaic PUBLIC ${MTF_RUNTIME_FLAGS})
target_include_directories(createMosaic PUBLIC  Examples/include ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(createMosaic mtf ${MTF_LIBS} ${Boost_LIBRARIES})
install(TARGETS createMosaic RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT mos)
add_custom_target(install_mos
  ${CMAKE_COMMAND}
  -D "CMAKE_INSTALL_COMPONENT=mos"
  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
   DEPENDS createMosaic
  )
add_custom_target(mos DEPENDS createMosaic)
add_custom_target(mtfm DEPENDS createMosaic install_mos)

find_package(PythonLibs 2.7)
if(PYTHONLIBS_FOUND)
	if(PYTHONLIBS_VERSION_STRING VERSION_LESS 3.0.0)
		add_library(pyMTF MODULE Examples/src/pyMTF.cc)
		set_target_properties(pyMTF PROPERTIES PREFIX "")
		target_compile_options(pyMTF PUBLIC ${MTF_RUNTIME_FLAGS})
		target_include_directories(pyMTF PUBLIC Examples/include ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
		target_link_libraries(pyMTF mtf ${MTF_LIBS} ${PYTHON_LIBRARIES} ${PYTHON_LIBS} ${Boost_LIBRARIES})	
		install(TARGETS pyMTF LIBRARY DESTINATION ${MTF_PY_INSTALL_DIR} COMPONENT py)
		add_custom_target(py DEPENDS pyMTF)
		add_custom_target(install_py
		  ${CMAKE_COMMAND}
		  -D "CMAKE_INSTALL_COMPONENT=py"
		  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
		   DEPENDS pyMTF
		  )
		  add_custom_target(mtfp DEPENDS pyMTF install_py)
	  else()
		message(STATUS "Incompatible version of Python library found so pyMTF is disabled: " ${PYTHONLIBS_VERSION_STRING})
	endif()		
else(PYTHONLIBS_FOUND)
	message(STATUS "Python library not found so pyMTF is disabled")
endif(PYTHONLIBS_FOUND)

add_executable(testMTF Examples/src/testMTF.cc)
target_compile_options(testMTF PUBLIC ${MTF_RUNTIME_FLAGS})
target_include_directories(testMTF PUBLIC Examples/include ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(testMTF mtf_test mtf ${MTF_LIBS} ${Boost_LIBRARIES})
install(TARGETS testMTF RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT test)
add_custom_target(test DEPENDS testMTF)
add_custom_target(install_test
  ${CMAKE_COMMAND}
  -D "CMAKE_INSTALL_COMPONENT=test"
  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
   DEPENDS testMTF
  )
add_custom_target(mtft DEPENDS testMTF mtf_test install_test install_test_lib)
 
add_custom_target(all DEPENDS runMTF createMosaic generateSyntheticSeq trackUAVTrajectory trackMarkers extractPatch pyMTF testMTF)
add_custom_target(install_all DEPENDS install_exe install_mos install_syn install_uav install_qr install_py)
add_custom_target(mtfall DEPENDS mtfe mtfu mtfm mtfs mtfq mtfpa mtft)
