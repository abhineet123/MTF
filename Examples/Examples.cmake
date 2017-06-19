if (WIN32)
	set(MTF_EXEC_INSTALL_DIR_DEFAULT C:/MTF/bin)
	set(MTF_PY_INSTALL_DIR_DEFAULT C:/Python27/Lib/site-packages)
	set(Matlab_ROOT_DIR_DEFAULT E:/Program\ Files/MATLAB/R2013a)
else()
	set(MTF_EXEC_INSTALL_DIR_DEFAULT /usr/local/bin)
	set(MTF_PY_INSTALL_DIR_DEFAULT /usr/local/lib/python2.7/dist-packages)
	set(Matlab_ROOT_DIR_DEFAULT /usr/local/MATLAB/MATLAB_Production_Server/R2013a/)
endif()
set(MTF_EXEC_INSTALL_DIR ${MTF_EXEC_INSTALL_DIR_DEFAULT} CACHE PATH "Directory to install the executable")
set(MTF_PY_INSTALL_DIR ${MTF_PY_INSTALL_DIR_DEFAULT} CACHE PATH "Directory to install the Python interface module (normally the CModules sub directory of PTF)") 
set(Matlab_ROOT_DIR ${Matlab_ROOT_DIR_DEFAULT} CACHE PATH "MATLAB root directory") 

# set(WARNING_FLAGS -Wfatal-errors -Wno-write-strings -Wno-unused-result)
# set(MTF_COMPILETIME_FLAGS -std=c++11)
# set(MTF_TOOLS inputCV inputBase objUtils PreProc)
# addPrefixAndSuffix("${MTF_TOOLS}" "Tools/" ".h" MTF_TOOLS_HEADERS)
# message(STATUS "MTF_TOOLS_HEADERS: ${MTF_TOOLS_HEADERS}")

# set(MTF_INCLUDE_DIRS ${MTF_INCLUDE_DIRS} PARENT_SCOPE)

# find_package(Boost REQUIRED COMPONENTS filesystem system)
# message(STATUS "Boost_LIBRARIES:")
# message(STATUS "Examples: MTF_RUNTIME_FLAGS: ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS}")

add_executable(runMTF Examples/cpp/runMTF.cc)
target_compile_definitions(runMTF PUBLIC ${MTF_DEFINITIONS})
target_compile_options(runMTF PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
target_include_directories(runMTF PUBLIC  ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
target_link_libraries(runMTF mtf ${MTF_LIBS})
install(TARGETS runMTF RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT exe)
add_custom_target(exe DEPENDS runMTF)
if(NOT WIN32)
	add_custom_target(install_exe
	  ${CMAKE_COMMAND}
	  -D "CMAKE_INSTALL_COMPONENT=exe"
	  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
	   DEPENDS runMTF
	  )
	add_custom_target(mtfe DEPENDS runMTF install_exe)
endif()

add_executable(trackUAVTrajectory Examples/cpp/trackUAVTrajectory.cc)
target_compile_definitions(trackUAVTrajectory PUBLIC ${MTF_DEFINITIONS})
target_compile_options(trackUAVTrajectory PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
target_include_directories(trackUAVTrajectory PUBLIC  ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
target_link_libraries(trackUAVTrajectory mtf ${MTF_LIBS})
install(TARGETS trackUAVTrajectory RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT uav)
add_custom_target(uav DEPENDS trackUAVTrajectory)
if(NOT WIN32)
	add_custom_target(install_uav
	  ${CMAKE_COMMAND}
	  -D "CMAKE_INSTALL_COMPONENT=uav"
	  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
	   DEPENDS trackUAVTrajectory
	  )
	add_custom_target(mtfu DEPENDS trackUAVTrajectory install_uav)
endif()

add_executable(trackMarkers Examples/cpp/trackMarkers.cc)
target_compile_definitions(trackMarkers PUBLIC ${MTF_DEFINITIONS})
target_compile_options(trackMarkers PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
target_include_directories(trackMarkers PUBLIC  ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
target_link_libraries(trackMarkers mtf ${MTF_LIBS})
install(TARGETS trackMarkers RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT qr)
add_custom_target(qr DEPENDS trackMarkers)
if(NOT WIN32)
	add_custom_target(install_qr
	  ${CMAKE_COMMAND}
	  -D "CMAKE_INSTALL_COMPONENT=qr"
	  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
	   DEPENDS trackMarkers
	  )
	add_custom_target(mtfq DEPENDS trackMarkers install_qr)
endif()

add_executable(extractPatch Examples/cpp/extractPatch.cc)
target_compile_definitions(extractPatch PUBLIC ${MTF_DEFINITIONS})
target_compile_options(extractPatch PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
target_include_directories(extractPatch PUBLIC  ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
target_link_libraries(extractPatch mtf ${MTF_LIBS})
install(TARGETS extractPatch RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT patch)
add_custom_target(patch DEPENDS extractPatch)
if(NOT WIN32)
	add_custom_target(install_patch
	  ${CMAKE_COMMAND}
	  -D "CMAKE_INSTALL_COMPONENT=patch"
	  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
	   DEPENDS extractPatch
	  )
	add_custom_target(mtfpa DEPENDS extractPatch install_patch)
endif()

add_executable(generateSyntheticSeq Examples/cpp/generateSyntheticSeq.cc)
target_compile_definitions(generateSyntheticSeq PUBLIC ${MTF_DEFINITIONS})
target_compile_options(generateSyntheticSeq PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
target_include_directories(generateSyntheticSeq PUBLIC  ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
target_link_libraries(generateSyntheticSeq mtf ${MTF_LIBS})
install(TARGETS generateSyntheticSeq RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT syn)
add_custom_target(syn DEPENDS generateSyntheticSeq)
if(NOT WIN32)
	add_custom_target(install_syn
	  ${CMAKE_COMMAND}
	  -D "CMAKE_INSTALL_COMPONENT=syn"
	  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
	   DEPENDS generateSyntheticSeq
	  )
	add_custom_target(mtfs DEPENDS generateSyntheticSeq install_syn)
endif()

add_executable(createMosaic Examples/cpp/createMosaic.cc)
target_compile_definitions(createMosaic PUBLIC ${MTF_DEFINITIONS})
target_compile_options(createMosaic PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
target_include_directories(createMosaic PUBLIC  ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
target_link_libraries(createMosaic mtf ${MTF_LIBS})
install(TARGETS createMosaic RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT mos)
add_custom_target(mos DEPENDS createMosaic)
if(NOT WIN32)
	add_custom_target(install_mos
	  ${CMAKE_COMMAND}
	  -D "CMAKE_INSTALL_COMPONENT=mos"
	  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
	   DEPENDS createMosaic
	  )
	add_custom_target(mtfm DEPENDS createMosaic install_mos)
endif()

find_package(PythonLibs 2.7)
find_package(NumPy)
if(PYTHONLIBS_FOUND AND PYTHON_NUMPY_FOUND)
	if(PYTHONLIBS_VERSION_STRING VERSION_LESS 3.0.0)
		add_library(pyMTF MODULE Examples/cpp/pyMTF.cc)
		set_target_properties(pyMTF PROPERTIES PREFIX "")
		if(WIN32)
			set_target_properties(pyMTF PROPERTIES SUFFIX ".pyd")
		endif()
		target_compile_definitions(pyMTF PUBLIC ${MTF_DEFINITIONS})
		target_compile_options(pyMTF PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
		target_include_directories(pyMTF PUBLIC ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} ${PYTHON_NUMPY_INCLUDE_DIR})
		message(STATUS "PYTHON_INCLUDE_DIRS: ${PYTHON_INCLUDE_DIRS}")
		message(STATUS "PYTHON_LIBRARIES: ${PYTHON_LIBRARIES}")		
		target_link_libraries(pyMTF mtf ${MTF_LIBS} ${PYTHON_LIBRARIES} ${PYTHON_LIBS})	
		install(TARGETS pyMTF LIBRARY DESTINATION ${MTF_PY_INSTALL_DIR} COMPONENT py)
		add_custom_target(py DEPENDS pyMTF)
		if(NOT WIN32)
			add_custom_target(install_py
			  ${CMAKE_COMMAND}
			  -D "CMAKE_INSTALL_COMPONENT=py"
			  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
			   DEPENDS pyMTF
			  )
			  add_custom_target(mtfp DEPENDS pyMTF install_py)
		  endif()	
	  else()
		message(STATUS "Incompatible version of Python library found so pyMTF is disabled: " ${PYTHONLIBS_VERSION_STRING})
	endif()		
else()
	message(STATUS "Python and/or Numpy not found so pyMTF is disabled")
endif()

find_package(Matlab 8.1 COMPONENTS MEX_COMPILER)
if(Matlab_FOUND)
	matlab_add_mex(
		NAME mexMTF
		SRC Examples/cpp/mexMTF.cc
		LINK_TO mtf ${MTF_LIBS}
	)
else()
	message(STATUS "Matlab not found so mexMTF installation is disabled")
	message(STATUS "If Matlab is installed but not detected, mexMTF can be compiled by running the following command at the MATLAB prompt with Examples/cpp as the root folder")
	addPrefixAndSuffix("${MTF_LIBS}" "-l"  " " MEX_MTF_LIBS)
	addPrefixAndSuffix("${MTF_DEFINITIONS}" "${DEFINITION_SWITCH}" " " MEX_MTF_DEFINITIONS)
	addPrefixAndSuffix("${MTF_INCLUDE_DIRS}" "${INCLUDE_SWITCH}" " " MEX_MTF_INCLUDE_DIRS)
	addPrefixAndSuffix("${MTF_EXT_INCLUDE_DIRS}" "${INCLUDE_SWITCH}" " " MEX_MTF_EXT_INCLUDE_DIRS)
	addSuffix("${MTF_RUNTIME_FLAGS}" " " MEX_MTF_RUNTIME_FLAGS)
	addSuffix("${MTF_COMPILETIME_FLAGS}" " " MEX_MTF_COMPILETIME_FLAGS)
	set(MEX_CFLAGS "-fPIC ${MEX_MTF_INCLUDE_DIRS} ${MEX_MTF_EXT_INCLUDE_DIRS} ${MEX_MTF_RUNTIME_FLAGS} ${MEX_MTF_COMPILETIME_FLAGS} ${MEX_MTF_DEFINITIONS}")
	set(MEX_COMMAND "mex CFLAGS='${MEX_CFLAGS}' CXXFLAGS='${MEX_CFLAGS}' ${MEX_MTF_LIBS} Examples/cpp/mexMTF.cc")
	message(STATUS ${MEX_COMMAND})

endif()

add_executable(testMTF Examples/cpp/testMTF.cc)
target_compile_definitions(testMTF PUBLIC ${MTF_DEFINITIONS})
target_compile_options(testMTF PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
target_include_directories(testMTF PUBLIC ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
target_link_libraries(testMTF mtf_test mtf ${MTF_LIBS})
install(TARGETS testMTF RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT test)
add_custom_target(test DEPENDS testMTF)
if(NOT WIN32)
	add_custom_target(install_test
	  ${CMAKE_COMMAND}
	  -D "CMAKE_INSTALL_COMPONENT=test"
	  -P "${MTF_BINARY_DIR}/cmake_install.cmake"
	   DEPENDS testMTF
	  )
	add_custom_target(mtft DEPENDS testMTF mtf_test install_test install_test_lib)
endif() 
add_custom_target(all DEPENDS runMTF createMosaic generateSyntheticSeq trackUAVTrajectory trackMarkers extractPatch pyMTF testMTF)
if(NOT WIN32)
	add_custom_target(install_all DEPENDS install_exe install_mos install_syn install_uav install_qr install_py)
	add_custom_target(mtfall DEPENDS mtfe mtfu mtfm mtfs mtfq mtfpa mtft)
endif() 
