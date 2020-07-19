option(WITH_PY "Enable compilation of the Python interface. This will be automatically disabled if Python is not found but can be manually disabled using this option if Python is found but some undocumented conflicts are causing build errors" ON)
option(WITH_MEX "Enable compilation of the Matlab interface. This will be automatically disabled if Matlab is not found. However, unresolved issues in Matlab cmake module can cause build errors even if a valid Matlab installation is found. This option can be turned off to avoid such issues." ON)

if (WIN32)
	set(MTF_PY_INSTALL_DIR_DEFAULT C:/Python27/Lib/site-packages)
	set(MTF_MEX_INSTALL_DIR_DEFAULT C:/Python27/Lib/site-packages)
	set(Matlab_ROOT_DIR_DEFAULT I:/MATLAB/R2014a/)
else()
	set(MTF_PY_INSTALL_DIR_DEFAULT /usr/local/lib/python2.7/dist-packages)
	set(MTF_MEX_INSTALL_DIR_DEFAULT /usr/local/lib/python2.7/dist-packages)
	set(Matlab_ROOT_DIR_DEFAULT /usr/local/MATLAB/MATLAB_Production_Server/R2013a/)
endif()
set(MTF_PY_INSTALL_DIR ${MTF_PY_INSTALL_DIR_DEFAULT} CACHE PATH "Directory to install the Python interface module") 
set(PY_VER 2.7 CACHE STRING "Python version for which to build")
set(Matlab_ROOT_DIR ${Matlab_ROOT_DIR_DEFAULT} CACHE PATH "MATLAB root directory") 
set(MTF_MEX_INSTALL_DIR "" CACHE PATH "Directory to install the Matlab interface module") 

# set(WARNING_FLAGS -Wfatal-errors -Wno-write-strings -Wno-unused-result)
# set(MTF_COMPILETIME_FLAGS -std=c++11)
# set(MTF_TOOLS inputCV inputBase objUtils PreProc)
# addPrefixAndSuffix("${MTF_TOOLS}" "Tools/" ".h" MTF_TOOLS_HEADERS)
# message(STATUS "MTF_TOOLS_HEADERS: ${MTF_TOOLS_HEADERS}")

# set(MTF_INCLUDE_DIRS ${MTF_INCLUDE_DIRS} PARENT_SCOPE)

# find_package(Boost REQUIRED COMPONENTS filesystem system)
# message(STATUS "Boost_LIBRARIES:")
# message(STATUS "Examples: MTF_RUNTIME_FLAGS: ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS}")
set(MTF_EXEC_INSTALL_DIR ${MTF_INSTALL_DIR}/bin)
set(EX_TARGET_NAMES runMTF createMosaic generateSyntheticSeq trackUAVTrajectory extractPatch diagnoseMTF) 
set(EX_INSTALL_TARGET_NAMES install_exe install_mos install_syn install_uav install_patch install_diag install_diag_lib) 
set(EX_COMBINED_TARGET_NAMES mtfe mtfm mtfs mtfu mtfpa mtft) 

add_executable(runMTF Examples/cpp/runMTF.cc)
target_compile_definitions(runMTF PUBLIC ${MTF_DEFINITIONS})
target_compile_options(runMTF PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
target_include_directories(runMTF PUBLIC  ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
target_link_libraries(runMTF mtf ${MTF_LIBS})
install(TARGETS runMTF RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT exe)
add_custom_target(exe DEPENDS runMTF)
# if(NOT WIN32)
	# add_custom_target(install_exe
	  # ${CMAKE_COMMAND}
	  # -D "CMAKE_INSTALL_COMPONENT=exe"
	  # -P "${MTF_BINARY_DIR}/cmake_install.cmake"
	   # DEPENDS runMTF
	  # )
	# add_custom_target(mtfe DEPENDS runMTF install_exe)
# endif()
add_executable(SimpleTrackingDemo Examples/cpp/SimpleTrackingDemo.cc)
target_compile_definitions(SimpleTrackingDemo PUBLIC ${MTF_DEFINITIONS})
target_compile_options(SimpleTrackingDemo PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
target_include_directories(SimpleTrackingDemo PUBLIC ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
target_link_libraries(SimpleTrackingDemo mtf ${MTF_LIBS})
install(TARGETS SimpleTrackingDemo RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT simple_demo)
add_custom_target(simple_demo DEPENDS SimpleTrackingDemo)

add_executable(CompositeTrackingDemo Examples/cpp/CompositeTrackingDemo.cc)
target_compile_definitions(CompositeTrackingDemo PUBLIC ${MTF_DEFINITIONS})
target_compile_options(CompositeTrackingDemo PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
target_include_directories(CompositeTrackingDemo PUBLIC ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
target_link_libraries(CompositeTrackingDemo mtf ${MTF_LIBS})
install(TARGETS CompositeTrackingDemo RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT composite_demo)
add_custom_target(composite_demo DEPENDS CompositeTrackingDemo)


add_executable(trackUAVTrajectory Examples/cpp/trackUAVTrajectory.cc)
target_compile_definitions(trackUAVTrajectory PUBLIC ${MTF_DEFINITIONS})
target_compile_options(trackUAVTrajectory PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
target_include_directories(trackUAVTrajectory PUBLIC  ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
target_link_libraries(trackUAVTrajectory mtf ${MTF_LIBS})
install(TARGETS trackUAVTrajectory RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT uav)
add_custom_target(uav DEPENDS trackUAVTrajectory)
# if(NOT WIN32)
	# add_custom_target(install_uav
	  # ${CMAKE_COMMAND}
	  # -D "CMAKE_INSTALL_COMPONENT=uav"
	  # -P "${MTF_BINARY_DIR}/cmake_install.cmake"
	   # DEPENDS trackUAVTrajectory
	  # )
	# add_custom_target(mtfu DEPENDS trackUAVTrajectory install_uav)
# endif()

if(FEAT_ENABLED)	
	add_executable(trackMarkers Examples/cpp/trackMarkers.cc)
	target_compile_definitions(trackMarkers PUBLIC ${MTF_DEFINITIONS})
	target_compile_options(trackMarkers PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
	target_include_directories(trackMarkers PUBLIC  ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
	target_link_libraries(trackMarkers mtf ${MTF_LIBS})
	install(TARGETS trackMarkers RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT qr)
	add_custom_target(qr DEPENDS trackMarkers)
	# if(NOT WIN32)
		# add_custom_target(install_qr
		  # ${CMAKE_COMMAND}
		  # -D "CMAKE_INSTALL_COMPONENT=qr"
		  # -P "${MTF_BINARY_DIR}/cmake_install.cmake"
		   # DEPENDS trackMarkers
		  # )
		# add_custom_target(mtfq DEPENDS trackMarkers install_qr)
		# set(EX_INSTALL_TARGET_NAMES ${EX_INSTALL_TARGET_NAMES} install_qr) 
		# set(EX_COMBINED_TARGET_NAMES ${EX_COMBINED_TARGET_NAMES} mtfq) 
	# endif()
	set(EX_TARGET_NAMES ${EX_TARGET_NAMES} trackMarkers) 	
endif()

add_executable(extractPatch Examples/cpp/extractPatch.cc)
target_compile_definitions(extractPatch PUBLIC ${MTF_DEFINITIONS})
target_compile_options(extractPatch PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
target_include_directories(extractPatch PUBLIC  ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
target_link_libraries(extractPatch mtf ${MTF_LIBS})
install(TARGETS extractPatch RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT patch)
add_custom_target(patch DEPENDS extractPatch)
# if(NOT WIN32)
	# add_custom_target(install_patch
	  # ${CMAKE_COMMAND}
	  # -D "CMAKE_INSTALL_COMPONENT=patch"
	  # -P "${MTF_BINARY_DIR}/cmake_install.cmake"
	   # DEPENDS extractPatch
	  # )
	# add_custom_target(mtfpa DEPENDS extractPatch install_patch)
# endif()

add_executable(generateSyntheticSeq Examples/cpp/generateSyntheticSeq.cc)
target_compile_definitions(generateSyntheticSeq PUBLIC ${MTF_DEFINITIONS})
target_compile_options(generateSyntheticSeq PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
target_include_directories(generateSyntheticSeq PUBLIC  ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
target_link_libraries(generateSyntheticSeq mtf ${MTF_LIBS})
install(TARGETS generateSyntheticSeq RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT syn)
add_custom_target(syn DEPENDS generateSyntheticSeq)
# if(NOT WIN32)
	# add_custom_target(install_syn
	  # ${CMAKE_COMMAND}
	  # -D "CMAKE_INSTALL_COMPONENT=syn"
	  # -P "${MTF_BINARY_DIR}/cmake_install.cmake"
	   # DEPENDS generateSyntheticSeq
	  # )
	# add_custom_target(mtfs DEPENDS generateSyntheticSeq install_syn)
# endif()

add_executable(createMosaic Examples/cpp/createMosaic.cc)
target_compile_definitions(createMosaic PUBLIC ${MTF_DEFINITIONS})
target_compile_options(createMosaic PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
target_include_directories(createMosaic PUBLIC  ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
target_link_libraries(createMosaic mtf ${MTF_LIBS})
install(TARGETS createMosaic RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT mos)
add_custom_target(mos DEPENDS createMosaic)
# if(NOT WIN32)
	# add_custom_target(install_mos
	  # ${CMAKE_COMMAND}
	  # -D "CMAKE_INSTALL_COMPONENT=mos"
	  # -P "${MTF_BINARY_DIR}/cmake_install.cmake"
	   # DEPENDS createMosaic
	  # )
	# add_custom_target(mtfm DEPENDS createMosaic install_mos)
# endif()

add_executable(registerSeq Examples/cpp/registerSeq.cc)
target_compile_definitions(registerSeq PUBLIC ${MTF_DEFINITIONS})
target_compile_options(registerSeq PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
target_include_directories(registerSeq PUBLIC  ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
target_link_libraries(registerSeq mtf ${MTF_LIBS})
install(TARGETS registerSeq RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT reg)
add_custom_target(reg DEPENDS registerSeq)
# if(NOT WIN32)
	# add_custom_target(install_reg
	  # ${CMAKE_COMMAND}
	  # -D "CMAKE_INSTALL_COMPONENT=reg"
	  # -P "${MTF_BINARY_DIR}/cmake_install.cmake"
	   # DEPENDS registerSeq
	  # )
	# add_custom_target(mtfr DEPENDS registerSeq install_reg)
# endif()

if(WITH_PY)
	find_package(PythonLibs ${PY_VER})
	find_package(NumPy)
	if(PYTHONLIBS_FOUND AND PYTHON_NUMPY_FOUND)
		execute_process (COMMAND ${PYTHON_EXECUTABLE} -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())" OUTPUT_VARIABLE MTF_PY_INSTALL_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
		message(STATUS "MTF_PY_INSTALL_DIR: ${MTF_PY_INSTALL_DIR}")

		# if(PYTHONLIBS_VERSION_STRING VERSION_LESS 3.0.0)
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
			# if(NOT WIN32)
				# add_custom_target(install_py
				  # ${CMAKE_COMMAND}
				  # -D "CMAKE_INSTALL_COMPONENT=py"
				  # -P "${MTF_BINARY_DIR}/cmake_install.cmake"
				   # DEPENDS pyMTF
				  # )
				  # add_custom_target(mtfp DEPENDS pyMTF install_py)
				  # set(EX_INSTALL_TARGET_NAMES ${EX_INSTALL_TARGET_NAMES} install_py)			  
				  # set(EX_COMBINED_TARGET_NAMES ${EX_COMBINED_TARGET_NAMES} mtfp)			  
			# endif()
			set(EX_TARGET_NAMES ${EX_TARGET_NAMES} pyMTF)

			
			find_package(Boost REQUIRED COMPONENTS thread)
			add_library(pyMTF2 MODULE Examples/cpp/pyMTF2.cc)
			set_target_properties(pyMTF2 PROPERTIES PREFIX "")
			if(WIN32)
				set_target_properties(pyMTF2 PROPERTIES SUFFIX ".pyd")
			endif()
			target_compile_definitions(pyMTF2 PUBLIC ${MTF_DEFINITIONS})
			target_compile_options(pyMTF2 PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
			target_include_directories(pyMTF2 PUBLIC ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} ${PYTHON_NUMPY_INCLUDE_DIR})
			message(STATUS "PYTHON_INCLUDE_DIRS: ${PYTHON_INCLUDE_DIRS}")
			message(STATUS "PYTHON_LIBRARIES: ${PYTHON_LIBRARIES}")		
			target_link_libraries(pyMTF2 mtf ${MTF_LIBS} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} ${PYTHON_LIBS})	
			install(TARGETS pyMTF2 LIBRARY DESTINATION ${MTF_PY_INSTALL_DIR} COMPONENT py2)
			add_custom_target(py2 DEPENDS pyMTF2)
			# install(CODE "execute_process(COMMAND python setup.py install -f --prefix=${MTF_PY_INSTALL_DIR} WORKING_DIRECTORY python)")
			install(DIRECTORY ${MTF_INCLUDE_DIRS}
				DESTINATION ${MTF_HEADER_INSTALL_DIR}
				COMPONENT header
				)
			# if(NOT WIN32)
				# add_custom_target(install_py2
				  # ${CMAKE_COMMAND}
				  # -D "CMAKE_INSTALL_COMPONENT=py2"
				  # -P "${MTF_BINARY_DIR}/cmake_install.cmake"
				   # DEPENDS pyMTF2
				  # )
				  # add_custom_target(mtfp2 DEPENDS pyMTF2 install_py2)
				  # set(EX_INSTALL_TARGET_NAMES ${EX_INSTALL_TARGET_NAMES} install_py2)			  
				  # set(EX_COMBINED_TARGET_NAMES ${EX_COMBINED_TARGET_NAMES} mtfp2)			  
			# endif()
			set(EX_TARGET_NAMES ${EX_TARGET_NAMES} pyMTF2)				
		# else()
			# message(STATUS "Incompatible version of Python library found so pyMTF is disabled: " ${PYTHONLIBS_VERSION_STRING})
		# endif()		
	else()
		message(STATUS "Python and/or Numpy not found so pyMTF is disabled")
	endif()
else(WITH_PY)
	message(STATUS "Python interface (pyMTF) is disabled")
endif(WITH_PY)


if(WITH_MEX)
	find_package(Matlab COMPONENTS MEX_COMPILER MX_LIBRARY)
	if(Matlab_FOUND)
		if ("${MTF_MEX_INSTALL_DIR}" STREQUAL "")
			set(MTF_MEX_INSTALL_DIR ${Matlab_ROOT_DIR}/toolbox/local) 
		endif()
		message(STATUS "Matlab_ROOT_DIR: ${Matlab_ROOT_DIR}")
		message(STATUS "Matlab_MEX_LIBRARY: ${Matlab_MEX_LIBRARY}")
		message(STATUS "Matlab_LIBRARIES: ${Matlab_LIBRARIES}")
		message(STATUS "MTF_MEX_INSTALL_DIR: ${MTF_MEX_INSTALL_DIR}")
		matlab_add_mex(
			NAME mexMTF
			SRC Examples/cpp/mexMTF.cc
			LINK_TO mtf ${MTF_LIBS} ${Matlab_LIBRARIES}
		)
		install(TARGETS mexMTF 
		RUNTIME DESTINATION ${MTF_MEX_INSTALL_DIR} 
		LIBRARY DESTINATION ${MTF_MEX_INSTALL_DIR}
		COMPONENT mex)
		add_custom_target(mex DEPENDS mexMTF)
		# if(NOT WIN32)
			# add_custom_target(install_mex
			  # ${CMAKE_COMMAND}
			  # -D "CMAKE_INSTALL_COMPONENT=mex"
			  # -P "${MTF_BINARY_DIR}/cmake_install.cmake"
			   # DEPENDS mexMTF
			  # )
			# add_custom_target(mtfx DEPENDS mexMTF install_mex)			
			# set(EX_INSTALL_TARGET_NAMES ${EX_INSTALL_TARGET_NAMES} install_mex)	
			# set(EX_COMBINED_TARGET_NAMES ${EX_COMBINED_TARGET_NAMES} mtfx)			  
		# endif()
		set(EX_TARGET_NAMES ${EX_TARGET_NAMES} mexMTF)
		
		find_package(Boost REQUIRED COMPONENTS thread)
		if(Boost_FOUND)
			matlab_add_mex(
				NAME mexMTF2
				SRC Examples/cpp/mexMTF2.cc
				LINK_TO mtf ${MTF_LIBS} ${Matlab_LIBRARIES} ${Boost_LIBRARIES}
			)
			install(TARGETS mexMTF2 
			RUNTIME DESTINATION ${MTF_MEX_INSTALL_DIR} 
			LIBRARY DESTINATION ${MTF_MEX_INSTALL_DIR}
			COMPONENT mex2)
			add_custom_target(mex2 DEPENDS mexMTF2)
			# if(NOT WIN32)
				# add_custom_target(install_mex2
				  # ${CMAKE_COMMAND}
				  # -D "CMAKE_INSTALL_COMPONENT=mex2"
				  # -P "${MTF_BINARY_DIR}/cmake_install.cmake"
				   # DEPENDS mexMTF2
				  # )
				# add_custom_target(mtfx2 DEPENDS mexMTF2 install_mex2)			
				# set(EX_INSTALL_TARGET_NAMES ${EX_INSTALL_TARGET_NAMES} install_mex2)	
				# set(EX_COMBINED_TARGET_NAMES ${EX_COMBINED_TARGET_NAMES} mtfx2)			  
			# endif()
			set(EX_TARGET_NAMES ${EX_TARGET_NAMES} mexMTF2)
		else(Boost_FOUND)
			message(STATUS "Boost thread module not found")	
		endif(Boost_FOUND)	
	else()
		message(STATUS "Matlab not found so mexMTF is disabled")
		if(NOT WIN32)
			message("\n\tIf Matlab is installed but not detected, mexMTF can be compiled by running the command in 'mtf_mex_cmd.txt' at the MATLAB prompt after:\n\t * removing all semi colons\n\t * replacing all occurences of the type '-l<full path to library>' with '<full path to library>'\n\t * replacing all occurences of the type '-l-L<path to library folder>' with '-L<path to library folder>'\n")
			addPrefixAndSuffix("${MTF_LIBS}" "-l"  " " MEX_MTF_LIBS)
			addPrefixAndSuffix("${MTF_DEFINITIONS}" "${DEFINITION_SWITCH}" " " MEX_MTF_DEFINITIONS)
			addPrefixAndSuffix("${MTF_INCLUDE_DIRS}" "${INCLUDE_SWITCH}\"" "\" " MEX_MTF_INCLUDE_DIRS)
			addPrefixAndSuffix("${MTF_EXT_INCLUDE_DIRS}" "${INCLUDE_SWITCH}\"" "\" " MEX_MTF_EXT_INCLUDE_DIRS)
			addSuffix("${MTF_RUNTIME_FLAGS}" " " MEX_MTF_RUNTIME_FLAGS)
			addSuffix("${MTF_COMPILETIME_FLAGS}" " " MEX_MTF_COMPILETIME_FLAGS)
			set(MEX_CFLAGS "-fPIC ${MEX_MTF_INCLUDE_DIRS} ${MEX_MTF_EXT_INCLUDE_DIRS} ${MEX_MTF_RUNTIME_FLAGS} ${MEX_MTF_COMPILETIME_FLAGS} ${MEX_MTF_DEFINITIONS}")
			set(MEX_COMMAND "mex -v CFLAGS='${MEX_CFLAGS}' CXXFLAGS='${MEX_CFLAGS}' -lmtf ${MEX_MTF_LIBS} Examples/cpp/mexMTF.cc")
			file(WRITE ${CMAKE_BINARY_DIR}/mtf_mex_cmd.txt "${MEX_COMMAND}")
		endif()		
	endif()
else(WITH_MEX)
	message(STATUS "Matlab interface (mexMTF) is disabled")
endif(WITH_MEX)

add_executable(diagnoseMTF Examples/cpp/diagnoseMTF.cc)
target_compile_definitions(diagnoseMTF PUBLIC ${MTF_DEFINITIONS})
target_compile_options(diagnoseMTF PUBLIC ${MTF_RUNTIME_FLAGS} ${MTF_COMPILETIME_FLAGS})
target_include_directories(diagnoseMTF PUBLIC ${MTF_INCLUDE_DIRS} ${MTF_EXT_INCLUDE_DIRS})
target_link_libraries(diagnoseMTF mtf_diag mtf ${MTF_LIBS})
install(TARGETS diagnoseMTF RUNTIME DESTINATION ${MTF_EXEC_INSTALL_DIR} COMPONENT diag)
add_custom_target(diag DEPENDS diagnoseMTF)
# if(NOT WIN32)
	# add_custom_target(install_diag
	  # ${CMAKE_COMMAND}
	  # -D "CMAKE_INSTALL_COMPONENT=diag"
	  # -P "${MTF_BINARY_DIR}/cmake_install.cmake"
	   # DEPENDS diagnoseMTF
	  # )
	# add_custom_target(mtft DEPENDS diagnoseMTF mtf_diag install_diag install_diag_lib)
# endif() 

# add_custom_target(all DEPENDS ${EX_TARGET_NAMES})

# if(NOT WIN32)
	# add_custom_target(install_all DEPENDS ${EX_INSTALL_TARGET_NAMES})
	# add_custom_target(mtfall DEPENDS ${EX_COMBINED_TARGET_NAMES})
# endif() 

