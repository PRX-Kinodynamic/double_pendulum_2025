file(GLOB EXECUTABLES_ML4KP_FILES	*.cpp)

set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/ML4KP )
	
foreach(f ${EXECUTABLES_ML4KP_FILES})
	GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
	add_executable(${F_EXEC} ${f})

	target_link_libraries(${F_EXEC} ${PROJECT_NAME}  	)

	if( nlohmann_json_FOUND )
		target_link_libraries(${F_EXEC}  )
	endif()

endforeach()
