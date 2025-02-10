cmake_minimum_required(VERSION 3.5)
project(DoublePendulum)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_STANDARD 17)
set(${PROJECT_NAME}_VERSION 0.1)

cmake_policy(SET CMP0079 NEW)

if (CMAKE_BUILD_TYPE MATCHES Release)
	set(BUILD_FLAGS "-O3")
  add_definitions(-DRELEASE_BUILD)
elseif(CMAKE_BUILD_TYPE MATCHES Debug)
	set(BUILD_FLAGS "-O0 -fno-inline -g")
  add_definitions(-DDEBUG_BUILD)
endif()

if (UNIX)
	if (NOT APPLE)
		# message(STATUS "Building with -fPIC")
    list(APPEND BUILD_FLAGS "-fPIC")
	endif ()
endif (UNIX)


add_definitions(${BUILD_FLAGS})

find_package(Eigen3 CONFIG REQUIRED)
find_package(YAMLCPP NAMES Yaml-cpp YamlCpp yaml-cpp)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${YAMLCPP_INCLUDE_DIR} ${YAML_CPP_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})

add_library(External INTERFACE)
set(${PROJECT_NAME}_LIBRARIES "External")
if(TARGET yaml-cpp )
	target_link_libraries(External INTERFACE yaml-cpp)
elseif(TARGET yaml-cpp::yaml-cpp )
	target_link_libraries(External INTERFACE yaml-cpp::yaml-cpp)
endif()


# include_directories("${PROJECT_SOURCE_DIR}/src")
link_directories("${PROJECT_SOURCE_DIR}/lib" ${Boost_LIBRARY_DIRS})

include_directories(".")
# include_directories(python) TODO
# include_directories(simulator/src)
# include_directories(utils/src)

add_subdirectory(controllers/ilqr)
add_subdirectory(model)
add_subdirectory(ML4KP_Interface)
# add_subdirectory(python)
add_subdirectory(simulator)
add_subdirectory(utils)


add_library(${PROJECT_NAME} INTERFACE )
foreach(LIBRARY IN ITEMS ${${PROJECT_NAME}_LIBRARIES} )
	add_dependencies(${PROJECT_NAME} ${LIBRARY})
	target_link_libraries(${PROJECT_NAME} INTERFACE ${LIBRARY})
endforeach()


set(EXECUTABLES 
	"simulator/src/test_sim.cpp"
	"controllers/ilqr/src/main_ilqr.cpp"
	"controllers/ilqr/src/main_ilqr_mpc.cpp")

foreach(f ${EXECUTABLES})
	GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
	add_executable(${F_EXEC} ${f})
	target_link_libraries(${F_EXEC} ${PROJECT_NAME}	)
endforeach()


# Install the library
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}/")
message(STATUS "Installing ${PROJECT_NAME} in: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "CMAKE_INSTALL_LIBDIR: ${CMAKE_INSTALL_LIBDIR}")

foreach(HEADER IN ITEMS ${${PROJECT_NAME}_HEADERS})
	get_filename_component(H_DIR ${HEADER} DIRECTORY )
	install(FILES ${HEADER} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${H_DIR})
endforeach()

# install(TARGETS ${${PROJECT_NAME}_LIBRARIES} EXPORT ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})
foreach(LIBRARY IN ITEMS ${${PROJECT_NAME}_LIBRARIES})
	install(TARGETS ${LIBRARY}
        EXPORT ${LIBRARY}Targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
	)
	install(EXPORT ${LIBRARY}Targets
        	FILE ${LIBRARY}Targets.cmake
	        NAMESPACE ${PROJECT_NAME}::
	        DESTINATION ${CMAKE_INSTALL_DESTINATION}
	)
endforeach()

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(EXPORT ${PROJECT_NAME}Targets
        FILE ${PROJECT_NAME}Targets.cmake
        NAMESPACE ${PROJECT_NAME}::
        EXPORT_LINK_INTERFACE_LIBRARIES
        DESTINATION ${CMAKE_INSTALL_DESTINATION}
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/DoublePendulumConfigVersion.cmake"
  VERSION ${${PROJECT_NAME}_VERSION}
  COMPATIBILITY AnyNewerVersion
)
configure_package_config_file(cmake/DoublePendulumConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/DoublePendulumConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_DESTINATION}
  PATH_VARS
  	CMAKE_INSTALL_INCLUDEDIR
  	CMAKE_INSTALL_LIBDIR 
  	${PROJECT_NAME}_LIBRARIES
  )

install(FILES 
			"${CMAKE_CURRENT_BINARY_DIR}/cmake/DoublePendulumConfig.cmake"
  			"${CMAKE_CURRENT_BINARY_DIR}/DoublePendulumConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_DESTINATION} )
