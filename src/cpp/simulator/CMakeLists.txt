file(GLOB_RECURSE HEADERES_SIM RELATIVE ${CMAKE_SOURCE_DIR} ./*.hpp)
file(GLOB_RECURSE SRC_SIM  ./*.cpp)

set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADERES_SIM}" PARENT_SCOPE)

add_library(Simulator ${SRC_SIM})

target_include_directories(Simulator PUBLIC
														PUBLIC
														"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
														)

set_target_properties(Simulator PROPERTIES COMPILE_FLAGS "-Wno-deprecated")



target_link_libraries(Simulator INTERFACE External )

set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};Simulator" PARENT_SCOPE)
