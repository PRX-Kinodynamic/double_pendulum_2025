file(GLOB_RECURSE HEADERES_MODEL RELATIVE ${CMAKE_SOURCE_DIR} ./*.hpp)
file(GLOB_RECURSE SRC_MODEL  ./*.cpp)

set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADERES_MODEL}" PARENT_SCOPE)

add_library(Model ${SRC_MODEL})

target_include_directories(Model PUBLIC
														PUBLIC
														"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
														)

set_target_properties(Model PROPERTIES COMPILE_FLAGS "-Wno-deprecated")



target_link_libraries(Model INTERFACE External )

set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};Model" PARENT_SCOPE)
