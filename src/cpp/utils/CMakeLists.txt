## Utilities library of DirtMP

file(GLOB_RECURSE HEADERES_UTILS RELATIVE ${CMAKE_SOURCE_DIR} ./*.hpp)
file(GLOB_RECURSE SRC_UTILS  ./*.cpp)

set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADERES_UTILS}" PARENT_SCOPE)

add_library(Utilities ${SRC_UTILS})

target_include_directories(Utilities PUBLIC
														PUBLIC
														"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
														)

set_target_properties(Utilities PROPERTIES COMPILE_FLAGS "-Wno-deprecated")



target_link_libraries(Utilities INTERFACE External )

set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};Utilities" PARENT_SCOPE)
