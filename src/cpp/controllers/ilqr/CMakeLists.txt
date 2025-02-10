
set(SRCS "src/ilqr.cpp;src/ilqr_mpc.cpp")

file(GLOB_RECURSE HEADERES_CONTROLLERS RELATIVE ${CMAKE_SOURCE_DIR} ./*.hpp)
set(${PROJECT_NAME}_HEADERS "${${PROJECT_NAME}_HEADERS};${HEADERES_CONTROLLERS}" PARENT_SCOPE)

add_library(Controllers ${SRCS})

target_include_directories(Controllers PUBLIC
														PUBLIC
														"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
														)

target_link_libraries(Controllers INTERFACE External )
target_link_libraries(Controllers INTERFACE Utilities )

set_target_properties(Controllers PROPERTIES COMPILE_FLAGS "-Wno-deprecated")

set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};Controllers" PARENT_SCOPE)
