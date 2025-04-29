set( EXAMPLE_FOLDER_PATH ${CMAKE_CURRENT_SOURCE_DIR} )
add_definitions(-DEXAMPLE_FOLDER_PATH="${EXAMPLE_FOLDER_PATH}")

add_executable(MainRedisInterfaceExample MainRedisInterfaceExample.cpp)
add_executable(AuxPandaSimvizOnly AuxPandaSimvizOnly.cpp)

# and link the library against the executable
target_link_libraries(MainRedisInterfaceExample ${SAI-INTERFACES_LIBRARIES})
target_link_libraries(AuxPandaSimvizOnly ${SAI-INTERFACES_LIBRARIES})