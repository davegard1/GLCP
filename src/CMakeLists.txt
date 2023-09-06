#set( LINK_LIBS stopUtil)

file(GLOB SOURCES "*.cpp")

add_library( src ${SOURCES})

#target_link_libraries( integration ${LINK_LIBS})