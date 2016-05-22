
set(SOURCES simple_test.cpp)
add_executable(simple_test ${SOURCES})
set_target_properties(simple_test PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(simple_test soem)

if(WIN32)
   target_link_libraries(simple_test wpcap.lib Packet.lib Ws2_32.lib Winmm.lib)
elseif(UNIX)
   target_link_libraries(simple_test pthread rt)
endif()

if(NOT DEFINED HOST_INSTALL)
   install(TARGETS simple_test DESTINATION bin)
endif()
