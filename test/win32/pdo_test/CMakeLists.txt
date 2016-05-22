
set(SOURCES pdo_test.cpp)
add_executable(pdo_test ${SOURCES})
set_target_properties(pdo_test PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(pdo_test soem)

if(WIN32)
   target_link_libraries(pdo_test wpcap.lib Packet.lib Ws2_32.lib Winmm.lib)
elseif(UNIX)
   target_link_libraries(pdo_test pthread rt)
endif()

if(NOT DEFINED HOST_INSTALL)
   install(TARGETS pdo_test DESTINATION bin)
endif()
