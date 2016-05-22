
set(SOURCES eepromtool.cpp)
add_executable(eepromtool ${SOURCES})
set_target_properties(eepromtool PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(eepromtool soem)

if(WIN32)
   target_link_libraries(eepromtool wpcap.lib Packet.lib Ws2_32.lib Winmm.lib)
elseif(UNIX)
   target_link_libraries(eepromtool pthread rt)
endif()

install(TARGETS eepromtool DESTINATION bin)
