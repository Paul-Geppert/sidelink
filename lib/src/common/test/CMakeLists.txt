
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-switch")
add_executable(slsch_pdu_cc slsch_pdu.cc)
target_link_libraries(slsch_pdu_cc srslte_common srslte_phy)
#  srslte_radio srslte_asn1 ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES})

add_test(slsch_mac_pdu_loop slsch_pdu_cc)
