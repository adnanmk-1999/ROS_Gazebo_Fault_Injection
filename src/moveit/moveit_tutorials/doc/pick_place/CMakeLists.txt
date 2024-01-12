add_executable(pick_place_tutorial src/pick_place_tutorial.cpp)
target_link_libraries(pick_place_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pick_place_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#add_executable(own_pick_place_V3 src/own_pick_place_V3.cpp)
#target_link_libraries(own_pick_place_V3 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#install(TARGETS own_pick_place_V3 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(own_pick_place_V4 src/own_pick_place_V4.cpp)
target_link_libraries(own_pick_place_V4 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS own_pick_place_V4 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(own_pick_place_VJZL src/own_pick_place_VJZL.cpp)
target_link_libraries(own_pick_place_VJZL ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS own_pick_place_VJZL DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

