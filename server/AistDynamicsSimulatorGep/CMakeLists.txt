# @author Shin'ichiro Nakaoka

set(program openhrp-aist-dynamics-simulator-gepetto)

set(sources DynamicsSimulator_impl.cpp server.cpp)

add_executable(${program} ${sources})

if(UNIX)
  target_link_libraries(${program}
    hrpGepUtil-${OPENHRP_LIBRARY_VERSION}
    hrpGepModel-${OPENHRP_LIBRARY_VERSION}
    hrpGepCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}
    ${OMNIORB_LIBRARIES} ${Boost_REGEX_LIBRARY} )
elseif(WIN32)
  set_target_properties(${program} PROPERTIES DEBUG_POSTFIX d)
  target_link_libraries(${program}
    optimized hrpGepUtil-${OPENHRP_LIBRARY_VERSION}
    optimized hrpGepModel-${OPENHRP_LIBRARY_VERSION}
    optimized hrpGepCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}
    debug hrpGepUtil-${OPENHRP_LIBRARY_VERSION}d
    debug hrpGepModel-${OPENHRP_LIBRARY_VERSION}d
    debug hrpGepCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}d
    ${OMNIORB_LIBRARIES})  
endif()

if(WIN32)
install(TARGETS ${program} DESTINATION ${PROJECT_BINARY_DIR}/bin CONFIGURATIONS Release)
endif()

install(TARGETS ${program} DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo)
# Installation to have the program launched through roslaunch.
install(TARGETS ${program} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME})
