set(program schedulergepetto)
add_executable(${program}
  mainschedulerproject.cpp
  loadproject.cpp
  schedulerproject.cpp
  logproject.cpp
)

target_link_libraries(${program}
  hrpGepUtil-${OPENHRP_LIBRARY_VERSION}
  hrpGepModel-${OPENHRP_LIBRARY_VERSION})

pkg_config_use_dependency(${program} libxml-2.0)

install(TARGETS ${program}
  DESTINATION example/${PROJECT_NAME}/scheduler
  CONFIGURATIONS Release Debug RelWithDebInfo)

# Installation to have the program launched through roslaunch.
install(TARGETS ${program} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME})