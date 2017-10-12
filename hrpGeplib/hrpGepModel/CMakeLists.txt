
configure_file(Config.h.in ${CMAKE_BINARY_DIR}/hrpGeplib/hrpGepModel/Config.h)

set(sources
  Sensor.cpp
  Light.cpp
  Body.cpp
  BodyCustomizerInterface.cpp
  Link.cpp
  LinkTraverse.cpp
  LinkPath.cpp
  JointPath.cpp
  ColdetLinkPair.cpp
  ForwardDynamics.cpp
  ForwardDynamicsABM.cpp
  ForwardDynamicsCBM.cpp
  World.cpp
  ConstraintForceSolver.cpp
  ModelNodeSet.cpp
  ModelLoaderUtil.cpp
  OnlineViewerUtil.cpp
  )

set(headers
  Body.h
  BodyCustomizerInterface.h
  ConstraintForceSolver.h
  ForwardDynamics.h
  ForwardDynamicsABM.h
  ForwardDynamicsCBM.h
  Link.h
  LinkTraverse.h
  LinkPath.h
  JointPath.h
  ColdetLinkPair.h
  InverseKinematics.h
  ModelNodeSet.h
  ModelLoaderUtil.h
  OnlineViewerUtil.h
  Sensor.h
  Light.h
  World.h
  ${CMAKE_BINARY_DIR}/hrpGeplib/hrpGepModel/Config.h
  )	


set(target hrpGepModel-${OPENHRP_LIBRARY_VERSION})

add_library(${target} SHARED ${sources})

# interface version 0.0.0 - version 3.0.0 beta1
set(HRPGEPMODEL_VERSION ${HRPSOVERSION}.0.0 )
set_target_properties(${target} PROPERTIES VERSION ${HRPGEPMODEL_VERSION} SOVERSION ${HRPSOVERSION})


if(UNIX)
  if(NOT QNXNTO)
    target_link_libraries(${target}
      hrpGepUtil-${OPENHRP_LIBRARY_VERSION}
      hrpGepCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}
      hrpGepCollision-${OPENHRP_LIBRARY_VERSION}
      ${OMNIORB_LIBRARIES}
      #boost_filesystem boost_regex
      ${Boost_REGEX_LIBRARY}
      #${Boost_LIBRARIES}
      dl)
  else(NOT QNXNTO)
    target_link_libraries(${target}
      hrpGepUtil-${OPENHRP_LIBRARY_VERSION}
      hrpGepCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}
      hrpGepCollision-${OPENHRP_LIBRARY_VERSION}
      #${OMNIORB_LIBRARIES}
      ${OPENRTM_LIBRARIES}
      #boost_filesystem boost_regex boost_system boost_signals
      ${Boost_LIBRARIES}
      )
  endif(NOT QNXNTO)
elseif(WIN32)
  add_definitions(-DHRPGEPMODEL_MAKE_DLL)
  target_link_libraries(${target}
    optimized hrpGepUtil-${OPENHRP_LIBRARY_VERSION}
    optimized hrpGepCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}
    optimized hrpGepCollision-${OPENHRP_LIBRARY_VERSION}
    debug hrpGepUtil-${OPENHRP_LIBRARY_VERSION}d
    debug hrpGepCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}d
    debug hrpGepCollision-${OPENHRP_LIBRARY_VERSION}d
    ${OMNIORB_LIBRARIES})
  set_target_properties(${target} PROPERTIES LINK_FLAGS /NODEFAULTLIB:LIBCMT )
  set_target_properties(${target} PROPERTIES DEBUG_POSTFIX d )
endif()

hrpGeplib_install_macro(${target} ${HRPGEPMODEL_VERSION})

install(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/hrpGepModel)

# Body Customizers
set(OPENHRPGEP_3_0_CUSTOMIZERS ${OPENHRPGEP_3_0_CUSTOMIZERS} CACHE FILEPATH "Source files of the body customizer for OpenHRP 3.0")

if(OPENHRPGEP_3_0_CUSTOMIZERS)

  foreach(src ${OPENHRPGEP_3_0_CUSTOMIZERS})

    get_filename_component(customizer ${src} NAME_WE)
    add_library(${customizer} SHARED ${src})
    set_target_properties(${customizer} PROPERTIES COMPILE_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}")
    set_target_properties(${customizer} PROPERTIES PREFIX "")
    set_target_properties(${customizer} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/customizer)

    if(WIN32)
      install(TARGETS ${customizer}
        DESTINATION ${RELATIVE_SHARE_INSTALL_PATH}/customizer CONFIGURATIONS Release Debug)
    else()
      install(TARGETS ${customizer}
        LIBRARY DESTINATION ${RELATIVE_SHARE_INSTALL_PATH}/customizer CONFIGURATIONS Release Debug RelWithDebInfo)
    endif()
  endforeach()
endif()


PKG_CONFIG_APPEND_LIBS("hrpGepModel-${OPENHRP_LIBRARY_VERSION}")