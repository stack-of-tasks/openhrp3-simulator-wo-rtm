macro(hrplib_install_macro HRPLIB_TARGET HRPLIB_VERSION)
  if(WIN32)
    install(TARGETS ${HRPLIB_TARGET}
      RUNTIME DESTINATION ${PROJECT_BINARY_DIR}/bin CONFIGURATIONS Release Debug
      LIBRARY DESTINATION ${PROJECT_BINARY_DIR}/lib CONFIGURATIONS Release Debug
      ARCHIVE DESTINATION ${PROJECT_BINARY_DIR}/lib CONFIGURATIONS Release Debug
      )
  endif()
  install(TARGETS ${HRPLIB_TARGET}
    RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
    LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
    ARCHIVE DESTINATION lib CONFIGURATIONS Release Debug
  )
endmacro()

macro(hrpGeplib_install_macro HRPGEPLIB_TARGET HRPGEPLIB_VERSION)
  if(WIN32)
    install(TARGETS ${HRPGEPLIB_TARGET}
      RUNTIME DESTINATION ${PROJECT_BINARY_DIR}/bin CONFIGURATIONS Release Debug
      LIBRARY DESTINATION ${PROJECT_BINARY_DIR}/lib CONFIGURATIONS Release Debug
      ARCHIVE DESTINATION ${PROJECT_BINARY_DIR}/lib CONFIGURATIONS Release Debug
      )
  endif()
  install(TARGETS ${HRPGEPLIB_TARGET}
    RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
    LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
    ARCHIVE DESTINATION lib CONFIGURATIONS Release Debug
  )
endmacro()

