if(n64 IN_LIST ARES_CORES)
  add_executable(n64-harness n64-harness.cpp)

  target_include_directories(n64-harness PRIVATE ${CMAKE_SOURCE_DIR})
  target_link_libraries(n64-harness PRIVATE ares::ares ares::mia ares::nall)

  if(ARES_ENABLE_CHD)
    target_link_libraries(n64-harness PRIVATE chdr-static)
  endif()

  set_target_properties(n64-harness PROPERTIES FOLDER tools PREFIX "")
  target_enable_subproject(n64-harness "headless scriptable N64 runner")
  set(CONSOLE TRUE)
  ares_configure_executable(n64-harness)
else()
  target_disable_subproject(n64-harness "headless scriptable N64 runner")
endif()
