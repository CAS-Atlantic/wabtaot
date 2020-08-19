
add_executable(wabtaot
  trap-with.cc
  aot-cd.cc
  aot-type-dictionary.cc
  aot-function-builder.cc
  relocation.cpp
  ../interp/binary-reader-interp.cc
)

target_link_libraries(wabtaot PUBLIC jitbuilder wabt)
get_property(JITBUILDER_INCLUDE TARGET jitbuilder PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(wabtaot SYSTEM PUBLIC "${JITBUILDER_INCLUDE}")
