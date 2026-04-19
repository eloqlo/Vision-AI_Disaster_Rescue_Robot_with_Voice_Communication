# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles\\appJetDash_autogen.dir\\AutogenUsed.txt"
  "CMakeFiles\\appJetDash_autogen.dir\\ParseCache.txt"
  "appJetDash_autogen"
  )
endif()
