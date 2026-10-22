# Shared compile settings. CI passes -DMSS_WERROR=ON.
option(MSS_WERROR "Treat warnings as errors" OFF)

function(mss_set_target_defaults target)
  target_compile_features(${target} PUBLIC cxx_std_17)
  set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  if(MSS_WERROR)
    target_compile_options(${target} PRIVATE -Werror)
  endif()
endfunction()
