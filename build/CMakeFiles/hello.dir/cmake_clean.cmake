file(REMOVE_RECURSE
  "../hello.exe"
  "../hello.exe.manifest"
  "../hello.pdb"
  "../libhello.dll.a"
  "CMakeFiles/hello.dir/main.cpp.obj"
  "CMakeFiles/hello.dir/main.cpp.obj.d"
)

# Per-language clean rules from dependency scanning.
foreach(lang CXX)
  include(CMakeFiles/hello.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
