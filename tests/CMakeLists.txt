set_source_files_properties(Test_OMP.cpp PROPERTIES COMPILE_FLAGS ${OpenMP_CXX_FLAGS})
add_executable(Test_OMP Test_OMP.cpp)