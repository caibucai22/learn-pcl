add_executable(fit_circle fit-Circle.cpp ../my_functions.cpp ../my_functions.h)
link_pcl_libraries(fit_circle)

add_executable(fit_circle_by_cc fit-Circle_from_cc.cpp ../my_functions.cpp ../my_functions.h Neighbourhood.h)
link_pcl_libraries(fit_circle_by_cc)