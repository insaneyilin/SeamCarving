add_executable(seam_carving seam_carving.cc)
target_link_libraries(seam_carving seam_carver ${OpenCV_LIBS})

