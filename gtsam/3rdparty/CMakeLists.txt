# install CCOLAMD headers
install(FILES CCOLAMD/Include/ccolamd.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gtsam/3rdparty/CCOLAMD)
install(FILES UFconfig/UFconfig.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gtsam/3rdparty/UFconfig)

# install Eigen - only the headers
install(DIRECTORY Eigen/Eigen 
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gtsam/3rdparty/Eigen
    FILES_MATCHING PATTERN "*.h")
file(GLOB eigen_dir_headers_all "Eigen/Eigen/*")
foreach(eigen_dir ${eigen_dir_headers_all})
    get_filename_component(filename ${eigen_dir} NAME)
    if (NOT ((${filename} MATCHES "CMakeLists.txt") OR (${filename} MATCHES "src")))
        install(FILES Eigen/Eigen/${filename} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gtsam/3rdparty/Eigen/Eigen)     
    endif()
endforeach(eigen_dir)