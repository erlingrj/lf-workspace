find_package(Eigen3 REQUIRED NO_MODULE)
target_link_libraries(${LF_MAIN_TARGET} Eigen3::Eigen)
# Copy source and header files into source directory
set(LF_SRC_DIR ${CMAKE_INSTALL_PREFIX}/src)
set(LF_SRC_GEN_DIR ${CMAKE_INSTALL_PREFIX}/src-gen)


file(COPY ${LF_SRC_DIR}/kf.cpp ${LF_SRC_DIR}/kf.hpp DESTINATION ${PROJECT_SOURCE_DIR})


target_sources(${LF_MAIN_TARGET} PRIVATE kf.cpp)
message("-- Custom CMAKE file processed")
