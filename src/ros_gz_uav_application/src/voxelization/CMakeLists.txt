cmake_minimum_required(VERSION 3.8)
project(voxelization)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

# Include directories
include_directories(include)

# Source files
set(SOURCES
    src/spatial_entities.cpp
    src/voxelization_algorithms.cpp
    src/voxelization_factory.cpp
)

# Header files
set(HEADERS
    include/voxelization_base.hpp
    include/spatial_entities.hpp
    include/voxelization_algorithms.hpp
)

# Create library
add_library(voxelization ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(voxelization
    Eigen3::Eigen
)

# Add OpenMP if available
if(OpenMP_CXX_FOUND)
    target_link_libraries(voxelization OpenMP::OpenMP_CXX)
    target_compile_options(voxelization PRIVATE ${OpenMP_CXX_FLAGS})
endif()

# Set compiler flags
target_compile_options(voxelization PRIVATE
    -Wall
    -Wextra
    -O3
)

# Create test executable
add_executable(test_voxelization test/test_voxelization.cpp)
target_link_libraries(test_voxelization voxelization)

# Install targets
install(TARGETS voxelization
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES ${HEADERS}
    DESTINATION include/voxelization
)

# Optional: CUDA support
option(ENABLE_CUDA "Enable CUDA support" OFF)
if(ENABLE_CUDA)
    find_package(CUDA REQUIRED)
    if(CUDA_FOUND)
        # Add CUDA source files
        set(CUDA_SOURCES
            src/obstacle_gpu_kernel.cu
        )
        
        # Create CUDA library
        cuda_add_library(voxelization_cuda ${CUDA_SOURCES})
        
        # Link CUDA libraries
        target_link_libraries(voxelization_cuda
            ${CUDA_LIBRARIES}
        )
        
        # Set CUDA compiler flags
        set_target_properties(voxelization_cuda PROPERTIES
            CUDA_SEPARABLE_COMPILATION ON
        )
        
        # Link with main library
        target_link_libraries(voxelization voxelization_cuda)
        
        # Install CUDA library
        install(TARGETS voxelization_cuda
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
        )
    endif()
endif()

# Documentation
# find_package(Doxygen)
# if(DOXYGEN_FOUND)
#     set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
#     set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
#     
#     configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
#     
#     add_custom_target(docs
#         COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
#         WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#         COMMENT "Generating API documentation with Doxygen"
#         VERBATIM
#     )
# endif() 