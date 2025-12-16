cmake_minimum_required(VERSION 3.5.0)

project(rmf_traffic VERSION 3.0.2)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  # Use the Release build type by default if the user has not specified one
  set(CMAKE_BUILD_TYPE Release)
endif()

include(GNUInstallDirs)

# We need to use the internal version of FCL because the upstream version has
# bugs that are not yet fixed.
#find_package(fcl 0.6 QUIET)

if(fcl_FOUND)
  message(STATUS "Using installed FCL version: ${FCL_VERSION}")
else()
  message(STATUS "Using in-source FCL")
  add_subdirectory(thirdparty/fcl)
endif()
# TODO(MXG): Remove everything related to maintaining compatibility with 0.5,
# since we are now requiring FCL-0.6+
set(using_new_fcl true)
set(FCL_LIBRARIES fcl)

find_package(rmf_utils REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(eigen3_cmake_module QUIET)
if (eigen3_cmake_module_FOUND)
  set(RMF_TRAFFIC_DEPENDS_EIGEN3_CMAKE_MODULE "find_dependency(eigen3_cmake_module)")
endif()
find_package(Eigen3 REQUIRED)

# ===== NVIDIA cuOpt Routing Integration =====
# Enables GPU-accelerated shortest path planning via NVIDIA cuOpt distance_engine
# Completely replaces A* and Dijkstra algorithms
message(STATUS "NVIDIA cuOpt routing integration enabled")

# Set cuOpt paths (check multiple possible locations)
# CMakeLists.txt is at: src/rmf/rmf_traffic/rmf_traffic/CMakeLists.txt
# Workspace root is 4 levels up: ../../../../ (not 5!)
get_filename_component(WORKSPACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
message(STATUS "Workspace root: ${WORKSPACE_ROOT}")

# Try routing_feature first (most likely location)
set(CUOPT_ENV_DIR "${WORKSPACE_ROOT}/src/routing_feature/cuopt-env")
if(NOT EXISTS "${CUOPT_ENV_DIR}")
  # Try alternative location
  set(CUOPT_ENV_DIR "${WORKSPACE_ROOT}/src/cuopt_ws/cuopt-env")
endif()
message(STATUS "Checking cuOpt environment at: ${CUOPT_ENV_DIR}")

if(EXISTS "${CUOPT_ENV_DIR}")
  message(STATUS "Found cuOpt environment: ${CUOPT_ENV_DIR}")
  
  # Set cuOpt paths
  set(CUOPT_ROOT "${CUOPT_ENV_DIR}/lib/python3.10/site-packages/libcuopt")
  set(CUOPT_INCLUDE_DIR "${CUOPT_ROOT}/include")
  set(CUOPT_LIB_DIR "${CUOPT_ROOT}/lib64")
  set(CUOPT_LIB "${CUOPT_LIB_DIR}/libcuopt.so")
  
  # Set RAFT and RMM paths
  set(RMM_ROOT "${CUOPT_ENV_DIR}/lib/python3.10/site-packages/librmm")
  set(RMM_INCLUDE_DIR "${RMM_ROOT}/include")
  set(RMM_LIB_DIR "${RMM_ROOT}/lib64")
  set(RMM_LIB "${RMM_LIB_DIR}/librmm.so")
  
  set(RAFT_ROOT "${CUOPT_ENV_DIR}/lib/python3.10/site-packages/libraft")
  set(RAFT_INCLUDE_DIR "${RAFT_ROOT}/include")
  
  # Set rapids_logger
  set(RAPIDS_LOGGER_ROOT "${CUOPT_ENV_DIR}/lib/python3.10/site-packages/rapids_logger")
  set(RAPIDS_LOGGER_INCLUDE_DIR "${RAPIDS_LOGGER_ROOT}/include")
  
  # CUDA runtime paths
  set(CUDA_RUNTIME_LIB_DIR "${CUOPT_ENV_DIR}/lib/python3.10/site-packages/nvidia/cuda_runtime/lib")
  set(CUDART_LIB "${CUDA_RUNTIME_LIB_DIR}/libcudart.so.12")
  
  # Find CUDA headers
  set(CUDA_RUNTIME_ROOT "${CUOPT_ENV_DIR}/lib/python3.10/site-packages/nvidia/cuda_runtime")
  set(CUDA_INCLUDE_DIR "${CUDA_RUNTIME_ROOT}/include")
  set(CUDA_NVCC_INCLUDE_DIR "${CUOPT_ENV_DIR}/lib/python3.10/site-packages/nvidia/cuda_nvcc/include")
  
  # Find all nvidia CUDA library includes
  file(GLOB NVIDIA_CUDA_PACKAGES "${CUOPT_ENV_DIR}/lib/python3.10/site-packages/nvidia/cuda_*/include")
  file(GLOB NVIDIA_CUBLAS_INCLUDE "${CUOPT_ENV_DIR}/lib/python3.10/site-packages/nvidia/cublas/include")
  file(GLOB NVIDIA_CUSPARSE_INCLUDE "${CUOPT_ENV_DIR}/lib/python3.10/site-packages/nvidia/cusparse/include")
  file(GLOB NVIDIA_CUSOLVER_INCLUDE "${CUOPT_ENV_DIR}/lib/python3.10/site-packages/nvidia/cusolver/include")
  file(GLOB NVIDIA_CURAND_INCLUDE "${CUOPT_ENV_DIR}/lib/python3.10/site-packages/nvidia/curand/include")
  
  list(APPEND NVIDIA_CUDA_INCLUDE_DIRS 
    ${NVIDIA_CUDA_PACKAGES}
    ${NVIDIA_CUBLAS_INCLUDE}
    ${NVIDIA_CUSPARSE_INCLUDE}
    ${NVIDIA_CUSOLVER_INCLUDE}
    ${NVIDIA_CURAND_INCLUDE}
  )
  
  message(STATUS "cuOpt include: ${CUOPT_INCLUDE_DIR}")
  message(STATUS "cuOpt lib: ${CUOPT_LIB}")
else()
  message(WARNING "cuOpt environment not found at ${CUOPT_ENV_DIR}. cuOpt routing will not be available.")
  set(CUOPT_AVAILABLE FALSE)
endif()

# ===== Traffic control library
file(GLOB_RECURSE core_lib_srcs "src/rmf_traffic/*.cpp")
add_library(rmf_traffic SHARED
  ${core_lib_srcs}
)

find_package(ament_cmake_catch2 QUIET)
find_package(ament_cmake_uncrustify QUIET)
if(BUILD_TESTING AND ament_cmake_catch2_FOUND AND ament_cmake_uncrustify_FOUND)
  file(GLOB_RECURSE unit_test_srcs "test/*.cpp")

  ament_add_catch2(
    test_rmf_traffic test/main.cpp ${unit_test_srcs}
    TIMEOUT 300)
  target_link_libraries(test_rmf_traffic
      rmf_traffic
      ${FCL_LIBRARIES}
      Threads::Threads
  )

  if(using_new_fcl)
    target_compile_definitions(test_rmf_traffic PRIVATE RMF_TRAFFIC__USING_FCL_0_6)
  endif()

  target_include_directories(test_rmf_traffic
    PUBLIC
      "${Eigen3_INCLUDE_DIRS}"
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/>
  )

  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../share/rmf_utils/")

  # Uncrustify is bugging out, giving nonsense false positives for the file
  # rmf_traffic/src/rmf_traffic/agv/Interpolate.cpp which have no apparent
  # solution. We will disable uncrustify until this big is fixed or a workaround
  # can be found.
#  ament_uncrustify(
#    ARGN include src test
#    CONFIG_FILE ${uncrustify_config_file}
#    MAX_LINE_LENGTH 80
#  )
endif()

if(using_new_fcl)
  target_compile_definitions(rmf_traffic PRIVATE RMF_TRAFFIC__USING_FCL_0_6)
endif()

target_link_libraries(rmf_traffic
  PUBLIC
    rmf_utils::rmf_utils
    Threads::Threads
  PRIVATE
    ${FCL_LIBRARIES}
    yaml-cpp
)

target_include_directories(rmf_traffic
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${EIGEN3_INCLUDE_DIRS}
)

# Add cuOpt includes and libraries if available
if(EXISTS "${CUOPT_ENV_DIR}")
  target_include_directories(rmf_traffic PRIVATE
    ${CUOPT_INCLUDE_DIR}
    ${RMM_INCLUDE_DIR}
    ${RAFT_INCLUDE_DIR}
    ${RAPIDS_LOGGER_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIR}
    ${CUDA_NVCC_INCLUDE_DIR}
    ${NVIDIA_CUDA_INCLUDE_DIRS}
  )
  
  target_link_directories(rmf_traffic PRIVATE
    ${CUOPT_LIB_DIR}
    ${RMM_LIB_DIR}
    ${CUDA_RUNTIME_LIB_DIR}
  )
  
  target_link_libraries(rmf_traffic PRIVATE
    ${CUOPT_LIB}
    ${RMM_LIB}
    ${CUDART_LIB}
    pthread
    dl
  )
  
  # Set RPATH so libraries can be found at runtime
  set_target_properties(rmf_traffic PROPERTIES
    BUILD_RPATH "${CUOPT_LIB_DIR};${RMM_LIB_DIR};${CUDA_RUNTIME_LIB_DIR}"
    INSTALL_RPATH "${CUOPT_LIB_DIR};${RMM_LIB_DIR};${CUDA_RUNTIME_LIB_DIR}"
    BUILD_RPATH_USE_LAUNCHER TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE
  )
  
  # Also add to LD_LIBRARY_PATH equivalent for runtime
  set(CUOPT_RUNTIME_LIB_PATH "${CUOPT_LIB_DIR}:${RMM_LIB_DIR}:${CUDA_RUNTIME_LIB_DIR}")
  target_compile_definitions(rmf_traffic PRIVATE
    LIBCUDACXX_ENABLE_EXPERIMENTAL_MEMORY_RESOURCE
    CUOPT_RUNTIME_LIB_PATH="${CUOPT_RUNTIME_LIB_PATH}"
  )
  
  message(STATUS "cuOpt libraries linked to rmf_traffic")
  message(STATUS "cuOpt runtime library path: ${CUOPT_RUNTIME_LIB_PATH}")
endif()

# Create cmake config files
include(CMakePackageConfigHelpers)

set(INSTALL_CONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/rmf_traffic/cmake")
set(PACKAGE_CONFIG_VERSION_FILE "${CMAKE_CURRENT_BINARY_DIR}/rmf_traffic-config-version.cmake")
set(PACKAGE_CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/rmf_traffic-config.cmake")

configure_package_config_file(
  "${CMAKE_CURRENT_LIST_DIR}/cmake/rmf_traffic-config.cmake.in"
  ${PACKAGE_CONFIG_FILE}
  INSTALL_DESTINATION ${INSTALL_CONFIG_DIR}
)

write_basic_package_version_file(
  ${PACKAGE_CONFIG_VERSION_FILE}
  COMPATIBILITY SameMajorVersion
)

install(
  TARGETS rmf_traffic
  EXPORT  rmf_traffic-targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  DIRECTORY include/rmf_traffic
  DESTINATION include
)

# Install zone planning public headers used by other packages
install(
  FILES
    src/rmf_traffic/agv/planning/zone_initialization.hpp
    src/rmf_traffic/agv/planning/zone_registry.hpp
    src/rmf_traffic/agv/planning/zone_planner_helper.hpp
    src/rmf_traffic/agv/planning/zone_planner_integration.hpp
    src/rmf_traffic/agv/planning/zone_graph_filter.hpp
  DESTINATION include/rmf_traffic/agv/planning
)

install(
  FILES
    ${PACKAGE_CONFIG_VERSION_FILE}
    ${PACKAGE_CONFIG_FILE}
  DESTINATION ${INSTALL_CONFIG_DIR}
)

install(
  EXPORT rmf_traffic-targets
  FILE rmf_traffic-targets.cmake
  NAMESPACE rmf_traffic::
  DESTINATION ${INSTALL_CONFIG_DIR}
)

export(
  EXPORT rmf_traffic-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/rmf_traffic-targets.cmake
  NAMESPACE rmf_traffic::
)

# Export ament index path
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv "prepend-non-duplicate;AMENT_PREFIX_PATH;")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv DESTINATION share/${PROJECT_NAME}/hook)
