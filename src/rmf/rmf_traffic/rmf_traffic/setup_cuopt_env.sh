#!/bin/bash
# Setup script for cuOpt runtime environment
# This script sets LD_LIBRARY_PATH for cuOpt libraries

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../../../.." && pwd)"

# Find cuopt-env directory
CUOPT_ENV_DIR="${WORKSPACE_ROOT}/src/routing_feature/cuopt-env"
if [ ! -d "${CUOPT_ENV_DIR}" ]; then
  CUOPT_ENV_DIR="${WORKSPACE_ROOT}/src/cuopt_ws/cuopt-env"
fi

if [ ! -d "${CUOPT_ENV_DIR}" ]; then
  echo "Warning: cuOpt environment not found. cuOpt routing may not work." >&2
  return 1
fi

# Set library paths
export LD_LIBRARY_PATH=\
${CUOPT_ENV_DIR}/lib/python3.10/site-packages/libcuopt/lib64:\
${CUOPT_ENV_DIR}/lib/python3.10/site-packages/librmm/lib64:\
${CUOPT_ENV_DIR}/lib/python3.10/site-packages/nvidia/cuda_runtime/lib:\
${CUOPT_ENV_DIR}/lib/python3.10/site-packages/nvidia/cublas/lib:\
${CUOPT_ENV_DIR}/lib/python3.10/site-packages/nvidia/cusparse/lib:\
${CUOPT_ENV_DIR}/lib/python3.10/site-packages/nvidia/cusolver/lib:\
${LD_LIBRARY_PATH}

echo "cuOpt environment configured: ${CUOPT_ENV_DIR}"
echo "LD_LIBRARY_PATH updated for cuOpt libraries"

