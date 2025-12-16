#!/bin/bash
# cuOpt Runtime Environment Setup for RMF
# Source this file before running RMF to enable cuOpt routing

# Get workspace root (assuming this script is in rmf_traffic/rmf_traffic/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Go up 4 levels: rmf_traffic/rmf_traffic -> rmf_traffic -> rmf -> src -> workspace_root
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

# Find cuopt-env directory
CUOPT_ENV_DIR="${WORKSPACE_ROOT}/src/routing_feature/cuopt-env"
if [ ! -d "${CUOPT_ENV_DIR}" ]; then
  CUOPT_ENV_DIR="${WORKSPACE_ROOT}/src/cuopt_ws/cuopt-env"
fi

if [ ! -d "${CUOPT_ENV_DIR}" ]; then
  echo "Warning: cuOpt environment not found at ${CUOPT_ENV_DIR}" >&2
  echo "cuOpt routing will not be available. Please ensure cuopt-env is installed." >&2
  return 1
fi

# Set library paths for cuOpt and dependencies
export LD_LIBRARY_PATH=\
${CUOPT_ENV_DIR}/lib/python3.10/site-packages/libcuopt/lib64:\
${CUOPT_ENV_DIR}/lib/python3.10/site-packages/librmm/lib64:\
${CUOPT_ENV_DIR}/lib/python3.10/site-packages/nvidia/cuda_runtime/lib:\
${CUOPT_ENV_DIR}/lib/python3.10/site-packages/nvidia/cublas/lib:\
${CUOPT_ENV_DIR}/lib/python3.10/site-packages/nvidia/cusparse/lib:\
${CUOPT_ENV_DIR}/lib/python3.10/site-packages/nvidia/cusolver/lib:\
${LD_LIBRARY_PATH}

echo "[cuOpt] Runtime environment configured"
echo "[cuOpt] Library path: ${CUOPT_ENV_DIR}"

