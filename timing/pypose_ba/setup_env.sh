#!/usr/bin/env bash
set -euo pipefail

ENV_NAME="${ENV_NAME:-pypose-bae}"
PYTHON_VERSION="${PYTHON_VERSION:-3.11}"
TORCH_INDEX_URL="${TORCH_INDEX_URL:-https://download.pytorch.org/whl/cu128}"
PYPOSE_SPEC="${PYPOSE_SPEC:-pypose>=0.9.5}"
BAE_SPEC="${BAE_SPEC:-git+https://github.com/sair-lab/bae.git@0.2}"

if ! command -v conda >/dev/null 2>&1; then
  echo "conda was not found on PATH." >&2
  exit 1
fi

if ! conda env list | awk '{print $1}' | grep -qx "${ENV_NAME}"; then
  conda create -y -n "${ENV_NAME}" "python=${PYTHON_VERSION}" pip
fi

conda run -n "${ENV_NAME}" python -m pip install --upgrade pip setuptools wheel packaging
conda run -n "${ENV_NAME}" python -m pip install torch torchvision torchaudio --index-url "${TORCH_INDEX_URL}"
conda run -n "${ENV_NAME}" python -m pip install "${PYPOSE_SPEC}" scipy matplotlib tqdm

# PyPose sparse parameters import the optional BAE backend. BAE builds native
# CUDA extensions, so keep nvcc in the env and expose NVIDIA wheel headers.
conda install -y -n "${ENV_NAME}" -c nvidia \
  cuda-nvcc=12.8 \
  libcudss-dev=0.4.0.2 \
  libcudss0=0.4.0.2 \
  libcusolver-dev=11.7.3.90 \
  libcusparse-dev=12.5.8.93

CONDA_BASE="$(conda info --base)"
# shellcheck source=/dev/null
source "${CONDA_BASE}/etc/profile.d/conda.sh"
conda activate "${ENV_NAME}"

export CUDA_HOME="${CONDA_PREFIX}"
export CC="${CC:-/usr/bin/gcc}"
export CXX="${CXX:-/usr/bin/g++}"
export CUDAHOSTCXX="${CUDAHOSTCXX:-/usr/bin/g++}"

if ! command -v nvcc >/dev/null 2>&1; then
  echo "nvcc was not found after installing cuda-nvcc; skipping BAE." >&2
elif [[ ! -x "${CXX}" ]]; then
  echo "CXX=${CXX} is not executable; skipping BAE." >&2
else
  SITE_PACKAGES="$(python -c 'import site; print(site.getsitepackages()[0])')"
  NVIDIA_INCLUDE_PATHS=(
    "${SITE_PACKAGES}/nvidia/cu12/include"
    "${SITE_PACKAGES}/nvidia/cusparse/include"
    "${SITE_PACKAGES}/nvidia/cusolver/include"
    "${SITE_PACKAGES}/nvidia/cublas/include"
    "${SITE_PACKAGES}/nvidia/cuda_runtime/include"
    "${SITE_PACKAGES}/nvidia/cuda_nvrtc/include"
  )
  NVIDIA_LIBRARY_PATHS=(
    "${SITE_PACKAGES}/nvidia/cu12/lib"
    "${SITE_PACKAGES}/nvidia/cusparse/lib"
    "${SITE_PACKAGES}/nvidia/cusolver/lib"
    "${SITE_PACKAGES}/nvidia/cublas/lib"
    "${SITE_PACKAGES}/nvidia/cuda_runtime/lib"
    "${SITE_PACKAGES}/nvidia/cuda_nvrtc/lib"
  )

  IFS=:
  export CPATH="${NVIDIA_INCLUDE_PATHS[*]}:${CPATH:-}"
  export LIBRARY_PATH="${NVIDIA_LIBRARY_PATHS[*]}:${LIBRARY_PATH:-}"
  export LD_LIBRARY_PATH="${NVIDIA_LIBRARY_PATHS[*]}:${CONDA_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
  unset IFS

  python -m pip install --no-build-isolation "${BAE_SPEC}"
fi

python - <<'PY'
import torch
import pypose as pp

print("torch", torch.__version__)
print("torch.cuda.is_available", torch.cuda.is_available())
print("torch.version.cuda", torch.version.cuda)
print("pypose", getattr(pp, "__version__", "unknown"))
try:
    import bae
except Exception as exc:
    print("bae unavailable:", exc)
else:
    print("bae", getattr(bae, "__version__", "installed"))
PY
