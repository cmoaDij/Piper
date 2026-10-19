from pathlib import Path
from setuptools import setup, find_packages
import os
this_dir = Path(__file__).parent.resolve()

long_description = (
    "Daimon tactile sensor SDK: camera driver + optical flow "
    "(FastFlowNet/TensorRT/ORT) + depth/shear reconstruction."
)

# 1) pip install .[gpu]
# 2) DMROBOTICS_GPU=1 pip install .
GPU_EXTRAS = [
    "tensorrt-cu12>=10.0.1",
    "onnx>=1.14.0,<=1.17.0",                   
    "cupy-cuda12x>=12.1.0,<=13.6.0"     
]

gpu_env_enabled = str(os.environ.get("DMROBOTICS_GPU", "")).lower() in ("1", "true", "yes", "on")

base_requires = [
    "numpy>=1.20,<1.27",
    "h5py>=3.8",
    "cryptography>=41.0.0",
    "opencv-contrib-python>=4.6.0.66,<=4.11.0.84",
    "scipy>=1.7.3,<=1.15.3",
    "grpcio>=1.48.0",
    "protobuf>=3.20.0",
    "grpcio-tools>=1.48.0",
    "pyudev",
    "onnxruntime-gpu>=1.19.0,<1.23.0"
]

install_requires = list(base_requires)
if gpu_env_enabled:
    install_requires += GPU_EXTRAS

setup(
    name="dmrobotics",
    version="1.2.6",
    description="Daimon tactile sensor SDK",
    long_description=long_description,
    long_description_content_type="text/plain",
    author="wanshixing",

    packages=find_packages(include=["dmrobotics", "dmrobotics.*","Daimon","Daimon.*"]),
    python_requires=">=3.8,<3.12",
    install_requires=install_requires,

    # pip install .[gpu]
    extras_require={
        "gpu": GPU_EXTRAS,
    },
    entry_points={
        "console_scripts": [
            "dmrobotics=dmrobotics.src.cli:main",
        ],
    },
    include_package_data=True,
    package_data={
         "dmrobotics.src": ["*.onnx", "*.pt"],
        "dmrobotics": [
            "lib/*.so",
            "lib/*.dll",
            "lib/*.lib",
            "lib/bundle_bins/*/ffnet_bundle*.so",
            "lib/bundle_bins/*/ffnet_bundle*.pyd",
            "lib/bundle_bins/*/ffnet_bundle*.dll",
        ],
        
    },
    zip_safe=False,

)
