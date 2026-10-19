````markdown
# Installation Guide

## Dependencies & Installation

### CPU Mode (CPU Inference Only)

If you only need CPU mode, install the package from the project root:

```bash
pip install .
````

After installation, you can run with `backend="cpu"`.

---

### GPU Mode (CUDA Acceleration)

If you plan to use GPU acceleration, make sure your system has a compatible NVIDIA GPU and the following are installed:

* NVIDIA driver
* CUDA Toolkit
* cuDNN

Follow the steps below:

1. Install the NVIDIA driver and CUDA Toolkit (matching your GPU and OS).
2. Download cuDNN from NVIDIA:
   [https://developer.download.nvidia.com/compute/cudnn/redist/cudnn/linux-x86_64/](https://developer.download.nvidia.com/compute/cudnn/redist/cudnn/linux-x86_64/)
3. Extract cuDNN, then copy the `include/` and `lib/` contents into your CUDA installation directory:

   * Copy headers into: `<CUDA_PATH>/include`
   * Copy libraries into: `<CUDA_PATH>/lib64`
4. Install dmrobotics with GPU extras:

```bash
pip install .[gpu]
```
build tensorrt
```bash
dmrobotics trt
```
After installation, you can run with `backend="cuda"`.

---

## Mode Overview

### 1) CPU Mode

Recommended for machines without an NVIDIA GPU or when GPU acceleration is not required.

Install:

```bash
pip install .
```

Run with:

* `backend="cpu"`

---

### 2) CUDA Mode (GPU)

Recommended for machines with an NVIDIA GPU and a properly configured CUDA + cuDNN environment.

Install:

```bash
pip install .[gpu]
```
Build tensorrt:
```bash
dmrobotics trt build
```
Run with:

* `backend="cuda"`

> Notes:
>
> * Make sure your NVIDIA driver / CUDA / cuDNN versions are compatible.
> * If you upgrade CUDA / driver / TensorRT, you may need to rebuild cached TensorRT engines (if your setup uses them).

---

```
```
