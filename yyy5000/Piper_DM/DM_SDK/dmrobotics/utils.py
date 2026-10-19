__all__ = [
    "put_arrows_on_image",
    "init_h5",
    "append_h5",
    "read_h5",
    "clean_logs",
]

import cv2
import h5py
from .src.dmSDK import DMTacImage
import numpy as np
from pathlib import Path

_H5_MEM_CACHE = {}

# ---------------------------------------------------------------------------
# Flow / arrow visualization helpers
# 光流箭头可视化
# ---------------------------------------------------------------------------


def put_arrows_on_image(image: np.ndarray,
                        flow_hw2: np.ndarray,
                        *,
                        step: int = 16,
                        scale: float = 1.0,
                        min_len: float = 0.5,
                        tip_ratio: float = 0.30,
                        tip_angle_deg: float = 25.0,
                        color=(0, 255, 0),
                        thickness: int = 1) -> np.ndarray:
    """
    Vectorized arrow drawing using cv2.polylines.
    使用 cv2.polylines 进行向量化绘制，大幅提高 FPS。
    """
    H, W = flow_hw2.shape[:2]
    img = image.copy()

    off = step // 2
    ys = np.arange(off, H, step, dtype=np.int32)
    xs = np.arange(off, W, step, dtype=np.int32)
    if ys.size == 0 or xs.size == 0:
        return img

    Y, X = np.meshgrid(ys, xs, indexing='ij')
    V = flow_hw2[Y, X, :] * float(scale)

    L = np.linalg.norm(V, axis=2)
    mask = L > float(min_len)
    if not np.any(mask):
        return img

    Y, X, V, L = Y[mask], X[mask], V[mask], L[mask]

    P0 = np.stack([X, Y], axis=1).astype(np.int32)
    P1 = (P0 + V).astype(np.int32)

    ang = np.deg2rad(tip_angle_deg)
    ca, sa = np.cos(ang), np.sin(ang)
    
    U = (V / (L[:, None] + 1e-6))
    
    vx, vy = U[:, 0], U[:, 1]
    
    rot1_x = ca * vx + sa * vy
    rot1_y = -sa * vx + ca * vy
    
    rot2_x = ca * vx - sa * vy
    rot2_y = sa * vx + ca * vy
    
    tip_len = L * tip_ratio
    
    Q1 = np.zeros_like(P1)
    Q1[:, 0] = P1[:, 0] - (rot1_x * tip_len).astype(np.int32)
    Q1[:, 1] = P1[:, 1] - (rot1_y * tip_len).astype(np.int32)
    
    Q2 = np.zeros_like(P1)
    Q2[:, 0] = P1[:, 0] - (rot2_x * tip_len).astype(np.int32)
    Q2[:, 1] = P1[:, 1] - (rot2_y * tip_len).astype(np.int32)

    arrow_paths = np.stack([Q1, P1, P0, P1, Q2], axis=1)
    
    cv2.polylines(img, list(arrow_paths), isClosed=False, 
                  color=color, thickness=thickness, lineType=cv2.LINE_AA)

    return img


# ---------------------------------------------------------------------------
# HDF5 logging helpers
# 采集数据到 HDF5 的工具
# ---------------------------------------------------------------------------

def init_h5(path: str,
            first_img,
            compression: str = "gzip",
            gzip_level: int = 4,
            use_shuffle: bool = True,
            use_fletcher32: bool = True) -> None:
    """
    Create a new HDF5 file and store the first frame.
    创建并初始化一个新的 HDF5 文件，并写入第一帧。

    The file stores:
      - global attrs["serial"]: sensor serial (enforces same sensor)
      - an extensible dataset "images": shape (N,H,W), dtype=uint8
    文件中包含:
      - 全局属性 attrs["serial"]：传感器序列号（保证同一传感器）
      - 可扩展数据集 "images"：(N,H,W), uint8

    first_img is expected to have:
      .img    -> 2D uint8 array [H,W]
      .serial -> device serial string
    first_img 需要提供:
      .img    -> 2D uint8 图像 [H,W]
      .serial -> 设备序列号
    """
    img = first_img.img
    H, W = img.shape

    with h5py.File(path, "w") as f:
        f.attrs["class"] = "DMTacImageSet"
        f.attrs["version"] = 1
        f.attrs["serial"] = first_img.serial

        ds_kwargs = dict(chunks=(1, H, W))
        if use_shuffle:
            ds_kwargs["shuffle"] = True
        if use_fletcher32:
            ds_kwargs["fletcher32"] = True

        if compression == "gzip":
            ds_kwargs["compression"] = "gzip"
            ds_kwargs["compression_opts"] = int(gzip_level)
        elif compression == "lzf":
            ds_kwargs["compression"] = "lzf"
        elif compression is None:
            pass
        else:
            raise ValueError("compression must be 'gzip', 'lzf', or None")

        dset = f.create_dataset(
            "images",
            shape=(1, H, W),
            maxshape=(None, H, W),
            dtype=np.uint8,
            **ds_kwargs
        )
        dset[0] = img


def append_h5(path: str, dmimg) -> int:
    """
    Append a new frame to an existing HDF5 file created by init_h5().
    向由 init_h5() 创建的 HDF5 文件中追加一帧。

    Returns the new dataset length (number of stored frames).
    返回写入后数据集的长度（样本总数）。

    dmimg must provide:
      .img    -> 2D uint8 array [H,W]
      .serial -> device serial (must match file)
    dmimg 需要提供:
      .img    -> 2D uint8 图像 [H,W]
      .serial -> 序列号 (必须与文件一致)
    """
    img = dmimg.img

    with h5py.File(path, "a") as f:
        file_serial = f.attrs.get("serial", None)
        if file_serial is None:
            raise RuntimeError(
                "HDF5 file is missing attrs['serial']; not a valid init_h5() file."
            )
        if dmimg.serial != file_serial:
            raise ValueError(
                "Serial mismatch: provided '%s' != file '%s'"
                % (dmimg.serial, file_serial)
            )

        if "images" not in f:
            raise RuntimeError(
                "HDF5 file is missing dataset 'images'; not a valid init_h5() file."
            )

        dset = f["images"]

        if img.shape != dset.shape[1:]:
            raise ValueError(
                "Image shape mismatch: provided %r != file %r"
                % (img.shape, dset.shape[1:])
            )

        n = dset.shape[0]
        dset.resize((n + 1,) + dset.shape[1:])
        dset[n] = img
        return n + 1

def read_h5(path: str, index: int, *, strict: bool = False):
    """
    从 HDF5 按索引取一帧，带内存缓存和越界保护。
    第一次访问某个 path 会整包读进内存，后面只按索引切。
    
    参数:
        path   : h5 文件路径
        index  : 想要的帧序号
        strict : True 时索引超范围会直接抛 IndexError；
                 False 时索引超范围会返回最后一帧并打印警告

    返回:
        DMTacImage(img=..., serial=...)
    """
    p = str(Path(path).expanduser())

    # 1) 确保缓存里有
    if p not in _H5_MEM_CACHE:
        with h5py.File(p, "r") as f:
            if "images" not in f:
                raise RuntimeError("HDF5 file is missing dataset 'images'.")
            imgs = f["images"][:]            # 一次性读进来
            serial = f.attrs.get("serial", "")
        _H5_MEM_CACHE[p] = {
            "imgs": imgs,
            "serial": serial,
        }

    cache = _H5_MEM_CACHE[p]
    imgs = cache["imgs"]
    serial = cache["serial"]

    if imgs.shape[0] == 0:
        raise RuntimeError("HDF5 dataset 'images' is empty.")

    max_idx = imgs.shape[0] - 1

    if 0 <= index <= max_idx:
        img = imgs[index]
        return  serial, max_idx, DMTacImage(img=img, serial=serial)
    else:
        if strict:
            raise IndexError(f"index {index} out of range (0..{max_idx}) for file {p}")
        else:
            print(f"[WARN] index {index} out of range (0..{max_idx}), "
                  f"return last frame {max_idx} from {p}")
            img = imgs[max_idx]
            return serial, max_idx, DMTacImage(img=img, serial=serial)
        

_BASE_DIR = Path.cwd()
_LOG_DIR = _BASE_DIR / "logs"
_LOG_PC_FLUX_DIR = _BASE_DIR / "logs_pc_flux"

def _clean_one_log_dir(log_dir: Path, label: str) -> int:
    """
    删除单个日志目录中的所有 .log 文件，并返回删除数量。
    Delete all .log files in a single log directory and return count.

    参数 / Args:
        log_dir: 目标目录，比如 _LOG_DIR 或 _LOG_PC_FLUX_DIR
                 Target directory, e.g., _LOG_DIR or _LOG_PC_FLUX_DIR
        label  : 打印时使用的标签，用于区别不同目录
                 Label used in print messages to distinguish directories

    返回 / Returns:
        删除的 .log 文件数量
        Number of .log files deleted
    """
    if not log_dir.exists():
        print(f"[clean_logs] {label}: directory not found -> {log_dir}")
        return 0

    count = 0
    for p in log_dir.glob("*.log"):
        try:
            p.unlink()
            count += 1
            print(f"[clean_logs] {label}: deleted {p.name}")
        except Exception as e:
            print(f"[clean_logs] {label}: failed to delete {p.name}: {e}")

    if count == 0:
        print(f"[clean_logs] {label}: no .log files to delete.")
    return count


def clean_logs():
    """
    清理当前包目录下的日志文件：
    - <dmrobotics>/logs
    - <dmrobotics>/logs_pc_flux

    Clean log files under this package:
    - <dmrobotics>/logs
    - <dmrobotics>/logs_pc_flux
    """
    total = 0
    total += _clean_one_log_dir(_LOG_DIR, "logs")
    total += _clean_one_log_dir(_LOG_PC_FLUX_DIR, "logs_pc_flux")

    print(f"[clean_logs] total deleted: {total} file(s).")
