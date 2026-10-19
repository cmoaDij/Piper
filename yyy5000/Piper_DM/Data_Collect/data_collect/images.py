"""RGB video encoding and version-independent raw episode image access."""

import io
from fractions import Fraction
from functools import lru_cache
from pathlib import Path

import av
import numpy as np

from .config import IMAGE_STREAMS, fingerprint


def _stream(container, encoder, width, height, fps, settings):
    stream = container.add_stream(encoder, rate=fps)
    stream.width, stream.height = width, height
    stream.pix_fmt = "yuv420p"
    stream.time_base = Fraction(1, fps)
    stream.codec_context.time_base = Fraction(1, fps)
    stream.codec_context.thread_count = settings.get("encoder_threads", 1)
    stream.codec_context.gop_size = fps
    stream.codec_context.max_b_frames = 0
    quality = str(settings.get("video_quality", 18))
    stream.options = (
        {"preset": "p4", "rc": "vbr", "cq": quality, "b": "0", "bf": "0"}
        if encoder == "h264_nvenc"
        else {"preset": "veryfast", "crf": quality, "tune": "zerolatency"}
    )
    return stream


@lru_cache(maxsize=8)
def _select_encoder(preference, profiles, threads, quality):
    """Actually encode/decode both streams concurrently; enumeration is insufficient."""
    errors = []
    for encoder in (("h264_nvenc", "libx264") if preference == "auto" else (preference,)):
        opened = []
        try:
            for width, height, fps in profiles:
                buffer = io.BytesIO()
                container = av.open(buffer, "w", format="mp4")
                opened.append((container, buffer))
                stream = _stream(container, encoder, width, height, fps,
                                 {"encoder_threads": threads, "video_quality": quality})
                for index in range(3):
                    frame = av.VideoFrame.from_ndarray(np.zeros((height, width, 3), np.uint8), format="rgb24")
                    frame.pts, frame.time_base = index, Fraction(1, fps)
                    for packet in stream.encode(frame):
                        container.mux(packet)
                for packet in stream.encode():
                    container.mux(packet)
            for container, buffer in opened:
                container.close()
                with av.open(io.BytesIO(buffer.getvalue())) as decoded:
                    if sum(1 for _ in decoded.decode(video=0)) != 3:
                        raise ValueError("编码探测帧数不符")
            print(f"RGB 视频编码器: {encoder}（双路试编码通过）" + (f"；回退原因: {errors}" if errors else ""), flush=True)
            return encoder
        except Exception as exc:
            errors.append(f"{encoder}: {type(exc).__name__}: {exc}")
        finally:
            for container, _ in opened:
                container.close()
    raise RuntimeError("没有可用的 RGB 视频编码器: " + "; ".join(errors))


def select_encoder(config):
    settings = config.get("storage", {})
    profiles = tuple((int(c["width"]), int(c["height"]), int(c["fps"])) for c in config["cameras"].values())
    return _select_encoder(settings.get("video_encoder", "auto"), profiles,
                           settings.get("encoder_threads", 1), settings.get("video_quality", 18))


class RGBVideoWriter:
    def __init__(self, path, shape, fps, encoder, settings):
        self.path = Path(path)
        self.partial = self.path.with_suffix(".mp4.partial")
        if self.path.exists() or self.partial.exists():
            raise FileExistsError(self.path)
        if len(shape) != 3 or shape[-1] != 3 or shape[0] % 2 or shape[1] % 2:
            raise ValueError("RGB 视频要求 H×W×3 且宽高为偶数")
        self.container = av.open(str(self.partial), "w", format="mp4")
        self.stream = _stream(self.container, encoder, shape[1], shape[0], fps, settings)
        self.shape, self.fps, self.count = tuple(shape), fps, 0

    def append(self, image):
        image = np.asarray(image)
        if image.dtype != np.uint8 or image.shape != self.shape or len(self.shape) != 3 or self.shape[-1] != 3:
            raise ValueError("RGB 视频帧尺寸或类型变化")
        index = self.count
        frame = av.VideoFrame.from_ndarray(image, format="rgb24")
        frame.pts, frame.time_base = index, Fraction(1, self.fps)
        for packet in self.stream.encode(frame):
            self.container.mux(packet)
        self.count += 1
        return index

    def finish(self):
        for packet in self.stream.encode():
            self.container.mux(packet)
        self.container.close()
        with av.open(str(self.partial)) as check:
            stream = check.streams.video[0]
            if stream.frames != self.count or (stream.height, stream.width, 3) != self.shape:
                raise ValueError("视频封盘帧数或尺寸不符")
        self.partial.replace(self.path)
        return fingerprint(self.path)

    def close(self):
        self.container.close()


def video_path(group):
    root = Path(group.file.filename).resolve().parent
    relative = Path(str(group.attrs["video_path"]))
    path = (root / relative).resolve()
    if relative.is_absolute() or not path.is_relative_to(root):
        raise ValueError("视频路径必须位于原始数据目录内")
    return path


def image_shape(group, *, rgb=False):
    stored = group["data"].shape[1:] if "data" in group else group.attrs["image_shape"]
    shape = tuple(int(size) for size in stored)
    if any(size <= 0 for size in shape):
        raise ValueError(f"{group.name} 图像宽高必须为正")
    if len(shape) == 2 and not rgb:
        return shape + (3,)
    if len(shape) != 3 or shape[-1] != 3:
        raise ValueError(f"{group.name} 图像尺寸非法: {shape}")
    return shape


def validate_images(file, *, decode=False):
    """Bind external videos to the HDF5 hash; optionally verify every decoded PTS."""
    for name in IMAGE_STREAMS:
        group = file[name]
        count = len(group["available_host_ts_ns"])
        if count == 0:
            raise ValueError(f"{name} 没有图像")
        shape = image_shape(group, rgb=name.startswith("cameras/"))
        if "data" in group:
            ds = group["data"]
            order = "GRAY" if ds.ndim == 3 else "RGB"
            if ds.dtype != np.uint8 or len(ds) != count or group.attrs.get("color_order") != order:
                raise ValueError(f"{name} 图像格式、颜色空间或长度非法")
            if name.startswith("tactile/") and (
                "baseline" not in group or group["baseline"].shape != ds.shape[1:]
                or group["baseline"].dtype != np.uint8
            ):
                raise ValueError(f"{name} 缺少同尺寸 uint8 无接触基线")
            continue
        if not name.startswith("cameras/") or group.attrs.get("storage") != "video" or group.attrs.get("color_order") != "RGB":
            raise ValueError(f"{name} 图像存储方式非法")
        for key in ("video_frame_index", "video_pts"):
            values = group[key][:]
            if values.dtype.kind not in "iu" or not np.array_equal(values, np.arange(count)):
                raise ValueError(f"{name} {key} 必须逐帧连续对应")
        fps = int(group.attrs["video_fps"])
        if fps <= 0 or group.attrs.get("video_time_base_num") != 1 or group.attrs.get("video_time_base_den") != fps:
            raise ValueError(f"{name} 视频时间基准非法")
        path = video_path(group)
        if fingerprint(path) != group.attrs.get("video_sha256"):
            raise ValueError(f"{name} 视频缺失或 SHA-256 不符")
        with av.open(str(path)) as container:
            stream = container.streams.video[0]
            stream.codec_context.thread_count = 1
            if stream.frames != count or (stream.height, stream.width, 3) != shape:
                raise ValueError(f"{name} 视频帧数或尺寸不符")
            if decode:
                total = 0
                for i, frame in enumerate(container.decode(stream)):
                    if frame.pts is None or frame.pts * frame.time_base * fps != i:
                        raise ValueError(f"{name} 视频 PTS 与帧索引不符")
                    total += 1
                if total != count:
                    raise ValueError(f"{name} 视频解码不完整")


class EpisodeImages:
    """Bounded sequential decoder; random review seeks back to a keyframe."""
    def __init__(self, file):
        self.file, self.readers = file, {}

    def read(self, name, index):
        group = self.file[name]
        index = int(index)
        if not 0 <= index < len(group["available_host_ts_ns"]):
            raise IndexError(index)
        if "data" in group:
            image = group["data"][index]
            return np.repeat(image[..., None], 3, axis=2) if image.ndim == 2 else image
        if name not in self.readers:
            container = av.open(str(video_path(group)))
            stream = container.streams.video[0]
            stream.codec_context.thread_count = 1
            self.readers[name] = [container, stream, iter(container.decode(stream)), -1, None]
        state = self.readers[name]
        container, stream, frames, previous, image = state
        if index == previous:
            return image
        fps = int(group.attrs["video_fps"])
        if index < previous or index - previous > fps:
            container.seek(int(Fraction(index, fps) / stream.time_base), stream=stream, backward=True)
            frames = iter(container.decode(stream))
            state[2] = frames
        for frame in frames:
            position = frame.pts * frame.time_base * fps
            if position == index:
                image = frame.to_ndarray(format="rgb24")
                state[3], state[4] = index, image
                return image
            if position > index:
                break
        raise ValueError(f"{name} 缺少视频帧 {index}")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        for container, *_ in self.readers.values():
            container.close()
