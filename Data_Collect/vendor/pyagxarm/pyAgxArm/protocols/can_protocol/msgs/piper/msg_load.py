#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import importlib
import pkgutil
import inspect
from typing import Optional


class _MsgNS:
    pass


class MsgBundle:
    def __init__(self):
        self.feedback = _MsgNS()
        self.transmit = _MsgNS()


def _find_msg_class(module):
    for _, obj in inspect.getmembers(module, inspect.isclass):
        if obj.__module__ == module.__name__:
            return obj
    return None


def _find_msg_classes(module):
    """
    找出模块中所有“消息类”
    """
    classes = []

    for _, obj in inspect.getmembers(module, inspect.isclass):
        if obj.__module__ != module.__name__:
            continue
        if obj.__name__.startswith("_"):
            continue
        # 可选：加一层保险
        if not obj.__name__.startswith("ArmMsg"):
            continue

        classes.append(obj)

    return classes


def _load_category(pkg_path: str, target_ns: object):
    pkg = importlib.import_module(pkg_path)

    for _, mod_name, _ in pkgutil.iter_modules(pkg.__path__):
        module = importlib.import_module(f"{pkg_path}.{mod_name}")
        # cls = _find_msg_class(module)
        # if cls is None:
        #     continue

        # setattr(target_ns, cls.__name__, cls)
        msg_classes = _find_msg_classes(module)
        for cls in msg_classes:
            setattr(target_ns, cls.__name__, cls)


def load_msgs(
    arm: str,
    version: Optional[str] = None,
) -> MsgBundle:
    """
    从【臂 → 版本 → 功能】加载消息
    """
    msgs = MsgBundle()

    # 当前 loader 所在的包：
    # pyAgxArm.protocols.can_protocol.msgs.piper
    root_pkg = __package__   # 关键点 ✅

    # 统一把 "" 当作 None
    if not version:
        base = f"{root_pkg}.default"
    else:
        base = f"{root_pkg}.versions.{version}"

    _load_category(f"{base}.feedback", msgs.feedback)
    _load_category(f"{base}.transmit", msgs.transmit)

    return msgs
