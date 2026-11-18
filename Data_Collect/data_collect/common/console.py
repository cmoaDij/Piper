"""Readable launch output; unfiltered process output still goes to launch logs."""

import codecs
import re


class CollectionConsole:
    def __init__(self, emit=print):
        self.emit = emit
        self.pending = {}
        self.decoders = {}
        self.tracebacks = set()

    def on_output(self, event):
        self.feed(event.pid, 1 if event.from_stdout else 2, event.text)

    def feed(self, pid, fd, data):
        key = (pid, fd)
        decoder = self.decoders.setdefault(key, codecs.getincrementaldecoder("utf-8")("replace"))
        text = self.pending.get(key, "") + decoder.decode(data)
        lines = text.split("\n")
        self.pending[key] = lines.pop()
        for line in lines:
            self._line(key, line.rstrip("\r"))

    def on_exit(self, event, context):
        self.flush(event.pid)

    def flush(self, pid):
        for key in [key for key in self.decoders if key[0] == pid]:
            tail = self.pending.pop(key, "") + self.decoders.pop(key).decode(b"", final=True)
            if tail:
                self._line(key, tail)
            self.tracebacks.discard(key)

    def _line(self, key, line):
        line = re.sub(r"\x1b\[[0-9;]*m", "", line)
        if not line.strip():
            return
        ros = re.match(r"\[(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]\s+\[[^]]+\]\s+\[[^]]+\]:\s*(.*)", line)
        python = re.match(r"(DEBUG|INFO|WARN|WARNING|ERROR|CRITICAL):[^:]+:\s*(.*)", line)
        level, message = (ros.groups() if ros else python.groups() if python else (None, line))
        if level:
            self.tracebacks.discard(key)
        if "Traceback (most recent call last)" in message:
            self.tracebacks.add(key)
        if key in self.tracebacks:
            self.emit(line)
        elif level in ("WARN", "WARNING", "ERROR", "FATAL", "CRITICAL"):
            self.emit(f"[{'警告' if level in ('WARN', 'WARNING') else '错误'}] {message}")
        elif any(tag in message for tag in ("[采集]", "[状态]", "[设备]", "[回零]", "[遥操]")):
            self.emit(message)
        elif "Moving home at " in message:
            self.emit("[回零] 开始，" + message.split("Moving home at ", 1)[1].replace(" speed", " 速度"))
        elif "moved to home position successfully" in message:
            self.emit("[回零] 六关节已到零位")
        elif "AgxGripper enabled and verified open at home" in message:
            self.emit("[回零] 夹爪已张开，到位检查通过")
        elif re.search(r"\b(?:WARN|WARNING|ERROR|FATAL|CRITICAL|failed|disconnected|disconnecting)\b|失败|断开|错误", message, re.IGNORECASE):
            # Native SDK print() calls may not carry a structured severity.
            self.emit(message)
        elif key[1] == 2 and level is None:
            # Keep unknown native stderr diagnostics, including exception tails.
            self.emit(line)
