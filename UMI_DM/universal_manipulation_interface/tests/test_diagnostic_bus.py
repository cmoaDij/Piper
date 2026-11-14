"""Diagnostics are best effort and do not block when no consumer is present."""
import json
import socket
import tempfile
import threading
from pathlib import Path

from umi.real_world.diagnostic_bus import AsyncJsonlWriter, DiagnosticPublisher, DiagnosticReceiver


def test():
    with tempfile.TemporaryDirectory(prefix='umi_diag_') as directory:
        root = Path(directory)
        socket_path = root / 'events.sock'
        received = []
        receiver = DiagnosticReceiver(socket_path, received.append)
        receiver.start()
        publisher = DiagnosticPublisher(socket_path)
        assert publisher.send({'source': 'test', 'value': 1})
        for _ in range(100):
            if received:
                break
            threading.Event().wait(.005)
        assert received == [{'source': 'test', 'value': 1}]
        publisher.close()
        receiver.close()

        missing = DiagnosticPublisher(root / 'missing.sock')
        assert missing.send({'source': 'test'})
        missing.close()

        output = root / 'events.jsonl'
        writer = AsyncJsonlWriter(output)
        writer.write({'event': 'written'})
        writer.close()
        assert json.loads(output.read_text().strip()) == {'event': 'written'}


if __name__ == '__main__':
    test()
    print('Diagnostic bus checks passed')
