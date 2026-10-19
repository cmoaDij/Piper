import threading
from queue import Queue
from .driver_context import DriverContext
from .protocol_parser_interface import ProtocolParserInterface
from .submodel_driver_context_interface import SubmodelDriverContextInterface

class SubmodelDriverContextAbstract(SubmodelDriverContextInterface):
    FPS_DATA_MONITOR = "DefaultDataMonitor"
    
    def __init__(self, config: dict, ctx: DriverContext, parser: ProtocolParserInterface):
        self._config = config
        self._ctx = ctx
        self._q_fps = Queue(maxsize=5)
        self._is_ok_mtx = threading.Lock()
        self._is_ok = False
        self._parser = parser
        self._ctx.fps.add_variable(self.FPS_DATA_MONITOR)
        self._ctx.register_parser_packet_fun(self.parse_packet)
        self._ctx.register_data_monitor_fun(self.fps_monitor)

    def get_fps(self):
        return self._ctx.fps.get_fps(self.FPS_DATA_MONITOR)

    def parse_packet(self, rx_data):
        recv_msg = self._parser.parse_packet(rx_data)
        if recv_msg:
            self._ctx.fps.increment(self.FPS_DATA_MONITOR)

    def is_ok(self):
        with self._is_ok_mtx:
            return self._is_ok 

    def fps_monitor(self):
        if self._q_fps.full():
            self._q_fps.get()
        self._q_fps.put(self.get_fps())
        with self._is_ok_mtx:
            if self._q_fps.full() and all(x == 0 for x in self._q_fps.queue):
                    self._is_ok = False
            else:
                self._is_ok = True
