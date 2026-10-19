from abc import abstractmethod

class SubmodelDriverContextInterface():

    @abstractmethod
    def get_fps(self):
        ...

    @abstractmethod
    def parse_packet(self, rx_data):
        ...

    @abstractmethod
    def is_ok(self):
        ...

    @abstractmethod
    def fps_monitor(self):
        ...
