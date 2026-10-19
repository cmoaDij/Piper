#!/usr/bin/env python3
# -*-coding:utf8-*-

from abc import ABC, abstractmethod

class ProtocolParserInterface(ABC):
    
    @abstractmethod
    def parse_packet(self, **kwargs):
        ...

    @abstractmethod
    def pack(self, **kwargs):
        ...

    