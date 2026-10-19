import pytest

from data_collect.common.config import load_config


@pytest.fixture
def config():
    return load_config()
