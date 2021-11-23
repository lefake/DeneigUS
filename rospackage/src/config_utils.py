
from logging_utils import get_logger
import yaml

class ConfigUtils:
    def __init__(self):
        self.logger = get_logger("config_utils.main")

    def load_yaml(self, path):
        with open(path, "r") as stream:
            try:
                data = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                self.logger.info(exc)
                data = None
        return data

    # TODO: write_yaml
    #def write_yaml(self, path):