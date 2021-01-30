import logging
import os
import sys
from importlib import reload
from pathlib import Path

FORMAT = '[%(asctime)s] %(name)s %(levelname)s :  %(message)s'
LOGGING_FORMAT = logging.Formatter(FORMAT)

def setup_logger(filepath, file_level=logging.DEBUG, print_level=logging.INFO):
    reload(logging)

    # Create the directory if it doesnt exits
    Path(os.path.split(filepath)[0]).mkdir(parents=True, exist_ok=True)

    logging.basicConfig(level=file_level,
                        format=FORMAT,
                        datefmt='%m-%d %H:%M',
                        filename=filepath,
                        filemode='w')

    console = logging.StreamHandler()
    console.setLevel(print_level)
    console.setFormatter(LOGGING_FORMAT)
    logging.getLogger('').addHandler(console)

def get_logger(name):
    return logging.getLogger(name)
