import logging
import sys
from datetime import datetime


def setup_logger(name: str = 'wro', level=logging.DEBUG) -> logging.Logger:
    """Create and return a named logger writing to stdout with timestamps."""
    logger = logging.getLogger(name)
    if logger.handlers:
        return logger  # already configured, avoid duplicate handlers

    logger.setLevel(level)

    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(level)

    # Format: [HH:MM:SS.mmm] LEVEL module: message
    fmt = '[%(asctime)s] %(levelname)-5s %(module)s: %(message)s'
    handler.setFormatter(logging.Formatter(fmt, datefmt='%H:%M:%S'))

    logger.addHandler(handler)
    return logger


# Module-level default logger — other modules can import and use directly
log = setup_logger()
