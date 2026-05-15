"""
logging_config.py

Structured logging setup.
Console: human-readable with colour-coded levels.
File: timestamped log for post-voyage analysis.
"""

from __future__ import annotations

import logging
import logging.handlers
from pathlib import Path


def setup_logging(
    level      : int  = logging.INFO,
    log_file   : str | None = "/tmp/maritime_perception.log",
    max_bytes  : int  = 10 * 1024 * 1024,   # 10MB
    backup_count: int = 3,
) -> None:
    """
    Configure root logger with console and optional rotating file handler.
    Call once at startup in main().
    """
    fmt = "%(asctime)s.%(msecs)03d | %(levelname)-8s | %(name)-30s | %(message)s"
    datefmt = "%H:%M:%S"

    root = logging.getLogger()
    root.setLevel(level)

    # console handler
    ch = logging.StreamHandler()
    ch.setLevel(level)
    ch.setFormatter(logging.Formatter(fmt, datefmt=datefmt))
    root.addHandler(ch)

    # rotating file handler
    if log_file:
        Path(log_file).parent.mkdir(parents=True, exist_ok=True)
        fh = logging.handlers.RotatingFileHandler(
            log_file,
            maxBytes=max_bytes,
            backupCount=backup_count,
            encoding="utf-8",
        )
        fh.setLevel(logging.DEBUG)   # file always gets DEBUG
        fh.setFormatter(logging.Formatter(fmt, datefmt=datefmt))
        root.addHandler(fh)

    # silence noisy third-party loggers
    logging.getLogger("rplidar").setLevel(logging.WARNING)