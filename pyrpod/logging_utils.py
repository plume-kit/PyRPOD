import logging
import os


def get_logger(name: str = "pyrpod", level: str | None = None) -> logging.Logger:
    """
    Return a configured logger for the PyRPOD project.

    - Uses a StreamHandler with a concise formatter
    - Respects PYRPOD_LOG_LEVEL and PYRPOD_LOG_FORMAT env vars
    - Avoids duplicate handlers on repeated calls
    """
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = logging.StreamHandler()
        fmt = os.environ.get(
            "PYRPOD_LOG_FORMAT",
            "%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        )
        handler.setFormatter(logging.Formatter(fmt))
        logger.addHandler(handler)

        level_name = (level or os.environ.get("PYRPOD_LOG_LEVEL", "INFO")).upper()
        logger.setLevel(getattr(logging, level_name, logging.INFO))

        # Prevent messages from bubbling to the root logger if user configured it
        logger.propagate = False

    return logger
