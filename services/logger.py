import logging
from pathlib import Path
from config import LOG_DIR

def set_logger_config(log_level="INFO", logger_name="main"):
    """Настройка логирования для указанного логгера"""
    try:
        # Создаём директорию для логов
        log_dir = Path(LOG_DIR)
        log_dir.mkdir(parents=True, exist_ok=True)

        # Определяем уровень и имя файла
        level = logging.INFO
        filename = "info.log" if logger_name == "main" else "debug.log"
        if log_level == "DEBUG":
            level = logging.DEBUG

        # Создаём или получаем логгер
        logger = logging.getLogger(logger_name)
        logger.setLevel(level)

        # Очищаем старые обработчики
        logger.handlers = []

        # Создаём обработчик для файла
        file_path = log_dir / filename
        file_handler = logging.FileHandler(file_path, mode="w")
        file_handler.setLevel(level)
        file_handler.setFormatter(logging.Formatter(
            "%(asctime)s %(levelname)s %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S"
        ))
        logger.addHandler(file_handler)

        # Добавляем консольный обработчик
        stream_handler = logging.StreamHandler()
        stream_handler.setLevel(level)
        stream_handler.setFormatter(logging.Formatter(
            "%(asctime)s %(levelname)s %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S"
        ))
        logger.addHandler(stream_handler)

        # Логируем успешную настройку
        logger.info(f"Logger '{logger_name}' configured: file={file_path}, level={log_level}")

    except Exception as e:
        # В случае ошибки выводим в консоль и используем базовое логирование
        print(f"Failed to configure logger '{logger_name}': {e}")
        logging.basicConfig(level=logging.INFO, handlers=[logging.StreamHandler()])
        logging.getLogger(logger_name).error(f"Logger setup failed: {e}")