import logging
from pathlib import Path

from config import LOG_DIR


def set_logger_config(log_level="INFO"):
    Path("logs/").mkdir(parents=True, exist_ok=True)

    level = logging.INFO
    filename = "info.log"

    if log_level == "DEBUG":
        level = logging.DEBUG
        filename = "debug.log"

    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        filename=LOG_DIR + filename
    )



# import logging
# from pathlib import Path
# from config import LOG_DIR
#
# def set_logger_config(log_level="INFO"):
#     """Настройка логирования в единый файл и консоль"""
#     try:
#         # Создаём директорию для логов
#         log_dir = Path(LOG_DIR)
#         log_dir.mkdir(parents=True, exist_ok=True)
#
#         # Определяем уровень логирования
#         level = logging.INFO
#         if log_level == "DEBUG":
#             level = logging.DEBUG
#
#         # Создаём или получаем логгер
#         logger = logging.getLogger()  # Корневой логгер для всех модулей
#         logger.setLevel(level)
#
#         # Очищаем старые обработчики
#         logger.handlers = []
#         logger.propagate = False  # Отключаем передачу корневым логгерам
#
#         # Создаём обработчик для файла
#         file_path = log_dir / "app.log"
#         file_handler = logging.FileHandler(file_path, mode="w")
#         file_handler.setLevel(level)
#         file_handler.setFormatter(logging.Formatter(
#             "%(asctime)s %(levelname)s [%(name)s] %(message)s",
#             datefmt="%Y-%m-%d %H:%M:%S"
#         ))
#         logger.addHandler(file_handler)
#
#         # Создаём обработчик для консоли
#         stream_handler = logging.StreamHandler()
#         stream_handler.setLevel(level)
#         stream_handler.setFormatter(logging.Formatter(
#             "%(asctime)s %(levelname)s [%(name)s] %(message)s",
#             datefmt="%Y-%m-%d %H:%M:%S"
#         ))
#         logger.addHandler(stream_handler)
#
#         # Логируем успешную настройку
#         logger.info(f"Logger configured: file={file_path}, level={log_level}")
#
#         # Принудительный flush для файлового обработчика
#         file_handler.flush()
#
#     except Exception as e:
#         # В случае ошибки выводим в консоль
#         print(f"Failed to configure logger: {e}")
#         logging.basicConfig(level=logging.INFO, handlers=[logging.StreamHandler()])
#         logging.getLogger().error(f"Logger setup failed: {e}")