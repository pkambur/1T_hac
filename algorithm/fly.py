import logging
import time
from pathlib import Path
from algorithm.PID import move, get_data, equal, concat_engines
from connection.SocketConnection import SocketConnection
from services.logger import set_logger_config
from config import LOG_LEVEL

# Создаём логгер для fly
logger = logging.getLogger("fly")

# Инициализация логгера
set_logger_config(LOG_LEVEL)  # Без logger_name
logger.debug("Fly module logger initialized")

connection = SocketConnection()
H = 8
T = 0.1
ANGLE = 5
ITER = [0, 300, 600, 900, 1200]

TAG = [None for _ in range(5)]
IS_X = [True for _ in range(5)]
INITIAL_POSITIONS = [None for _ in range(5)]
RETURNING = [False for _ in range(5)]
TRAJECTORIES = [[] for _ in range(5)]
CURRENT_TARGETS = [None for _ in range(5)]
FIRES_STATUS = []

def check_lidars(directions, lidars):
    try:
        logger.debug(f"Checking lidars for directions: {directions}")
        if 0 < lidars.get(directions[0], float('inf')) < 5:
            logger.debug(f"Obstacle detected in main direction {directions[0]}: {lidars.get(directions[0])}")
            return False
        for direction in directions[1:]:
            if 0 < lidars.get(direction, float('inf')) < 3:
                logger.debug(f"Obstacle detected in side direction {direction}: {lidars.get(direction)}")
                return False
        logger.debug(f"Path clear for directions {directions}")
        return True
    except Exception as e:
        logger.error(f"Error in check_lidars: {e}")
        return False

def get_direction1(drone_z, fire_z):
    try:
        direction = "b"
        if drone_z - fire_z > 0:
            direction = "f"
        logger.debug(f"Z direction: {direction} (drone_z={drone_z}, fire_z={fire_z})")
        return direction, direction + "r", direction + "l"
    except Exception as e:
        logger.error(f"Error in get_direction1: {e}")
        raise

def get_direction2(drone_x, fire_x):
    try:
        direction = "l"
        if drone_x - fire_x > 0:
            direction = "r"
        logger.debug(f"X direction: {direction} (drone_x={drone_x}, fire_x={fire_x})")
        return direction, "f" + direction, "b" + direction
    except Exception as e:
        logger.error(f"Error in get_direction2: {e}")
        raise

def follow_trajectory(drone_position, trajectory, i):
    try:
        if not trajectory:
            logger.debug(f"Drone {i} trajectory empty, reached initial position")
            return None, None
        target_point = trajectory[-1]
        if equal(drone_position["x"], target_point["x"]) and equal(drone_position["z"], target_point["z"]):
            logger.debug(f"Drone {i} reached trajectory point {target_point}, removing it")
            TRAJECTORIES[i].pop()
            return follow_trajectory(drone_position, trajectory, i)
        logger.debug(f"Drone {i} moving to trajectory point {target_point}")
        return get_direction(drone_position, target_point, {}, i)
    except Exception as e:
        logger.error(f"Error in follow_trajectory for drone {i}: {e}")
        raise

def assign_next_target(i, targets):
    try:
        logger.debug(f"Assigning next target for drone {i}")
        global CURRENT_TARGETS, FIRES_STATUS
        for idx, status in enumerate(FIRES_STATUS):
            if not status and targets[idx] is not None:
                CURRENT_TARGETS[i] = targets[idx]
                logger.info(f"Drone {i} assigned to new target: {CURRENT_TARGETS[i]}")
                return True
        CURRENT_TARGETS[i] = None
        logger.info(f"Drone {i} has no new targets, hovering")
        return False
    except Exception as e:
        logger.error(f"Error in assign_next_target for drone {i}: {e}")
        raise

def next_step(targets, iter, initial_positions):
    try:
        logger.debug(f"Next step iteration {iter}")
        global TAG, ITER, IS_X, RETURNING, INITIAL_POSITIONS, TRAJECTORIES, CURRENT_TARGETS, FIRES_STATUS
        try:
            data = get_data(connection.receive_data())
            logger.debug(f"Received data: {data}")
        except Exception as e:
            logger.error(f"Connection error: {e}")
            return [0 for _ in range(len(targets))]

        fires = [0 for _ in range(len(data))]
        result = []

        if iter == 0:
            global FIRES_STATUS
            FIRES_STATUS = [False for _ in range(len(targets))]
            logger.debug(f"Initialized FIRES_STATUS: {FIRES_STATUS}")
            for i, drone in enumerate(data):
                if i < len(INITIAL_POSITIONS):
                    INITIAL_POSITIONS[i] = {"x": drone["droneVector"]["x"], "z": drone["droneVector"]["z"]}
                    TRAJECTORIES[i] = [INITIAL_POSITIONS[i]]
                    logger.info(f"Drone {i} initial position: {INITIAL_POSITIONS[i]}")
                    if i < len(targets) and targets[i] is not None:
                        CURRENT_TARGETS[i] = targets[i]
                        logger.info(f"Drone {i} assigned initial target: {CURRENT_TARGETS[i]}")
                    else:
                        CURRENT_TARGETS[i] = None
                        logger.warning(f"Drone {i} has no initial target")

        for i, drone in enumerate(data):
            if i >= len(ITER):
                logger.warning(f"Drone {i} inactive, hovering at height {H}")
                new_data = move("f", drone, 0, H)
                result.append(new_data)
                continue

            if not RETURNING[i] and CURRENT_TARGETS[i] is not None:
                current_pos = {"x": drone["droneVector"]["x"], "z": drone["droneVector"]["z"]}
                if not TRAJECTORIES[i] or not (equal(current_pos["x"], TRAJECTORIES[i][-1]["x"]) and
                                              equal(current_pos["z"], TRAJECTORIES[i][-1]["z"])):
                    TRAJECTORIES[i].append(current_pos)
                    logger.debug(f"Drone {i} added position to trajectory: {current_pos}")

            if RETURNING[i]:
                logger.debug(f"Drone {i} returning, following trajectory")
                direction, TAG[i] = follow_trajectory(drone["droneVector"], TRAJECTORIES[i], i)
            else:
                if CURRENT_TARGETS[i] is None:
                    logger.warning(f"Drone {i} has no target, hovering")
                    new_data = move("f", drone, 0, H)
                    result.append(new_data)
                    continue
                logger.debug(f"Drone {i} moving to target: {CURRENT_TARGETS[i]}")
                if TAG[i] is None:
                    direction, TAG[i] = get_direction(drone["droneVector"], CURRENT_TARGETS[i], drone["lidarInfo"], i)
                elif "x" in TAG[i]:
                    direction, TAG[i] = go_x(drone["droneVector"], CURRENT_TARGETS[i], drone["lidarInfo"], TAG[i][1])
                    IS_X[i] = False
                elif "z" in TAG[i]:
                    direction, TAG[i] = go_z(drone["droneVector"], CURRENT_TARGETS[i], drone["lidarInfo"], TAG[i][1])

            if ITER[i] > iter:
                logger.debug(f"Drone {i} waiting for activation (iter={iter}, activation={ITER[i]})")
                new_data = move("f", drone, 0, 0)
            elif direction is None:
                if not RETURNING[i]:
                    logger.info(f"Drone {i} reached fire, extinguishing and starting return")
                    new_data = move("f", drone, 0, H, drop=True)
                    fires[i] = 1
                    for idx, target in enumerate(targets):
                        if target == CURRENT_TARGETS[i]:
                            FIRES_STATUS[idx] = True
                            logger.info(f"Fire {idx} extinguished")
                            break
                    RETURNING[i] = True
                    TAG[i] = None
                    IS_X[i] = True
                else:
                    logger.info(f"Drone {i} reached initial position, refueling")
                    new_data = move("f", drone, 0, H)
                    RETURNING[i] = False
                    TAG[i] = None
                    IS_X[i] = True
                    TRAJECTORIES[i] = [INITIAL_POSITIONS[i]]
                    if assign_next_target(i, targets):
                        logger.info(f"Drone {i} refueled and assigned new target: {CURRENT_TARGETS[i]}")
                    else:
                        logger.info(f"Drone {i} refueled, no new targets, hovering")
            else:
                logger.debug(f"Drone {i} moving in direction {direction}")
                new_data = move(direction, drone, ANGLE, H)

            result.append(new_data)

        try:
            logger.debug("Sending data to connection")
            connection.send_data(concat_engines(result, T))
        except Exception as e:
            logger.error(f"Failed to send data: {e}")
        time.sleep(T)
        logger.debug(f"Completed next_step iteration {iter}")
        return fires
    except Exception as e:
        logger.error(f"Error in next_step: {e}")
        raise

def run(targets):
    try:
        logger.debug("Starting drone control run")
        global TRAJECTORIES, FIRES_STATUS
        logger.info(f"Starting run with targets: {targets}")
        i = 0
        fires = next_step(targets, i, INITIAL_POSITIONS)
        while not all(FIRES_STATUS) or not all_at_initial_positions():
            fires = next_step(targets, i, INITIAL_POSITIONS)
            i += 1
        logger.info(f"Run completed. All fires extinguished. Final trajectories: {TRAJECTORIES}")
    except Exception as e:
        logger.error(f"Error in run: {e}")
        raise

def all_at_initial_positions():
    try:
        logger.debug("Checking if all drones are at initial positions")
        global INITIAL_POSITIONS, RETURNING, CURRENT_TARGETS
        try:
            data = get_data(connection.receive_data())
            logger.debug(f"Received data for position check: {data}")
        except Exception as e:
            logger.error(f"Connection error in all_at_initial_positions: {e}")
            return False

        for i, drone in enumerate(data):
            if i >= len(INITIAL_POSITIONS):
                continue
            if RETURNING[i] or CURRENT_TARGETS[i] is None:
                initial = INITIAL_POSITIONS[i]
                current = drone["droneVector"]
                if not (equal(current["x"], initial["x"]) and equal(current["z"], initial["z"])):
                    logger.debug(f"Drone {i} not at initial position: current={current}, initial={initial}")
                    return False
                else:
                    logger.debug(f"Drone {i} at initial position: {current}")
        return True
    except Exception as e:
        logger.error(f"Error in all_at_initial_positions: {e}")
        raise

def go_x(drone_position, target_position, lidars, direction):
    try:
        directions1 = get_direction1(drone_position["z"], target_position["z"])
        if check_lidars(directions1, lidars):
            logger.debug(f"Switching to Z direction: {directions1[0]}")
            return directions1[0], None
        logger.debug(f"Continuing in X direction: {direction}")
        return direction, "x" + direction
    except Exception as e:
        logger.error(f"Error in go_x: {e}")
        raise

def go_z(drone_position, target_position, lidars, direction):
    try:
        directions2 = get_direction2(drone_position["x"], target_position["x"])
        if check_lidars(directions2, lidars):
            logger.debug(f"Switching to X direction: {directions2[0]}")
            return directions2[0], None
        logger.debug(f"Continuing in Z direction: {direction}")
        return direction, "z" + direction
    except Exception as e:
        logger.error(f"Error in go_z: {e}")
        raise

def get_direction(drone_position, target_position, lidars, i):
    try:
        global IS_X
        if target_position is None:
            logger.warning(f"Drone {i} has no target, hovering")
            return None, None
        if equal(drone_position["z"], target_position["z"]):
            IS_X[i] = True
            logger.debug(f"Drone {i} aligned on Z, enabling X alignment")
        if not equal(drone_position["x"], target_position["x"]) and IS_X[i]:
            directions2 = get_direction2(drone_position["x"], target_position["x"])
            if not lidars or check_lidars(directions2, lidars):
                logger.debug(f"Drone {i} moving in X direction: {directions2[0]}")
                return directions2[0], None
            directions1 = get_direction1(drone_position["z"], target_position["z"])
            logger.debug(f"Drone {i} obstructed in X, moving in Z: {directions1[0]}")
            return directions1[0], "z" + directions1[0]
        if not equal(drone_position["z"], target_position["z"]):
            directions1 = get_direction1(drone_position["z"], target_position["z"])
            if not lidars or check_lidars(directions1, lidars):
                logger.debug(f"Drone {i} moving in Z direction: {directions1[0]}")
                return directions1[0], None
            directions2 = get_direction2(drone_position["x"], target_position["x"])
            logger.debug(f"Drone {i} obstructed in Z, moving in X: {directions2[0]}")
            return directions2[0], "x" + directions2[0]
        logger.debug(f"Drone {i} reached target: {target_position}")
        return None, None
    except Exception as e:
        logger.error(f"Error in get_direction for drone {i}: {e}")
        raise



# import logging
# import time
# from pathlib import Path
# from algorithm.PID import move, get_data, equal, concat_engines
# from connection.SocketConnection import SocketConnection
#
# # Предполагаем, что LOG_DIR определен
# LOG_DIR = "logs/"
#
# def set_logger_config(log_level="INFO"):
#     """Настройка логирования с перезаписью файла"""
#     Path(LOG_DIR).mkdir(parents=True, exist_ok=True)
#
#     level = logging.INFO
#     filename = "info.log"
#
#     if log_level == "DEBUG":
#         level = logging.DEBUG
#         filename = "debug.log"
#
#     logging.basicConfig(
#         level=level,
#         format="%(asctime)s %(levelname)s %(message)s",
#         datefmt="%Y-%m-%d %H:%M:%S",
#         filename=str(Path(LOG_DIR) / filename),
#         filemode="w"
#     )
#
# # Инициализация логгера
# set_logger_config("DEBUG")
#
# connection = SocketConnection()
# H = 8  # высота, на которой летит дрон
# T = 0.1  # время, через которое симулятор пришлет новый пакет данных
# ANGLE = 5  # угол наклона дрона
# ITER = [0, 300, 600, 900, 1200]  # задержка для следующих дронов
#
# # Списки для состояния дронов (максимум 5 дронов)
# TAG = [None for _ in range(5)]
# IS_X = [True for _ in range(5)]
# INITIAL_POSITIONS = [None for _ in range(5)]
# RETURNING = [False for _ in range(5)]
# TRAJECTORIES = [[] for _ in range(5)]  # Список траекторий для каждого дрона
# CURRENT_TARGETS = [None for _ in range(5)]  # Текущие цели для каждого дрона
# FIRES_STATUS = []  # Статус очагов: True - потушен, False - активен
#
# def check_lidars(directions, lidars):
#     """Проверка лидаров. True - если можно лететь, False - если рядом преграда"""
#     if 0 < lidars.get(directions[0], float('inf')) < 5:
#         logging.debug(f"Obstacle detected in main direction {directions[0]}: {lidars.get(directions[0])}")
#         return False
#     for direction in directions[1:]:
#         if 0 < lidars.get(direction, float('inf')) < 3:
#             logging.debug(f"Obstacle detected in side direction {direction}: {lidars.get(direction)}")
#             return False
#     logging.debug(f"Path clear for directions {directions}")
#     return True
#
# def get_direction1(drone_z, fire_z):
#     """Направление по оси Z: f - вперед, b - назад"""
#     direction = "b"
#     if drone_z - fire_z > 0:
#         direction = "f"
#     logging.debug(f"Z direction: {direction} (drone_z={drone_z}, fire_z={fire_z})")
#     return direction, direction + "r", direction + "l"
#
# def get_direction2(drone_x, fire_x):
#     """Направление по оси X: r - вправо, l - влево"""
#     direction = "l"
#     if drone_x - fire_x > 0:
#         direction = "r"
#     logging.debug(f"X direction: {direction} (drone_x={drone_x}, fire_x={fire_x})")
#     return direction, "f" + direction, "b" + direction
#
# def follow_trajectory(drone_position, trajectory, i):
#     """Следование по сохраненной траектории в обратном порядке"""
#     if not trajectory:
#         logging.debug(f"Drone {i} trajectory empty, reached initial position")
#         return None, None
#     target_point = trajectory[-1]
#     if equal(drone_position["x"], target_point["x"]) and equal(drone_position["z"], target_point["z"]):
#         logging.debug(f"Drone {i} reached trajectory point {target_point}, removing it")
#         TRAJECTORIES[i].pop()
#         return follow_trajectory(drone_position, trajectory, i)
#     logging.debug(f"Drone {i} moving to trajectory point {target_point}")
#     return get_direction(drone_position, target_point, {}, i)
#
# def assign_next_target(i):
#     """Назначает дрона на следующий непотушенный очаг"""
#     global CURRENT_TARGETS, FIRES_STATUS
#     for idx, status in enumerate(FIRES_STATUS):
#         if not status:  # Очаг не потушен
#             CURRENT_TARGETS[i] = targets[idx]
#             logging.info(f"Drone {i} assigned to new target: {CURRENT_TARGETS[i]}")
#             return True
#     CURRENT_TARGETS[i] = None
#     logging.info(f"Drone {i} has no new targets, hovering")
#     return False
#
# def next_step(targets, iter, initial_positions):
#     """Один шаг управления дронами"""
#     global TAG, ITER, IS_X, RETURNING, INITIAL_POSITIONS, TRAJECTORIES, CURRENT_TARGETS, FIRES_STATUS
#     try:
#         data = get_data(connection.receive_data())
#     except Exception as e:
#         logging.error(f"Connection error: {e}")
#         return [0 for _ in range(len(targets))]
#
#     fires = [0 for _ in range(len(data))]
#     result = []
#
#     # Инициализация на первой итерации
#     if iter == 0:
#         global FIRES_STATUS
#         FIRES_STATUS = [False for _ in range(len(targets))]  # Инициализируем статус очагов
#         for i, drone in enumerate(data):
#             if i < len(INITIAL_POSITIONS):
#                 INITIAL_POSITIONS[i] = {"x": drone["droneVector"]["x"], "z": drone["droneVector"]["z"]}
#                 TRAJECTORIES[i] = [INITIAL_POSITIONS[i]]
#                 logging.info(f"Drone {i} initial position: {INITIAL_POSITIONS[i]}")
#                 # Назначаем начальные цели
#                 if i < len(targets) and targets[i] is not None:
#                     CURRENT_TARGETS[i] = targets[i]
#                     logging.info(f"Drone {i} assigned initial target: {CURRENT_TARGETS[i]}")
#                 else:
#                     CURRENT_TARGETS[i] = None
#                     logging.warning(f"Drone {i} has no initial target")
#
#     for i, drone in enumerate(data):
#         if i >= len(ITER):
#             logging.warning(f"Drone {i} inactive, hovering at height {H}")
#             new_data = move("f", drone, 0, H)
#             result.append(new_data)
#             continue
#
#         # Записываем текущую позицию в траекторию, если дрон не возвращается
#         if not RETURNING[i] and CURRENT_TARGETS[i] is not None:
#             current_pos = {"x": drone["droneVector"]["x"], "z": drone["droneVector"]["z"]}
#             if not TRAJECTORIES[i] or not (equal(current_pos["x"], TRAJECTORIES[i][-1]["x"]) and
#                                           equal(current_pos["z"], TRAJECTORIES[i][-1]["z"])):
#                 TRAJECTORIES[i].append(current_pos)
#                 logging.debug(f"Drone {i} added position to trajectory: {current_pos}")
#
#         # Определяем текущую цель
#         if RETURNING[i]:
#             logging.debug(f"Drone {i} returning, following trajectory")
#             direction, TAG[i] = follow_trajectory(drone["droneVector"], TRAJECTORIES[i], i)
#         else:
#             if CURRENT_TARGETS[i] is None:
#                 logging.warning(f"Drone {i} has no target, hovering")
#                 new_data = move("f", drone, 0, H)
#                 result.append(new_data)
#                 continue
#             logging.debug(f"Drone {i} moving to target: {CURRENT_TARGETS[i]}")
#             if TAG[i] is None:
#                 direction, TAG[i] = get_direction(drone["droneVector"], CURRENT_TARGETS[i], drone["lidarInfo"], i)
#             elif "x" in TAG[i]:
#                 direction, TAG[i] = go_x(drone["droneVector"], CURRENT_TARGETS[i], drone["lidarInfo"], TAG[i][1])
#                 IS_X[i] = False
#             elif "z" in TAG[i]:
#                 direction, TAG[i] = go_z(drone["droneVector"], CURRENT_TARGETS[i], drone["lidarInfo"], TAG[i][1])
#
#         if ITER[i] > iter:
#             logging.debug(f"Drone {i} waiting for activation (iter={iter}, activation={ITER[i]})")
#             new_data = move("f", drone, 0, 0)
#         elif direction is None:
#             if not RETURNING[i]:
#                 # Дрон достиг очага
#                 logging.info(f"Drone {i} reached fire, extinguishing and starting return")
#                 new_data = move("f", drone, 0, H, drop=True)
#                 fires[i] = 1
#                 # Отмечаем очаг как потушенный
#                 for idx, target in enumerate(targets):
#                     if target == CURRENT_TARGETS[i]:
#                         FIRES_STATUS[idx] = True
#                         logging.info(f"Fire {idx} extinguished")
#                         break
#                 RETURNING[i] = True
#                 TAG[i] = None
#                 IS_X[i] = True
#             else:
#                 # Дрон достиг исходной позиции
#                 logging.info(f"Drone {i} reached initial position, refueling")
#                 new_data = move("f", drone, 0, H)
#                 # Заправка и назначение нового очага
#                 RETURNING[i] = False
#                 TAG[i] = None
#                 IS_X[i] = True
#                 TRAJECTORIES[i] = [INITIAL_POSITIONS[i]]  # Сбрасываем траекторию
#                 if assign_next_target(i):
#                     logging.info(f"Drone {i} refueled and assigned new target: {CURRENT_TARGETS[i]}")
#                 else:
#                     logging.info(f"Drone {i} refueled, no new targets, hovering")
#         else:
#             logging.debug(f"Drone {i} moving in direction {direction}")
#             new_data = move(direction, drone, ANGLE, H)
#
#         result.append(new_data)
#
#     try:
#         connection.send_data(concat_engines(result, T))
#     except Exception as e:
#         logging.error(f"Failed to send data: {e}")
#     time.sleep(T)
#     return fires
#
# def run(targets):
#     """Основной цикл управления дронами"""
#     global TRAJECTORIES, FIRES_STATUS
#     logging.info(f"Starting run with targets: {targets}")
#     i = 0
#     fires = next_step(targets, i, INITIAL_POSITIONS)
#     # Продолжаем, пока не потушены все очаги или дроны не вернулись
#     while not all(FIRES_STATUS) or not all_at_initial_positions():
#         fires = next_step(targets, i, INITIAL_POSITIONS)
#         i += 1
#     logging.info(f"Run completed. All fires extinguished. Final trajectories: {TRAJECTORIES}")
#
# def all_at_initial_positions():
#     """Проверяет, вернулись ли все дроны в исходные позиции, если нет активных целей"""
#     global INITIAL_POSITIONS, RETURNING, CURRENT_TARGETS
#     try:
#         data = get_data(connection.receive_data())
#     except Exception as e:
#         logging.error(f"Connection error in all_at_initial_positions: {e}")
#         return False
#
#     for i, drone in enumerate(data):
#         if i >= len(INITIAL_POSITIONS):
#             continue
#         if RETURNING[i] or CURRENT_TARGETS[i] is None:
#             initial = INITIAL_POSITIONS[i]
#             current = drone["droneVector"]
#             if not (equal(current["x"], initial["x"]) and equal(current["z"], initial["z"])):
#                 logging.debug(f"Drone {i} not at initial position: current={current}, initial={initial}")
#                 return False
#             else:
#                 logging.debug(f"Drone {i} at initial position: {current}")
#     return True
#
# def go_x(drone_position, target_position, lidars, direction):
#     directions1 = get_direction1(drone_position["z"], target_position["z"])
#     if check_lidars(directions1, lidars):
#         logging.debug(f"Switching to Z direction: {directions1[0]}")
#         return directions1[0], None
#     logging.debug(f"Continuing in X direction: {direction}")
#     return direction, "x" + direction
#
# def go_z(drone_position, target_position, lidars, direction):
#     directions2 = get_direction2(drone_position["x"], target_position["x"])
#     if check_lidars(directions2, lidars):
#         logging.debug(f"Switching to X direction: {directions2[0]}")
#         return directions2[0], None
#     logging.debug(f"Continuing in Z direction: {direction}")
#     return direction, "z" + direction
#
# def get_direction(drone_position, target_position, lidars, i):
#     global IS_X
#     if target_position is None:
#         logging.warning(f"Drone {i} has no target, hovering")
#         return None, None
#     if equal(drone_position["z"], target_position["z"]):
#         IS_X[i] = True
#         logging.debug(f"Drone {i} aligned on Z, enabling X alignment")
#     if not equal(drone_position["x"], target_position["x"]) and IS_X[i]:
#         directions2 = get_direction2(drone_position["x"], target_position["x"])
#         if not lidars or check_lidars(directions2, lidars):
#             logging.debug(f"Drone {i} moving in X direction: {directions2[0]}")
#             return directions2[0], None
#         directions1 = get_direction1(drone_position["z"], target_position["z"])
#         logging.debug(f"Drone {i} obstructed in X, moving in Z: {directions1[0]}")
#         return directions1[0], "z" + directions1[0]
#     if not equal(drone_position["z"], target_position["z"]):
#         directions1 = get_direction1(drone_position["z"], target_position["z"])
#         if not lidars or check_lidars(directions1, lidars):
#             logging.debug(f"Drone {i} moving in Z direction: {directions1[0]}")
#             return directions1[0], None
#         directions2 = get_direction2(drone_position["x"], target_position["x"])
#         logging.debug(f"Drone {i} obstructed in Z, moving in X: {directions2[0]}")
#         return directions2[0], "x" + directions2[0]
#     logging.debug(f"Drone {i} reached target: {target_position}")
#     return None, None