import logging
import time
from pathlib import Path
from algorithm.PID import move, get_data, equal, concat_engines
from connection.SocketConnection import SocketConnection

# Предполагаем, что LOG_DIR определен
LOG_DIR = "logs/"  # Замените на актуальное значение из config

def set_logger_config(log_level="INFO"):
    """Настройка логирования с перезаписью файла"""
    Path(LOG_DIR).mkdir(parents=True, exist_ok=True)

    level = logging.INFO
    filename = "info.log"

    if log_level == "DEBUG":
        level = logging.DEBUG
        filename = "debug.log"

    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        filename=str(Path(LOG_DIR) / filename),
        filemode="w"  # Перезаписываем файл при каждом запуске
    )

# Инициализация логгера
set_logger_config("DEBUG")

connection = SocketConnection()
H = 8  # высота, на которой летит дрон
T = 0.1  # время, через которое симулятор пришлет новый пакет данных
ANGLE = 5  # угол наклона дрона
ITER = [0, 300, 600, 900, 1200]  # задержка для следующих дронов

# Списки для состояния дронов (максимум 5 дронов)
TAG = [None for _ in range(5)]
IS_X = [True for _ in range(5)]
INITIAL_POSITIONS = [None for _ in range(5)]
RETURNING = [False for _ in range(5)]
TRAJECTORIES = [[] for _ in range(5)]  # Список траекторий для каждого дрона

def check_lidars(directions, lidars):
    """Проверка лидаров. True - если можно лететь, False - если рядом преграда"""
    if 0 < lidars.get(directions[0], float('inf')) < 5:
        logging.debug(f"Obstacle detected in main direction {directions[0]}: {lidars.get(directions[0])}")
        return False
    for direction in directions[1:]:
        if 0 < lidars.get(direction, float('inf')) < 3:
            logging.debug(f"Obstacle detected in side direction {direction}: {lidars.get(direction)}")
            return False
    logging.debug(f"Path clear for directions {directions}")
    return True

def get_direction1(drone_z, fire_z):
    """Направление по оси Z: f - вперед, b - назад"""
    direction = "b"
    if drone_z - fire_z > 0:
        direction = "f"
    logging.debug(f"Z direction: {direction} (drone_z={drone_z}, fire_z={fire_z})")
    return direction, direction + "r", direction + "l"

def get_direction2(drone_x, fire_x):
    """Направление по оси X: r - вправо, l - влево"""
    direction = "l"
    if drone_x - fire_x > 0:
        direction = "r"
    logging.debug(f"X direction: {direction} (drone_x={drone_x}, fire_x={fire_x})")
    return direction, "f" + direction, "b" + direction

def follow_trajectory(drone_position, trajectory, i):
    """Следование по сохраненной траектории в обратном порядке"""
    if not trajectory:
        logging.debug(f"Drone {i} trajectory empty, reached initial position")
        return None, None
    target_point = trajectory[-1]
    if equal(drone_position["x"], target_point["x"]) and equal(drone_position["z"], target_point["z"]):
        logging.debug(f"Drone {i} reached trajectory point {target_point}, removing it")
        TRAJECTORIES[i].pop()
        return follow_trajectory(drone_position, trajectory, i)
    logging.debug(f"Drone {i} moving to trajectory point {target_point}")
    return get_direction(drone_position, target_point, {}, i)

def next_step(targets, iter, initial_positions):
    """Один шаг управления дронами"""
    global TAG, ITER, IS_X, RETURNING, INITIAL_POSITIONS, TRAJECTORIES
    try:
        data = get_data(connection.receive_data())
    except Exception as e:
        logging.error(f"Connection error: {e}")
        return [0 for _ in range(len(targets))]

    fires = [0 for _ in range(len(data))]
    result = []

    # Сохраняем исходные позиции на первой итерации
    if iter == 0:
        for i, drone in enumerate(data):
            if i < len(INITIAL_POSITIONS):
                INITIAL_POSITIONS[i] = {"x": drone["droneVector"]["x"], "z": drone["droneVector"]["z"]}
                TRAJECTORIES[i] = [INITIAL_POSITIONS[i]]
                logging.info(f"Drone {i} initial position: {INITIAL_POSITIONS[i]}")

    for i, drone in enumerate(data):
        if i >= len(ITER) or (i < len(targets) and targets[i] is None):
            logging.warning(f"Drone {i} inactive or no target, hovering at height {H}")
            new_data = move("f", drone, 0, H)
            result.append(new_data)
            continue

        # Записываем текущую позицию в траекторию, если дрон не возвращается
        if not RETURNING[i]:
            current_pos = {"x": drone["droneVector"]["x"], "z": drone["droneVector"]["z"]}
            if not TRAJECTORIES[i] or not (equal(current_pos["x"], TRAJECTORIES[i][-1]["x"]) and
                                          equal(current_pos["z"], TRAJECTORIES[i][-1]["z"])):
                TRAJECTORIES[i].append(current_pos)
                logging.debug(f"Drone {i} added position to trajectory: {current_pos}")

        # Определяем текущую цель: пожар или траектория
        if RETURNING[i]:
            logging.debug(f"Drone {i} returning, following trajectory")
            direction, TAG[i] = follow_trajectory(drone["droneVector"], TRAJECTORIES[i], i)
        else:
            current_target = targets[i]
            logging.debug(f"Drone {i} moving to target: {current_target}")
            if TAG[i] is None:
                direction, TAG[i] = get_direction(drone["droneVector"], current_target, drone["lidarInfo"], i)
            elif "x" in TAG[i]:
                direction, TAG[i] = go_x(drone["droneVector"], current_target, drone["lidarInfo"], TAG[i][1])
                IS_X[i] = False
            elif "z" in TAG[i]:
                direction, TAG[i] = go_z(drone["droneVector"], current_target, drone["lidarInfo"], TAG[i][1])

        if ITER[i] > iter:
            logging.debug(f"Drone {i} waiting for activation (iter={iter}, activation={ITER[i]})")
            new_data = move("f", drone, 0, 0)
        elif direction is None:
            if not RETURNING[i]:
                logging.info(f"Drone {i} reached fire, extinguishing and starting return")
                new_data = move("f", drone, 0, H, drop=True)
                fires[i] = 1
                RETURNING[i] = True
                TAG[i] = None
                IS_X[i] = True
            else:
                logging.info(f"Drone {i} reached initial position, hovering")
                new_data = move("f", drone, 0, H)
        else:
            logging.debug(f"Drone {i} moving in direction {direction}")
            new_data = move(direction, drone, ANGLE, H)

        result.append(new_data)

    try:
        connection.send_data(concat_engines(result, T))
    except Exception as e:
        logging.error(f"Failed to send data: {e}")
    time.sleep(T)
    return fires

def run(targets):
    """Основной цикл управления дронами"""
    global TRAJECTORIES
    logging.info(f"Starting run with targets: {targets}")
    i = 0
    fires = next_step(targets, i, INITIAL_POSITIONS)
    while not all(RETURNING) or not all_at_initial_positions():
        fires = next_step(targets, i, INITIAL_POSITIONS)
        i += 1
    logging.info(f"Run completed. Final trajectories: {TRAJECTORIES}")

def all_at_initial_positions():
    """Проверяет, вернулись ли все дроны в исходные позиции"""
    global INITIAL_POSITIONS, RETURNING
    try:
        data = get_data(connection.receive_data())
    except Exception as e:
        logging.error(f"Connection error in all_at_initial_positions: {e}")
        return False

    for i, drone in enumerate(data):
        if i >= len(INITIAL_POSITIONS):
            continue
        if RETURNING[i]:
            initial = INITIAL_POSITIONS[i]
            current = drone["droneVector"]
            if not (equal(current["x"], initial["x"]) and equal(current["z"], initial["z"])):
                logging.debug(f"Drone {i} not at initial position: current={current}, initial={initial}")
                return False
            else:
                logging.debug(f"Drone {i} at initial position: {current}")
    return True

def go_x(drone_position, target_position, lidars, direction):
    directions1 = get_direction1(drone_position["z"], target_position["z"])
    if check_lidars(directions1, lidars):
        logging.debug(f"Switching to Z direction: {directions1[0]}")
        return directions1[0], None
    logging.debug(f"Continuing in X direction: {direction}")
    return direction, "x" + direction

def go_z(drone_position, target_position, lidars, direction):
    directions2 = get_direction2(drone_position["x"], target_position["x"])
    if check_lidars(directions2, lidars):
        logging.debug(f"Switching to X direction: {directions2[0]}")
        return directions2[0], None
    logging.debug(f"Continuing in Z direction: {direction}")
    return direction, "z" + direction

def get_direction(drone_position, target_position, lidars, i):
    global IS_X
    if target_position is None:
        logging.warning(f"Drone {i} has no target, hovering")
        return None, None
    if equal(drone_position["z"], target_position["z"]):
        IS_X[i] = True
        logging.debug(f"Drone {i} aligned on Z, enabling X alignment")
    if not equal(drone_position["x"], target_position["x"]) and IS_X[i]:
        directions2 = get_direction2(drone_position["x"], target_position["x"])
        # Пропускаем проверку лидаров, если словарь пуст (возврат по траектории)
        if not lidars or check_lidars(directions2, lidars):
            logging.debug(f"Drone {i} moving in X direction: {directions2[0]}")
            return directions2[0], None
        directions1 = get_direction1(drone_position["z"], target_position["z"])
        logging.debug(f"Drone {i} obstructed in X, moving in Z: {directions1[0]}")
        return directions1[0], "z" + directions1[0]
    if not equal(drone_position["z"], target_position["z"]):
        directions1 = get_direction1(drone_position["z"], target_position["z"])
        # Пропускаем проверку лидаров, если словарь пуст (возврат по траектории)
        if not lidars or check_lidars(directions1, lidars):
            logging.debug(f"Drone {i} moving in Z direction: {directions1[0]}")
            return directions1[0], None
        directions2 = get_direction2(drone_position["x"], target_position["x"])
        logging.debug(f"Drone {i} obstructed in Z, moving in X: {directions2[0]}")
        return directions2[0], "x" + directions2[0]
    logging.debug(f"Drone {i} reached target: {target_position}")
    return None, None