import time
import math
import logging
import json

from algorithm.PID import move, get_data, equal, concat_engines
from connection.SocketConnection import SocketConnection

# Настройка логирования
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(levelname)s %(message)s')

connection = SocketConnection()
H = 8  # высота полета
T = 0.1  # интервал симулятора
ANGLE = 3  # уменьшенный угол наклона для стабильности
ITER = [0, 500, 1000, 1500, 2000]  # увеличенные задержки
MIN_DRONE_DISTANCE = 8  # увеличенное расстояние
STABILIZATION_STEPS = 100  # шаги стабилизации
GROUND_THRESHOLD = 0.5  # минимальная высота

# Хранение состояний
BASE_POSITIONS = [None for _ in range(5)]  # начальные позиции
DRONE_STATE = ["STABILIZE" for _ in range(5)]  # состояния: STABILIZE, TO_FIRE, RETURN
TAG = [None for _ in range(5)]  # тег направления
IS_X = [True for _ in range(5)]  # равняем ли X?
TRAJECTORIES = [[] for _ in range(5)]  # траектории дронов
STABILIZATION_COUNTER = [0 for _ in range(5)]  # счетчик шагов стабилизации


def check_lidars(directions, lidars):
    """Проверка лидаров. True - можно лететь, False - препятствие"""
    if directions[0] in lidars and 0 < lidars[directions[0]] < 5:
        return False
    for direction in directions[1:]:
        if direction in lidars and 0 < lidars[direction] < 3:
            return False
    return True


def check_ground_collision(drone):
    """Проверка столкновения с землей"""
    if drone["droneVector"]["y"] < GROUND_THRESHOLD or ("d" in drone["lidarInfo"] and 0 < drone["lidarInfo"]["d"] < 1):
        logging.warning(
            f"Drone {drone['id']} too close to ground: y={drone['droneVector']['y']}, lidar_d={drone['lidarInfo'].get('d', 0)}")
        return True
    return False


def get_avoidance_direction(current_pos, other_pos, current_id, other_id):
    """Направление для избегания столкновения"""
    dx = current_pos["x"] - other_pos["x"]
    dz = current_pos["z"] - other_pos["z"]
    if current_id > other_id:
        if abs(dx) > abs(dz):
            return "r" if dx > 0 else "l"
        else:
            return "f" if dz > 0 else "b"
    return None


def check_drone_collision(drone_positions, current_drone_idx):
    """Проверка и коррекция столкновений"""
    current_pos = drone_positions[current_drone_idx]["droneVector"]
    avoidance_direction = None
    for i, drone in enumerate(drone_positions):
        if i == current_drone_idx:
            continue
        other_pos = drone["droneVector"]
        distance = math.sqrt(
            (current_pos["x"] - other_pos["x"]) ** 2 +
            (current_pos["z"] - other_pos["z"]) ** 2
        )
        if distance < MIN_DRONE_DISTANCE:
            logging.debug(f"Drone {current_drone_idx} too close to drone {i}: {distance}")
            avoidance_direction = get_avoidance_direction(current_pos, other_pos, current_drone_idx, i)
            if avoidance_direction:
                break
    return avoidance_direction


def get_direction1(drone_z, fire_z):
    """Направление по Z: f - вперед, b - назад"""
    direction = "f" if drone_z - fire_z > 0 else "b"
    return direction, direction + "r", direction + "l"


def get_direction2(drone_x, fire_x):
    """Направление по X: r - вправо, l - влево"""
    direction = "r" if drone_x - fire_x > 0 else "l"
    return direction, "f" + direction, "b" + direction


def save_trajectory(drone_idx, position):
    """Сохранение текущей позиции в траекторию дрона"""
    if not TRAJECTORIES[drone_idx] or TRAJECTORIES[drone_idx][-1] != position:
        TRAJECTORIES[drone_idx].append(position)
        logging.debug(f"Drone {drone_idx} trajectory updated: {position}")


def save_trajectories_to_file():
    """Сохранение траекторий в файл"""
    with open("drone_trajectories.json", "w") as f:
        json.dump({f"drone_{i}": TRAJECTORIES[i] for i in range(5)}, f, indent=2)
    logging.info("Trajectories saved to drone_trajectories.json")


def next_step(targets, iter):
    """Один шаг: анализ данных и отправка команд"""
    global TAG, ITER, IS_X, DRONE_STATE, BASE_POSITIONS, STABILIZATION_COUNTER

    data = get_data(connection.receive_data())

    # Сохранение начальных позиций
    if BASE_POSITIONS[0] is None:
        for i, drone in enumerate(data):
            BASE_POSITIONS[i] = {
                "x": drone["droneVector"]["x"],
                "y": drone["droneVector"]["y"],
                "z": drone["droneVector"]["z"]
            }
            logging.info(f"Drone {i} base position: {BASE_POSITIONS[i]}")

    fires = [0 for _ in range(len(targets))]
    result = []

    for i, drone in enumerate(data):
        # Сохранение текущей позиции в траекторию
        current_pos = {
            "x": drone["droneVector"]["x"],
            "y": drone["droneVector"]["y"],
            "z": drone["droneVector"]["z"]
        }
        save_trajectory(i, current_pos)

        # Проверка на аварию
        if drone["isDroneCrushed"]:
            logging.warning(f"Drone {i} crashed!")
            new_data = move("f", drone, 0, 0)
            result.append(new_data)
            continue

        # Проверка столкновения с землей
        if check_ground_collision(drone):
            new_data = move("f", drone, 0, H)
            result.append(new_data)
            continue

        # Выбор цели в зависимости от состояния
        target = targets[i] if DRONE_STATE[i] == "TO_FIRE" else BASE_POSITIONS[i]
        logging.debug(f"Drone {i}: state={DRONE_STATE[i]}, pos={current_pos}, target={target}")

        # Проверка на None в target
        if target is None:
            logging.error(f"Drone {i} has no valid target position")
            new_data = move("f", drone, 0, H)
            result.append(new_data)
            continue

        avoidance_direction = check_drone_collision(data, i)

        if DRONE_STATE[i] == "STABILIZE":
            new_data = move("f", drone, 0, H)
            STABILIZATION_COUNTER[i] += 1
            if STABILIZATION_COUNTER[i] >= STABILIZATION_STEPS and abs(drone["droneVector"]["y"] - H) < 0.5:
                DRONE_STATE[i] = "TO_FIRE"
                logging.info(f"Drone {i} stabilized at y={drone['droneVector']['y']}, moving to fire")
            result.append(new_data)

        elif DRONE_STATE[i] == "TO_FIRE":
            if ITER[i] > iter:
                new_data = move("f", drone, 0, H)
            elif avoidance_direction:
                new_data = move(avoidance_direction, drone, ANGLE, H)
                logging.debug(f"Drone {i} avoiding collision, direction={avoidance_direction}")
            elif TAG[i] is None:
                direction, TAG[i] = get_direction(drone["droneVector"], targets[i], drone["lidarInfo"], i)
                new_data = move(direction, drone, ANGLE, H) if direction else move("f", drone, 0, H)
            elif "x" in TAG[i]:
                direction, TAG[i] = go_x(drone["droneVector"], targets[i], drone["lidarInfo"], TAG[i][1])
                IS_X[i] = False
                new_data = move(direction, drone, ANGLE, H) if direction else move("f", drone, 0, H)
            elif "z" in TAG[i]:
                direction, TAG[i] = go_z(drone["droneVector"], targets[i], drone["lidarInfo"], TAG[i][1])
                new_data = move(direction, drone, ANGLE, H) if direction else move("f", drone, 0, H)

            if equal(drone["droneVector"]["x"], targets[i]["x"]) and equal(drone["droneVector"]["z"], targets[i]["z"]):
                new_data = move("f", drone, 0, H, drop=True)
                fires[i] = 1
                DRONE_STATE[i] = "RETURN"
                TAG[i] = None
                IS_X[i] = True
                logging.info(f"Drone {i} extinguished fire at {targets[i]}, returning to base")
            result.append(new_data)

        elif DRONE_STATE[i] == "RETURN":
            if equal(drone["droneVector"]["x"], BASE_POSITIONS[i]["x"]) and equal(drone["droneVector"]["z"],
                                                                                  BASE_POSITIONS[i]["z"]):
                new_data = move("f", drone, 0, 0)
                fires[i] = 1
                logging.info(f"Drone {i} returned to base {BASE_POSITIONS[i]}")
            else:
                if avoidance_direction:
                    new_data = move(avoidance_direction, drone, ANGLE, H)
                    logging.debug(f"Drone {i} avoiding collision, direction={avoidance_direction}")
                else:
                    direction, TAG[i] = get_direction(drone["droneVector"], BASE_POSITIONS[i], drone["lidarInfo"], i)
                    new_data = move(direction, drone, ANGLE, H) if direction else move("f", drone, 0, H)
            result.append(new_data)

    connection.send_data(concat_engines(result, T))
    time.sleep(T)

    return fires


def run(targets):
    """Основной цикл"""
    i = 0
    fires = next_step(targets, i)
    while sum(fires) != len(targets):
        fires = next_step(targets, i)
        i += 1
        logging.debug(f"Iteration {i}, fires extinguished: {sum(fires)}/{len(targets)}")
    save_trajectories_to_file()


def go_x(drone_position, target_position, lidars, direction):
    """Движение по Z, если нельзя, то по X"""
    directions1 = get_direction1(drone_position["z"], target_position["z"])
    if check_lidars(directions1, lidars):
        return directions1[0], None
    return direction, "x" + direction


def go_z(drone_position, target_position, lidars, direction):
    """Движение по X, если нельзя, то по Z"""
    directions2 = get_direction2(drone_position["x"], target_position["x"])
    if check_lidars(directions2, lidars):
        return directions2[0], None
    return direction, "z" + direction


def get_direction(drone_position, target_position, lidars, i):
    """Определение направления полета"""
    global IS_X

    if target_position is None:
        logging.error(f"Target position for drone {i} is None")
        return None, None

    if equal(drone_position["z"], target_position["z"]):
        IS_X[i] = True

    if not equal(drone_position["x"], target_position["x"]) and IS_X[i]:
        directions2 = get_direction2(drone_position["x"], target_position["x"])
        if check_lidars(directions2, lidars):
            return directions2[0], None
        directions1 = get_direction1(drone_position["z"], target_position["z"])
        return directions1[0], "z" + directions1[0]

    if not equal(drone_position["z"], target_position["z"]):
        directions1 = get_direction1(drone_position["z"], target_position["z"])
        if check_lidars(directions1, lidars):
            return directions1[0], None
        directions2 = get_direction2(drone_position["x"], target_position["x"])
        return directions2[0], "x" + directions2[0]

    return None, None


def equal(a, b):
    return abs(a - b) < 0.5  # уменьшенный допуск для точности