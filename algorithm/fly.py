# import time
# import math
# from connection.SocketConnection import SocketConnection  # Добавлен импорт
# from algorithm.PID import move, get_data, concat_engines
#
# connection = SocketConnection()
# H = 8  # высота, на которой летит дрон
# T = 0.1  # время, через которое симулятор пришлет новый пакет данных
# ANGLE = 5  # стандартный угол наклона дрона
# PRECISION_ANGLE = 0.5  # угол для точной корректировки
# ITER = [0, 200, 400, 600, 800]  # задержка для следующих дронов
# LIDAR_THRESHOLD = 0.5  # порог для близких препятствий
# SAFE_LIDAR_THRESHOLD = 1.5  # порог для безопасных направлений
# PRECISION_THRESHOLD = 3.0  # порог расстояния для точной корректировки
# BYPASS_STEPS = 7  # максимальное количество шагов в обходе
# DANGER_COUNT_THRESHOLD = 2  # порог для сброса маршрута при риске
# DANGER_LIDAR_THRESHOLD = 0.7  # порог для опасных лидаров
#
# # Теги: None, "x<dir>", "z<dir>", "bypass", "precision", "return"
# TAG = [None for _ in range(5)]
#
# # Равняем ли координату X?
# IS_X = [True for _ in range(5)]
#
# # Список для хранения маршрутов каждого дрона
# PATHS = [[] for _ in range(5)]
#
# # Статус активации дронов
# IS_ACTIVE = [False for _ in range(5)]
#
# # Флаг для отслеживания сброса огнетушителя
# DROPPED = [False for _ in range(5)]
#
# # Статус возвращения
# IS_RETURNING = [False for _ in range(5)]
#
# # Индексы для возвращения по маршруту
# RETURN_INDICES = [0 for _ in range(5)]
#
# # Состояние обхода: {направление, шаги, исходное направление}
# BYPASS_STATE = [{} for _ in range(5)]  # Например, {"direction": "r", "steps": 7, "original": "f"}
#
# # Счетчик опасных шагов
# DANGER_COUNT = [0 for _ in range(5)]
#
# # Состояние точной корректировки: {оставшиеся шаги, этап}
# PRECISION_STATE = [{} for _ in range(5)]  # Например, {"steps": 30, "stage": "x"}
#
# def equal(a, b):
#     """Проверяет равенство с допуском 0.2 для высокой точности"""
#     return abs(a - b) < 0.2
#
# def calculate_distance(drone_position, target_position):
#     """Вычисляет евклидово расстояние до цели"""
#     dx = drone_position["x"] - target_position["x"]
#     dz = drone_position["z"] - target_position["z"]
#     return math.sqrt(dx**2 + dz**2)
#
# def get_perpendicular_directions(direction):
#     """Возвращает перпендикулярные направления"""
#     if direction == 'f':
#         return ['r', 'l']
#     if direction == 'b':
#         return ['r', 'l']
#     if direction == 'r':
#         return ['f', 'b']
#     if direction == 'l':
#         return ['f', 'b']
#     return []
#
# def reverse_direction(direction):
#     """Возвращает противоположное направление"""
#     if direction == 'f':
#         return 'b'
#     if direction == 'b':
#         return 'f'
#     if direction == 'r':
#         return 'l'
#     if direction == 'l':
#         return 'r'
#     return 'f'
#
# def check_lidars(lidars, desired_direction, drone_index):
#     """Проверяет лидары для всех направлений, возвращает безопасные и флаг необходимости обхода"""
#     safe_directions = []
#     needs_bypass = False
#     print(f"Drone {drone_index}: Checking lidars, desired={desired_direction}, values={lidars}")
#
#     for direction in ['f', 'b', 'r', 'l']:
#         value = lidars.get(direction, float('inf'))
#         if value == -1 or value >= SAFE_LIDAR_THRESHOLD:
#             safe_directions.append(direction)
#         if 0 < value < LIDAR_THRESHOLD:
#             print(f"Drone {drone_index}: Direction {direction} blocked (lidar={value:.2f})")
#             needs_bypass = True
#
#     return safe_directions, needs_bypass
#
# def avoid_obstacle(lidars, current_direction, drone_position, target_position, drone_index):
#     """Выбирает направление для обхода препятствия"""
#     safe_directions, _ = check_lidars(lidars, current_direction, drone_index)
#     perpendicular = get_perpendicular_directions(current_direction)
#
#     # Приоритет перпендикулярным направлениям с лидаром > SAFE_LIDAR_THRESHOLD
#     for direction in perpendicular:
#         if direction in safe_directions:
#             print(f"Drone {drone_index}: Bypassing, chose {direction}, lidars={lidars}")
#             BYPASS_STATE[drone_index] = {
#                 "direction": direction,
#                 "steps": BYPASS_STEPS,
#                 "original": current_direction
#             }
#             return direction
#
#     # Пробуем любое безопасное направление
#     if safe_directions:
#         chosen_direction = safe_directions[0]
#         print(f"Drone {drone_index}: Bypassing, chose {chosen_direction} (fallback), lidars={lidars}")
#         BYPASS_STATE[drone_index] = {
#             "direction": chosen_direction,
#             "steps": BYPASS_STEPS,
#             "original": current_direction
#         }
#         return chosen_direction
#
#     # Если нет безопасных направлений, отскок назад
#     reverse_dir = reverse_direction(current_direction)
#     print(f"Drone {drone_index}: No safe directions, attempting backoff to {reverse_dir}, lidars={lidars}")
#     BYPASS_STATE[drone_index] = {
#         "direction": reverse_dir,
#         "steps": 3,  # Увеличенный отскок
#         "original": current_direction
#     }
#     return reverse_dir
#
# def precision_align(drone_position, target_position, lidars, drone_index):
#     """Точно корректирует позицию дрона по x, затем z"""
#     dx = target_position["x"] - drone_position["x"]
#     dz = target_position["z"] - drone_position["z"]
#     stage = PRECISION_STATE[drone_index].get("stage", "x")
#     safe_directions, needs_bypass = check_lidars(lidars, "f", drone_index)
#
#     print(f"Drone {drone_index}: Precision aligning, stage={stage}, dx={dx:.3f}, dz={dz:.3f}, lidars={lidars}")
#
#     if needs_bypass:
#         direction = avoid_obstacle(lidars, "f", drone_position, target_position, drone_index)
#         return direction, ANGLE
#
#     # Этап 1: выравнивание по x
#     if stage == "x" and abs(dx) > 0.05:
#         desired_direction = "r" if dx > 0 else "l"
#         if desired_direction in safe_directions:
#             print(f"Drone {drone_index}: Aligning x, moving {desired_direction}, dx={dx:.3f}")
#             return desired_direction, PRECISION_ANGLE
#         perpendicular = get_perpendicular_directions(desired_direction)
#         for alt_dir in perpendicular:
#             if alt_dir in safe_directions:
#                 print(f"Drone {drone_index}: Aligning x, moving {alt_dir} (alt), dx={dx:.3f}")
#                 return alt_dir, PRECISION_ANGLE
#         return "f", 0
#     # Переход к этапу z
#     elif stage == "x" and abs(dx) <= 0.05:
#         PRECISION_STATE[drone_index]["stage"] = "z"
#         print(f"Drone {drone_index}: X aligned, switching to z, dx={dx:.3f}, dz={dz:.3f}")
#
#     # Этап 2: выравнивание по z
#     if stage == "z" and abs(dz) > 0.05:
#         desired_direction = "f" if dz < 0 else "b"
#         if desired_direction in safe_directions:
#             print(f"Drone {drone_index}: Aligning z, moving {desired_direction}, dz={dz:.3f}")
#             return desired_direction, PRECISION_ANGLE
#         perpendicular = get_perpendicular_directions(desired_direction)
#         for alt_dir in perpendicular:
#             if alt_dir in safe_directions:
#                 print(f"Drone {drone_index}: Aligning z, moving {alt_dir} (alt), dz={dz:.3f}")
#                 return alt_dir, PRECISION_ANGLE
#         return "f", 0
#     elif stage == "z" and abs(dz) <= 0.05:
#         print(f"Drone {drone_index}: Z aligned, checking target, dx={dx:.3f}, dz={dz:.3f}")
#         return "f", 0
#
#     return "f", 0
#
# def return_step(drone, drone_index, lidars):
#     """Выполняет шаг возвращения по сохраненному маршруту"""
#     if RETURN_INDICES[drone_index] >= len(PATHS[drone_index]):
#         return move("f", drone, 0, H)
#
#     path_index = len(PATHS[drone_index]) - 1 - RETURN_INDICES[drone_index]
#     last_direction = PATHS[drone_index][path_index]
#     return_direction = reverse_direction(last_direction)
#
#     safe_directions, needs_bypass = check_lidars(lidars, return_direction, drone_index)
#     if needs_bypass:
#         alt_direction = avoid_obstacle(lidars, return_direction, drone["droneVector"], {"x": 0, "z": 0}, drone_index)
#         PATHS[drone_index].append(alt_direction)
#         print(f"Drone {drone_index}: Return bypass, moving {alt_direction}, lidars={lidars}")
#         return move(alt_direction, drone, ANGLE, H)
#
#     if return_direction in safe_directions:
#         RETURN_INDICES[drone_index] += 1
#         print(f"Drone {drone_index}: Returning, moving {return_direction}, index={RETURN_INDICES[drone_index]}, lidars={lidars}")
#         return move(return_direction, drone, ANGLE, H)
#
#     alt_direction = avoid_obstacle(lidars, return_direction, drone["droneVector"], {"x": 0, "z": 0}, drone_index)
#     PATHS[drone_index].append(alt_direction)
#     print(f"Drone {drone_index}: Return bypass, moving {alt_direction}, lidars={lidars}")
#     return move(alt_direction, drone, ANGLE, H)
#
# def next_step(targets, iter):
#     """Функция анализирует данные с симулятора и делает один шаг"""
#     global TAG, ITER, IS_X, PATHS, IS_ACTIVE, DROPPED, BYPASS_STATE, DANGER_COUNT, IS_RETURNING, RETURN_INDICES, PRECISION_STATE
#
#     data = get_data(connection.receive_data())
#     fires = [0 for _ in range(len(targets))]
#     result = []
#
#     for i, drone in enumerate(data):
#         if not IS_ACTIVE[i]:
#             if iter < ITER[i]:
#                 result.append(move("f", drone, 0, 0))
#                 continue
#             else:
#                 IS_ACTIVE[i] = True
#                 result.append(move("f", drone, 0, H))
#                 continue
#
#         current_distance = calculate_distance(drone["droneVector"], targets[i])
#         dx = drone["droneVector"]["x"] - targets[i]["x"]
#         dz = drone["droneVector"]["z"] - targets[i]["z"]
#
#         # Проверяем, нужно ли активировать точную корректировку
#         if current_distance < PRECISION_THRESHOLD and not DROPPED[i] and TAG[i] != "precision" and not IS_RETURNING[i]:
#             print(f"Drone {i}: Near target (dist={current_distance:.2f}), switching to precision")
#             TAG[i] = "precision"
#             PRECISION_STATE[i] = {"steps": 30, "stage": "x"}
#
#         # Проверяем лидары для всех направлений
#         safe_directions, needs_bypass = check_lidars(drone["lidarInfo"], "f", i)
#
#         # Если требуется обход, немедленно переключаемся в bypass
#         if needs_bypass and TAG[i] != "bypass" and TAG[i] != "return":
#             direction = avoid_obstacle(drone["lidarInfo"], "f", drone["droneVector"], targets[i], i)
#             TAG[i] = "bypass"
#             angle = ANGLE
#         else:
#             # Определяем направление
#             if TAG[i] is None:
#                 direction, TAG[i] = get_direction(drone["droneVector"], targets[i], drone["lidarInfo"], i)
#                 angle = ANGLE
#             elif "x" in TAG[i]:
#                 direction, TAG[i] = go_x(drone["droneVector"], targets[i], drone["lidarInfo"], TAG[i][1])
#                 angle = ANGLE
#                 IS_X[i] = False
#             elif "z" in TAG[i]:
#                 direction, TAG[i] = go_z(drone["droneVector"], targets[i], drone["lidarInfo"], TAG[i][1])
#                 angle = ANGLE
#             elif TAG[i] == "bypass":
#                 direction = BYPASS_STATE[i]["direction"]
#                 angle = ANGLE
#                 BYPASS_STATE[i]["steps"] -= 1
#                 original_dir = BYPASS_STATE[i]["original"]
#                 original_lidar = drone["lidarInfo"].get(original_dir, float('inf'))
#                 if original_lidar == -1 or original_lidar >= LIDAR_THRESHOLD or BYPASS_STATE[i]["steps"] <= 0:
#                     print(f"Drone {i}: Bypass completed, original_lidar={original_lidar:.2f}, resetting TAG")
#                     TAG[i] = None
#                     PATHS[i] = []  # Перерасчет маршрута
#                     BYPASS_STATE[i] = {}
#                 else:
#                     print(f"Drone {i}: Continuing bypass, direction={direction}, steps={BYPASS_STATE[i]['steps']}, original_lidar={original_lidar:.2f}")
#             elif TAG[i] == "precision":
#                 direction, angle = precision_align(drone["droneVector"], targets[i], drone["lidarInfo"], i)
#                 PRECISION_STATE[i]["steps"] -= 1
#                 if PRECISION_STATE[i]["steps"] <= 0:
#                     print(f"Drone {i}: Precision steps exhausted, forcing drop")
#                     direction, angle = "f", 0
#             elif TAG[i] == "return":
#                 new_data = return_step(drone, i, drone["lidarInfo"])
#                 if equal(drone["droneVector"]["x"], 0) and equal(drone["droneVector"]["z"], 0):
#                     print(f"Drone {i}: Reached origin, resetting for recharge")
#                     IS_RETURNING[i] = False
#                     DROPPED[i] = False
#                     TAG[i] = None
#                     PATHS[i] = []
#                     RETURN_INDICES[i] = 0
#                     BYPASS_STATE[i] = {}
#                     DANGER_COUNT[i] = 0
#                     PRECISION_STATE[i] = {}
#                 result.append(new_data)
#                 continue
#
#         # Проверяем попадание в цель
#         if equal(drone["droneVector"]["x"], targets[i]["x"]) and equal(drone["droneVector"]["z"], targets[i]["z"]) and not DROPPED[i]:
#             print(f"Drone {i}: Reached target at x={targets[i]['x']:.2f}, z={targets[i]['z']:.2f}, dx={dx:.3f}, dz={dz:.3f}, dropping extinguisher")
#             new_data = move("f", drone, 0, H, drop=True)
#             DROPPED[i] = True
#             fires[i] = 1
#             TAG[i] = "return"
#             IS_RETURNING[i] = True
#             RETURN_INDICES[i] = 0
#             PRECISION_STATE[i] = {}
#             result.append(new_data)
#             continue
#
#         # Проверяем безопасность направления
#         if direction and direction not in safe_directions and TAG[i] != "bypass" and TAG[i] != "return":
#             direction = avoid_obstacle(drone["lidarInfo"], direction, drone["droneVector"], targets[i], i)
#             TAG[i] = "bypass"
#             angle = ANGLE
#
#         if direction is None or (direction == "f" and angle == 0):
#             new_data = move("f", drone, 0, H)
#         else:
#             lidar_value = drone["lidarInfo"].get(direction, float('inf'))
#             if lidar_value != -1 and lidar_value < DANGER_LIDAR_THRESHOLD:
#                 DANGER_COUNT[i] += 1
#                 print(f"Drone {i}: Moving in risky direction {direction} (lidar={lidar_value:.2f}), DANGER_COUNT={DANGER_COUNT[i]}")
#             else:
#                 DANGER_COUNT[i] = 0
#             new_data = move(direction, drone, angle, H)
#             if TAG[i] != "bypass" and TAG[i] != "return":
#                 PATHS[i].append(direction)
#             print(f"Drone {i}: Moving {direction}, angle={angle}, lidars={drone['lidarInfo']}, TAG={TAG[i]}, BYPASS={BYPASS_STATE[i]}, DIST={current_distance:.2f}, POS=x={drone['droneVector']['x']:.2f},z={drone['droneVector']['z']:.2f}, TARGET=x={targets[i]['x']:.2f},z={targets[i]['z']:.2f}")
#
#         # Проверяем DANGER_COUNT
#         if DANGER_COUNT[i] >= DANGER_COUNT_THRESHOLD:
#             print(f"Drone {i}: Too many risky moves (DANGER_COUNT={DANGER_COUNT[i]}), resetting TAG and PATHS")
#             TAG[i] = None
#             PATHS[i] = []
#             BYPASS_STATE[i] = {}
#             DANGER_COUNT[i] = 0
#             PRECISION_STATE[i] = {}
#
#         result.append(new_data)
#
#     connection.send_data(concat_engines(result, T))
#     time.sleep(T)
#
#     return fires
#
# def run(targets):
#     """Запускает миссию"""
#     print("Using code v11")
#     i = 0
#     fires = next_step(targets, i)
#     while sum(fires) != len(targets):
#         fires = next_step(targets, i)
#         i += 1
#
# def go_x(drone_position, target_position, lidars, direction):
#     """Хочу двигаться по Z, но не могу - мешает препятствие"""
#     desired_direction = get_direction1(drone_position["z"], target_position["z"])
#     safe_directions, _ = check_lidars(lidars, desired_direction, 0)
#     if desired_direction in safe_directions:
#         return desired_direction, None
#     return direction, "x" + direction
#
# def go_z(drone_position, target_position, lidars, direction):
#     """Хочу двигаться по X, но не могу - мешает препятствие"""
#     desired_direction = get_direction2(drone_position["x"], target_position["x"])
#     safe_directions, _ = check_lidars(lidars, desired_direction, 0)
#     if desired_direction in safe_directions:
#         return desired_direction, None
#     return direction, "z" + direction
#
# def get_direction1(drone_z, fire_z):
#     """Направление по оси Z"""
#     return "f" if drone_z - fire_z > 0 else "b"
#
# def get_direction2(drone_x, fire_x):
#     """Направление по оси X"""
#     return "r" if drone_x - fire_x > 0 else "l"
#
# def get_direction(drone_position, target_position, lidars, i):
#     """Определяет направление полета дрона и тег"""
#     global IS_X
#
#     if equal(drone_position["x"], target_position["x"]) and equal(drone_position["z"], target_position["z"]):
#         return None, None
#
#     if equal(drone_position["z"], target_position["z"]):
#         IS_X[i] = True
#
#     if not equal(drone_position["x"], target_position["x"]) and IS_X[i]:
#         desired_direction = get_direction2(drone_position["x"], target_position["x"])
#         safe_directions, _ = check_lidars(lidars, desired_direction, i)
#         if desired_direction in safe_directions:
#             return desired_direction, None
#         alt_direction = get_direction1(drone_position["z"], target_position["z"])
#         return alt_direction, "z" + alt_direction
#
#     if not equal(drone_position["z"], target_position["z"]):
#         desired_direction = get_direction1(drone_position["z"], target_position["z"])
#         safe_directions, _ = check_lidars(lidars, desired_direction, i)
#         if desired_direction in safe_directions:
#             return desired_direction, None
#         alt_direction = get_direction2(drone_position["x"], target_position["x"])
#         return alt_direction, "x" + alt_direction
#
#     return None, None




import time
import uuid

from algorithm.PID import move, get_data, equal, concat_engines
from connection.SocketConnection import SocketConnection

connection = SocketConnection()
H = 8  # высота, на которой летит дрон
T = 0.1  # время, через которое симулятор пришлет новый пакет данных
ANGLE = 5  # угол наклона дрона
ITER = [0, 300, 600, 900, 1200]  # задержка для следующих дронов (в итерациях)

# Если tag = None то просто запускаем get_direction()
# Если tag = "x", то нужно лететь по х, пока не сможем лететь по z
# Если tag = "z", то нужно лететь по z, пока не сможем лететь по x
# Если tag = "x" или tag = "z", то добавляется второй символ - направление полета
# Если tag = "return", то дрон возвращается по сохраненному маршруту
TAG = [None for _ in range(5)]

# Равняем ли координату X?
IS_X = [True for _ in range(5)]

# Список для хранения маршрутов каждого дрона
PATHS = [[] for _ in range(5)]

# Статус возвращения дронов
IS_RETURNING = [False for _ in range(5)]

# Статус активации дронов (взлетели ли они)
IS_ACTIVE = [False for _ in range(5)]

# Индексы для отслеживания текущей позиции в пути возвращения
RETURN_INDICES = [0 for _ in range(5)]


def check_lidars(directions, lidars):
    """Проверка лидаров. True - если можно лететь в заданном направлении дальше, False - если рядом преграда"""
    if 0 < lidars[directions[0]] < 5:
        return False
    for direction in directions[1:]:
        if 0 < lidars[direction] < 3:
            return False
    return True


def get_direction1(drone_z, fire_z):
    """Направление по оси Z. f - forward - вперед, b - backward - назад
    Возвращается кортеж из 3 элементов: еще добавляются боковые направления"""
    direction = "b"
    if drone_z - fire_z > 0:
        direction = "f"
    return direction, direction + "r", direction + "l"


def get_direction2(drone_x, fire_x):
    """Направление по оси X. r - right - вправо, l - left - влево
    Возвращается кортеж из 3 элементов: еще добавляются боковые направления"""
    direction = "l"
    if drone_x - fire_x > 0:
        direction = "r"
    return direction, "f" + direction, "b" + direction


def reverse_direction(direction):
    """Возвращает противоположное направление для возвращения"""
    if direction == "f":
        return "b"
    if direction == "b":
        return "f"
    if direction == "r":
        return "l"
    if direction == "l":
        return "r"
    return direction


def return_step(drone, drone_index, lidars):
    """Выполняет один шаг возвращения дрона по сохраненному маршруту"""
    if RETURN_INDICES[drone_index] >= len(PATHS[drone_index]):  # Если маршрут пуст или пройден
        return move("f", drone, 0, H)  # Остаемся на месте

    # Берем направление из маршрута, начиная с конца, и инвертируем его
    path_index = len(PATHS[drone_index]) - 1 - RETURN_INDICES[drone_index]
    last_direction = PATHS[drone_index][path_index]
    return_direction = reverse_direction(last_direction)

    # Проверяем, можно ли двигаться в обратном направлении
    directions = [return_direction]
    if return_direction in ["f", "b"]:
        directions.extend([return_direction + "r", return_direction + "l"])
    else:
        directions.extend(["f" + return_direction, "b" + return_direction])

    if check_lidars(directions, lidars):
        RETURN_INDICES[drone_index] += 1  # Переходим к следующему шагу возврата
        return move(return_direction, drone, ANGLE, H)

    # Если нельзя двигаться в обратном направлении, пытаемся обойти препятствие
    alt_directions = get_direction(drone["droneVector"], {"x": 0, "z": 0}, lidars, drone_index)[0]
    if alt_directions:
        PATHS[drone_index].append(alt_directions)  # Добавляем новое направление в путь
        return move(alt_directions, drone, ANGLE, H)

    return move("f", drone, 0, H)  # Если не можем двигаться, стоим на месте


def next_step(targets, iter):
    """Функция анализирует данные с симулятора и делает один шаг, то есть одну отправку на симулятор
    :param targets: список точек, к которым летит дрон
    :param iter: текущая итерация
    """
    global TAG, ITER, IS_X, PATHS, IS_RETURNING, IS_ACTIVE, RETURN_INDICES

    data = get_data(connection.receive_data())
    fires = [0 for _ in range(len(targets))]
    result = []

    for i, drone in enumerate(data):
        # Проверяем, достиг ли дрон времени активации
        if not IS_ACTIVE[i]:
            if iter < ITER[i]:
                # Дрон ждет, остается на месте (y=0)
                result.append(move("f", drone, 0, 0))
                continue
            else:
                IS_ACTIVE[i] = True  # Активируем дрон
                # Поднимаем дрон на высоту H
                result.append(move("f", drone, 0, H))
                continue

        if IS_RETURNING[i]:
            # Если дрон возвращается, используем обратный маршрут
            new_data = return_step(drone, i, drone["lidarInfo"])
            # Проверяем, достиг ли дрон начальной точки (x=0, z=0)
            if equal(drone["droneVector"]["x"], 0) and equal(drone["droneVector"]["z"], 0) and RETURN_INDICES[i] >= len(
                    PATHS[i]):
                IS_RETURNING[i] = False  # Завершаем возврат
                RETURN_INDICES[i] = 0  # Сбрасываем индекс для возможного повторного использования
            result.append(new_data)
            continue

        if TAG[i] is None:
            direction, TAG[i] = get_direction(drone["droneVector"], targets[i], drone["lidarInfo"], i)
        elif "x" in TAG[i]:
            direction, TAG[i] = go_x(drone["droneVector"], targets[i], drone["lidarInfo"], TAG[i][1])
            IS_X[i] = False
        elif "z" in TAG[i]:
            direction, TAG[i] = go_z(drone["droneVector"], targets[i], drone["lidarInfo"], TAG[i][1])

        if direction is None:
            new_data = move("f", drone, 0, H, drop=True)
            fires[i] = 1
            IS_RETURNING[i] = True  # Начинаем возврат после тушения
            TAG[i] = "return"
            RETURN_INDICES[i] = 0  # Инициализируем индекс возврата
        else:
            new_data = move(direction, drone, ANGLE, H)
            PATHS[i].append(direction)  # Сохраняем направление в маршрут

        result.append(new_data)

    connection.send_data(concat_engines(result, T))
    time.sleep(T)

    return fires


def run(targets):
    """Запускает миссию и возвращает дроны в исходную точку"""
    i = 0
    fires = next_step(targets, i)
    while any(IS_RETURNING) or sum(fires) != len(targets):
        fires = next_step(targets, i)
        i += 1


def go_x(drone_position, target_position, lidars, direction):
    """Хочу двигаться по Z, но не могу - мешает препятствие.
    Значит нужно лететь по X пока не смогу лететь по Z"""
    directions1 = get_direction1(drone_position["z"], target_position["z"])
    if check_lidars(directions1, lidars):
        return directions1[0], None
    return direction, "x" + direction


def go_z(drone_position, target_position, lidars, direction):
    """Хочу двигаться по X, но не могу - мешает препятствие.
    Значит нужно лететь по Z пока не смогу лететь по X"""
    directions2 = get_direction2(drone_position["x"], target_position["x"])
    if check_lidars(directions2, lidars):
        return directions2[0], None
    return direction, "z" + direction


def get_direction(drone_position, target_position, lidars, i):
    """Функция определяет дальнейшее направление полета дрона и тег"""
    global IS_X

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





# import time
#
# from algorithm.PID import move, get_data, equal, concat_engines
# from connection.SocketConnection import SocketConnection
#
# connection = SocketConnection()
# H = 8  # высота, на которой летит дрон
# T = 0.1  # время, через которое симулятор пришлет новый пакет данных
# ANGLE = 5  # угол наклона дрона
# ITER = [0, 300, 600, 900, 1200]  # задержка для следующих дронов
#
# # Если tag = None то просто запускаем get_direction()
# # Если tag = "x", то нужно лететь по х, пока не сможем лететь по z
# # Если tag = "z", то нужно лететь по z, пока не сможем лететь по x
# # Если tag = "x" или tag = "z", то добавляется второй символ - направление полета
# TAG = [None for _ in range(5)]
#
# # Равняем ли координату X?
# IS_X = [True for _ in range(5)]
#
#
# def check_lidars(directions, lidars):
#     """Проверка лидаров. True - если можно лететь в заданном направлении дальне, False - если рядом преграда"""
#     if 0 < lidars[directions[0]] < 5:
#         return False
#     for direction in directions[1:]:
#         if 0 < lidars[direction] < 3:
#             return False
#     return True
#
#
# def get_direction1(drone_z, fire_z):
#     """Направление по оси Z. f - forward - вперед, b - backward - назад
#     Возвращается кортеж из 3 элементов: еще добавляются боковые направления"""
#     direction = "b"
#     if drone_z - fire_z > 0:
#         return "f"
#     return direction, direction + "r", direction + "l"
#
#
# def get_direction2(drone_x, fire_x):
#     """Направление по оси X. r - right - вправо, l - left - влево
#     Возвращается кортеж из 3 элементов: еще добавляются боковые направления"""
#     direction = "l"
#     if drone_x - fire_x > 0:
#         return "r"
#     return direction, "f" + direction, "b" + direction
#
#
# def next_step(targets, iter):
#     """Функция анализирует данны с симулятора и делает один шаг, то есть одну отправку на симулятор
#     :param targets: список точек, к которым летит дрон
#     :param iter: текущая итерация
#     """
#     global TAG, ITER, IS_X
#
#     data = get_data(connection.receive_data())
#     fires = [0 for _ in range(len(targets))]
#     result = []
#
#     for i, drone in enumerate(data):
#         if TAG[i] is None:
#             direction, TAG[i] = get_direction(drone["droneVector"], targets[i], drone["lidarInfo"], i)
#         elif "x" in TAG[i]:
#             direction, TAG[i] = go_x(drone["droneVector"], targets[i], drone["lidarInfo"], TAG[i][1])
#             IS_X[i] = False
#         elif "z" in TAG[i]:
#             direction, TAG[i] = go_z(drone["droneVector"], targets[i], drone["lidarInfo"], TAG[i][1])
#
#         if ITER[i] > iter:
#             new_data = move("f", drone, 0, 0)
#         elif direction is None:
#             new_data = move("f", drone, 0, H, drop=True)
#             fires[i] = 1
#         else:
#             new_data = move(direction, drone, ANGLE, H)
#
#         result.append(new_data)
#
#     connection.send_data(concat_engines(result, T))
#     time.sleep(T)
#
#     return fires
#
#
# def run(targets):
#     i = 0
#     fires = next_step(targets, i)
#     while sum(fires) != len(targets):
#         fires = next_step(targets, i)
#         i += 1
#
#
# def go_x(drone_position, target_position, lidars, direction):
#     """Хочу двигаться по Z, но не могу - мешает препятствие.
#     Значит нужно лететь по X пока не смогу лететь по Z"""
#
#     directions1 = get_direction1(drone_position["z"], target_position["z"])
#     if check_lidars(directions1, lidars):
#         return directions1[0], None
#
#     return direction, "x" + direction
#
#
# def go_z(drone_position, target_position, lidars, direction):
#     """Хочу двигаться по X, но не могу - мешает препятствие.
#     Значит нужно лететь по Z пока не смогу лететь по X"""
#
#     directions2 = get_direction2(drone_position["x"], target_position["x"])
#     if check_lidars(directions2, lidars):
#         return directions2[0], None
#     return direction, "z" + direction
#
#
# def get_direction(drone_position, target_position, lidars, i):
#     """Функция определяет дальнейшее направления полета дрона и тег"""
#     global IS_X
#
#     if equal(drone_position["z"], target_position["z"]):
#         IS_X[i] = True
#
#     if not equal(drone_position["x"], target_position["x"]) and IS_X[i]:
#         directions2 = get_direction2(drone_position["x"], target_position["x"])
#         if check_lidars(directions2, lidars):
#             return directions2[0], None
#         directions1 = get_direction1(drone_position["z"], target_position["z"])
#         return directions1[0], "z" + directions1[0]
#
#     if not equal(drone_position["z"], target_position["z"]):
#         directions1 = get_direction1(drone_position["z"], target_position["z"])
#         if check_lidars(directions1, lidars):
#             return directions1[0], None
#         directions2 = get_direction2(drone_position["x"], target_position["x"])
#         return directions2[0], "x" + directions2[0]
#
#     return None, None
