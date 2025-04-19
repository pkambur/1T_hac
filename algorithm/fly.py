import time
from algorithm.PID import move, get_data, equal, concat_engines
from connection.SocketConnection import SocketConnection

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

def check_lidars(directions, lidars):
    """Проверка лидаров. True - если можно лететь, False - если рядом преграда"""
    if 0 < lidars[directions[0]] < 5:
        return False
    for direction in directions[1:]:
        if 0 < lidars[direction] < 3:
            return False
    return True

def get_direction1(drone_z, fire_z):
    """Направление по оси Z: f - вперед, b - назад"""
    direction = "b"
    if drone_z - fire_z > 0:
        return "f"
    return direction, direction + "r", direction + "l"

def get_direction2(drone_x, fire_x):
    """Направление по оси X: r - вправо, l - влево"""
    direction = "l"
    if drone_x - fire_x > 0:
        return "r"
    return direction, "f" + direction, "b" + direction

def next_step(targets, iter, initial_positions):
    """Один шаг управления дронами"""
    global TAG, ITER, IS_X, RETURNING, INITIAL_POSITIONS
    data = get_data(connection.receive_data())
    fires = [0 for _ in range(len(data))]  # Статус пожаров для каждого дрона
    result = []

    # Сохраняем исходные позиции на первой итерации
    if iter == 0:
        for i, drone in enumerate(data):
            if i < len(INITIAL_POSITIONS):
                INITIAL_POSITIONS[i] = {"x": drone["droneVector"]["x"], "z": drone["droneVector"]["z"]}

    for i, drone in enumerate(data):
        # Пропускаем, если индекс дрона превышает количество дронов или цель отсутствует
        if i >= len(ITER) or (i < len(targets) and targets[i] is None):
            new_data = move("f", drone, 0, H)  # Зависаем на месте
            result.append(new_data)
            continue

        # Определяем текущую цель: пожар или исходная позиция
        current_target = INITIAL_POSITIONS[i] if RETURNING[i] else targets[i]

        if TAG[i] is None:
            direction, TAG[i] = get_direction(drone["droneVector"], current_target, drone["lidarInfo"], i)
        elif "x" in TAG[i]:
            direction, TAG[i] = go_x(drone["droneVector"], current_target, drone["lidarInfo"], TAG[i][1])
            IS_X[i] = False
        elif "z" in TAG[i]:
            direction, TAG[i] = go_z(drone["droneVector"], current_target, drone["lidarInfo"], TAG[i][1])

        if ITER[i] > iter:
            new_data = move("f", drone, 0, 0)  # Дрон ждет активации
        elif direction is None:
            if not RETURNING[i]:
                # Дрон достиг пожара, тушим и начинаем возврат
                new_data = move("f", drone, 0, H, drop=True)
                fires[i] = 1
                RETURNING[i] = True
                TAG[i] = None
                IS_X[i] = True
            else:
                # Дрон достиг исходной позиции, зависаем
                new_data = move("f", drone, 0, H)
        else:
            new_data = move(direction, drone, ANGLE, H)

        result.append(new_data)

    connection.send_data(concat_engines(result, T))
    time.sleep(T)
    return fires

def run(targets):
    """Основной цикл управления дронами"""
    i = 0
    fires = next_step(targets, i, INITIAL_POSITIONS)
    # Продолжаем, пока все дроны не потушат пожары и не вернутся
    while not all(RETURNING) or not all_at_initial_positions():
        fires = next_step(targets, i, INITIAL_POSITIONS)
        i += 1

def all_at_initial_positions():
    """Проверяет, вернулись ли все дроны в исходные позиции"""
    global INITIAL_POSITIONS, RETURNING
    data = get_data(connection.receive_data())
    for i, drone in enumerate(data):
        if i >= len(INITIAL_POSITIONS):
            continue
        if RETURNING[i]:
            initial = INITIAL_POSITIONS[i]
            current = drone["droneVector"]
            if not (equal(current["x"], initial["x"]) and equal(current["z"], initial["z"])):
                return False
    return True

def go_x(drone_position, target_position, lidars, direction):
    directions1 = get_direction1(drone_position["z"], target_position["z"])
    if check_lidars(directions1, lidars):
        return directions1[0], None
    return direction, "x" + direction

def go_z(drone_position, target_position, lidars, direction):
    directions2 = get_direction2(drone_position["x"], target_position["x"])
    if check_lidars(directions2, lidars):
        return directions2[0], None
    return direction, "z" + direction

def get_direction(drone_position, target_position, lidars, i):
    global IS_X
    # Если цель отсутствует, зависаем
    if target_position is None:
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