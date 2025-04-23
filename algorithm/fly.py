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





