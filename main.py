import asyncio
import json
import logging
import sys
import numpy as np
from sklearn.cluster import KMeans
from config import LOG_LEVEL
from services.logger import set_logger_config

# Проверка версии Python
if sys.version_info < (3, 7):
    print("Error: Python 3.7 or higher is required")
    sys.exit(1)

# Создаём логгер для main
logger = logging.getLogger("main")

def calculate_distance(point1, point2):
    """Вычисляет евклидово расстояние между двумя точками (x, z)"""
    try:
        distance = ((point1["x"] - point2["x"])**2 + (point1["z"] - point2["z"])**2)**0.5
        logger.debug(f"Calculated distance between {point1} and {point2}: {distance}")
        return distance
    except Exception as e:
        logger.error(f"Error calculating distance: {e}")
        raise

def build_tsp_route(fires, start_point):
    """Строит маршрут для дрона по очагам, начиная с start_point"""
    try:
        if not fires:
            logger.debug("No fires to build route for")
            return []
        route = [start_point]
        unvisited = fires.copy()
        current = start_point

        while unvisited:
            distances = [calculate_distance(current, fire) for fire in unvisited]
            nearest_idx = np.argmin(distances)
            nearest_fire = unvisited.pop(nearest_idx)
            route.append(nearest_fire)
            current = nearest_fire

        logger.debug(f"Built TSP route: {route[1:]}")
        return route[1:]
    except Exception as e:
        logger.error(f"Error building TSP route: {e}")
        raise

def assign_fires_to_drones(fires, num_drones):
    """Распределяет очаги между дронами с помощью KMeans и строит маршруты"""
    try:
        logger.debug(f"Assigning {len(fires)} fires to {num_drones} drones")
        valid_fires = [
            fire for fire in fires
            if fire is not None and isinstance(fire, dict) and "x" in fire and "z" in fire
        ]
        if not valid_fires:
            logger.error("No valid fire positions available")
            return [None] * num_drones * 3

        logger.info(f"Valid fires: {valid_fires}")

        coordinates = np.array([[fire["x"], fire["z"]] for fire in valid_fires])
        kmeans = KMeans(n_clusters=min(num_drones, len(valid_fires)), random_state=42)
        labels = kmeans.fit_predict(coordinates)

        clusters = [[] for _ in range(num_drones)]
        for fire, label in zip(valid_fires, labels):
            clusters[label].append(fire)

        start_point = {"x": 0, "z": 0}
        drone_routes = []
        for cluster in clusters:
            route = build_tsp_route(cluster, start_point)
            drone_routes.append(route)
            logger.info(f"Cluster assigned: {route}")

        # Логируем разбиение очагов по дронам
        for drone_idx, route in enumerate(drone_routes):
            logger.info(f"Drone {drone_idx} assigned fires: {route}")

        max_len = max(len(route) for route in drone_routes) if drone_routes else 1
        fire_position = []
        for i in range(max_len):
            for drone_idx, route in enumerate(drone_routes):
                if i < len(route):
                    fire_position.append(route[i])
                else:
                    fire_position.append(None)

        logger.debug(f"Final fire position list: {fire_position}")
        return fire_position
    except Exception as e:
        logger.error(f"Error assigning fires to drones: {e}")
        raise

async def start_websocket():
    try:
        from algorithm.fly import run, connection
        from algorithm.PID import concat_engine, concat_engines

        logger.debug("Starting websocket connection")
        await connection.set_connection()

        logger.debug("Receiving fire positions")
        try:
            fire_position = json.loads(connection.receive_data())["firesPositions"]
            logger.debug(f"Raw fire positions: {fire_position}")
        except (json.JSONDecodeError, KeyError) as e:
            logger.error(f"Failed to parse fire positions: {e}")
            fire_position = []

        for i, pos in enumerate(fire_position):
            if pos is None or not isinstance(pos, dict) or "x" not in pos or "z" not in pos:
                logger.warning(f"Invalid fire position at index {i}: {pos}")
            else:
                logger.info(f"Fire {i}: {pos}")

        num_drones = 5
        logger.debug(f"Assigning fires to {num_drones} drones")
        fire_position = assign_fires_to_drones(fire_position, num_drones)
        logger.info(f"Optimized fire positions: {fire_position}")

        logger.debug("Sending initial command")
        connection.send_data(concat_engines(concat_engine([0 for _ in range(8)], {"id": 0}), 0))
        logger.debug("Running drone control")
        run(fire_position)
        logger.debug("Closing connection")
        connection.close_connection()
    except Exception as e:
        logger.error(f"Error in start_websocket: {e}")
        raise

if __name__ == "__main__":
    try:
        # Очищаем обработчики корневого логгера
        logging.getLogger('').handlers = []
        set_logger_config(LOG_LEVEL)  # Без logger_name
        logger.debug("Main program started")
        logger.debug(f"Python version: {sys.version}")
        asyncio.run(start_websocket())
        logger.debug("Program completed successfully")
    except Exception as e:
        logger.error(f"Main program error: {e}")
        raise




# import asyncio
# import json
#
# from config import LOG_LEVEL
# from services.logger import set_logger_config
#
#
# async def start_websocket():
#     from algorithm.fly import run, connection
#     from algorithm.PID import concat_engine, concat_engines
#
#     await connection.set_connection()
#
#     fire_position = json.loads(connection.receive_data())["firesPositions"]
#     for i, pos in enumerate(fire_position):
#         print(i, pos)
#
#     fire_position = [fire_position[14],
#          fire_position[13],
#          fire_position[2],
#          fire_position[3],
#          fire_position[4]]
#
#     connection.send_data(concat_engines(concat_engine([0 for _ in range(8)], {"id": 0}), 0))
#     run(fire_position)
#     connection.close_connection()
#
#
# if __name__ == "__main__":
#     set_logger_config(LOG_LEVEL)
#     asyncio.run(start_websocket())
