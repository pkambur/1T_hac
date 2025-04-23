import asyncio
import json

from config import LOG_LEVEL
from services.logger import set_logger_config


async def start_websocket():
    from algorithm.fly import run, connection
    from algorithm.PID import concat_engine, concat_engines

    await connection.set_connection()

    fire_position = json.loads(connection.receive_data())["firesPositions"]
    for i, pos in enumerate(fire_position):
        print(i, pos)

    fire_position = [fire_position[14],
                     fire_position[13],
                     fire_position[1],
                     fire_position[2],
                     fire_position[10]]

    connection.send_data("sendData")
    run(fire_position)
    connection.close_connection()


if __name__ == "__main__":
    set_logger_config(LOG_LEVEL)
    asyncio.run(start_websocket())
