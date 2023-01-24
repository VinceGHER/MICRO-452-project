import asyncio
import time
import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from modules.Tymio_interface import Tymio_interface
import modules.Tools as Tools
import numpy as np
async def test():
    config = Tools.read_config_file("./config.yaml")
    tymio_interface_config = config["Tymio_interface"]
    tymio_interface = Tymio_interface(
            tymio_interface_config["tolerance_pos"],
            tymio_interface_config["tolerance_angle"]*np.pi/180,
            tymio_interface_config["k_dist"],
            tymio_interface_config["k_angle"],
            50,
            100,
            tymio_interface_config["tymio_speed_to_mms"],
        )
    await tymio_interface.connect()
    start = time.time()
    tymio_interface.apply_motor_control((1,1),False)
    speed_l = []
    speed_r = []
    while time.time() - start < 30:
        await tymio_interface.node.wait_for_variables({
            "prox.horizontal",
            "motor.left.speed",
            "motor.right.speed",
        })
        speed_l.append(tymio_interface.node.v.motor.left.speed)
        speed_r.append(tymio_interface.node.v.motor.left.speed)
    tymio_interface.apply_motor_control((0,0),False)
    print(speed_l)
    print(speed_r)
    print(np.mean(speed_l))
    print(np.mean(speed_r))
    print(np.var(speed_l))
    print(np.var(speed_r))
if __name__ == "__main__":
    asyncio.run(test())