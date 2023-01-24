import asyncio
import time
import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import modules.Tymio_interface as Tymio_interface
import modules.Tools as Tools
import numpy as np
async def test():
    config = Tools.read_config_file("./config.yaml")
    tymio_interface_config = config["Tymio_interface"]
    tymio_interface = Tymio_interface.Tymio_interface(
            tymio_interface_config["tolerance_pos"],
            tymio_interface_config["tolerance_angle"]*np.pi/180,
            tymio_interface_config["k_dist"],
            tymio_interface_config["k_angle"],
            50,
            tymio_interface_config["speed_forward"],
            tymio_interface_config["tymio_speed_to_mms"],
        )
    await tymio_interface.connect()
    start = time.time()
    tymio_interface.apply_motor_control((-1,1),False)
    while time.time() - start < 50:
        pass
    tymio_interface.apply_motor_control((0,0),False)
if __name__ == "__main__":
    asyncio.run(test())