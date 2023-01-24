from Tools import extend_config_file
from tdmclient import ClientAsync, aw
import numpy as np
import sys
import time
import asyncio

calibration_distance_cm = 15
"""Calibrate the proximity sensors """
async def prox_sensor_calibrate(speed_cm_s:float,max_distance_cm:float=calibration_distance_cm,interval_cm:float=0.1):

    lookup_int_all = []
    lookup_cm_all = []

    client = ClientAsync()
    time.sleep(0.1)
    client.process_waiting_messages()
    node = client.nodes[0]
    aw(node.lock())

    aw(node.register_events([("walk_back", 1)]))
    period = round(interval_cm*10000/speed_cm_s)
    speed_int = round(speed_cm_s * 500/20)
    
    program = f"""

    var walking = 0
    var need_walking = 0

    timer.period[0] = {period}

    onevent timer0
        if need_walking == 1 and walking == 0 then
            need_walking = 0
            walking = 1
            motor.left.target = -{speed_int}
            motor.right.target = -{speed_int}
        else
            need_walking = 0
            walking = 0
            motor.left.target = 0
            motor.right.target = 0
        end

    onevent walk_back
        need_walking = event.args[0]
    """

    aw(node.compile(program))
    aw(node.run())
    for i in range(round(max_distance_cm/interval_cm)):
        aw(node.send_events({"walk_back": [1]}))

        while True:
            await node.wait_for_variables({"motor.right.target"})
            if node.v.motor.right.target != speed_int:
                break
            else:
                time.sleep(0.1)

        await node.wait_for_variables({"prox.horizontal"})
        lookup_int_all.append(list(node.v.prox.horizontal)[3])
        lookup_cm_all.append(round(i*interval_cm,1))
    
    lookup_cm = []
    lookup_int = []

    for v_int,v_dist in zip(lookup_int_all,lookup_cm_all):
        if not v_int in lookup_int:
            lookup_cm.append(v_dist)
            lookup_int.append(v_int)

    lookup_cm = np.asarray(lookup_cm)
    lookup_int = np.asarray(lookup_int)

    lookup_cm = np.append(lookup_cm,lookup_cm[len(lookup_cm)-1] + interval_cm)
    lookup_cm = lookup_cm.tolist()

    lookup_int = (lookup_int[lookup_int != 0])
    lookup_int = np.append(lookup_int,0)
    lookup_int = lookup_int.tolist()

    out_dict = {'Robot':{'Sensor Properties':{'cm_lookup':lookup_cm, 'int_lookup':lookup_int}}}
    extend_config_file(out_dict)
    sys.exit()

asyncio.run(prox_sensor_calibrate(3))