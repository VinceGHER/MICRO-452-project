import asyncio
from asyncio import events
from tdmclient import ClientAsync
import math
import modules.Tools as Tools
import sys
class Tymio_interface:

    def __init__(self, tolerance_pos,tolerance_angle,k_dist,k_angle,speed_turning,speed_forward,tymio_speed_to_mms):
        self.tolerance_angle = tolerance_angle
        self.tolerance_pos = tolerance_pos
        self.k_angle = k_angle
        self.k_dist = k_dist
        self.speed_turning = speed_turning
        self.speed_forward = speed_forward
        self.tymio_speed_to_mms = tymio_speed_to_mms
        self.client = ClientAsync()

    async def connect(self):
        self.node = await self.client.wait_for_node()
        await self.node.lock()
        return
    async def release(self):
        await self.node.unlock()
        await self.node.stop()

    """ Get the sensors data
    Return the list of the prox sensors, the right and left motor speed
    """
    async def get_sensors(self):
        await self.node.wait_for_variables({
            "prox.horizontal",
            "motor.left.speed",
            "motor.right.speed",
        })
        return (list(self.node.v.prox.horizontal),self.node.v.motor.right.speed*self.tymio_speed_to_mms,self.node.v.motor.left.speed*self.tymio_speed_to_mms)
   
    """ Apply the motor control
        Return if checkpoint is reached and the robot is going toward the checkpoint
    """
    async def move_toward_pos(self, goal_pos, orientation, pos):
        if math.dist(goal_pos,pos) < self.tolerance_pos:
            self.apply_motor_control((0,0),0)
            return True,True
        vector_direction = (goal_pos[0]-pos[0],goal_pos[1]-pos[1])
        orientation_traj = Tools.angle_between((1,0),vector_direction)
        print("goal_angle: ",orientation_traj*180/math.pi,"vector: ",vector_direction)
        diff =( orientation_traj - orientation)

        if (diff > math.pi):
            diff -= 2*math.pi
        if (diff < -math.pi):
            diff += 2*math.pi

        # diff = min(max(diff,-360),360)
        print("diff: ",diff,"(",diff*180/math.pi,")","dist",math.dist([vector_direction[0]],[vector_direction[1]]))
        is_in_tolerance = abs(diff) < self.tolerance_angle
        if is_in_tolerance: # move forward
            v = self.min_max_slide(math.dist([vector_direction[0]],[vector_direction[1]]),True)
            self.apply_motor_control((v,v),True)
        else : # move counter clockwise for > 0 clockwise for < 0
            v = self.min_max_slide(diff[0],False)
            self.apply_motor_control((-v,v), False)
        return False,is_in_tolerance
    def min_max_slide(self,value,forward):
        if forward:
            coef = max(min(self.k_dist*value,1),-1)
        else:
            coef = max(min(self.k_angle*value,1),-1)
        return coef

    """
    Apply the motor control
    Convert the -1 1 value to the speed of the robot depending if the robt is turning or going forward
    """

    def apply_motor_control(self,motor_control,forward):
        print("control: ",motor_control)
        control = {
            "motor.left.target": [int(motor_control[0]* (self.speed_forward if forward == True else self.speed_turning))],
            "motor.right.target": [int(motor_control[1]* (self.speed_forward if forward == True else self.speed_turning))],
        }
        self.node.send_set_variables(control)
    
async def test():
    try:
        tymio_interface = Tymio_interface(0.1,3,1,1,50,50,0.43)
        await tymio_interface.connect()
        goal = (3,3)
        success = False

        while not success:
            print(await tymio_interface.get_sensors())
            a=input ('Entre new pos: ')  # value entered is 1,1
            data = tuple(int(x) for x in a.split(","))
            pos = (data[0],data[1])
            ori=data[2]
            print(pos,ori)
            success = await tymio_interface.move_toward_pos(goal,ori,pos)
            print(success)
            await tymio_interface.client.sleep(1)

            tymio_interface.apply_motor_control((0,0),False)
    except KeyboardInterrupt:
        print("ff")
        # await tymio_interface.release()
        loop = asyncio.get_event_loop()
        return


if __name__ == "__main__":
    loop = events.new_event_loop()
    try:
        events.set_event_loop(loop)
        loop.run_until_complete(test())
    except:
        print("ddd")
        loop.close()