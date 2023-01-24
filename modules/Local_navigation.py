import numpy as np

RIGHT = 1
LEFT = 2       

low_speed = 0.5
high_speed = 1

class Local_navigation:

    def __init__(self,prox_sensor_properties:dict):
        self.number_prox_sensor = prox_sensor_properties['nb_sensor']
        self.sensor_angle_from_direction = np.asarray(prox_sensor_properties['sensor_angles'])

        self.cm_lookup:list = np.asarray(prox_sensor_properties['cm_lookup'])
        self.int_lookup:list = np.asarray(prox_sensor_properties['int_lookup'])

        self.min_distance = np.asarray(prox_sensor_properties['min_distance'])
        self.threshold_int = np.array(self.cm_to_int(self.min_distance))
        self.min_distance_avoidance = prox_sensor_properties['min_distance_avoidance']

        self.which_obstable:list[int]
        self.obstacle_int:list[int]
        self.full_prox_cm:np.array
        
        self.compute_avoidance_started:bool = False
        self.avoidance_occurring:bool = False

        self.start_avoidance_point:np.array
        self.start_to_goal_vec:np.array
        self.direction_of_avoidance:int

        self.max_corner_iter = 5
        self.corner_iter_count = 0

        self.safe_dist_cm = np.max(self.min_distance)
        self.dist_margin_cm = 5

    
    def cm_to_int(self,distance_cm:list[float]) -> list[int]:
        return list(map(self._cm_to_int,distance_cm))
    
    
    def _cm_to_int(self,distance_cm:float) -> int:
        idx = (np.abs(self.cm_lookup - distance_cm)).argmin()
        return self.int_lookup[idx]

            
    def int_to_cm(self,distance_int:list[int]) -> list[float]:
        return list(map(self._cm_to_int,distance_int))
    
    def _int_to_cm(self,distance_int:int) -> float:
        idx = (np.abs(self.int_lookup - distance_int)).argmin()
        return self.cm_lookup[idx]

    def is_obstacle(self,prox_sensor:tuple[int],current_pos,next_pos,current_angle)->bool:
        prox_sensor = np.array(prox_sensor)
        self.obstacle_int = prox_sensor
        self.full_prox_cm = np.asarray(self.int_to_cm(prox_sensor))

        is_obstacle_per_sensor = np.greater(prox_sensor,self.threshold_int)
        self.avoidance_occurring = np.any(is_obstacle_per_sensor)

        self.which_obstable = np.where(is_obstacle_per_sensor)[0]
        self.which_obstable = self.which_obstable.tolist()

        # obstacle_in_path = False
        # for obs_dist,sensor_angle in zip(self.min_distance[self.which_obstable],self.sensor_angle_from_direction[self.which_obstable]):
        #     object_pos =current_pos+obs_dist*np.array([np.cos(sensor_angle*np.pi/180+current_angle),np.sin(sensor_angle*np.pi/180+current_angle)])
        #     obstacle_in_path = obstacle_in_path or self.point_is_in(current_pos,next_pos,object_pos)
        # self.avoidance_occurring = self.avoidance_occurring and obstacle_in_path

        return self.avoidance_occurring
    
    def point_is_in(self,point1,point2,point_to_ckeck):
        unit_vector1 = np.squeeze(point1 / np.linalg.norm(point1))
        unit_vector2 = np.squeeze(point2 / np.linalg.norm(point2))
        unit_vector_to_check = np.squeeze(point_to_ckeck / np.linalg.norm(point_to_ckeck))

        angle_to_1 = np.arccos(np.dot(unit_vector1, unit_vector_to_check))
        angle_to_2 = np.arccos(np.dot(unit_vector2, unit_vector_to_check))
        print(angle_to_2,'**************')

        if angle_to_1 >= np.pi/2 or angle_to_2 >= np.pi/2:
            return False
        return True

    def start_avoidance(self,current_position:np.array,next_step_position:np.array):
        self.start_avoidance_point = current_position
        self.start_to_goal_vec = next_step_position - self.start_avoidance_point
        where_obstacle = np.sign(self.sensor_angle_from_direction[self.which_obstable]) @ np.transpose(self.obstacle_int[self.which_obstable])
        if where_obstacle >= 0:
            self.direction_of_avoidance = RIGHT
        else:
            self.direction_of_avoidance = LEFT
        self.compute_avoidance_started = True

    def stop_avoidance(self):
        self.compute_avoidance_started = False
        self.avoidance_occurring = False


    def has_avoided(self,current_position:np.array):
        current_vec:np.array = current_position - self.start_avoidance_point
        if np.linalg.norm(current_vec) < self.min_distance_avoidance:
            return False
        has_avoided = self._is_point_good_side(self.direction_of_avoidance, current_vec, self.start_to_goal_vec)
        return has_avoided
    

    def _is_point_good_side(self,direction_of_avoidance, current_vec, start_to_goal_vec):
        cross_p = np.cross(current_vec,start_to_goal_vec)
        if direction_of_avoidance == LEFT:
            cross_p = -cross_p

        return (cross_p >= 0)


    def compute_avoidance(self,prox_sensor_int:np.array,current_position:np.array,next_step_position:np.array):
        if not self.compute_avoidance_started:
            self.start_avoidance(current_position,next_step_position)

        elif self.has_avoided(current_position):
            self.stop_avoidance()

        return not self.avoidance_occurring,*self.get_avoidance_vel(prox_sensor_int)

    

    def get_avoidance_vel(self,prox_sensor_int):
        if not self.avoidance_occurring:
            return None,None

        orientation_factor = 1
        interesting_sensors = [0,1,2]

        if self.direction_of_avoidance == LEFT:
            orientation_factor = -1
            interesting_sensors = [4,3,2]

        prox_sensor_cm = np.asarray(self.int_to_cm(np.asarray(prox_sensor_int)[interesting_sensors]))
        
        if prox_sensor_int[interesting_sensors[0]] == 0:
            self.corner_iter_count += 1
            if self.corner_iter_count <= self.max_corner_iter:
                velocity = np.array([high_speed,high_speed])
            else:
                if self.corner_iter_count%2 == 0:
                    velocity = orientation_factor * np.array([-1,1])
                else:
                    velocity = np.array([high_speed,high_speed])

        else:
            if self.corner_iter_count >= self.max_corner_iter:
                self.corner_iter_count = 0

            if prox_sensor_cm[2] < self.safe_dist_cm:
                velocity = orientation_factor * np.array([high_speed,-high_speed])
            elif prox_sensor_cm[1] < self.safe_dist_cm:
                velocity = orientation_factor * np.array([high_speed,-high_speed])
            elif prox_sensor_cm[0] < self.safe_dist_cm:
                velocity = orientation_factor * np.array([high_speed,-high_speed])
            elif prox_sensor_cm[0] < self.safe_dist_cm-self.dist_margin_cm:
                velocity = orientation_factor * np.array([low_speed,-low_speed])
            elif prox_sensor_cm[0] > self.safe_dist_cm+self.dist_margin_cm:
                velocity = orientation_factor * np.array([-low_speed,low_speed])
            else:
                self.corner_iter_count += 1
                if self.corner_iter_count <= self.max_corner_iter:
                    velocity = np.array([high_speed,high_speed])
                else:
                    if self.corner_iter_count%2 == 0:
                        velocity = orientation_factor * np.array([-1,1])
                    else:
                        velocity = np.array([high_speed,high_speed])

        return velocity


