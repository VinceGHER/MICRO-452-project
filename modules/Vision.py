import time
import cv2
import numpy as np
import modules.Tools as Tools
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}


class Vision:

    def __init__(self,cam_id,nb_blocks_x,nb_blocks_y,grid_x_mm,grid_y_mm,corners_grid_ids=[11,22,33,44],robot_id=55,goal_id=66,black_threshold=180,aruco_type = "DICT_5X5_1000"):
        self.nb_blocks_x = nb_blocks_x
        self.nb_blocks_y = nb_blocks_y
        self.grid_x_mm = grid_x_mm
        self.grid_y_mm = grid_y_mm
        self.aruco_type = aruco_type
        self.black_threshold = black_threshold

        self.corners_grid_ids = corners_grid_ids
        self.robot_id = robot_id
        self.goal_id = goal_id
        self.cam = cv2.VideoCapture(cam_id, cv2.CAP_DSHOW)
        # self.cam.set(cv2.CAP_PROP_EXPOSURE, -8.0)
        self.cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)

    """Capture and Image and return the grid and the robot position"""    
    def compute_vision(self):
        ret, image = self.cam.read()
        # cv2.imshow('raw_capture',image)

        return self.process_image(image)
    
    """Process an image and return the robot position"""
    def get_position_orientation(self):
        ret, image = self.cam.read()
        image_output = image.copy()
        centers,_ = self.get_centers_orientation_markers(image_output)

        image_flat = self.get_flat_perpective(image,centers)
        height, width = image_flat.shape[0:2]
        print(height,width)
        centers_corrected, orientation_corrected = self.get_centers_orientation_markers(image_flat)
        if self.robot_id not in centers_corrected:
            raise Exception("Robot not detected")

        print(centers_corrected)
        orientation_traj = Tools.angle_between((1,0),orientation_corrected[self.robot_id])

        pixel_size_x = self.grid_x_mm/width
        pixel_size_y = self.grid_y_mm/height

        centers_corrected = {k: (v[0]*pixel_size_x,v[1]*pixel_size_y) for k,v in centers_corrected.items()}

        return centers_corrected[self.robot_id],orientation_traj,image_flat

    """ Plot the robot position and orientation on the image and the covariance for the position and orientation"""
    def plot_state(self,image, pos, angle, cov, cov_angle,goal,path=None):
        height, width = image.shape[0:2]
        to_pixel_x = width/self.grid_x_mm
        to_pixel_y = height/self.grid_y_mm
        cov = np.array(cov)[0]
        pos = (int(pos[0]*to_pixel_x),int(pos[1]*to_pixel_y))
        goal = (int(goal[0]*to_pixel_x),int(goal[1]*to_pixel_y))

        if path is not None:
            for p in path:
                p = (int(p[0]*to_pixel_x),int(p[1]*to_pixel_y))
                cv2.circle(image, p, radius=0, color=(255, 165, 0), thickness=2)
        try:

            pearson = cov[0, 1]/np.sqrt(cov[0, 0] * cov[1, 1])
            # Using a special case to obtain the eigenvalues of this
            # two-dimensional dataset.
            ell_radius_x = np.sqrt(1 + pearson)*5
            ell_radius_y = np.sqrt(1 - pearson)*5

            angle_variance_radius = np.sqrt(cov_angle)

            cv2.circle(image, pos, radius=0, color=(255, 0, 0), thickness=2)
            cv2.circle(image, goal, radius=0, color=(230,144,255), thickness=5)
            cv2.ellipse(image, pos, (int(ell_radius_x * 2), int(ell_radius_y * 2)), 0,0, 360, (0, 0, 255), 1)
            cv2.arrowedLine(image, pos, (int(pos[0]+20*np.cos(angle)),int(pos[1]-20*np.sin(angle))),
                                        (0, 0, 255), 1)
            cv2.arrowedLine(image, pos, (int(pos[0]+20*np.cos(angle+angle_variance_radius)),int(pos[1]-20*np.sin(angle+angle_variance_radius))),
                                        (255, 0, 0), 1)
            cv2.arrowedLine(image, pos, (int(pos[0]+20*np.cos(angle-angle_variance_radius)),int(pos[1]-20*np.sin(angle-angle_variance_radius))),
                                        (255, 0, 0), 1)
            
            self.show_image(image,"state")
        except Exception as e:
            print("error",e)
            
    """ Display the Aruco markers detected on the image"""
    def display_markers(self,corners, ids, rejected, image):
        if len(corners) > 0:
            
            ids = ids.flatten()
            
            for (markerCorner, markerID) in zip(corners, ids):
                
                corners = markerCorner.reshape((4, 2))
               
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
               
                # cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                
                cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                # print("[Inference] ArUco marker ID: {}".format(markerID))
    
    """Get the centers and orientation of the Aruco markers"""
    def get_centers_orientation_markers(self,image):                 
        arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[self.aruco_type])
        arucoParams = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
      
        self.display_markers(corners, ids, rejected, image)
        
        centers = {}
        orientations = {}
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                centers[markerID] = (cX, cY)
                
                # print(markerID,corners)
                orientations[markerID] = (topLeft[0]-bottomLeft[0],(topLeft[1]-bottomLeft[1]))
        return centers,orientations

    def process_image(self, image):
        image_output = image.copy()
        centers,_ = self.get_centers_orientation_markers(image_output)
        print("perspective",centers)
        image_flat = self.get_flat_perpective(image,centers)
        image_flat_output = image_flat.copy()

        # convert in gray scale
        gray = cv2.cvtColor(image_flat, cv2.COLOR_BGR2GRAY)
        # blur the image
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # threshold the image
        mask = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

        # self.show_image(mask,"mask")
        height, width, channels = image_flat.shape
        grid = np.zeros((self.nb_blocks_y,self.nb_blocks_x))
        
        block_size_x = round( width/ self.nb_blocks_x)
        block_size_y = round( height/ self.nb_blocks_y)
        self.block_size_x = block_size_x
        self.block_size_y = block_size_y
        # cropped_image = mask[0*block_size_y:0+block_size_y,3*block_size_x:3*block_size_x+block_size_x]
        # print(np.mean(cropped_image))
        # self.show_image(cropped_image,"cropped")
        for i in range(self.nb_blocks_y):
            for j in range(self.nb_blocks_x):
                x = j*block_size_x
                y = i*block_size_y
                
                cropped_image = mask[y:y+block_size_y,x:x+block_size_x]
                cv2.rectangle(image_flat_output, (x,y), (x+block_size_x,y+block_size_y), (0,0,255), 2)
                
                average= np.mean(cropped_image)
                # print(i,j,average)
                if (average <= self.black_threshold):
                    grid[i][j] = 1
                else:
                    grid[i][j] = 0
    
        centers_2,_ = self.get_centers_orientation_markers(image_flat)
        # self.show_image(image_flat_output,"image_flat_output")  
        # self.show_image(image_flat,"image_flat")  
        if self.goal_id not in centers_2:
            raise Exception("Goal not detected")
        if  self.robot_id not in centers_2:
            raise Exception("Robot not detected")
        goal = np.floor(np.array((centers_2[self.goal_id][0]/block_size_x,centers_2[self.goal_id][1]/block_size_y))).astype(int)
        start = np.floor(np.array((centers_2[self.robot_id][0]/block_size_x,centers_2[self.robot_id][1]/block_size_y))).astype(int)
    
        grid[goal[1]][goal[0]] = 0
        start = start[::-1]
        goal = goal[::-1]
        return grid, start, goal


    """Get the flat perspective of the image"""
    def get_flat_perpective(self, image,centers):
        if not all(corner in centers for corner in self.corners_grid_ids):
            raise Exception("corners not detected")
        pt_A = np.array(centers[self.corners_grid_ids[0]])
        pt_B = np.array(centers[self.corners_grid_ids[2]])
        pt_C = np.array(centers[self.corners_grid_ids[3]])
        pt_D = np.array(centers[self.corners_grid_ids[1]])

        width_AD = np.sqrt(((pt_A[0] - pt_D[0]) ** 2) + ((pt_A[1] - pt_D[1]) ** 2))
        width_BC = np.sqrt(((pt_B[0] - pt_C[0]) ** 2) + ((pt_B[1] - pt_C[1]) ** 2))
        maxWidth = max(int(width_AD), int(width_BC))


        height_AB = np.sqrt(((pt_A[0] - pt_B[0]) ** 2) + ((pt_A[1] - pt_B[1]) ** 2))
        height_CD = np.sqrt(((pt_C[0] - pt_D[0]) ** 2) + ((pt_C[1] - pt_D[1]) ** 2))
        maxHeight = max(int(height_AB), int(height_CD))

        input_pts = np.float32([pt_A, pt_B, pt_C, pt_D])
        output_pts = np.float32([[0, 0],
                                [0, maxHeight - 1],
                                [maxWidth - 1, maxHeight - 1],
                                [maxWidth - 1, 0]])

        # Compute the perspective transform M
        M = cv2.getPerspectiveTransform(input_pts,output_pts)
        out = cv2.warpPerspective(image,M,(maxWidth, maxHeight),flags=cv2.INTER_LINEAR)
        
        return out

    def show_image(self,image,name):
        h, w = image.shape[0:2]
        neww = 800
        newh = int(neww*(h/w))
        cv2.namedWindow(name, cv2.WINDOW_KEEPRATIO)
        cv2.imshow(name, image)
        cv2.resizeWindow(name, neww, newh)

    def calibrate_camera(self):
        while 1:
            ret, img = self.cam.read()
            self.show_image(img,"calibration")
            if cv2.waitKey(0) == -1:
                break
    def wait_for_color_calibration(self):
        self.cam.read()
        time.sleep(1)
        self.cam.read()
    def calibrate_position_angle(self, nb_samples=10):
        samples_pos = []
        samples_angle = []
        while 1:
            try:
                pos, angle, _ = self.get_position_orientation()
                print(pos,angle)
                samples_pos.append(pos)
                samples_angle.append(angle)
                if cv2.waitKey(10) == 27:
                    break
                print("sample added")
                if len(samples_angle) > nb_samples:
                    break
            except Exception as e:
                print("calibration error",e)
                continue
        return np.var(samples_pos,axis=0), np.var(samples_angle,axis=0)
    def __del__(self):
        self.cam.release()
if __name__ == "__main__":

    
    vision = Vision(0,7,5,corners_grid_ids=[11,22,33,44],robot_id=55,goal_id=66,aruco_type = "DICT_4X4_250")
    # Time to do color calibration
    vision.wait_for_color_calibration()

    # main 
    while 1:
        print(vision.compute_vision())
        if cv2.waitKey(0) == -1:
            break

    # frame = cv2.imread('background_inv.jpg')
    # grid,start,goal = vision.process_image(frame)
    # print(grid,start,goal)

    # pos, ori = vision.get_position_orientation(frame)
    # print(pos,ori)

    cv2.destroyAllWindows()