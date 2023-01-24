import math
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button
class Robot_state_estimator:

    def __init__(self, Ks,Q, R, mean_init, cov_init):
        self.Ks = Ks
        self.Q = Q
        self.R = R
        self.mean_state = mean_init
        self.cov_state = cov_init

    # Pre parse the measurement to get the right format
    def compute_robot_state(self, x, y, theta, vr, vl, camera_meas,Ts, plot=None):
        mean_state_prev = self.mean_state
        cov_state_prev = self.cov_state
        if camera_meas:
            measurement = np.array([[x],[y],[theta],[vr],[vl]])
        else:
            measurement = np.array([[vr],[vl]])
        self.kalman_filter(measurement,camera_meas,Ts)

        if plot != None:
            self.plot_state(**plot,title="Position estimation before and after kalman filter",mu=[mean_state_prev[:2],self.mean_state[:2]],
                Sigma=[np.array([i[:2] for i in cov_state_prev[:2]]),np.array([i[:2] for i in self.cov_state[:2]])])
        return self.mean_state.T[0][:2]

    def __str__(self): 
        return """
== Robot estimator states ==
Mean_state:
{mean_state}

Cov_state:
{cov_state}
====
        """.format(mean_state=np.array2string(self.mean_state),cov_state=np.array2string(self.cov_state))

    # All the math implementation of the extented kalman filter
    def kalman_filter(self,measurement,camera_meas,Ts):

        if camera_meas:
            R = self.R
        else:
            R = np.array(self.R)[[3,4],:][:,[3,4]]
        G = self.compute_jacobian_motion(self.mean_state,Ts)
        mean_state_a_priori = self.motion_model(self.mean_state, Ts)
        cov_state_a_priori = G @ self.cov_state @ G.T + self.Q
        H = self.compute_jacobian_measurement(mean_state_a_priori,camera_meas,Ts)
        i = measurement - self.measurement_model(mean_state_a_priori,camera_meas)
        s = H @ cov_state_a_priori @ H.T + R
        k = cov_state_a_priori @ H.T @ np.linalg.inv(s)

        self.mean_state = mean_state_a_priori + k @ i
        self.cov_state = cov_state_a_priori - k @ H @ cov_state_a_priori
    
    
    
    def get_position_angle_mean_cov(self):
        return (self.mean_state[:2], self.mean_state[2], np.array([i[:2] for i in self.cov_state[:2]]), self.cov_state[2][2])
    
    
    # 3d plot of the gaussian distribution
    def plot_state_position(self,**kargs):
        self.plot_state(**kargs,title="Position estimation",mu=[self.mean_state[:2]],Sigma=[np.array([i[:2] for i in self.cov_state[:2]])])

    def plot_state(self,N,x_min,x_max,y_min,y_max, title,mu, Sigma):
        X = np.linspace(x_min, x_max, N)
        Y = np.linspace(y_min, y_max, N)
        X, Y = np.meshgrid(X, Y)

        # Pack X and Y into a single 3-dimensional array
        pos = np.empty(X.shape + (2,))
        pos[:, :, 0] = X
        pos[:, :, 1] = Y

        Z = []

        for i in range(len(mu)):
            Z.append( self.multivariate_gaussian(pos, mu[i].T, Sigma[i]))

    
        # Plot
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        axslider = plt.axes([0.1, 0.02, 0.5, 0.03])
        axplay = plt.axes([0.65, 0.02, 0.2, 0.03])
        index_slider = Slider(
            axslider, "Timestep", 0, len(Z)-1,
            valinit=0, valstep=1
        )
        bplay = Button(axplay, 'Play')
        def make_plot(Z):
        
            ax.plot_surface(X, Y, Z, cmap="viridis", linewidth=0)
            ax.contourf(X, Y, Z, zdir='z',offset=-1, cmap=cm.get_cmap("viridis"))
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Probability Density')
            # ax.set_zlim(-0.15,0.2)
            # ax.set_zticks(np.linspace(0,0.2,5))
        make_plot(Z[index_slider.val])
 
        ax.view_init(67, -21)
        plt.title(title)
        def play(val):
            for i in range(len(Z)):
                ax.clear()
                make_plot(Z[i])
                index_slider.set_val(i)
                fig.canvas.draw()
                plt.pause(0.2)
              

        def update(val):
            ax.clear()
            make_plot(Z[index_slider.val])
            fig.canvas.draw_idle()
        index_slider.on_changed(update)
        bplay.on_clicked(play)
        plt.show()

    def multivariate_gaussian(self,pos, mu, Sigma):
        """Return the multivariate Gaussian distribution on array pos."""

        n = mu.shape[0]
        Sigma_det = np.linalg.det(Sigma)
        Sigma_inv = np.linalg.inv(Sigma)
        N = np.sqrt((2*np.pi)**n * Sigma_det)
        # This einsum call calculates (x-mu)T.Sigma-1.(x-mu) in a vectorized
        # way across all the input variables.
        fac = np.einsum('...k,kl,...l->...', pos-mu, Sigma_inv, pos-mu)

        return np.exp(-fac / 2) / N
    def get_states(self,mean_state):
        return tuple( mean_state[i][0] for i in range(len(mean_state)))
    
    
    # Here is defined the motion model and jacobian model
    def motion_model(self,mean_state, Ts):
        (x,y,theta,vr,vl) = self.get_states(mean_state)
        return np.array([
            [x + (vr+vl)/2 * math.cos(theta)* Ts],
            [y - (vr+vl)/2 * math.sin(theta)* Ts],
            [theta + self.Ks * (vr - vl) * Ts],
            [vr],
            [vl],
        ])
    def compute_jacobian_motion(self, mean_state, Ts):
        (x,y,theta,vr,vl) = self.get_states(mean_state)
        return np.array([
                [1,0,(vr+vl)/2*(-math.sin(theta)*Ts),math.cos(theta)/2*Ts,math.cos(theta)/2*Ts],
                [0,1,(vr+vl)/2*(math.cos(theta)*Ts),math.sin(theta)/2*Ts,math.sin(theta)/2*Ts],
                [0,0,1,self.Ks * Ts,-self.Ks * Ts],
                [0,0,0,1,0],
                [0,0,0,0,1],
     
                ])
    # Here is defined the measurement model and jacobian model
    def measurement_model(self, mean_state,camera_meas):
        (x,y,theta,vr,vl) = self.get_states(mean_state)
        if camera_meas:
            return np.array([
                [x],
                [y],
                [theta],
                [vr],
                [vl],
            ])
        else:
            return np.array([
                [vr],
                [vl],
            ])
    def compute_jacobian_measurement(self, mean_state,camera_meas, Ts):
        (x,y,theta,vr,vl) = self.get_states(mean_state)
        if camera_meas:
            return np.array([
                    [1,0,0,0,0],
                    [0,1,0,0,0],
                    [0,0,1,0,0],
                    [0,0,0,1,0],
                    [0,0,0,0,1],
                    ])
        else:
            return np.array([
                    [0,0,0,1,0],
                    [0,0,0,0,1],
                    ])


# Test function to test the module.
def test():
    Ks=1
    Q = np.array([
                [0.04,0,0,0,0],
                [0,0.04,0,0,0],
                [0,0,0.004,0,0],
                [0,0,0,6.153,0],
                [0,0,0,0,6.153],
                ])
    R = np.array([
                [0.25,0,0,0,0],
                [0,0.25,0,0,0],
                [0,0,0.025,0,0],
                [0,0,0,6.153,0],
                [0,0,0,0,6.153],
                ])
    mean_init = np.array([[1], [1], [0], [0], [0]])
    cov_init = np.array([
                [0.1,0,0,0,0],
                [0,0.1,0,0,0],
                [0,0,0.03,0,0],
                [0,0,0,0.001,0],
                [0,0,0,0,0.001],
                ])
    robot_estimator = Robot_state_estimator(Ks,Q,R,mean_init,cov_init)
    print(robot_estimator)

    plot_params = {
        'N':100,
        'x_min':0,
        'x_max':10,
        'y_min':0,
        'y_max':10,
    }
    mean_state = []
    cov_state = []
    (mean,angle,cov, cov_angle) = robot_estimator.get_position_angle_mean_cov()

    mean_state.append(mean)
    cov_state.append(cov)
    for i in range(10):
        robot_estimator.compute_robot_state(
                5+np.random.uniform(-2,2),
                5+np.random.uniform(-2,2),
                0,
                np.random.uniform(-2,2),
                np.random.uniform(-2,2), True, 1)
        (mean,angle,cov, cov_angle) = robot_estimator.get_position_angle_mean_cov()
        print(robot_estimator)
        mean_state.append(mean)
        cov_state.append(cov)

    robot_estimator.plot_state(**plot_params,title="Position estimation",mu=np.array(mean_state),Sigma=np.array(cov_state)) 

    
    
if __name__ == "__main__":
    test()
    print("Everything passed")