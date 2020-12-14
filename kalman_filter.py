import numpy as np
from plot import plot_trajectory, plot_point, plot_covariance_2d

class UserCode:

        def __init__(self):
            dt = 0.005

            #State-transition model
            self.A = np.array([
                   [1,0,dt,0],
                   [0,1,0,dt],
                   [0,0,1,0],
                   [0,0,0,1]
                   ])
            #Observation model (GPS)
            self.H = np.array([[1,0,0,0],[0,1,0,0]])

            #Process/State noise
            vel_noise_std = 0.005
            pos_noise_std = 0.005
            self.Q = np.array([
                   [pos_noise_std*pos_noise_std,0,0,0],
                   [0,pos_noise_std*pos_noise_std,0,0],
                   [0,0,vel_noise_std*vel_noise_std,0],
                   [0,0,0,vel_noise_std*vel_noise_std]
                   ])

        #Sensor/Measurement noise (GPS)
        measurement_noise_std = 0.5
        self.R = measurement_noise_std * measurement_noise_std * np.identity(2)

        self.x = np.zeros((4,1)) #Initial state vector [x,y,vx,vy]
        self.sigma = np.identity(4) #Initial covariance matrix

        def predictState(self, A, x):
        # param A: State-transition model matrix
        # param x: Current state vector
        # return x_p: Predicted state vector as 4x1 numpy array

        x_p = np.dot(A, x) #state equation

        return x_p

    def predictCovariance(self, A, sigma, Q): #compute sigma bar
        sigma_p = np.dot(np.dot(A, sigma), np.transpose(A))+Q)
        return sigma_p

    def calculateKalmanGain(self, sigma_p, H, R): #compute Kalman gain
        k = np.dot(np.dot(sigma_p, np.transpose(H)), np.linalg.inv(np.dot(H, np.dot(sigma_p, np.transpose(H)))+R))
        return k

    def correctState(self, z, x_p, k, H):

    #param z: Measurement vector
    #param x_p: Predicted state vector
    #param k: Kalman gain
    #param H: Observation model
    #return x: Corrected state vector as 4x1 numpy array

        x = x_p + np.dot(k, (z - np.dot(H, x_p))) # on définit l'état de correction non pas en passant par
    # la loi  normale mais tout simplement en utilisant l'équation pour déterminer mu
    # et en remplacant mu par x


        return x

    def correctCovariance(self, sigma_p, k, H): #on calcule sigma
        sigma = np.dot((np.identity(4)-np.dot(k, H)), sigma_p)
        return sigma

    def state_callback(self):  #cette méthode est appelée à chaque simulation step,
        # elle réalise une implémentation du KF prediction
        self.x = self.predictState(self.A, self.x)
        self.sigma = self.predictCovariance(self.A, self.sigma, self.Q)

    # visualize position state
        plot_trajectory("kalman", self.x[0:2]) #on trace la position estimée d'après le motion model
        plot_covariance_2d("kalman", self.sigma[0:2,0:2]) #on trace la zone d'incertitude autour de la postion estimée d'après le motion model

    def measurement_callback(self, measurement): #cette méthode est appelée à chaque nouvelle mesure du gps
# elle réalise une implémentation du KF de correction
#param measurement: vector of measured coordinates

# visualize measurement
        plot_point("gps", measurement)

        k = self.calculateKalmanGain(self.sigma, self.H, self.R) #on calcule d'abord la nouvelle valeur de # k

        self.x = self.correctState(measurement, self.x, k, self.H) #on calcule la valeur de x estimée
        self.sigma = self.correctCovariance(self.sigma, k, self.H) #on calcule la covariance de correction,
# pour savoir quelle est l'incertitude autour de la position du drone

# visualize position state
        plot_trajectory("kalman", self.x[0:2]) #on trace la position (selon toute probabilité)
        plot_covariance_2d("kalman", self.sigma[0:2,0:2]) #on trace la zone d'incertitude autour de la
# valeur calculée)

