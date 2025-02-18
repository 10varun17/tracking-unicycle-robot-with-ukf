import numpy as np
import scipy
from filterpy.kalman import unscented_transform

""" 
UKF Implementation
"""
class UKF:
    def __init__(self, dim_x, dim_z, dt, fx, hx, Q, R, alpha, beta, kappa, initial_state, initial_covariance):
        """
        dim_x: dimension of the state vector, eg. [x, y, theta].T dimension
        dim_z: dimension of the measurement vector, eg. [x, y].T dimension
        dt: time step
        fx: state transition function f(x, dt, u)
        hx: measurement function h(x)
        Q: process noise covariance (dim_x x dim_x)
        R: measurement noise covariance (dim_z x dim_z)
        alpha, beta, kappa: UKF scaling parameters for sigma points
        initial_state: initial state estimate (numpy array of shape (dim_x,))
        initial_covariance: initial covariance matrix (dim_x x dim_x numpy array)
        """
        # Initialize variables
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dt = dt
        self.fx = fx
        self.hx = hx
        self.Q = Q
        self.R = R
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        
        # Initialize variables for sigma points calculation
        self.n = dim_x
        self.lambda_ = alpha**2 * (self.n + kappa) - self.n
        self.num_sigmas = 2 * self.n + 1

        # Calculate weights for the sigma points
        self.Wm = np.full(self.num_sigmas, 1.0 / (2 * (self.n + self.lambda_)))
        self.Wc = np.full(self.num_sigmas, 1.0 / (2 * (self.n + self.lambda_)))
        self.Wm[0] = self.lambda_ / (self.n + self.lambda_)
        self.Wc[0] = self.lambda_ / (self.n + self.lambda_) + 1 - alpha**2 + beta

        # Initialize state and covariance
        self.x = initial_state
        self.P = initial_covariance
        
        # Variables to store sigma points in state space and measurement space
        self.sigmas_f = np.zeros((self.num_sigmas, dim_x))
        self.sigmas_h = np.zeros((self.num_sigmas, dim_z))

    def compute_sigma_points(self, x, P):
        """
        Compute sigma points for the given state and covariance.
        """
        n = self.n
        num_sigma = self.num_sigmas
        sigma_points = np.zeros((num_sigma, n))     # num_sigma sigma points for each of n dimensions of state variable
        U = scipy.linalg.cholesky((n + self.lambda_) * P)
        sigma_points[0] = x
        for k in range(n):
            sigma_points[k+1]   = x + U[k]
            sigma_points[n+k+1] = x - U[k]
        return sigma_points

    def predict(self, dt, u):
        """
        Predict step: propagate sigma points through the process model.
    
        u: control command (Twist message with v and w)
        """
        self.dt = dt 
        sigmas = self.compute_sigma_points(self.x, self.P)
        
        # Propagate each sigma point using the process model fx
        for i, sigma_point in enumerate(sigmas):
            self.sigmas_f[i] = self.fx(sigma_point, dt, u)
        
        # Compute the predicted state mean and covariance
        self.x, self.P = unscented_transform(self.sigmas_f, self.Wm, self.Wc, self.Q)

    def update(self, z):
        """
        Update step: update the state using measurement z
        """
        # Transform the predicted sigma points into measurement space
        for i in range(self.num_sigmas):
            self.sigmas_h[i] = self.hx(self.sigmas_f[i])
            
        # Compute predicted mean and covariance
        zp, Pz = unscented_transform(self.sigmas_h, self.Wm, self.Wc, self.R)
        
        # Compute cross covariance of state and measurement
        Pxz = np.zeros((self.dim_x, self.dim_z))
        for i in range(self.num_sigmas):
            Pxz += self.Wc[i] * np.outer(self.sigmas_f[i] - self.x, self.sigmas_h[i] - zp)
            
        # Compute Kalman gain.
        K = np.dot(Pxz, scipy.linalg.inv(Pz))
        
        # Update state and covariance with measurement residual
        y = z - zp
        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(K, np.dot(Pz, K.T))