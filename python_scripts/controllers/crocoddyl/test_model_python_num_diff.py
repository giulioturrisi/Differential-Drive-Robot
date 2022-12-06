import numpy as np

import crocoddyl
import time

class UnicycleModelDerived(crocoddyl.DifferentialActionModelAbstract):

    def __init__(self):
        crocoddyl.DifferentialActionModelAbstract.__init__(self, crocoddyl.StateVector(3), 2, 5)  # nu = 1; nr = 6
        self.costWeights = [20., 50., 20., 10, 10]

    def calc(self, data, x, u=None):
        if u is None:
            u = self.unone
        v, w = u
        px, py, theta = x
        c, s = np.cos(theta), np.sin(theta)
        
        # Defining the equation of motions
        x_d = c *v 
        y_d = s * v
        theta_d = w
        data.xout = np.matrix([x_d, y_d, theta_d]).T

        
         # Computing the cost residual and value
        data.r = np.matrix(self.costWeights * np.array([np.asscalar(x[0]), np.asscalar(x[1]), np.asscalar(x[2]), np.asscalar(u[0]), np.asscalar(u[1])])).T
        data.cost = .5 * np.asscalar(sum(np.asarray(data.r)**2))


    def calcDiff(self, data, x, u=None):
        pass

# Creating an action model for the unicycle system
unicycleDAM = UnicycleModelDerived()
unicycleData = unicycleDAM.createData()
unicycleDAM = model = UnicycleModelDerived()

# Setting up the cost weights
model.r = [
    10.,  # state weight
    1.  # control weight
]


# Using NumDiff for computing the derivatives. We specify the
# withGaussApprox=True to have approximation of the Hessian based on the
# Jacobian of the cost residuals.
unicycleND = crocoddyl.DifferentialActionModelNumDiff(unicycleDAM, True)

# Getting the IAM using the simpletic Euler rule
timeStep = 0.1
unicycleIAM = crocoddyl.IntegratedActionModelEuler(unicycleND, timeStep)


terminalUnicycle = UnicycleModelDerived()
terminalUnicycleDAM = crocoddyl.DifferentialActionModelNumDiff(terminalUnicycle, True)
terminalUnicycleIAM = crocoddyl.IntegratedActionModelEuler(terminalUnicycleDAM)
terminalUnicycle.costWeights[0] = 1
terminalUnicycle.costWeights[1] = 10
terminalUnicycle.costWeights[2] = 1
terminalUnicycle.costWeights[3] = 0
terminalUnicycle.costWeights[4] = 0


# Formulating the optimal control problem
T = 50  # number of knots
x0 = np.matrix([-1., -1., 1.]).T  #x,y,theta
problem = crocoddyl.ShootingProblem(x0, [unicycleIAM] * T, terminalUnicycleIAM)

start_time = time.time()
# Creating the DDP solver for this OC problem, defining a logger
ddp = crocoddyl.SolverDDP(problem)
ddp.setCallbacks([crocoddyl.CallbackLogger(), crocoddyl.CallbackVerbose()])

# Solving it with the DDP algorithm
ddp.solve()

print("comp. time: ", time.time()-start_time)


# Plotting the solution, solver convergence and unicycle motion
log = ddp.getCallbacks()[0]
crocoddyl.plotOCSolution(log.xs, log.us, figIndex=1, show=True)
