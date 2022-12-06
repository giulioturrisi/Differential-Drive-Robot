import numpy as np

import crocoddyl
import time

class UnicycleModelDerived(crocoddyl.ActionModelAbstract):

    def __init__(self):
        crocoddyl.ActionModelAbstract.__init__(self, crocoddyl.StateVector(3), 2, 5)
        self.dt = .01
        self.costWeights = [10., 1.]

    def calc(self, data, x, u=None):
        if u is None:
            u = self.unone
        v, w = u
        px, py, theta = x
        c, s, dt = np.cos(theta), np.sin(theta), self.dt
        # Rollout the dynamics
        data.xnext[0] = px + c * v * dt
        data.xnext[1] = py + s * v * dt
        data.xnext[2] = theta + w * dt
        # Compute the cost value
        data.r[:3] = self.costWeights[0] * x
        data.r[3:] = self.costWeights[1] * u
        data.cost = .5 * sum(data.r**2)

    def calcDiff(self, data, x, u=None):
        if u is None:
            u = self.unone
        v = u[0]
        theta = x[2]
        # Cost derivatives
        data.Lx[:] = x * ([self.costWeights[0]**2] * self.state.nx)
        data.Lu[:] = u * ([self.costWeights[1]**2] * self.nu)
        # Dynamic derivatives
        c, s, dt = np.cos(theta), np.sin(theta), self.dt
        data.Fx[0, 2] = -s * v * dt
        data.Fx[1, 2] = c * v * dt
        data.Fu[0, 0] = c * dt
        data.Fu[1, 0] = s * dt
        data.Fu[2, 1] = dt

    def createData(self):
        data = UnicycleDataDerived(self)
        return data

class UnicycleDataDerived(crocoddyl.ActionDataAbstract):

    def __init__(self, model):
        crocoddyl.ActionDataAbstract.__init__(self, model)
        nx, nu = model.state.nx, model.nu
        self.Lxx[range(nx), range(nx)] = [model.costWeights[0]**2] * nx
        self.Luu[range(nu), range(nu)] = [model.costWeights[1]**2] * nu
        self.Fx[0, 0] = 1
        self.Fx[1, 1] = 1
        self.Fx[2, 2] = 1

# Creating an action model for the unicycle system
model = UnicycleModelDerived()

# Setting up the cost weights
model.r = [
    10.,  # state weight
    1.  # control weight
]

# Formulating the optimal control problem
T = 50  # number of knots
x0 = np.matrix([-1., -1., 1.]).T  #x,y,theta
problem = crocoddyl.ShootingProblem(x0, [model] * T, model)

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
