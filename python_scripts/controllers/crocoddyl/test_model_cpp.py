import numpy as np

import crocoddyl
import time

# Creating an action model for the unicycle system
model = crocoddyl.ActionModelUnicycle()
model.dt = 0.01

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
