from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from acados_model import export_robot_model
import numpy as np
import scipy.linalg

import time

import sys
sys.path.append('/home/python_scripts/')
from robot_model import Robot

X0 = np.array([0.0, 1., 0.001])  # Intitalize the states 
N_horizon = 50  # Define the number of discretization steps
T_horizon = 1.0  # Define the prediction horizon
dt = 1.0/50

unicycle = Robot(dt = 0.01)



def create_ocp_solver_description() -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    model = export_robot_model()
    ocp.model = model
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu

    # set dimensions
    ocp.dims.N = N_horizon

    # set cost
    Q_mat = np.diag([5, 5, 0])  # [x,0,yaw]
    R_mat = np.diag([1, 1])

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    ny = nx + nu
    ny_e = nx

    ocp.cost.W_e = Q_mat*10
    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[nx : nx + nu, 0:nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))

    # set constraints
    #ocp.constraints.lbu = np.array([-F_max])
    #ocp.constraints.ubu = np.array([+F_max])
    #ocp.constraints.idxbu = np.array([0])

    ocp.constraints.x0 = X0

    # set options
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = "IRK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # SQP_RTI, SQP
    ocp.solver_options.nlp_solver_max_iter = 400
    # ocp.solver_options.levenberg_marquardt = 1e-2

    # set prediction horizon
    ocp.solver_options.tf = T_horizon

    return ocp


def closed_loop_simulation():

    # create solvers
    ocp = create_ocp_solver_description()
    #acados_ocp_solver = AcadosOcpSolver(
    #    ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
    #)

    acados_ocp_solver = AcadosOcpSolver(
        ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
    )


    # prepare simulation
    Nsim = 20
    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]

    simX = np.ndarray((Nsim + 1, nx))
    simU = np.ndarray((Nsim, nu))

    xcurrent = X0
    print("initial state: ", X0)
    simX[0, :] = xcurrent

    # initialize solver
    for stage in range(N_horizon + 1):
        acados_ocp_solver.set(stage, "x", 0.0 * np.ones(xcurrent.shape))
    for stage in range(N_horizon):
        acados_ocp_solver.set(stage, "u", np.zeros((nu,)))

    for j in range(N_horizon):
        yref = np.array([0, 2, 0, 0, 0])
        acados_ocp_solver.set(j, "yref", yref)
    yref_N = np.array([0, 2, 0])
    acados_ocp_solver.set(N_horizon, "yref", yref_N)

    # closed loop
    for i in range(Nsim):
        start_time = time.time()
        # set initial state constraint
        acados_ocp_solver.set(0, "lbx", xcurrent)
        acados_ocp_solver.set(0, "ubx", xcurrent)


        # solve ocp
        status = acados_ocp_solver.solve()

        simU[i, :] = acados_ocp_solver.get(0, "u")

        
        print("control time: ", time.time()-start_time)


        print("status", status)
        # simulate system
        xcurrent = unicycle.integrate(xcurrent, simU[i, 0], simU[i, 1])


        print("state: ", xcurrent)
        simX[i + 1, :] = xcurrent



if __name__ == "__main__":
    closed_loop_simulation()