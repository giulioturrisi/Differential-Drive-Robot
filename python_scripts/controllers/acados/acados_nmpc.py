from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from acados_model import export_robot_model
import numpy as np
import scipy.linalg

import time

import sys

class NMPC:
    def __init__(self, horizon, dt):
        self.horizon = horizon  # Define the number of discretization steps
        self.dt = dt

        self.T_horizon = self.horizon*self.dt

        
        self.ocp = self.create_ocp_solver_description()
        self.acados_ocp_solver = AcadosOcpSolver(
            self.ocp, json_file="acados_nmpc_" + self.ocp.model.name + ".json"
        )

        self.state_dim = self.ocp.model.x.size()[0]
        self.control_dim = self.ocp.model.u.size()[0]

    def create_ocp_solver_description(self,) -> AcadosOcp:
        # create ocp object to formulate the OCP
        ocp = AcadosOcp()

        model = export_robot_model()
        ocp.model = model
        nx = model.x.size()[0]
        nu = model.u.size()[0]
        ny = nx + nu

        # set dimensions
        ocp.dims.N = self.horizon


        # set cost
        Q_mat = 2 * np.diag([5, 5, 1])  # [x,y,yaw]
        R_mat = 1 * np.diag([1, 1])

        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.cost_type_e = "LINEAR_LS"

        ny = nx + nu
        ny_e = nx

        ocp.cost.W_e = Q_mat
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
        v_max = 0.5 
        ocp.constraints.lbu = np.array([-v_max, -v_max])
        ocp.constraints.ubu = np.array([+v_max, +v_max])
        ocp.constraints.idxbu = np.array([0,1])

        X0 = np.array([0.0, 0.0, 0.0])
        ocp.constraints.x0 = X0

        # set options
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # 'GAUSS_NEWTON', 'EXACT'
        ocp.solver_options.integrator_type = "IRK"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"  # SQP_RTI, SQP
        ocp.solver_options.nlp_solver_max_iter = 400
        # ocp.solver_options.levenberg_marquardt = 1e-2

        # set prediction horizon
        ocp.solver_options.tf = self.T_horizon

        return ocp

    #def compute_control(self, state, state_des):
    def compute_control(self, state, reference_x, reference_y):

        state[2] += 0.02 
     
        # initialize solver
        for stage in range(self.horizon + 1):
            self.acados_ocp_solver.set(stage, "x", 0.0 * np.ones((self.state_dim,)))
        for stage in range(self.horizon):
            self.acados_ocp_solver.set(stage, "u", np.zeros((self.control_dim,)))

 
        ref_yaw = 0

        for j in range(self.horizon):
            ref_x = reference_x[j]
            ref_y = reference_y[j]

            ref_x_d = (reference_x[j+1] - ref_x)/self.dt
            ref_y_d = (reference_y[j+1] - ref_y)/self.dt
            ref_yaw = np.arctan2(ref_y_d, ref_x_d) 

            yref = np.array([ref_x, ref_y, ref_yaw, 0, 0])
            #print("yref", yref)
            self.acados_ocp_solver.set(j, "yref", yref)
        
        ref_x = reference_x[self.horizon]
        ref_y = reference_y[self.horizon]
        yref_N = np.array([ref_x, ref_y, ref_yaw])
        self.acados_ocp_solver.set(self.horizon, "yref", yref_N)


        # set initial state constraint
        self.acados_ocp_solver.set(0, "lbx", state)
        self.acados_ocp_solver.set(0, "ubx", state)


        # solve ocp
        status = self.acados_ocp_solver.solve()

        control = self.acados_ocp_solver.get(0, "u")

        print("control", control)
        



        return control[0],control[1]
