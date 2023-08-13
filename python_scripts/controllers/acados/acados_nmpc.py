from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from acados_model import export_robot_model
import numpy as np
import scipy.linalg

import time

import sys

class Acados_NMPC:
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

        self.last_solutions_x = None
        self.last_solutions_u = None

    def reset(self,):
        """Every control class should have a reset function
        """
        return


    def create_ocp_solver_description(self,) -> AcadosOcp:
        # Create ocp object to formulate the OCP
        ocp = AcadosOcp()

        model = export_robot_model()
        ocp.model = model
        nx = model.x.size()[0]
        nu = model.u.size()[0]
        ny = nx + nu

        # Set dimensions
        ocp.dims.N = self.horizon


        # Set cost
        Q_mat = 2 * np.diag([5, 5, 2])  # [x,y,yaw]
        R_mat = 1 * np.diag([0.1, 0.1])

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

        '''# Set constraints
        v_max = 1 
        w_max = 1 
        ocp.constraints.lbu = np.array([-v_max, -w_max])
        ocp.constraints.ubu = np.array([+v_max, +w_max])
        ocp.constraints.idxbu = np.array([0,1])'''

        X0 = np.array([0.0, 0.0, 0.0])
        ocp.constraints.x0 = X0

        # Set options
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # 'GAUSS_NEWTON', 'EXACT'
        ocp.solver_options.integrator_type = "IRK"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"  # SQP_RTI, SQP
        ocp.solver_options.nlp_solver_max_iter = 400
        # ocp.solver_options.levenberg_marquardt = 1e-2

        # Set prediction horizon
        ocp.solver_options.tf = self.T_horizon

        return ocp


    def compute_control(self, state, reference_x, reference_y):

        
        '''# Initialize solver
        for stage in range(self.horizon + 1):
            self.acados_ocp_solver.set(stage, "x", state * np.ones((self.state_dim,)))
        for stage in range(self.horizon):
            self.acados_ocp_solver.set(stage, "u", np.zeros((self.control_dim,)))'''

 
        ref_yaw = 0
        # Fill cost function using flat outputs ---------------------------------------------------
        for j in range(self.horizon):
            ref_x = reference_x[j]
            ref_y = reference_y[j]

            ref_x_dot = (reference_x[j+1] - ref_x)/self.dt
            ref_y_dot = (reference_y[j+1] - ref_y)/self.dt
            ref_yaw = np.arctan2(ref_y_dot, ref_x_dot) 

            ref_v = 0
            ref_w = 0
            if(j < self.horizon-1):
                ref_x_ddot = (((reference_x[j+2] - reference_x[j+1])/self.dt ) - ref_x_dot)/self.dt
                ref_y_ddot = (((reference_y[j+2] - reference_y[j+1])/self.dt ) - ref_y_dot)/self.dt
                
                ref_v = np.sqrt(ref_x_dot*ref_x_dot + ref_y_dot*ref_y_dot)
                ref_w = (ref_y_ddot*ref_x_dot - ref_x_ddot*ref_y_dot)/(ref_x_dot*ref_x_dot + ref_y_dot*ref_y_dot + 0.01)

            yref = np.array([ref_x, ref_y, ref_yaw, ref_v, ref_w])
            self.acados_ocp_solver.set(j, "yref", yref)
            #print("yref", yref)
        
        # Fill last step horizon ---------------------------------------------------
        ref_x = reference_x[self.horizon]
        ref_y = reference_y[self.horizon]
        yref_N = np.array([ref_x, ref_y, ref_yaw])
        self.acados_ocp_solver.set(self.horizon, "yref", yref_N)


        # Set initial state constraint ---------------------------------------------
        self.acados_ocp_solver.set(0, "lbx", state)
        self.acados_ocp_solver.set(0, "ubx", state)


        # Solve ocp -----------------------------------------------------------------
        status = self.acados_ocp_solver.solve()

        control = self.acados_ocp_solver.get(0, "u")




        return control[0],control[1]
