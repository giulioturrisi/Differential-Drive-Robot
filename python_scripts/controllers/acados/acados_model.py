from acados_template import AcadosModel
from casadi import SX, vertcat

import sys
sys.path.append('/home/python_scripts/')
from robot_model import Robot

# Reference for model equations:
# http://users.isr.ist.utl.pt/~jag/publications/08-JETC-RCarona-vcontrol.pdf

def export_robot_model() -> AcadosModel:
    model_name = "unicycle"

    # set up states & controls
    x = SX.sym("x")
    y = SX.sym("y")
    yaw = SX.sym("yaw")

    #x_d = SX.sym("x_d")
    #pitch_d = SX.sym("pitch_d")
    #yaw_d = SX.sym("yaw_d")

    x = vertcat(x, y, yaw)

    v = SX.sym("v")
    w = SX.sym("w")
    u = vertcat(v, w)

    # xdot
    x_dot = SX.sym("x_dot")
    y_dot = SX.sym("y_dot")
    yaw_dot = SX.sym("yaw_dot")



    xdot = vertcat(x_dot, y_dot, yaw_dot)

    # algebraic variables
    # z = None

    # parameters
    p = []

    # dynamics
    #f_expl = vertcat(v * cos(theta), v * sin(theta), F, theta_d, T)
    unicycle = Robot(dt = 0.01)
    f_expl = unicycle.fk(x, u)

    print("f_expl", f_expl)

    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    # model.z = z
    model.p = p
    model.name = model_name

    return model