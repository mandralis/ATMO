# Acados/Casadi
from acados_template import AcadosOcp
import scipy.linalg
import numpy as np
from atmo.mpc.parameters import params_
from atmo.mpc.dynamics import export_robot_model

# constraint variables
u_max                       = params_.get('u_max')
v_max_absolute              = params_.get('v_max_absolute')

# collocation parameters in flight
N_horizon                   = params_.get('N_horizon')
T_horizon                   = params_.get('T_horizon')

# cost function in flight
Q_mat                       = params_.get('Q_mat')
R_mat                       = params_.get('R_mat')
Q_mat_terminal              = params_.get('Q_mat_terminal')

# temporary references (these get overwritten the first time mpc is called)
x_ref                       = np.zeros(12,dtype='float')
u_ref                       = np.zeros(4,dtype='float')
x0                          = np.zeros(12,dtype='float')
varphi0                     = 0.0
grounded0                   = True

# export directory
acados_ocp_path             = params_.get('acados_ocp_path')

def create_ocp_solver_description() -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    model = export_robot_model())

    ocp.model = model
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu

    # set dimensions
    ocp.dims.N = N_horizon

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    ny = nx + nu
    ny_e = nx

    ocp.cost.W_e = Q_mat_terminal
    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[nx : nx + nu, 0:nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.hstack((x_ref,u_ref))
    ocp.cost.yref_e = x_ref

    # set constraints
    ocp.constraints.lbu = np.array([0,0,0,0])
    ocp.constraints.ubu = np.array([+u_max,+u_max,+u_max,+u_max])
    ocp.constraints.idxbu = np.array([0,1,2,3])
    
    ocp.constraints.x0 = x0

    # parameters
    ocp.parameter_values = np.array([varphi0])
    
    # set options
    ocp.solver_options.qp_solver       = "FULL_CONDENSING_HPIPM"       # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx  = "GAUSS_NEWTON"                # GAUSS_NEWTON, EXACT
    ocp.solver_options.integrator_type = "ERK"                         # ERK      
    ocp.solver_options.nlp_solver_type = "SQP_RTI"                     # SQP_RTI, SQP

    # set prediction horizon
    ocp.solver_options.tf = T_horizon

    # export directory
    ocp.code_export_directory = acados_ocp_path

    return ocp
