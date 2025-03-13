import numpy as np
from os import getenv

# Declare parameter dictionary
params_ = {}

# High level parameters
params_['Ts']                    = 0.007                                                                 # control frequency of MPC
params_['Ts_tilt_controller']    = params_.get('Ts')                                                     # control frequency of TiltController
params_['Ts_drive_controller']   = params_.get('Ts')                                                     # control frequency of DriveController
params_['queue_size']            = 1                                                                     # queue size of ros2 messages
params_['warmup_time']           = 1.0                                                                   # time after which mpc controller is started (seconds)
params_['max_tilt_in_flight']    = np.deg2rad(50)
params_['max_tilt_on_land']      = np.deg2rad(85)

# RC channels
params_['tilt_channel']          = 9
params_['encoder_channel']       = 11
params_['mpc_channel']           = 7
params_['offboard_channel']      = 8
params_['roll_channel']          = 0
params_['pitch_channel']         = 1
params_['throttle_channel']      = 2
params_['yaw_channel']           = 3
params_['min']                   = 1094
params_['max']                   = 1934
params_['dead']                  = 1514

# Manual control parameters
params_['max_dx']                = 0.75                           # max x velocity
params_['max_dy']                = 0.75                           # max y velocity
params_['max_dz']                = 0.75                           # max z velocity
params_['max_dpsi']              = np.pi/4                        # max yaw angular velocity
params_['tilt_height']           = -1.2                           # height at which tilt is enabled
params_['initial_tilt_sim']      = 0.0                            # initial tilt angle in simulation (radian)
params_['d2f_throttle_thresh']   = 0.3                            # throttle value above which we switch from driving to flight mode

# Transition parameters
params_['l_pivot_wheel']         = 0.261                                                                                                                                                                                # distance from pivot point to wheel exterior (look at y distance in SDF and add radius of wheel (0.125))
params_['h_bot_pivot']           = 0.094                                                                                                                                                                                # distance from bottom plate to pivot point (look at z distance in SDF)
params_['varphi_g']              = np.arctan(params_.get('h_bot_pivot')/params_.get('l_pivot_wheel'))                                                                                                                   # angle of pivot point when robot is on ground and wheels just touch the ground
params_['z_ground_base']         = -0.127                                                                                                                                                                               # (exp: -0.113 TBD) height that optitrack registers when robot is on ground with arms at 0 degrees
params_['h_wheel_ground']        = -0.17                                                                                                                                                                                # distance from wheel to ground when transition begins                                                 
params_['z_star']                = params_.get('h_wheel_ground') + params_.get('z_ground_base') - (params_.get('l_pivot_wheel') * np.sin(params_.get('max_tilt_in_flight')) - params_.get('h_bot_pivot'))               # transition height
params_['u_ramp_down_rate']      = 0.15  

# State machine parameters
params_['land_tolerance']        = -0.03                          # tolerance to consider robot landed
params_['takeoff_height']        = -1.0                           # height at which we consider robot in flight

# Trajectory parameters
params_['z0']                    = -0.16                        
params_['zf']                    = -0.30

# Roboclaw addresses
params_['tilt_roboclaw_address']         = "/dev/ttyACM1"
params_['drive_roboclaw_address']        = "/dev/ttyACM0"

# kinematic driving parameters
params_['wheel_base']                    = 0.135      # half the distance between the wheels
params_['wheel_radius']                  = 0.125      # the wheel radius 
params_['max_wheel_angular_velocity']    = 10.0       # rad/s
params_['max_drive_speed']               = params_.get('wheel_radius') * params_.get('max_wheel_angular_velocity')  # m/s
params_['max_turn_speed']                = params_.get('wheel_radius') * params_.get('max_wheel_angular_velocity') / params_.get('wheel_base')   # rad/s

# MPC parameters
params_['acados_ocp_path']       = getenv("ATMO") +'/atmo_ws/src/atmo/atmo/mpc/acados/'                  # path that acados model is compiled to
params_['generate_mpc']          = True                                                                  # generate acados model
params_['build_mpc']             = True                                                                  # build acados model
params_['cost_update_freq']      = 10 * params_.get('Ts')                                                # frequency at which cost is updated (seconds)
params_['N_horizon']             = 10
params_['T_horizon']             = 1.2

# MPC constraints
params_['u_max']            = 1.0
params_['v_max_absolute']   = (np.pi/2)/4
params_['T_max']            = 4*params_['kT']

# cost function parameters
params_['w_x']        = 1.0     
params_['w_y']        = 1.0     
params_['w_z']        = 1.0     
params_['w_dx']       = 10.0
params_['w_dy']       = 10.0
params_['w_dz']       = 20.0
params_['w_phi']      = 0.1     
params_['w_th']       = 0.1     
params_['w_psi']      = 0.1   
params_['w_ox']       = 3.0   
params_['w_oy']       = 5.0   
params_['w_oz']       = 1.5   
params_['w_u']        = 1.0       
params_['rho']        = 0.1     
params_['gamma']      = 1.0     

# cost function
params_['Q_mat']          = np.diag([params_['w_x'],
                                     params_['w_y'],
                                     params_['w_z'],
                                     params_['w_psi'],
                                     params_['w_th'],
                                     params_['w_phi'],
                                     params_['w_dx'],
                                     params_['w_dy'],
                                     params_['w_dz'],
                                     params_['w_ox'],
                                     params_['w_oy'],
                                     params_['w_oz']])
params_['R_mat']          = params_['rho'] * np.diag([params_['w_u'],params_['w_u'],params_['w_u'],params_['w_u']])
params_['Q_mat_terminal'] = params_['gamma'] * params_['Q_mat']

# cost function parameters near ground
params_['w_x_near_ground']        = 0.0  
params_['w_y_near_ground']        = 0.0
params_['w_z_near_ground']        = 0.0
params_['w_dx_near_ground']       = 0.0
params_['w_dy_near_ground']       = 0.0
params_['w_dz_near_ground']       = 0.0
params_['w_phi_near_ground']      = 0.0
params_['w_th_near_ground']       = 0.0
params_['w_psi_near_ground']      = 0.0  
params_['w_ox_near_ground']       = 3.0
params_['w_oy_near_ground']       = 5.0
params_['w_oz_near_ground']       = 1.5
params_['w_u_near_ground']        = 1.0
params_['rho_near_ground']        = 0.1
params_['gamma_near_ground']      = 1.0

# cost function near ground
params_['Q_mat_near_ground']          = np.diag([params_['w_x_near_ground'],
                                                 params_['w_y_near_ground'],
                                                 params_['w_z_near_ground'],
                                                 params_['w_psi_near_ground'],
                                                 params_['w_th_near_ground'],
                                                 params_['w_phi_near_ground'],
                                                 params_['w_dx_near_ground'],
                                                 params_['w_dy_near_ground'],
                                                 params_['w_dz_near_ground'],
                                                 params_['w_ox_near_ground'],
                                                 params_['w_oy_near_ground'],
                                                 params_['w_oz_near_ground']])
params_['R_mat_near_ground']          = params_['rho_near_ground'] * np.diag([params_['w_u_near_ground'],params_['w_u_near_ground'],params_['w_u_near_ground'],params_['w_u_near_ground']])
params_['Q_mat_terminal_near_ground'] = params_['gamma_near_ground'] * params_['Q_mat_near_ground']

# dynamic model parameters
params_['g']                = 9.81                                                                   # gravitational acceleration
params_['kT']               = 28.15                                                                  # thrust coefficient
params_['kM']               = 0.018                                                                  # moment coefficient
params_['m_base']           = 2.33                                                                   # mass of base
params_['m_arm']            = 1.537                                                                  # mass of arm
params_['m_rotor']          = 0.021                                                                  # mass of rotor 
params_['m']                = params_['m_base'] + 2*params_['m_arm'] + 4*params_['m_rotor']          # total mass 
params_['I_base_xx']        = 0.0067
params_['I_base_yy']        = 0.011
params_['I_base_zz']        = 0.0088
params_['I_base_xy']        = -0.000031
params_['I_base_xz']        = 0.00046
params_['I_base_yz']        = 0.00004
params_['I_arm_xx']         = 0.008732
params_['I_arm_yy']         = 0.036926
params_['I_arm_zz']         = 0.043822
params_['I_arm_xy']         = 0.000007
params_['I_arm_xz']         = -0.000012
params_['I_arm_yz']         = 0.000571
params_['I_rotor_xx']       = 0.000022
params_['I_rotor_yy']       = 0.000022
params_['I_rotor_zz']       = 0.000043
params_['r_BA_right_x']     = 0.0066
params_['r_BA_right_y']     = 0.0685
params_['r_BA_right_z']     = -0.021
params_['r_AG_right_x']     = -0.00032
params_['r_AG_right_y']     = 0.16739
params_['r_AG_right_z']     = -0.02495
params_['r_AR1_x']          = 0.16491
params_['r_AR1_y']          = 0.13673
params_['r_AR1_z']          = -0.069563