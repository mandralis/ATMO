from __future__ import print_function, division
import numpy as np 
from atmo.mpc.parameters import params_

# get parameters
v_max_absolute             = params_.get('v_max_absolute')
m                          = params_.get('m')
g                          = params_.get('g')
T_max                      = params_.get('T_max')
u_max                      = params_.get('u_max')
z0                         = params_.get('z0')
zf                         = params_.get('zf')
Ts                         = params_.get('Ts')

def traj_fly_up(t):
    drive_vel = [0.0,0.0]
    done = False
    H = -5.0
    v_up = -0.5
    z0 = 0.0

    t1 = H/v_up

    tilt_vel = 0.0
    
    if (t<t1): 
        z, dz    = z0 + v_up*t, v_up
        if t>0.5*t1:
            tilt_vel = 1.0
    else:
        z,dz = z0 + H, 0.0
        done = True

    x_ref = np.zeros(12)
    x_ref[2] = z
    x_ref[8] = dz
    u_ref = m*g/T_max*np.ones(4)
    # tilt_vel = 1.0
    return x_ref,u_ref,tilt_vel,drive_vel,done

def traj_jump_time(t):

    done = False
    drive_vel = [0.0,0.0] # drive speed, turn speed 

    H         = -5.0         # 1.5 m 
    H_down    =  H - zf
    v_up      = -0.50
    v_down    =  0.30 
    v_forward =  0.0         # 0.0
    a_forward =  0.0

    t1 = H/v_up
    t2 = t1 + 2.0
    t3 = t2 + (-H_down/v_down)
    t_tilt = 0.8*t2 + 0.2*t3
    
    if (t<t1): 
        x, dx    = 0.0, 0.0
        z, dz    = z0 + v_up*t, v_up
        tilt_vel = 0.0
    elif ((t>t1) and (t<t2)):
        x, dx    = 0.0, 0.0
        z, dz    = z0 + H, 0.0
        tilt_vel = 0.0
    elif ((t>t2) and (t<t3)):
        x, dx    = v_forward*(t-t2) + a_forward*(t-t2)*(t-t2), v_forward + a_forward*(t-t2)
        z,dz = z0 + H + v_down * (t-t2), v_down
        tilt_vel = 0.0
        if t>t_tilt:
            tilt_vel = 1.0
    elif (t>t3):
        x,dx = v_forward*(t3-t2)+ a_forward*(t3-t2)*(t3-t2),0.0
        z,dz = z0 + H + v_down * (t3-t2),0.0
        tilt_vel = 1.0
        done = True

    x_ref = np.zeros(12)
    x_ref[0] = x
    x_ref[2] = z
    x_ref[6] = dx
    x_ref[8] = dz

    u_ref = m*g/T_max*np.ones(4)
    return x_ref,u_ref,tilt_vel,drive_vel,done

def traj_descent_time(t):
    out = np.zeros(12)
    out[2] = -2.0 + t*0.5
    out[8] = 0.5
    if out[2] >= 0.0 : 
        out[2] = 0.0
        out[0] = 4.0*1.0
    else:
        out[0] = t*1.0
    return out

def traj_circle_time(t):

    x_ref,u_ref,tilt_vel,drive_vel,done_up = traj_fly_up(t)

    if done_up:
        drive_vel = [0.0,0.0]
        done = False 
        if t > 5.0 : done = True

        x_ref = np.zeros(12)
        x_ref[0] = -4.0 + 4.0*np.cos(2*np.pi/5.0*t)
        x_ref[1] = 4.0*np.sin(2*np.pi/5.0*t)
        x_ref[2] = -5.0

        u_ref = m*g/T_max*np.ones(4)
        tilt_vel = 0.0
    else:
        done = False
    return x_ref,u_ref,tilt_vel,drive_vel,done
