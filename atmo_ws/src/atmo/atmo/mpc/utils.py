import numpy as np
from atmo.mpc.parameters import params_

varphi_g      = params_.get('varphi_g')
l_pivot_wheel = params_.get('l_pivot_wheel')
h_bot_pivot   = params_.get('h_bot_pivot')
wheel_base    = params_.get('wheel_base')
wheel_radius  = params_.get('wheel_radius')
z_ground_base = params_.get('z_ground_base')
max_wheel_angular_velocity = params_.get('max_wheel_angular_velocity')
max_drive_speed = params_.get('max_drive_speed')
max_turn_speed  = params_.get('max_turn_speed')

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Below should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw    

def quaternion_from_euler(phi,th,psi):
    w = np.cos(phi/2)*np.cos(th/2)*np.cos(psi/2) + np.sin(phi/2)*np.sin(th/2)*np.sin(psi/2)
    x = np.sin(phi/2)*np.cos(th/2)*np.cos(psi/2) - np.cos(phi/2)*np.sin(th/2)*np.sin(psi/2)
    y = np.cos(phi/2)*np.sin(th/2)*np.cos(psi/2) + np.sin(phi/2)*np.cos(th/2)*np.sin(psi/2)
    z = np.cos(phi/2)*np.cos(th/2)*np.sin(psi/2) - np.sin(phi/2)*np.sin(th/2)*np.cos(psi/2)
    return np.array([w,x,y,z])

def drive_mixer(drive_speed,turn_speed):
    R,l = wheel_radius,wheel_base
    u_right = 1/R * (drive_speed*max_drive_speed - l*turn_speed*max_turn_speed)
    u_left  = 1/R * (drive_speed*max_drive_speed + l*turn_speed*max_turn_speed)
    return u_left/max_wheel_angular_velocity, u_right/max_wheel_angular_velocity

def z_schedule(z,zstar,zg):
    z,zstar,zg = abs(z),abs(zstar),abs(zg)
    if z >= zstar:
        return 1.0
    elif zg <= z <= zstar:
        return (z - zg)/(zstar - zg)
    else:
        return 0.0

def distance_ground_robot(x_current, phi_current):
    z_base = abs(x_current[2]) 
    dH     = 0.0
    if (varphi_g <= phi_current <= np.pi/2):
        dH = l_pivot_wheel*np.sin(phi_current) - h_bot_pivot
    return -(z_base - dH) + abs(z_ground_base)