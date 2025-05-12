
import numpy as np

# DH parameters of the UR5 robot
ur5_dh_params = [   
    {'a':0, 'd':0.089159, 'alpha': np.pi/2, 'theta':0}, 
    {'a':-0.425, 'd':0, 'alpha': 0, 'theta':0},
    {'a':-0.39225, 'd':0, 'alpha':0, 'theta':0},
    {'a':0, 'd':0.10915, 'alpha': np.pi/2,'theta':0},
    {'a':0, 'd':0.09465, 'alpha': -np.pi/2, 'theta':0},
    {'a':0, 'd':0.0823, 'alpha': 0, 'theta':0}
]


def forward_kinematics( dh_params:list, q:list, base_world_transform:np.ndarray=np.identity(4))->list:
    kinematic_chain = [base_world_transform]
    for i in range(0, len(dh_params)):
        a_i, d_i, alpha_i = (dh_params[i]["a"], dh_params[i]["d"], dh_params[i]["alpha"])

        theta_tot = q[i] + dh_params[i]["theta"]
        
        c_theta_i, s_theta_i = np.cos(theta_tot), np.sin(theta_tot) 
        c_alpha_i, s_alpha_i = np.cos(alpha_i), np.sin(alpha_i)

        T_i = kinematic_chain[-1]@np.array([[c_theta_i, -s_theta_i*c_alpha_i, s_theta_i*s_alpha_i, a_i*c_theta_i],
                                            [s_theta_i, c_theta_i*c_alpha_i, -c_theta_i*s_alpha_i, a_i*s_theta_i],
                                            [0, s_alpha_i, c_alpha_i, d_i],
                                            [0, 0, 0, 1]
                                            ])
        kinematic_chain.append(T_i)
    
    return kinematic_chain   