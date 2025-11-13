from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class JAMALRoughCfg( LeggedRobotCfg ):
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.40] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            # 'FL_hip_joint': 0.0,   # [rad] was 0.1
            # 'RL_hip_joint': 0.0,   # [rad]
            # 'FR_hip_joint': -0.0 ,  # [rad]
            # 'RR_hip_joint': -0.0,   # [rad]
            'FL_hip_joint': -0.1,   # [rad] was 0.1
            'RL_hip_joint': -0.1,   # [rad]
            'FR_hip_joint': 0.1 ,  # [rad]
            'RR_hip_joint': 0.1,   # [rad]

            'FL_thigh_joint': 0.793,     # [rad]
            'RL_thigh_joint': 0.593,   # [rad] was 0.793
            'FR_thigh_joint': 0.793,     # [rad]
            'RR_thigh_joint': 0.593,   # [rad] was 0.793

            'FL_calf_joint': -1.509,   # [rad]
            'RL_calf_joint': -1.509,    # [rad]
            'FR_calf_joint': -1.509,  # [rad]
            'RR_calf_joint': -1.509,    # [rad]
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        # stiffness = {'joint': 20.}  # [N*m/rad]
        # damping = {'joint': 0.5}     # [N*m*s/rad]
        stiffness = {'joint': 60.0}  # [N*m/rad]
        damping = {'joint': 1.0}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4
        hip_reduction = 0.5 #was 1.0

    class asset( LeggedRobotCfg.asset ):
        # file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go2/urdf/go2.urdf'
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/jamal/urdf/jamal.urdf'
        name = "jamal"
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf", "base"]
        terminate_after_contacts_on = ["base"]
        privileged_contacts_on = ["base", "thigh", "calf"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = True

    class commands( LeggedRobotCfg.commands ):
            curriculum = True
            max_curriculum = 2.0
            num_commands = 4 # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
            resampling_time = 10. # time before command are changed[s]
            # heading_command =True # if true: compute ang vel command from heading error
            heading_command = False # if true: compute ang vel command from heading error
            
            class ranges( LeggedRobotCfg.commands.ranges):
                lin_vel_x = [1, 1] # min max [m/s]
                lin_vel_y = [-1, 1]   # min max [m/s]
                ang_vel_yaw = [-3.14, 3.14]    # min max [rad/s]
                # lin_vel_y = [0, 0]   # min max [m/s]
                # ang_vel_yaw = [0, 0]    # min max [rad/s]
                heading = [-3.14, 3.14]

    class rewards( LeggedRobotCfg.rewards ):
        # /////////////////working rewards////////////////
        # soft_dof_pos_limit = 0.9
        # base_height_target = 0.35
        # //////////////////new rewards////////////////
        only_positive_rewards = False # if true negative total rewards are clipped at zero (avoids early termination problems)
        tracking_sigma = 0.25 # tracking reward = exp(-error^2/sigma)
        soft_dof_pos_limit = 1. # percentage of urdf limits, values above this limit are penalized
        soft_dof_vel_limit = 1.
        soft_torque_limit = 1.
        base_height_target = 0.34
        max_contact_force = 100. # forces above this value are penalized
        clearance_height_target = -0.1 # was-0.2
        class scales( LeggedRobotCfg.rewards.scales ):
            # /////////////////working rewards////////////////
            # torques = -0.0002
            # dof_pos_limits = -12.0

            # //////////////////new rewards////////////////
            # # Height / posture stabilizers
            # base_height = -5.0         # was -0.; enable it
            # # orientation = -0.5         # was -0.; penalize bad orientation (pitch/roll/yaw err)
            # # # Effort
            # # torques = -0.0002          # was -1e-5; give effort some weight
            # # termination = -0.05         # was -0.0; make falling clearly bad
            # # # Gait shaping
            # feet_air_time = 1.2      # was +1.0; reduce a lot to avoid pogo
            # # # Optional (if your env supports it):
            # # dof_pos_limits = -14.0
            # stumble = -5.0

            termination = -0.0
            tracking_lin_vel = 1.0
            tracking_ang_vel = 0.5
            lin_vel_z = -1 # was-2
            ang_vel_xy = -0.05
            orientation = -5.0# was -0.2
            dof_acc = -2.5e-7
            joint_power = -2e-5
            base_height = -10 # was-1
            foot_clearance = -0.04 # was -0.01
            action_rate = -0.01
            smoothness = -0.01
            feet_air_time =  1 #was 0
            collision = -1.0 #was 0
            feet_stumble = -0.0
            stand_still = 0.
            torques = -0.0
            dof_vel = -0.0
            dof_pos_limits = -5.0 # was 0
            dof_vel_limits = -0.0
            torque_limits = -0.0
            
        

class JAMALRoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'rough_jamal'
