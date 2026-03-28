import time
import os
import mujoco.viewer
import mujoco
import numpy as np
import torch
import yaml
import sys
from scipy.spatial.transform import Rotation as R
import gamepad_reader_btp


# --- 辅助函数 ---
def get_gravity_orientation(quaternion):
    qw, qx, qy, qz = quaternion
    gravity_orientation = np.zeros(3)
    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)
    return gravity_orientation


def pd_control(target_q, q, kp, dq, kd):
    return (target_q - q) * kp - dq * kd


def update_hist(hist_buf, cur, dim):
    cur = cur.to(hist_buf.device)
    res = torch.cat([hist_buf[:, dim:], cur], dim=-1)
    return res


if __name__ == "__main__":
    import argparse

    gamepad = gamepad_reader_btp.Gamepad()
    command_function = gamepad.get_command

    parser = argparse.ArgumentParser()
    parser.add_argument("--config_file", type=str, default="configs/p2_prop.yaml")
    args = parser.parse_args()

    with open(f"{args.config_file}", "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        # 加载基础参数
        policy_path = config["policy_path"]
        xml_path = config["xml_path"]
        simulation_dt = config["simulation_dt"]
        control_decimation = config["control_decimation"]
        kps = np.array(config["kps"], dtype=np.float32)
        kds = np.array(config["kds"], dtype=np.float32)
        default_angles = np.array(config["default_angles"], dtype=np.float32)
        lin_vel_scale = config["lin_vel_scale"]
        ang_vel_scale = config["ang_vel_scale"]
        rpy_scale = config["rpy_scale"]
        dof_pos_scale = config["dof_pos_scale"]
        dof_vel_scale = config["dof_vel_scale"]
        action_scale = config["action_scale"]
        cmd_scale = np.array(config["cmd_scale"], dtype=np.float32)
        num_actions = config["num_actions"]
        num_obs = config["num_obs"]
        his_obs = config["his_obs"]
        controller_scale = np.array(config["controller_scale"], dtype=np.float32)
        cycle_time = 0.6

    # --- 变量初始化 ---
    action = np.zeros(num_actions, dtype=np.float32)
    target_dof_pos = default_angles.copy()
    current_smooth_cmd = np.zeros(3, dtype=np.float32)

    # --- 核心参数设定 ---
    VEL_LIMITS = np.array([2.5, 1.0, 1.5])
    MAX_ACCEL = 3.0
    MAX_DECEL = 2.0  # 增大刹车力度，让减速立竿见影

    # 针对你的北通手柄：松开时输出 1，所以 1.1 以下全部丢弃
    FILTER_THRESHOLD = 0.1
    # ------------------

    device = torch.device('cpu')
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    d.qpos[7:7 + len(default_angles)] = default_angles
    m.opt.timestep = simulation_dt
    policy = torch.jit.load(policy_path).to(device).eval()

    history = {k: torch.zeros((1, (3 if k in ['ang_vel', 'gravity', 'cmd'] else
                                   num_actions if k in ['qj', 'dqj', 'actions'] else 2) * his_obs), device=device)
               for k in ["ang_vel", "gravity", "cmd", "qj", "dqj", "actions", "gait"]}

    print(f"\n--- Hard Filter Mode (Ignore < {FILTER_THRESHOLD}) ---")

    with mujoco.viewer.launch_passive(m, d) as viewer:
        counter = 0
        while viewer.is_running():
            step_start = time.time()

            # 物理步进
            tau = pd_control(target_dof_pos, d.qpos[7:], kps, d.qvel[6:], kds)
            d.ctrl[:] = tau
            mujoco.mj_step(m, d)

            # 1. 获取手柄原始输入
            raw_lin, raw_ang, e_stop, _ = command_function()
            if e_stop:
                sys.exit(0)
            raw_input = np.array([raw_lin[0], raw_lin[1], raw_ang])

            # 2. 【硬过滤逻辑】：小于 1.1 直接不要，大于 1.1 原样保留
            filtered_joy = np.zeros(3)
            for i in range(3):
                if np.abs(raw_input[i]) > FILTER_THRESHOLD:
                    filtered_joy[i] = raw_input[i]  # 直接用原值，不减去偏移
                else:
                    filtered_joy[i] = 0.0  # 彻底丢弃

            # 3. 计算期望速度并应用上限
            target_joy_cmd = np.array([
                filtered_joy[0] * 0.5 * controller_scale[0],
                filtered_joy[1] * 0.5 * controller_scale[1],
                filtered_joy[2] * 1.0 * controller_scale[2]
            ])
            target_joy_cmd = np.clip(target_joy_cmd, -VEL_LIMITS, VEL_LIMITS)

            # 4. 加速与自动减速
            for i in range(3):
                # 手柄只要被过滤成 0，立刻进入刹车模式
                joy_released = (filtered_joy[i] == 0.0)
                is_braking = joy_released or (np.sign(target_joy_cmd[i]) != np.sign(current_smooth_cmd[i]) and np.abs(
                    current_smooth_cmd[i]) > 0.01)

                # 刹车用大加速度 (MAX_DECEL)，起步用普通加速度 (MAX_ACCEL)
                step_accel = MAX_DECEL if is_braking else MAX_ACCEL
                step_val = step_accel * simulation_dt

                diff = target_joy_cmd[i] - current_smooth_cmd[i]
                current_smooth_cmd[i] += np.clip(diff, -step_val, step_val)

                # 微小速度清零，防止漂移
                if joy_released and np.abs(current_smooth_cmd[i]) < 0.01:
                    current_smooth_cmd[i] = 0.0

            cmd = current_smooth_cmd.copy()

            # 5. 模型推理
            if counter % control_decimation == 0:
                qj = d.qpos[7:]
                dqj = d.qvel[6:]
                quat = d.qpos[3:7]
                omega = d.qvel[3:6]
                gravity_orientation = get_gravity_orientation(quat)
                phase = (counter * simulation_dt) / cycle_time
                sin_phase = np.sin(2 * np.pi * phase)
                stance_mask = np.array([sin_phase >= 0, sin_phase < 0], dtype=np.float32)

                if np.all(cmd == 0):
                    stance_mask[:] = 1.0

                cur_obs = {
                    "ang_vel": torch.from_numpy(omega * ang_vel_scale).float().unsqueeze(0),
                    "gravity": torch.from_numpy(gravity_orientation * rpy_scale).float().unsqueeze(0),
                    "cmd": torch.from_numpy(cmd * cmd_scale).float().unsqueeze(0),
                    "qj": torch.from_numpy((qj - default_angles) * dof_pos_scale).float().unsqueeze(0),
                    "dqj": torch.from_numpy(dqj * dof_vel_scale).float().unsqueeze(0),
                    "actions": torch.from_numpy(action).float().unsqueeze(0),
                    "gait": torch.from_numpy(stance_mask).float().unsqueeze(0)
                }
                for k in history:
                    history[k] = update_hist(history[k], cur_obs[k], cur_obs[k].shape[-1])

                obs_tensor = torch.cat(
                    [history[k] for k in ["ang_vel", "gravity", "cmd", "qj", "dqj", "actions", "gait"]], dim=-1)
                with torch.no_grad():
                    action = policy(obs_tensor).cpu().numpy().squeeze()

                target_dof_pos = (action * action_scale) + default_angles

            # 实时看板：观察刹车状态
            if counter % 50 == 0:
                viewer.sync()
                print(
                    f"\r[RAW] {raw_input[0]:>5.2f} | [FILT] {filtered_joy[0]:>5.2f} | [CMD] {cmd[0]:>5.2f} | {'!! BRAKE !!' if np.all(filtered_joy == 0) else 'DRIVING'}",
                    end="")

            counter += 1
            time_to_sleep = m.opt.timestep - (time.time() - step_start)
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)