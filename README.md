### Install
```bash
conda env create -f magiclab_mujoco.yaml -n magiclab_sim2sim
```

### Run

Run simulator:
```bash
python deploy_mujoco/z1_12dof_prop_mujoco.py --config=deploy_mujoco/configs/z1_12dof.yaml
```

Test command input program (in another terminal):
```bash
python deploy_mujoco/gamepad_reader_btp.py
```



修改后的magiclab_mujoco，完善了手柄操控，键盘操控不太对，不要用
