# 🦾 Ubuntu 从零安装 MuJoCo（官方新版）+ Python 仿真环境完整教程
(余达洋 2025.11.28)

本教程从最基础的系统配置开始，直到成功运行自定义的机械臂仿真程序，适用于所有 Ubuntu 用户（特别是 20.04 / 22.04）   
全流程基于 **官方新版 MuJoCo（mujoco 3.x）**

---

# 目录

1. [下载MuJoCo在Linux系统的官方文件](#1-下载MuJoCo在Linux系统的官方文件)
2. [下载Pycharm](#2-下载Pycharm)
3. [配置 Python 虚拟环境](#3-配置-Python-虚拟环境)
4. [运行程序](#4-运行程序)

---

# 1.下载MuJoCo在Linux系统的官方文件

访问MuJoCo官网：https://github.com/google-deepmind/mujoco/releases 进行下载

对于Linux的Ubuntu系统，应下载mujoco-版本号-linux-x86_64.tar.gz格式的文件选项

```python
#路径替换为你文件储存的真实路径
echo "export PATH=\$PATH:/home/dar/MuJoCoBin/mujoco-3.3.7/bin" >> ~/.bashrc
#修改立刻生效
source ~/.bashrc
```
下载完成后，将MuJoCo的可执行文件路径加入系统 PATH，使你可以在任何目录运行它

```python
simulate ~/MuJoCoBin/mujoco-3.3.7/model/humanoid/humanoid.xml
```
接下来可以执行上面的指令，应该会看到一个下载MuJoCo文件时附赠的一个小人建模，说明安装成功了

# 2.下载Pycharm

步骤 1：下载 PyCharm

👉 https://www.jetbrains.com/pycharm/download/

步骤 2：解压
```python
#替换真实下载文件
tar -xzf pycharm-*.tar.gz -C ~/opt
```

如果没有 opt 文件夹：
```python
mkdir -p ~/opt
```
步骤 3：运行 PyCharm
```python
#先cd到下载文件路径里的bin然后运行
./pycharm.sh
#也可以直接通过绝对路径运行
```

# 3.配置 Python 虚拟环境

这里我来提供一个openarm建模展示的脚本，可以直接粘贴使用
```python
import mujoco
import mujoco.viewer
import pyautogui

def viewer_init(viewer):
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.lookat[:] = [0, 0.5, 0.5]
    viewer.cam.distance = 2.5
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -30

def main():
    model = mujoco.MjModel.from_xml_path('scene.xml')  # 使用我提供的 XML
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer_init(viewer)

        while viewer.is_running():

            mujoco.mj_step(model, data)
            viewer.sync()

main()
```
下面是需要导入的模型和场景的xml，直接展开全文复制就可以用了

---

### openarm_bimanual.xml

<details>
<summary>点击展开（注意这个很多，真的很多）</summary>

```
<?xml version='1.0' encoding='utf-8'?>
<mujoco model="openarm">
  <default>
    <!-- <geom friction="0.0001" margin="0.0005" /> -->
    <joint stiffness="0" ref="0.0" />
    <default class="robot">
      <default class="motor_DM8009">
        <joint frictionloss="0.1" damping="0.4" limited="true" type="hinge" />
        <motor forcelimited="true" forcerange="-40 40" />
      </default>
      <default class="motor_DM4340">
        <joint frictionloss="0.1" damping="0.4" limited="true" type="hinge" />
        <motor forcelimited="true" forcerange="-27 27" />
      </default>
      <default class="motor_DM4310">
        <joint frictionloss="0.1" damping="0.4" limited="true" type="hinge" />
        <motor forcelimited="true" forcerange="-7 7" />
      </default>
      <default class="motor_finger">
        <joint limited="true" type="slide" damping="2" stiffness="0.5" />
        <position forcelimited="true" forcerange="-333 333" ctrllimited="true" ctrlrange="0 0.044"
          kp="100" />
      </default>
      <default class="collision">
        <geom material="collision_material" condim="3" conaffinity="1" priority="1" group="3"
          solref="0.005 1" friction="1 0.01 0.01" />
        <!-- <material name="collision_material" rgba="1 0 0 0.5" specular="0.2" shininess="0.1"/> -->
        <equality solimp="0.99 0.999 1e-05" solref="0.005 1" />
      </default>
      <default class="visual">
        <geom material="visualgeom" contype="0" conaffinity="0" group="2" />
      </default>
    </default>
  </default>

  <compiler angle="radian" meshdir="meshes" />

  <asset>
    <material name="default_material" rgba="0.7 0.7 0.7 1" />
    <material name="collision_material" rgba="1.0 0.28 0.1 0.9" />

    <mesh name="body_collision" file="collision/body/body_link0.stl" scale="0.001 0.001 0.001" />
    <mesh name="link0_collision" file="collision/arm/link0.stl" scale="0.001 0.001 0.001" />
    <mesh name="link0_collision_left" file="collision/arm/link0.stl" scale="0.001 -0.001 0.001" />
    <mesh name="link1_collision" file="collision/arm/link1.stl" scale="0.001 0.001 0.001" />
    <mesh name="link1_collision_left" file="collision/arm/link1.stl" scale="0.001 -0.001 0.001" />
    <mesh name="link2_collision" file="collision/arm/link2.stl" scale="0.001 0.001 0.001" />
    <mesh name="link2_collision_left" file="collision/arm/link2.stl" scale="0.001 -0.001 0.001" />
    <mesh name="link3_collision" file="collision/arm/link3.stl" scale="0.001 0.001 0.001" />
    <mesh name="link3_collision_left" file="collision/arm/link3.stl" scale="0.001 -0.001 0.001" />
    <mesh name="link4_collision" file="collision/arm/link4.stl" scale="0.001 0.001 0.001" />
    <mesh name="link5_collision" file="collision/arm/link5.stl" scale="0.001 0.001 0.001" />
    <mesh name="link5_collision_left" file="collision/arm/link5.stl" scale="0.001 -0.001 0.001" />
    <mesh name="link6_collision" file="collision/arm/link6.stl" scale="0.001 0.001 0.001" />
    <mesh name="link6_collision_left" file="collision/arm/link6.stl" scale="0.001 -0.001 0.001" />
    <mesh name="link7_collision" file="collision/arm/link7.stl" scale="0.001 0.001 0.001" />
    <mesh name="link7_collision_left" file="collision/arm/link7.stl" scale="0.001 -0.001 0.001" />
    <mesh name="hand_collision" file="collision/gripper/hand.stl" scale="0.001 0.001 0.001" />
    <mesh name="finger_collision" file="collision/gripper/finger.stl" scale="0.001 0.001 0.001" />
    <mesh name="finger_collision_left" file="collision/gripper/finger.stl"
      scale="0.001 -0.001 0.001" />

    <mesh name="body_link0_0.obj" file="visual/body/body_link0_0.obj" scale="0.001 0.001 0.001" />
    <mesh name="body_link0_1.obj" file="visual/body/body_link0_1.obj" scale="0.001 0.001 0.001" />
    <mesh name="body_link0_2.obj" file="visual/body/body_link0_2.obj" scale="0.001 0.001 0.001" />
    <mesh name="body_link0_3.obj" file="visual/body/body_link0_3.obj" scale="0.001 0.001 0.001" />
    <mesh name="body_link0_4.obj" file="visual/body/body_link0_4.obj" scale="0.001 0.001 0.001" />
    <mesh name="body_link0_5.obj" file="visual/body/body_link0_5.obj" scale="0.001 0.001 0.001" />

    <mesh name="link0_0.obj" file="visual/arm/link0_0.obj" scale="0.001 0.001 0.001" />
    <mesh name="link0_1.obj" file="visual/arm/link0_1.obj" scale="0.001 0.001 0.001" />
    <mesh name="link1_0.obj" file="visual/arm/link1_0.obj" scale="0.001 0.001 0.001" />
    <mesh name="link1_1.obj" file="visual/arm/link1_1.obj" scale="0.001 0.001 0.001" />
    <mesh name="link1_2.obj" file="visual/arm/link1_2.obj" scale="0.001 0.001 0.001" />
    <mesh name="link1_3.obj" file="visual/arm/link1_3.obj" scale="0.001 0.001 0.001" />
    <mesh name="link2_0.obj" file="visual/arm/link2_0.obj" scale="0.001 0.001 0.001" />
    <mesh name="link2_1.obj" file="visual/arm/link2_1.obj" scale="0.001 0.001 0.001" />
    <mesh name="link2_2.obj" file="visual/arm/link2_2.obj" scale="0.001 0.001 0.001" />
    <mesh name="link3_0.obj" file="visual/arm/link3_0.obj" scale="0.001 0.001 0.001" />
    <mesh name="link3_1.obj" file="visual/arm/link3_1.obj" scale="0.001 0.001 0.001" />
    <mesh name="link3_2.obj" file="visual/arm/link3_2.obj" scale="0.001 0.001 0.001" />
    <mesh name="link4_0.obj" file="visual/arm/link4_0.obj" scale="0.001 0.001 0.001" />
    <mesh name="link4_1.obj" file="visual/arm/link4_1.obj" scale="0.001 0.001 0.001" />
    <mesh name="link4_2.obj" file="visual/arm/link4_2.obj" scale="0.001 0.001 0.001" />
    <mesh name="link5_0.obj" file="visual/arm/link5_0.obj" scale="0.001 0.001 0.001" />
    <mesh name="link5_1.obj" file="visual/arm/link5_1.obj" scale="0.001 0.001 0.001" />
    <mesh name="link5_2.obj" file="visual/arm/link5_2.obj" scale="0.001 0.001 0.001" />
    <mesh name="link6_0.obj" file="visual/arm/link6_0.obj" scale="0.001 0.001 0.001" />
    <mesh name="link6_1.obj" file="visual/arm/link6_1.obj" scale="0.001 0.001 0.001" />
    <mesh name="link6_0_left.obj" file="visual/arm/link6_0.obj" scale="0.001 -0.001 0.001" />
    <mesh name="link6_1_left.obj" file="visual/arm/link6_1.obj" scale="0.001 -0.001 0.001" />
    <mesh name="link7_0.obj" file="visual/arm/link7_0.obj" scale="0.001 0.001 0.001" />
    <mesh name="link7_1.obj" file="visual/arm/link7_1.obj" scale="0.001 0.001 0.001" />
    <mesh name="link7_0_left.obj" file="visual/arm/link7_0.obj" scale="0.001 -0.001 0.001" />
    <mesh name="link7_1_left.obj" file="visual/arm/link7_1.obj" scale="0.001 -0.001 0.001" />

    <mesh name="hand_0.obj" file="visual/gripper/hand_0.obj" scale="0.001 0.001 0.001" />
    <mesh name="hand_1.obj" file="visual/gripper/hand_1.obj" scale="0.001 0.001 0.001" />
    <mesh name="finger_0.obj" file="visual/gripper/finger_0.obj" scale="0.001 0.001 0.001" />
    <mesh name="finger_0_flip.obj" file="visual/gripper/finger_0.obj" scale="0.001 -0.001 0.001" />
    <mesh name="finger_1.obj" file="visual/gripper/finger_1.obj" scale="0.001 0.001 0.001" />
    <mesh name="finger_1_flip.obj" file="visual/gripper/finger_1.obj" scale="0.001 -0.001 0.001" />

    <material name="matte_black" rgba="0.24705882 0.24705882 0.24705882 1.0"
      reflectance="0.05098039000000001" shininess="0.5" />
    <material name="metal_silver" rgba="0.79607843 0.79607843 0.79607843 1.0" reflectance="1.0"
      shininess="1.0" />
  </asset>

  <worldbody>

    <body name="openarm_body_link0" pos="0 0 0" quat="1.0 0.0 0.0 0.0">
      <inertial pos="0.0 0.0 0.0" quat="1.0 0.0 0.0 0.0" mass="13.89"
        diaginertia="1.653 1.653 0.051" />
      <geom name="openarm_body_link0_collision" pos="0.0 0.0 0.0" quat="1.0 0.0 0.0 0.0" type="mesh"
        mesh="body_link0_3.obj" class="collision" />
      <geom name="openarm_body_link0_visual_0" pos="0.0 0.0 0.0" quat="1.0 0.0 0.0 0.0"
        material="matte_black" type="mesh" mesh="body_link0_0.obj" class="visual" />
      <geom name="openarm_body_link0_visual_1" pos="0.0 0.0 0.0" quat="1.0 0.0 0.0 0.0"
        material="metal_silver" type="mesh" mesh="body_link0_1.obj" class="visual" />
      <geom name="openarm_body_link0_visual_2" pos="0.0 0.0 0.0" quat="1.0 0.0 0.0 0.0"
        material="matte_black" type="mesh" mesh="body_link0_2.obj" class="visual" />
      <geom name="openarm_body_link0_visual_3" pos="0.0 0.0 0.0" quat="1.0 0.0 0.0 0.0"
        material="matte_black" type="mesh" mesh="body_link0_3.obj" class="visual" />
      <geom name="openarm_body_link0_visual_4" pos="0.0 0.0 0.0" quat="1.0 0.0 0.0 0.0"
        material="metal_silver" type="mesh" mesh="body_link0_4.obj" class="visual" />
      <geom name="openarm_body_link0_visual_5" pos="0.0 0.0 0.0" quat="1.0 0.0 0.0 0.0"
        material="matte_black" type="mesh" mesh="body_link0_5.obj" class="visual" />
      <body name="openarm_left_link0" pos="0.0 0.031 0.698"
        quat="0.7071054825112363 -0.7071080798594735 0.0 0.0">
        <inertial pos="-0.0009483362816297526 0.0001580207020448382 0.03076860287587199"
          quat="1.0 0.0 0.0 0.0" mass="1.1432284943239561" diaginertia="0.001128 0.000962 0.00147" />
        <geom name="openarm_left_link0_collision" pos="0.0 0.0 0.0" quat="1.0 0.0 0.0 0.0"
          type="mesh" mesh="link0_collision_left" class="collision" />
        <geom name="openarm_left_link0_visual_0" pos="0.0 0.0 0.0" quat="1.0 0.0 0.0 0.0"
          material="metal_silver" type="mesh" mesh="link0_0.obj" class="visual" />
        <geom name="openarm_left_link0_visual_1" pos="0.0 0.0 0.0" quat="1.0 0.0 0.0 0.0"
          material="matte_black" type="mesh" mesh="link0_1.obj" class="visual" />
        <body name="openarm_left_link1" pos="0.0 0.0 0.0625" quat="1.0 0.0 0.0 0.0">
          <joint name="openarm_left_joint1" type="hinge" ref="0.0" class="motor_DM8009"
            range="-3.490659 1.3962629999999998" axis="0 0 1" />
          <inertial pos="0.0011467657911800769 3.319987657026362e-05 0.05395284380736254"
            quat="1.0 0.0 0.0 0.0" mass="1.1416684646202298"
            diaginertia="0.001567 0.001273 0.001016" />
          <geom name="openarm_left_link1_collision" pos="-0.0 0.0 -0.0625" quat="1.0 0.0 0.0 0.0"
            type="mesh" mesh="link1_collision_left" class="collision" />
          <geom name="openarm_left_link1_visual_0" pos="-0.0 0.0 -0.0625" quat="1.0 0.0 0.0 0.0"
            material="matte_black" type="mesh" mesh="link1_0.obj" class="visual" />
          <geom name="openarm_left_link1_visual_2" pos="-0.0 0.0 -0.0625" quat="1.0 0.0 0.0 0.0"
            material="matte_black" type="mesh" mesh="link1_2.obj" class="visual" />
          <geom name="openarm_left_link1_visual_3" pos="-0.0 0.0 -0.0625" quat="1.0 0.0 0.0 0.0"
            material="matte_black" type="mesh" mesh="link1_3.obj" class="visual" />
          <body name="openarm_left_link2" pos="-0.0301 0.0 0.06"
            quat="0.7071067811882787 -0.7071067811848163 0.0 0.0">
            <joint name="openarm_left_joint2" type="hinge" ref="0.0" class="motor_DM8009"
              range="-3.3161253267948965 0.17453267320510335" axis="-1 0 0" />
            <inertial pos="0.00839629182351943 -2.0145102027597523e-08 0.03256649300522363"
              quat="1.0 0.0 0.0 0.0" mass="0.2775092746011571"
              diaginertia="0.000359 0.000376 0.000232" />
            <geom name="openarm_left_link2_collision" pos="0.0301 0.0 -0.1225"
              quat="1.0 0.0 0.0 0.0" type="mesh" mesh="link2_collision_left" class="collision" />
            <geom name="openarm_left_link2_visual_0" pos="0.0301 0.0 -0.1225" quat="1.0 0.0 0.0 0.0"
              material="matte_black" type="mesh" mesh="link2_0.obj" class="visual" />
            <geom name="openarm_left_link2_visual_1" pos="0.0301 0.0 -0.1225" quat="1.0 0.0 0.0 0.0"
              material="matte_black" type="mesh" mesh="link2_1.obj" class="visual" />
            <geom name="openarm_left_link2_visual_2" pos="0.0301 0.0 -0.1225" quat="1.0 0.0 0.0 0.0"
              material="matte_black" type="mesh" mesh="link2_2.obj" class="visual" />
            <body name="openarm_left_link3" pos="0.0301 0.0 0.06625" quat="1.0 0.0 0.0 0.0">
              <joint name="openarm_left_joint3" type="hinge" ref="0.0" class="motor_DM4340"
                range="-1.570796 1.570796" axis="0 0 1" />
              <inertial pos="-0.002104752099628911 0.0005549085042607548 0.08847470545721961"
                quat="1.0 0.0 0.0 0.0" mass="1.073863338202347"
                diaginertia="0.004372 0.004319 0.000661" />
              <geom name="openarm_left_link3_collision" pos="-0.0 -0.0 -0.18875"
                quat="1.0 0.0 0.0 0.0" type="mesh" mesh="link3_collision_left" class="collision" />
              <geom name="openarm_left_link3_visual_0" pos="-0.0 -0.0 -0.18875"
                quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh" mesh="link3_0.obj"
                class="visual" />
              <geom name="openarm_left_link3_visual_1" pos="-0.0 -0.0 -0.18875"
                quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh" mesh="link3_1.obj"
                class="visual" />
              <geom name="openarm_left_link3_visual_2" pos="-0.0 -0.0 -0.18875"
                quat="1.0 0.0 0.0 0.0" material="metal_silver" type="mesh" mesh="link3_2.obj"
                class="visual" />
              <body name="openarm_left_link4" pos="-0.0 0.0315 0.15375" quat="1.0 0.0 0.0 0.0">
                <joint name="openarm_left_joint4" type="hinge" ref="0.0" class="motor_DM4340"
                  range="0.0 2.443461" axis="0 1 0" />
                <inertial pos="-0.0029006831074562967 -0.03030575826634669 0.06339637422196209"
                  quat="1.0 0.0 0.0 0.0" mass="0.6348534566833373"
                  diaginertia="0.001577 0.001346 0.001104" />
                <geom name="openarm_left_link4_collision" pos="0.0 -0.0315 -0.3425"
                  quat="1.0 0.0 0.0 0.0" type="mesh" mesh="link4_collision" class="collision" />
                <geom name="openarm_left_link4_visual_0" pos="0.0 -0.0315 -0.3425"
                  quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh" mesh="link4_0.obj"
                  class="visual" />
                <geom name="openarm_left_link4_visual_1" pos="0.0 -0.0315 -0.3425"
                  quat="1.0 0.0 0.0 0.0" material="metal_silver" type="mesh" mesh="link4_1.obj"
                  class="visual" />
                <geom name="openarm_left_link4_visual_2" pos="0.0 -0.0315 -0.3425"
                  quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh" mesh="link4_2.obj"
                  class="visual" />
                <body name="openarm_left_link5" pos="0.0 -0.0315 0.0955" quat="1.0 0.0 0.0 0.0">
                  <joint name="openarm_left_joint5" type="hinge" ref="0.0" class="motor_DM4310"
                    range="-1.570796 1.570796" axis="0 0 1" />
                  <inertial pos="-0.003049665024221911 0.0008866902457326625 0.043079803024980934"
                    quat="1.0 0.0 0.0 0.0" mass="0.6156588026168502"
                    diaginertia="0.000423 0.000445 0.000324" />
                  <geom name="openarm_left_link5_collision" pos="-0.0 -0.0 -0.438"
                    quat="1.0 0.0 0.0 0.0" type="mesh" mesh="link5_collision_left" class="collision" />
                  <geom name="openarm_left_link5_visual_0" pos="-0.0 -0.0 -0.438"
                    quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh" mesh="link5_0.obj"
                    class="visual" />
                  <geom name="openarm_left_link5_visual_1" pos="-0.0 -0.0 -0.438"
                    quat="1.0 0.0 0.0 0.0" material="metal_silver" type="mesh" mesh="link5_1.obj"
                    class="visual" />
                  <geom name="openarm_left_link5_visual_2" pos="-0.0 -0.0 -0.438"
                    quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh" mesh="link5_2.obj"
                    class="visual" />
                  <body name="openarm_left_link6" pos="0.0375 0.0 0.1205" quat="1.0 0.0 0.0 0.0">
                    <joint name="openarm_left_joint6" type="hinge" ref="0.0" class="motor_DM4310"
                      range="-0.785398 0.785398" axis="1 0 0" />
                    <inertial
                      pos="-0.037136587005447405 0.00033230528343419053 -9.498374522309838e-05"
                      quat="1.0 0.0 0.0 0.0" mass="0.475202773187987"
                      diaginertia="0.000143 0.000157 0.000159" />
                    <geom name="openarm_left_link6_collision" pos="-0.0375 -0.0 -0.5585"
                      quat="1.0 0.0 0.0 0.0" type="mesh" mesh="link6_collision_left"
                      class="collision" />
                    <geom name="openarm_left_link6_visual_0" pos="-0.0375 -0.0 -0.5585"
                      quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh"
                      mesh="link6_0_left.obj" class="visual" />
                    <geom name="openarm_left_link6_visual_1" pos="-0.0375 -0.0 -0.5585"
                      quat="1.0 0.0 0.0 0.0" material="metal_silver" type="mesh"
                      mesh="link6_1_left.obj" class="visual" />
                    <body name="openarm_left_link7" pos="-0.0375 0.0 0.0" quat="1.0 0.0 0.0 0.0">
                      <joint name="openarm_left_joint7" type="hinge" ref="0.0" class="motor_DM4310"
                        range="-1.570796 1.570796" axis="0 -1 0" />
                      <inertial pos="6.875510271106056e-05 -0.01766175250761268 0.06651945409987448"
                        quat="1.0 0.0 0.0 0.0" mass="0.4659771327380578"
                        diaginertia="0.000639 0.000497 0.000342" />
                      <geom name="openarm_left_link7_collision" pos="0.0 -0.0 -0.5585"
                        quat="1.0 0.0 0.0 0.0" type="mesh" mesh="link7_collision_left"
                        class="collision" />
                      <geom name="openarm_left_link7_visual_0" pos="0.0 -0.0 -0.5585"
                        quat="1.0 0.0 0.0 0.0" material="metal_silver" type="mesh"
                        mesh="link7_0_left.obj" class="visual" />
                      <geom name="openarm_left_link7_visual_1" pos="0.0 -0.0 -0.5585"
                        quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh"
                        mesh="link7_1_left.obj" class="visual" />
                      <body name="openarm_left_link8" pos="1e-06 0.0205 0.0" quat="1.0 0.0 0.0 0.0">
                        <body name="openarm_left_hand" pos="0 -0.025 0.1001" quat="1.0 0.0 0.0 0.0">
                          <inertial pos="0.0 0.002 0.01" quat="1.0 0.0 0.0 0.0" mass="0.15"
                            diaginertia="0.0001 0.00025 0.00017" />
                          <geom name="openarm_left_hand_collision" pos="0.0 0.005 -0.6585"
                            quat="1.0 0.0 0.0 0.0" type="mesh" mesh="hand_collision"
                            class="collision" />
                          <geom name="openarm_left_hand_visual_0" pos="0.0 0.005 -0.6585"
                            quat="1.0 0.0 0.0 0.0" material="metal_silver" type="mesh"
                            mesh="hand_0.obj" class="visual" />
                          <geom name="openarm_left_hand_visual_1" pos="0.0 0.005 -0.6585"
                            quat="1.0 0.0 0.0 0.0" material="metal_silver" type="mesh"
                            mesh="hand_1.obj" class="visual" />
                          <body name="openarm_left_hand_tcp" pos="0 -0.0 0.08"
                            quat="1.0 0.0 0.0 0.0" />
                          <body name="openarm_left_right_finger" pos="0 0.00 0.015"
                            quat="1.0 0.0 0.0 0.0">
                            <joint name="openarm_left_finger_joint1" type="slide" ref="0.0"
                              class="motor_finger" range="0.0 0.044" axis="0 -1 0" />
                            <inertial pos="0.0064528 -0.01702 0.0219685" quat="1.0 0.0 0.0 0.0"
                              mass="0.03602545343277134"
                              diaginertia="2.3749999999999997e-06 2.3749999999999997e-06 7.5e-07" />
                            <geom name="openarm_left_right_finger_collision"
                              pos="0.0 0.05 -0.673001" quat="1.0 0.0 0.0 0.0" type="mesh"
                              mesh="finger_collision_left" class="collision" />
                            <geom name="openarm_left_right_finger_visual_0" pos="0.0 0.05 -0.673001"
                              quat="1.0 0.0 0.0 0.0" material="default_material" type="mesh"
                              mesh="finger_0_flip.obj" class="visual" />
                            <geom name="openarm_left_right_finger_visual_1" pos="0.0 0.05 -0.673001"
                              quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh"
                              mesh="finger_1_flip.obj" class="visual" />
                          </body>
                          <body name="openarm_left_left_finger" pos="0 0.012 0.015"
                            quat="1.0 0.0 0.0 0.0">
                            <joint name="openarm_left_finger_joint2" type="slide" ref="0.0"
                              class="motor_finger" range="0.0 0.044" axis="0 1 0" />
                            <inertial pos="0.0064528 0.01702 0.0219685" quat="1.0 0.0 0.0 0.0"
                              mass="0.03602545343277134"
                              diaginertia="2.3749999999999997e-06 2.3749999999999997e-06 7.5e-07" />
                            <geom name="openarm_left_left_finger_collision"
                              pos="0.0 -0.05 -0.673001" quat="1.0 0.0 0.0 0.0" type="mesh"
                              mesh="finger_collision" class="collision" />
                            <geom name="openarm_left_left_finger_visual_0" pos="0.0 -0.05 -0.673001"
                              quat="1.0 0.0 0.0 0.0" material="default_material" type="mesh"
                              mesh="finger_0.obj" class="visual" />
                            <geom name="openarm_left_left_finger_visual_1" pos="0.0 -0.05 -0.673001"
                              quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh"
                              mesh="finger_1.obj" class="visual" />
                          </body>
                        </body>
                      </body>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
      <body name="openarm_right_link0" pos="0.0 -0.031 0.698"
        quat="0.7071054825112363 0.7071080798594735 0.0 0.0">
        <inertial pos="-0.0009483362816297526 0.0001580207020448382 0.03076860287587199"
          quat="1.0 0.0 0.0 0.0" mass="1.1432284943239561" diaginertia="0.001128 0.000962 0.00147" />
        <geom name="openarm_right_link0_collision" pos="0.0 0.0 0.0" quat="1.0 0.0 0.0 0.0"
          type="mesh" mesh="link0_collision" class="collision" />
        <geom name="openarm_right_link0_visual_0" pos="0.0 0.0 0.0" quat="1.0 0.0 0.0 0.0"
          material="metal_silver" type="mesh" mesh="link0_0.obj" class="visual" />
        <geom name="openarm_right_link0_visual_1" pos="0.0 0.0 0.0" quat="1.0 0.0 0.0 0.0"
          material="matte_black" type="mesh" mesh="link0_1.obj" class="visual" />
        <body name="openarm_right_link1" pos="0.0 0.0 0.0625" quat="1.0 0.0 0.0 0.0">
          <joint name="openarm_right_joint1" type="hinge" ref="0.0" class="motor_DM8009"
            range="-1.396263 3.490659" axis="0 0 1" />
          <inertial pos="0.0011467657911800769 3.319987657026362e-05 0.05395284380736254"
            quat="1.0 0.0 0.0 0.0" mass="1.1416684646202298"
            diaginertia="0.001567 0.001273 0.001016" />
          <geom name="openarm_right_link1_collision" pos="-0.0 0.0 -0.0625" quat="1.0 0.0 0.0 0.0"
            type="mesh" mesh="link1_collision" class="collision" />
          <geom name="openarm_right_link1_visual_0" pos="-0.0 0.0 -0.0625" quat="1.0 0.0 0.0 0.0"
            material="matte_black" type="mesh" mesh="link1_0.obj" class="visual" />
          <geom name="openarm_right_link1_visual_2" pos="-0.0 0.0 -0.0625" quat="1.0 0.0 0.0 0.0"
            material="matte_black" type="mesh" mesh="link1_2.obj" class="visual" />
          <geom name="openarm_right_link1_visual_3" pos="-0.0 0.0 -0.0625" quat="1.0 0.0 0.0 0.0"
            material="matte_black" type="mesh" mesh="link1_3.obj" class="visual" />
          <body name="openarm_right_link2" pos="-0.0301 0.0 0.06"
            quat="0.7071067811882787 0.7071067811848163 0.0 0.0">
            <joint name="openarm_right_joint2" type="hinge" ref="0.0" class="motor_DM8009"
              range="-0.17453267320510335 3.3161253267948965" axis="-1 0 0" />
            <inertial pos="0.00839629182351943 -2.0145102027597523e-08 0.03256649300522363"
              quat="1.0 0.0 0.0 0.0" mass="0.2775092746011571"
              diaginertia="0.000359 0.000376 0.000232" />
            <geom name="openarm_right_link2_collision" pos="0.0301 0.0 -0.1225"
              quat="1.0 0.0 0.0 0.0" type="mesh" mesh="link2_collision" class="collision" />
            <geom name="openarm_right_link2_visual_0" pos="0.0301 0.0 -0.1225"
              quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh" mesh="link2_0.obj"
              class="visual" />
            <geom name="openarm_right_link2_visual_1" pos="0.0301 0.0 -0.1225"
              quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh" mesh="link2_1.obj"
              class="visual" />
            <geom name="openarm_right_link2_visual_2" pos="0.0301 0.0 -0.1225"
              quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh" mesh="link2_2.obj"
              class="visual" />
            <body name="openarm_right_link3" pos="0.0301 0.0 0.06625" quat="1.0 0.0 0.0 0.0">
              <joint name="openarm_right_joint3" type="hinge" ref="0.0" class="motor_DM4340"
                range="-1.570796 1.570796" axis="0 0 1" />
              <inertial pos="-0.002104752099628911 0.0005549085042607548 0.08847470545721961"
                quat="1.0 0.0 0.0 0.0" mass="1.073863338202347"
                diaginertia="0.004372 0.004319 0.000661" />
              <geom name="openarm_right_link3_collision" pos="-0.0 -0.0 -0.18875"
                quat="1.0 0.0 0.0 0.0" type="mesh" mesh="link3_collision" class="collision" />
              <geom name="openarm_right_link3_visual_0" pos="-0.0 -0.0 -0.18875"
                quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh" mesh="link3_0.obj"
                class="visual" />
              <geom name="openarm_right_link3_visual_1" pos="-0.0 -0.0 -0.18875"
                quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh" mesh="link3_1.obj"
                class="visual" />
              <geom name="openarm_right_link3_visual_2" pos="-0.0 -0.0 -0.18875"
                quat="1.0 0.0 0.0 0.0" material="metal_silver" type="mesh" mesh="link3_2.obj"
                class="visual" />
              <body name="openarm_right_link4" pos="-0.0 0.0315 0.15375" quat="1.0 0.0 0.0 0.0">
                <joint name="openarm_right_joint4" type="hinge" ref="0.0" class="motor_DM4340"
                  range="0.0 2.443461" axis="0 1 0" />
                <inertial pos="-0.0029006831074562967 -0.03030575826634669 0.06339637422196209"
                  quat="1.0 0.0 0.0 0.0" mass="0.6348534566833373"
                  diaginertia="0.001577 0.001346 0.001104" />
                <geom name="openarm_right_link4_collision" pos="0.0 -0.0315 -0.3425"
                  quat="1.0 0.0 0.0 0.0" type="mesh" mesh="link4_collision" class="collision" />
                <geom name="openarm_right_link4_visual_0" pos="0.0 -0.0315 -0.3425"
                  quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh" mesh="link4_0.obj"
                  class="visual" />
                <geom name="openarm_right_link4_visual_1" pos="0.0 -0.0315 -0.3425"
                  quat="1.0 0.0 0.0 0.0" material="metal_silver" type="mesh" mesh="link4_1.obj"
                  class="visual" />
                <geom name="openarm_right_link4_visual_2" pos="0.0 -0.0315 -0.3425"
                  quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh" mesh="link4_2.obj"
                  class="visual" />
                <body name="openarm_right_link5" pos="0.0 -0.0315 0.0955" quat="1.0 0.0 0.0 0.0">
                  <joint name="openarm_right_joint5" type="hinge" ref="0.0" class="motor_DM4310"
                    range="-1.570796 1.570796" axis="0 0 1" />
                  <inertial pos="-0.003049665024221911 0.0008866902457326625 0.043079803024980934"
                    quat="1.0 0.0 0.0 0.0" mass="0.6156588026168502"
                    diaginertia="0.000423 0.000445 0.000324" />
                  <geom name="openarm_right_link5_collision" pos="-0.0 -0.0 -0.438"
                    quat="1.0 0.0 0.0 0.0" type="mesh" mesh="link5_collision" class="collision" />
                  <geom name="openarm_right_link5_visual_0" pos="-0.0 -0.0 -0.438"
                    quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh" mesh="link5_0.obj"
                    class="visual" />
                  <geom name="openarm_right_link5_visual_1" pos="-0.0 -0.0 -0.438"
                    quat="1.0 0.0 0.0 0.0" material="metal_silver" type="mesh" mesh="link5_1.obj"
                    class="visual" />
                  <geom name="openarm_right_link5_visual_2" pos="-0.0 -0.0 -0.438"
                    quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh" mesh="link5_2.obj"
                    class="visual" />
                  <body name="openarm_right_link6" pos="0.0375 0.0 0.1205" quat="1.0 0.0 0.0 0.0">
                    <joint name="openarm_right_joint6" type="hinge" ref="0.0" class="motor_DM4310"
                      range="-0.785398 0.785398" axis="1 0 0" />
                    <inertial
                      pos="-0.037136587005447405 0.00033230528343419053 -9.498374522309838e-05"
                      quat="1.0 0.0 0.0 0.0" mass="0.475202773187987"
                      diaginertia="0.000143 0.000157 0.000159" />
                    <geom name="openarm_right_link6_collision" pos="-0.0375 -0.0 -0.5585"
                      quat="1.0 0.0 0.0 0.0" type="mesh" mesh="link6_collision" class="collision" />
                    <geom name="openarm_right_link6_visual_0" pos="-0.0375 -0.0 -0.5585"
                      quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh" mesh="link6_0.obj"
                      class="visual" />
                    <geom name="openarm_right_link6_visual_1" pos="-0.0375 -0.0 -0.5585"
                      quat="1.0 0.0 0.0 0.0" material="metal_silver" type="mesh" mesh="link6_1.obj"
                      class="visual" />
                    <body name="openarm_right_link7" pos="-0.0375 0.0 0.0" quat="1.0 0.0 0.0 0.0">
                      <joint name="openarm_right_joint7" type="hinge" ref="0.0" class="motor_DM4310"
                        range="-1.570796 1.570796" axis="0 1 0" />
                      <inertial pos="6.875510271106056e-05 -0.01766175250761268 0.06651945409987448"
                        quat="1.0 0.0 0.0 0.0" mass="0.4659771327380578"
                        diaginertia="0.000639 0.000497 0.000342" />
                      <geom name="openarm_right_link7_collision" pos="0.0 -0.0 -0.5585"
                        quat="1.0 0.0 0.0 0.0" type="mesh" mesh="link7_collision" class="collision" />
                      <geom name="openarm_right_link7_visual_0" pos="0.0 -0.0 -0.5585"
                        quat="1.0 0.0 0.0 0.0" material="metal_silver" type="mesh"
                        mesh="link7_0.obj" class="visual" />
                      <geom name="openarm_right_link7_visual_1" pos="0.0 -0.0 -0.5585"
                        quat="1.0 0.0 0.0 0.0" material="matte_black" type="mesh" mesh="link7_1.obj"
                        class="visual" />
                      <body name="openarm_right_link8" pos="1e-06 0.0205 0.0" quat="1.0 0.0 0.0 0.0">
                        <body name="openarm_right_hand" pos="0 -0.025 0.1001" quat="1.0 0.0 0.0 0.0">
                          <inertial pos="0.0 0.002 0.01" quat="1.0 0.0 0.0 0.0" mass="0.15"
                            diaginertia="0.0001 0.00025 0.00017" />
                          <geom name="openarm_right_hand_collision" pos="0.0 0.005 -0.6585"
                            quat="1.0 0.0 0.0 0.0" type="mesh" mesh="hand_collision"
                            class="collision" />
                          <geom name="openarm_right_hand_visual_0" pos="0.0 0.005 -0.6585"
                            quat="1.0 0.0 0.0 0.0" material="metal_silver" type="mesh"
                            mesh="hand_0.obj" class="visual" />
                          <geom name="openarm_right_hand_visual_1" pos="0.0 0.005 -0.6585"
                            quat="1.0 0.0 0.0 0.0" material="metal_silver" type="mesh"
                            mesh="hand_1.obj" class="visual" />
                          <body name="openarm_right_hand_tcp" pos="0 -0.0 0.08"
                            quat="1.0 0.0 0.0 0.0" />
                          <body name="openarm_right_right_finger" pos="0 0.00 0.015"
                            quat="1.0 0.0 0.0 0.0">
                            <joint name="openarm_right_finger_joint1" type="slide" ref="0.0"
                              class="motor_finger" range="0.0 0.044" axis="0 -1 0" />
                            <inertial pos="0.0064528 -0.01702 0.0219685" quat="1.0 0.0 0.0 0.0"
                              mass="0.03602545343277134"
                              diaginertia="2.3749999999999997e-06 2.3749999999999997e-06 7.5e-07" />
                            <geom name="openarm_right_right_finger_collision"
                              pos="0.0 0.05 -0.673001" quat="1.0 0.0 0.0 0.0" type="mesh"
                              mesh="finger_collision_left" class="collision" />
                            <geom name="openarm_right_right_finger_visual_0"
                              pos="0.0 0.05 -0.673001" quat="1.0 0.0 0.0 0.0"
                              material="default_material" type="mesh" mesh="finger_0_flip.obj"
                              class="visual" />
                            <geom name="openarm_right_right_finger_visual_1"
                              pos="0.0 0.05 -0.673001" quat="1.0 0.0 0.0 0.0" material="matte_black"
                              type="mesh" mesh="finger_1_flip.obj" class="visual" />
                          </body>
                          <body name="openarm_right_left_finger" pos="0 0.012 0.015"
                            quat="1.0 0.0 0.0 0.0">
                            <joint name="openarm_right_finger_joint2" type="slide" ref="0.0"
                              class="motor_finger" range="0.0 0.044" axis="0 1 0" />
                            <inertial pos="0.0064528 0.01702 0.0219685" quat="1.0 0.0 0.0 0.0"
                              mass="0.03602545343277134"
                              diaginertia="2.3749999999999997e-06 2.3749999999999997e-06 7.5e-07" />
                            <geom name="openarm_right_left_finger_collision"
                              pos="0.0 -0.05 -0.673001" quat="1.0 0.0 0.0 0.0" type="mesh"
                              mesh="finger_collision" class="collision" />
                            <geom name="openarm_right_left_finger_visual_0"
                              pos="0.0 -0.05 -0.673001" quat="1.0 0.0 0.0 0.0"
                              material="default_material" type="mesh" mesh="finger_0.obj"
                              class="visual" />
                            <geom name="openarm_right_left_finger_visual_1"
                              pos="0.0 -0.05 -0.673001" quat="1.0 0.0 0.0 0.0"
                              material="matte_black" type="mesh" mesh="finger_1.obj" class="visual" />
                          </body>
                        </body>
                      </body>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
    <site name="world_site" pos="0 0 0" quat="1 0 0 0" />
    <camera name="front_camera" mode="track" fovy="90.0"
      quat="4.329780281177467e-17 4.329780281177466e-17 0.7071067811865475 0.7071067811865476"
      pos="0.0 2.0 0.5" />
    <camera name="side_camera" mode="track" fovy="90.0"
      quat="-0.5 -0.4999999999999999 0.5 0.5000000000000001" pos="-2.0 0.0 0.5" />

  </worldbody>

  <tendon>
    <fixed name="split_right">
      <joint joint="openarm_right_finger_joint1" coef="0.5" />
      <joint joint="openarm_right_finger_joint2" coef="0.5" />
    </fixed>
    <fixed name="split_left">
      <joint joint="openarm_left_finger_joint1" coef="0.5" />
      <joint joint="openarm_left_finger_joint2" coef="0.5" />
    </fixed>
  </tendon>
  <equality>
    <joint joint1="openarm_right_finger_joint1" joint2="openarm_right_finger_joint2"
      solimp="0.95 0.99 0.001" solref="0.005 1" />
    <joint joint1="openarm_left_finger_joint1" joint2="openarm_left_finger_joint2"
      solimp="0.95 0.99 0.001" solref="0.005 1" />
  </equality>


  <actuator>
    <motor name="left_joint1_ctrl" joint="openarm_left_joint1" class="motor_DM8009" />
    <motor name="left_joint2_ctrl" joint="openarm_left_joint2" class="motor_DM8009" />
    <motor name="left_joint3_ctrl" joint="openarm_left_joint3" class="motor_DM4340" />
    <motor name="left_joint4_ctrl" joint="openarm_left_joint4" class="motor_DM4340" />
    <motor name="left_joint5_ctrl" joint="openarm_left_joint5" class="motor_DM4310" />
    <motor name="left_joint6_ctrl" joint="openarm_left_joint6" class="motor_DM4310" />
    <motor name="left_joint7_ctrl" joint="openarm_left_joint7" class="motor_DM4310" />
    <motor name="left_finger1_ctrl" joint="openarm_left_finger_joint1" class="motor_finger" />
    <motor name="left_finger2_ctrl" joint="openarm_left_finger_joint2" class="motor_finger" />
    <motor name="right_joint1_ctrl" joint="openarm_right_joint1" class="motor_DM8009" />
    <motor name="right_joint2_ctrl" joint="openarm_right_joint2" class="motor_DM8009" />
    <motor name="right_joint3_ctrl" joint="openarm_right_joint3" class="motor_DM4340" />
    <motor name="right_joint4_ctrl" joint="openarm_right_joint4" class="motor_DM4340" />
    <motor name="right_joint5_ctrl" joint="openarm_right_joint5" class="motor_DM4310" />
    <motor name="right_joint6_ctrl" joint="openarm_right_joint6" class="motor_DM4310" />
    <motor name="right_joint7_ctrl" joint="openarm_right_joint7" class="motor_DM4310" />
    <position name="right_finger1_ctrl" joint="openarm_right_finger_joint1" class="motor_finger" />
    <position name="right_finger2_ctrl" joint="openarm_right_finger_joint2" class="motor_finger" />
  </actuator>

  <contact>
    <exclude body1="openarm_body_link0" body2="openarm_left_link0" />
    <exclude body1="openarm_left_link0" body2="openarm_left_link1" />
    <exclude body1="openarm_left_link0" body2="openarm_left_link2" />
    <exclude body1="openarm_left_link0" body2="openarm_left_link3" />
    <exclude body1="openarm_left_link1" body2="openarm_left_link2" />
    <exclude body1="openarm_left_link1" body2="openarm_left_link3" />
    <exclude body1="openarm_left_link2" body2="openarm_left_link3" />
    <exclude body1="openarm_left_link3" body2="openarm_left_link4" />
    <exclude body1="openarm_left_link4" body2="openarm_left_link5" />
    <exclude body1="openarm_left_link5" body2="openarm_left_link6" />
    <exclude body1="openarm_left_link5" body2="openarm_left_link7" />
    <exclude body1="openarm_left_link6" body2="openarm_left_link7" />
    <exclude body1="openarm_body_link0" body2="openarm_right_link0" />
    <exclude body1="openarm_right_link0" body2="openarm_right_link1" />
    <exclude body1="openarm_right_link0" body2="openarm_right_link2" />
    <exclude body1="openarm_right_link0" body2="openarm_right_link3" />
    <exclude body1="openarm_right_link1" body2="openarm_right_link2" />
    <exclude body1="openarm_right_link1" body2="openarm_right_link3" />
    <exclude body1="openarm_right_link2" body2="openarm_right_link3" />
    <exclude body1="openarm_right_link3" body2="openarm_right_link4" />
    <exclude body1="openarm_right_link4" body2="openarm_right_link5" />
    <exclude body1="openarm_right_link5" body2="openarm_right_link6" />
    <exclude body1="openarm_right_link5" body2="openarm_right_link7" />
    <exclude body1="openarm_right_link6" body2="openarm_right_link7" />
    <exclude body1="openarm_left_hand" body2="openarm_left_right_finger" />
    <exclude body1="openarm_left_hand" body2="openarm_left_left_finger" />
    <exclude body1="openarm_right_hand" body2="openarm_right_right_finger" />
    <exclude body1="openarm_right_hand" body2="openarm_right_left_finger" />
  </contact>

  <sensor>
    <framepos name="world_site_pos" objtype="site" objname="world_site" />
    <framequat name="world_site_quat" objtype="site" objname="world_site" />
    <framelinvel name="world_site_linvel" objtype="site" objname="world_site" />
    <frameangvel name="world_site_angvel" objtype="site" objname="world_site" />
    <velocimeter name="world_site_vel" site="world_site" />
  </sensor>
</mujoco>
```

</details>

---

### scene.xml

<details>
<summary>点击展开</summary>

```
<mujoco model="openarm_bimanual scene">
  <include file="openarm_bimanual.xml"/>

  <statistic center="0 0 0.43" extent=".85" meansize="0.05"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="160" elevation="-10"/>
    <quality shadowsize="8192"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>

  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  </worldbody>
</mujoco>
```

</details>

---
（注意如果将文件保存在别的地方，记得修改引入model的路径）
---
📌在这之前，你的Ubuntu系统可能没有安装pip和venv组件（都是后续需要的）
```
sudo apt update
sudo apt install python3-pip
sudo apt install python3-venv
```
如果安装好了也请检查一遍（确保更新到最新版本）

验证 pip：
```
pip3 --version
```
正常输出例如：
```
pip 23.x.x from /usr/lib/python3/dist-packages/pip (python 3.12)
```
验证 venv 可用：
```
python3 -m venv testenv
```
如果不报错，说明 venv 成功。

---
步骤 1：创建虚拟环境（建议用于 MuJoCo 项目）
在终端执行（任意位置都可以）：
```
python3 -m venv ~/myenv
```
激活它：
```
source ~/myenv/bin/activate
```
你会看到：
```
#user是你自己的设备名
(myenv) user:
```
步骤 2：安装所有依赖（在 myenv 中）
```
pip install --upgrade pip
pip install mujoco
pip install pyautogui numpy scipy ikpy
```
这些依赖都是在我提供的脚本里需要使用的，如果后续需要更多的库需再进行安装

检查：
```
python3 -c "import mujoco; print('mujoco ok')"
```

# 4.运行程序

现在确保你在虚拟环境中并且安装了所有依赖之后，你就可以cd到保存openarm脚本的目录运行：
```
#替换你自己命名的文件
python3 <filename>
```
然后就可以开心的看到openarm了

---

但是这样每次在终端里先启动虚拟环境再运行程序非常的麻烦，这时候我们就可以在pycharm中直接添加编译器，让他自动选择我们已经调试好的虚拟环境

步骤 1：打开 PyCharm 的解释器设置

菜单路径：
```
File → Settings → Project: <你的项目名> → Python Interpreter
```
你会看到当前选中的解释器可能是这样的（错误示例）：
```
Python 3.12 (python)  ~/myenv/bin/python/bin/python   ❌
```

这个路径是不对的，需要更换。

步骤 2：添加你的虚拟环境解释器

在同一页面右上角点击：
```
⚙ → Add Interpreter...
```

选择：
```
Add Existing Environment
```

然后文件选择器中逐级进入：
```
/home/<你的用户名>/myenv/bin/
```

选择以下其中之一：
```
python3.12   ← 推荐
python       ← 也可以
```

点击 OK / Apply。

步骤 3：验证解释器是否正确加载

成功之后，PyCharm 的解释器下拉框里应该显示：
```
Python 3.12  (/home/<你的用户名>/myenv/bin/python3.12)
```

并且包列表中应该能看到：

mujoco、numpy、scipy、pyautogui、ikpy等依赖。

这时候，我们就可以直接通过点击文件右上角的运行（绿色的三角🟢▶️）或者右键文件进行运行

恭喜你，接下来可以开始你的MuJoCo之旅了🎉🎉🎉
