# Diagrama de Dependencias del Proyecto

```mermaid
flowchart TD
    %% ===== ROS Infrastructure =====
    subgraph ROS_MSG["intera_core_msgs"]
        direction LR
        JCM[JointCommand]
        EST[EndpointState]
        RST[RobotAssemblyState]
        IKSRV[SolvePositionIK.srv]
        FKSRV[SolvePositionFK.srv]
    end

    subgraph MOT_MSG["intera_motion_msgs"]
        TRAJ[Trajectory]
        WPT[Waypoint]
    end

    %% ===== intera_sdk Packages =====
    subgraph DATAFLOW["intera_dataflow"]
        WAIT[wait_for.py]
        SIG[signals.py]
    end

    subgraph CTRL["intera_control"]
        PID[pid.py]
    end

    subgraph IO["intera_io"]
        IOIF[io_interface.py]
        IOCMD[io_command.py]
    end

    subgraph IINTERFACE["intera_interface"]
        LIMB[limb.py]
        GRIP[gripper.py]
        GRIPF[gripper_factory.py]
        CSP[clicksmart_plate.py]
        HEAD[head.py]
        HDISP[head_display.py]
        CAM[camera.py]
        CUFF[cuff.py]
        DIO[digital_io.py]
        JLIM[joint_limits.py]
        LIGHT[lights.py]
        NAV[navigator.py]
        REN[robot_enable.py]
        RPARAM[robot_params.py]
        SET[settings.py]
    end

    subgraph JTRAJ["intera_joint_trajectory_action"]
        JTAS[joint_trajectory_action.py]
        BEZ[bezier.py]
        MJ[minjerk.py]
    end

    subgraph MOTIF["intera_motion_interface"]
        MTRJ[motion_trajectory.py]
        MWPT[motion_waypoint.py]
        MCAC[motion_controller_action_client.py]
        MWOPT[motion_waypoint_options.py]
        IOPT[interaction_options.py]
        IPUB[interaction_publisher.py]
        RW[random_walk.py]
        UTL[utility_functions.py]
    end

    subgraph MEXAMPLES["intera_examples"]
        REC[recorder.py]
    end

    subgraph MEXTDEV["intera_external_devices"]
        GETCH[getch.py]
        JOY[joystick.py]
    end

    %% ===== Custom Python Scripts =====
    subgraph SCRIPTS["python_scripts_v1"]
        PP1[pick_place1.py]
        PP3[pick_place3.py]
        TPP[test_pick_place.py]
        TPPD[test_pick_place_daniel.py]
        LLM[llm_api.py]
        RT[run_task.py]
        GEN[generated_task.py]
        F2W[frame2world.py]
        B2W[Bbox_to_world.py]
        CF[capture_frame.py]
        DM[detection_markers.py]
        OLLAMA[ollama_client.py]
        OPENAI[openai_client.py]
        STB[sawyer_trajectory_bridge.py]
        GTH[go_to_top_hand_camera_pos.py]
        GCP[get_camera_pose.py]
        CPU[camera_pose_up.py]
        PTP[print_tip_pose.py]
        SSP[send_safe_pose.py]
        MTC[move_tip_coordinates.py]
        DPC[dome_photos_capture.py]
        CD[compare_distances.py]
    end

    %% ===== Sawyer MoveIt =====
    subgraph SMOVEIT["sawyer_moveit"]
        MSAW[move_sawyer.py]
        CONF[sawyer_moveit_config]
    end

    %% ===== Sawyer Simulator =====
    subgraph SSIM["sawyer_simulator"]
        subgraph GAZ["sawyer_gazebo"]
            SGRCP[sawyer_gazebo_ros_control_plugin.cpp]
            ASM[assembly_interface.cpp]
            ACI[arm_controller_interface.cpp]
            AKI[arm_kinematics_interface.cpp]
            HI[head_interface.cpp]
            CSI[cameras_sim_io_node.py]
        end
        HWIF[sawyer_hardware_interface]
        SIMCTRL[sawyer_sim_controllers]
        SIMEX[ik_pick_and_place_demo.py]
    end

    %% ===== SNS IK =====
    subgraph SNS["sns_ik"]
        SNSLIB[sns_ik_lib]
        SNSKIN[sns_ik_kinematics_plugin]
        SNSEX[ik_tests.cpp]
    end

    %% ===== Sawyer Robot =====
    subgraph SROBOT["sawyer_robot"]
        URDF[sawyer_description / URDF + meshes]
    end

    %% ===== Dependencies: intera_sdk internal =====
    IO --> DATAFLOW
    IO --> ROS_MSG
    IINTERFACE --> DATAFLOW
    IINTERFACE --> IO
    IINTERFACE --> RPARAM
    IINTERFACE --> SET
    JTRAJ --> CTRL
    JTRAJ --> IINTERFACE
    MOTIF --> IINTERFACE
    MOTIF --> MOT_MSG
    MOTIF --> UTL
    MEXAMPLES --> IINTERFACE

    %% ===== python_scripts_v1 internal =====
    B2W --> CF
    B2W --> F2W
    LLM --> F2W
    LLM --> IINTERFACE
    LLM --> ROS_MSG
    RT --> OLLAMA
    RT --> LLM
    GEN --> LLM
    STB --> IINTERFACE
    PP1 --> IINTERFACE
    PP3 --> IINTERFACE
    TPP --> IINTERFACE
    TPPD --> IINTERFACE
    GTH --> IINTERFACE
    GCP --> IINTERFACE
    CPU --> IINTERFACE
    PTP --> IINTERFACE
    SSP --> IINTERFACE
    MTC --> IINTERFACE
    DPC --> IINTERFACE

    %% ===== Sawyer simulator internal =====
    GAZ --> HWIF
    GAZ --> SIMCTRL
    GAZ --> SNSLIB
    AKI --> SNSLIB
    SIMCTRL --> HWIF
    SIMEX --> IINTERFACE

    %% ===== SNS IK internal =====
    SNSKIN --> SNSLIB
    SNSEX --> SNSLIB

    %% ===== Cross-package =====
    LLM --> SNSLIB
    SNSKIN --> SMOVEIT
    CONF --> SNSKIN

    %% ===== ROS message usage =====
    IINTERFACE --> ROS_MSG
    MOTIF --> MOT_MSG
    GAZ --> ROS_MSG
    STB --> ROS_MSG

    %% ===== URDF usage =====
    SNSLIB --> URDF
    GAZ --> URDF
    SMOVEIT --> URDF

    %% ===== Style classes =====
    classDef ros fill="#e1f5fe",stroke:#0288d1
    classDef sdk fill="#e8f5e9",stroke:#388e3c
    classDef script fill="#fff3e0",stroke:#f57c00
    classDef moveit fill="#f3e5f5",stroke:#7b1fa2
    classDef sim fill="#fce4ec",stroke:#c62828
    classDef sns fill="#e0f7fa",stroke:#00695c
    classDef robot fill="#f9fbe7",stroke:#827717

    class ROS_MSG,MOT_MSG ros
    class DATAFLOW,CTRL,IO,IINTERFACE,JTRAJ,MOTIF,MEXAMPLES,MEXTDEV sdk
    class SCRIPTS script
    class SMOVEIT moveit
    class SSIM sim
    class SNS sns
    class SROBOT robot
```
