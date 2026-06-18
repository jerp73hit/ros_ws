from llm_api import make_scene, adjust_grasp_waypoints, execute_waypoints, init_api

def execute_task():
    scene = make_scene()
    source = scene["objects"]["banana"]
    target = scene["objects"]["bowl"]

    table_z = scene["table_z"]

    # 2. Compute Phase 1: Pickup coordinates
    p_yaw = source["orientation"]
    p_hover_z = source["center"][2] + source["half_extents"][2] + 0.12
    p_hover = [source["center"][0], source["center"][1], p_hover_z]
    p_grasp = [source["center"][0], source["center"][1], source["center"][2]]

    # 3. Compute Phase 2: Placement coordinates
    dest_x = target["center"][0]
    dest_y = target["center"][1] + 0.08  # Adjusting for placement from the bowl's center
    dest_z = table_z + target["half_extents"][2] + source["half_extents"][2]
    dest_hover_z = max(target["center"][2] + target["half_extents"][2], dest_z) + 0.12

    dest_hover = [dest_x, dest_y, dest_hover_z]
    dest_drop = [dest_x, dest_y, dest_z + .05]

    # 4. Construct sequential trajectory
    waypoints = [
        {"pos": p_hover, "ori": [180, 0, p_yaw], "gripper": "open"},
        {"pos": p_grasp, "ori": [180, 0, p_yaw], "gripper": "close"},
        {"pos": p_hover, "ori": [180, 0, p_yaw], "gripper": "close"},
        {"pos": dest_hover, "ori": [180, 0, p_yaw], "gripper": "close"},
        {"pos": dest_drop, "ori": [180, 0, p_yaw], "gripper": "open"},
        {"pos": dest_hover, "ori": [180, 0, p_yaw], "gripper": "open"},
    ]

    # 5. Execute pipeline
    adjusted = adjust_grasp_waypoints(scene, waypoints)
    execute_waypoints(adjusted)


init_api()
execute_task()
