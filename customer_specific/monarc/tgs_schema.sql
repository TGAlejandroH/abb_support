CREATE TABLE cad_assets(
     node_id TEXT PRIMARY KEY,
     cad_type TEXT NOT NULL,
     source_mode TEXT NOT NULL,
     source_path TEXT,
     embedded_blob BLOB,
     sha256 TEXT,
     unit TEXT,
     color_json TEXT,
     visible INTEGER NOT NULL DEFAULT 1,
     FOREIGN KEY(node_id) REFERENCES nodes(id) ON DELETE CASCADE
    );
CREATE TABLE external_axis_groups(
     node_id TEXT PRIMARY KEY,
     joint_json TEXT,
     kind TEXT,
     shared INTEGER,
     attached_robot_groups_json TEXT,
     ownership_mode TEXT,
     positioner_base_link TEXT,
     positioner_ee_link TEXT,
     workpiece_frame_name TEXT,
     metadata_json TEXT,
     FOREIGN KEY(node_id) REFERENCES nodes(id) ON DELETE CASCADE
    );
CREATE TABLE nodes(
     id TEXT PRIMARY KEY,
     type TEXT NOT NULL,
     name TEXT NOT NULL,
     parent_id TEXT,
     parent_ref_type TEXT NOT NULL DEFAULT 'node',
     parent_ref_value TEXT,
     order_index INTEGER NOT NULL DEFAULT 0,
     is_fixed INTEGER NOT NULL DEFAULT 1,
     created_at TEXT NOT NULL,
     updated_at TEXT NOT NULL,
     FOREIGN KEY(parent_id) REFERENCES nodes(id) ON DELETE SET NULL
    );
CREATE TABLE program_instructions(
     node_id TEXT PRIMARY KEY,
     program_node_id TEXT,
     instruction_id TEXT,
     instr_type TEXT,
     enabled INTEGER,
     payload_json TEXT,
     order_index INTEGER,
     FOREIGN KEY(node_id) REFERENCES nodes(id) ON DELETE CASCADE,
     FOREIGN KEY(program_node_id) REFERENCES nodes(id) ON DELETE CASCADE
    );
CREATE TABLE programs(
     node_id TEXT PRIMARY KEY,
     program_id TEXT UNIQUE,
     parent_frame_name TEXT,
     robot_group_name TEXT,
     enabled INTEGER,
     FOREIGN KEY(node_id) REFERENCES nodes(id) ON DELETE CASCADE
    );
CREATE TABLE project_meta(
     key TEXT PRIMARY KEY,
     value TEXT NOT NULL
    );
CREATE TABLE robot_bundle_files(
     path TEXT PRIMARY KEY,
     content_blob BLOB NOT NULL,
     sha256 TEXT NOT NULL
    );
CREATE TABLE robot_groups(
     node_id TEXT PRIMARY KEY,
     arm_joint_json TEXT,
     base_link TEXT,
     tcp_link TEXT,
     default_mechanism_groups_json TEXT, robot_brand TEXT NOT NULL DEFAULT '', output_profile_id TEXT NOT NULL DEFAULT '', home_poses_json TEXT NOT NULL DEFAULT '{}',
     FOREIGN KEY(node_id) REFERENCES nodes(id) ON DELETE CASCADE
    );
CREATE TABLE robot_state(
     id TEXT PRIMARY KEY,
     joint_json TEXT NOT NULL,
     tcp_json TEXT,
     rail_value REAL,
     lock_joints_json TEXT,
     world_from_base_json TEXT,
     active_robot_group_name TEXT,
     active_external_axis_group_name TEXT,
     updated_at TEXT NOT NULL
    );
CREATE TABLE targets(
     node_id TEXT PRIMARY KEY,
     mode TEXT NOT NULL,
     joint_json TEXT NOT NULL,
     cartesian_json TEXT NOT NULL,
     config_json TEXT NOT NULL,
     FOREIGN KEY(node_id) REFERENCES nodes(id) ON DELETE CASCADE
    );
CREATE TABLE tools(
     node_id TEXT PRIMARY KEY,
     robot_group_name TEXT,
     xyz_rpy_json TEXT,
     FOREIGN KEY(node_id) REFERENCES nodes(id) ON DELETE CASCADE
    );
CREATE TABLE transforms(
     node_id TEXT PRIMARY KEY,
     tx REAL,
     ty REAL,
     tz REAL,
     rx REAL,
     ry REAL,
     rz REAL,
     ref_frame_id TEXT,
     FOREIGN KEY(node_id) REFERENCES nodes(id) ON DELETE CASCADE
    );
CREATE TABLE ui_state(
     key TEXT PRIMARY KEY,
     value TEXT NOT NULL
    );
CREATE TABLE weld_capture_poses(
     capture_pose_id TEXT PRIMARY KEY,
     weld_id TEXT NOT NULL,
     order_index INTEGER NOT NULL DEFAULT 0,
     payload_json TEXT NOT NULL DEFAULT '{}',
     geometry_payload_json TEXT NOT NULL DEFAULT '{}',
     FOREIGN KEY(weld_id) REFERENCES weld_items(weld_id) ON DELETE CASCADE
    );
CREATE TABLE weld_cropboxes(
     cropbox_id TEXT PRIMARY KEY,
     weld_id TEXT NOT NULL,
     order_index INTEGER NOT NULL DEFAULT 0,
     payload_json TEXT NOT NULL DEFAULT '{}',
     geometry_payload_json TEXT NOT NULL DEFAULT '{}',
     FOREIGN KEY(weld_id) REFERENCES weld_items(weld_id) ON DELETE CASCADE
    );
CREATE TABLE weld_datums(
     datum_id TEXT PRIMARY KEY,
     weld_id TEXT NOT NULL,
     order_index INTEGER NOT NULL DEFAULT 0,
     payload_json TEXT NOT NULL DEFAULT '{}',
     geometry_payload_json TEXT NOT NULL DEFAULT '{}',
     FOREIGN KEY(weld_id) REFERENCES weld_items(weld_id) ON DELETE CASCADE
    );
CREATE TABLE weld_environment_obstacles(
     obstacle_id TEXT PRIMARY KEY,
     project_node_id TEXT NOT NULL,
     name TEXT NOT NULL,
     order_index INTEGER NOT NULL DEFAULT 0,
     payload_json TEXT NOT NULL DEFAULT '{}',
     geometry_payload_json TEXT NOT NULL DEFAULT '{}',
     FOREIGN KEY(project_node_id) REFERENCES weld_projects(node_id) ON DELETE CASCADE
    );
CREATE TABLE weld_fixtures(
     fixture_id TEXT PRIMARY KEY,
     project_node_id TEXT NOT NULL,
     name TEXT NOT NULL,
     order_index INTEGER NOT NULL DEFAULT 0,
     payload_json TEXT NOT NULL DEFAULT '{}',
     geometry_payload_json TEXT NOT NULL DEFAULT '{}',
     FOREIGN KEY(project_node_id) REFERENCES weld_projects(node_id) ON DELETE CASCADE
    );
CREATE TABLE weld_generated_outputs(
     output_id TEXT PRIMARY KEY,
     project_node_id TEXT NOT NULL,
     owner_kind TEXT NOT NULL,
     owner_id TEXT NOT NULL,
     output_kind TEXT NOT NULL,
     order_index INTEGER NOT NULL DEFAULT 0,
     binding_json TEXT NOT NULL DEFAULT '{}',
     status_json TEXT NOT NULL DEFAULT '{}',
     FOREIGN KEY(project_node_id) REFERENCES weld_projects(node_id) ON DELETE CASCADE
    );
CREATE TABLE weld_global_capture_poses(
     global_capture_pose_id TEXT PRIMARY KEY,
     capture_set_id TEXT NOT NULL,
     order_index INTEGER NOT NULL DEFAULT 0,
     payload_json TEXT NOT NULL DEFAULT '{}',
     geometry_payload_json TEXT NOT NULL DEFAULT '{}',
     FOREIGN KEY(capture_set_id) REFERENCES weld_global_capture_sets(capture_set_id) ON DELETE CASCADE
    );
CREATE TABLE weld_global_capture_sets(
     capture_set_id TEXT PRIMARY KEY,
     project_node_id TEXT NOT NULL,
     name TEXT NOT NULL,
     order_index INTEGER NOT NULL DEFAULT 0,
     payload_json TEXT NOT NULL DEFAULT '{}',
     generation_state_json TEXT NOT NULL DEFAULT '{}',
     FOREIGN KEY(project_node_id) REFERENCES weld_projects(node_id) ON DELETE CASCADE
    );
CREATE TABLE weld_item_runtime_state(
     weld_id TEXT PRIMARY KEY,
     touch_offset_matrix_json TEXT NOT NULL DEFAULT '[[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]]',
     saved_localization_matrix_json TEXT NOT NULL DEFAULT '[[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]]',
     is_capture_enabled INTEGER NOT NULL DEFAULT 1,
     is_selected_to_weld INTEGER NOT NULL DEFAULT 1,
     FOREIGN KEY(weld_id) REFERENCES weld_items(weld_id) ON DELETE CASCADE
    );
CREATE TABLE weld_items(
     weld_id TEXT PRIMARY KEY,
     project_node_id TEXT NOT NULL,
     name TEXT NOT NULL,
     robot_group_name TEXT NOT NULL DEFAULT '',
     order_index INTEGER NOT NULL DEFAULT 0,
     payload_json TEXT NOT NULL DEFAULT '{}',
     generation_state_json TEXT NOT NULL DEFAULT '{}',
     FOREIGN KEY(project_node_id) REFERENCES weld_projects(node_id) ON DELETE CASCADE
    );
CREATE TABLE weld_project_runtime_state(
     project_node_id TEXT PRIMARY KEY,
     is_path_planning_optimized INTEGER NOT NULL DEFAULT 1,
     FOREIGN KEY(project_node_id) REFERENCES weld_projects(node_id) ON DELETE CASCADE
    );
CREATE TABLE weld_projects(
     node_id TEXT PRIMARY KEY,
     project_id TEXT UNIQUE NOT NULL,
     payload_json TEXT NOT NULL DEFAULT '{}',
     FOREIGN KEY(node_id) REFERENCES nodes(id) ON DELETE CASCADE
    );
CREATE TABLE weld_segments(
     segment_id TEXT PRIMARY KEY,
     weld_id TEXT NOT NULL,
     order_index INTEGER NOT NULL DEFAULT 0,
     payload_json TEXT NOT NULL DEFAULT '{}',
     geometry_payload_json TEXT NOT NULL DEFAULT '{}',
     FOREIGN KEY(weld_id) REFERENCES weld_items(weld_id) ON DELETE CASCADE
    );
CREATE TABLE weld_touch_points(
     touch_point_id TEXT PRIMARY KEY,
     weld_id TEXT NOT NULL,
     order_index INTEGER NOT NULL DEFAULT 0,
     payload_json TEXT NOT NULL DEFAULT '{}',
     FOREIGN KEY(weld_id) REFERENCES weld_items(weld_id) ON DELETE CASCADE
    );
CREATE INDEX idx_nodes_parent ON nodes(parent_id);
CREATE INDEX idx_nodes_type ON nodes(type);
CREATE INDEX idx_program_instr_program ON program_instructions(program_node_id, order_index);
CREATE INDEX idx_weld_capture_poses_weld_order ON weld_capture_poses(weld_id, order_index);
CREATE INDEX idx_weld_cropboxes_weld_order ON weld_cropboxes(weld_id, order_index);
CREATE INDEX idx_weld_datums_weld_order ON weld_datums(weld_id, order_index);
CREATE INDEX idx_weld_environment_obstacles_project_order ON weld_environment_obstacles(project_node_id, order_index);
CREATE INDEX idx_weld_fixtures_project_order ON weld_fixtures(project_node_id, order_index);
CREATE INDEX idx_weld_generated_outputs_project_owner_order ON weld_generated_outputs(project_node_id, owner_kind, owner_id, order_index);
CREATE INDEX idx_weld_global_capture_poses_set_order ON weld_global_capture_poses(capture_set_id, order_index);
CREATE INDEX idx_weld_global_capture_sets_project_order ON weld_global_capture_sets(project_node_id, order_index);
CREATE INDEX idx_weld_items_project_order ON weld_items(project_node_id, order_index);
CREATE INDEX idx_weld_segments_weld_order ON weld_segments(weld_id, order_index);
CREATE INDEX idx_weld_touch_points_weld_order ON weld_touch_points(weld_id, order_index);
