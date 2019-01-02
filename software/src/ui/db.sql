
CREATE TABLE 'poses'
(
    'id' INTEGER PRIMARY KEY,
    'qx' FLOAT,
    'qy' FLOAT,
    'qz' FLOAT,
    'qw' FLOAT,
    'x' FLOAT,
	'y' FLOAT,
	'z' FLOAT
);

CREATE TABLE 'camera_parameters'
(
	'id' INTEGER PRIMARY KEY,
	'name' TEXT,
	'date' DATE,
    'image_width' INTEGER,
    'image_height' INTEGER,
	'fx' FLOAT,
	'fy' FLOAT,
	'cx' FLOAT,
	'cy' FLOAT,
	'distortion_model' INTEGER
);

CREATE TABLE 'distortion_coefficients'
(
	'id' INTEGER PRIMARY KEY,
	'camera_id' INTEGER,
	'rank' INTEGER,
	'value' FLOAT
);

CREATE TABLE 'rig_parameters'
(
	'id' INTEGER PRIMARY KEY,
	'name' TEXT,
	'date' DATE,
	'number_of_cameras' INTEGER
);

CREATE TABLE 'rig_cameras'
(
	'id' INTEGER PRIMARY KEY,
    'rig_id' INTEGER,
	'rank' INTEGER,
	'camera_to_rig' INTEGER,
	'camera_id' INTEGER
);

CREATE TABLE 'recordings'
(
	'id' INTEGER PRIMARY KEY,
	'name' TEXT,
	'date' DATE,
    'directory' TEXT,
	'number_of_views' INTEGER
);

CREATE TABLE 'recording_frames'
(
    'id' INTEGER PRIMARY KEY,
    'recording_id' INTEGER,
    'rank' INTEGER,
    'time' REAL
);

CREATE TABLE 'recording_views'
(
    'id' INTEGER PRIMARY KEY,
    'frame_id' INTEGER,
    'view' INTEGER,
    'filename' TEXT
);

CREATE TABLE 'reconstructions'
(
    'id' INTEGER PRIMARY KEY,
    'name' TEXT,
    'date' DATE,
    'rig_id' INTEGER,
    'recording_id' INTEGER
);

CREATE TABLE 'frames'
(
	'id' INTEGER PRIMARY KEY,
	'reconstruction_id' INTEGER,
	'rank' INTEGER,
    'rank_in_recording' INTEGER,
    'timestamp' FLOAT,
	'rig_to_world' INTEGER,
	'aligned_wrt_previous' BOOLEAN
);

CREATE TABLE 'mappoints'
(
	'id' INTEGER PRIMARY KEY,
	'rank' INTEGER,
	'world_x' FLOAT,
	'world_y' FLOAT,
	'world_z' FLOAT
);

CREATE TABLE 'keypoints'
(
    'id' INTEGER PRIMARY KEY,
    'frame_id' INTEGER,
    'view' INTEGER,
    'u' FLOAT,
    'v' FLOAT
);

CREATE TABLE 'descriptors'
(
    'id' INTEGER PRIMARY KEY,
    'keypoint_id' INTEGER,
    'rank' INTEGER,
    'value' FLOAT
);

CREATE TABLE 'projections'
(
	'id' INTEGER PRIMARY KEY,
    'frame_id' INTEGER,
    'view' INTEGER,
	'x_distorted' FLOAT,
    'y_distorted' FLOAT,
	'type' INTEGER,
	'mappoint_id' INTEGER
);

CREATE TABLE 'densepoints'
(
	'id' INTEGER PRIMARY KEY,
	'frame_id' INTEGER,
	'rig_x' FLOAT,
	'rig_y' FLOAT,
	'rig_z' FLOAT,
	'color_red' FLOAT,
	'color_green' FLOAT,
	'color_blue' FLOAT
);

