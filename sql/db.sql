CREATE TABLE poses(
  id INTEGER PRIMARY KEY,
  qx FLOAT,
  qy FLOAT,
  qz FLOAT,
  qw FLOAT,
  x FLOAT,
  y FLOAT,
  z FLOAT
);

CREATE TABLE reconstructions(
  id INTEGER PRIMARY KEY,
  name TEXT,
  reconstruction_date DATE
);

CREATE TABLE frames(
  id INTEGER PRIMARY KEY,
  reconstruction_id INTEGER,
  rank INTEGER,
  rig_to_world INTEGER,
  aligned_wrt_previous BOOLEAN
);

CREATE TABLE views(
  id INTEGER PRIMARY KEY,
  frame_id INTEGER,
  rank INTEGER
);

CREATE TABLE mappoints(
  id INTEGER PRIMARY KEY,
  world_x FLOAT,
  world_y FLOAT,
  world_z FLOAT
);

CREATE TABLE projections(
  id INTEGER PRIMARY KEY,
  view_id INTEGER,
  type INTEGER,
  u FLOAT,
  v FLOAT,
  mappoint_id INTEGER
);

CREATE TABLE densepoints(
  id INTEGER PRIMARY KEY,
  frame_id INTEGER,
  rig_x FLOAT,
  rig_y FLOAT,
  rig_z FLOAT,
  color_red FLOAT,
  color_green FLOAT,
  color_blue FLOAT
);

