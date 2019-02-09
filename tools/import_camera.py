
import json
import sqlite3
import shutil
import os
import sys

assert( len(sys.argv) == 3 )

input_recording_dir = sys.argv[1]
output_project = sys.argv[2]
output_db = os.path.join(output_project, 'db.sqlite')

conn = sqlite3.connect(output_db)

def import_camera(name):

    f = open(os.path.join(input_recording_dir, name), "r")
    doc = json.load(f)
    f.close()

    values = ("imported calibration", doc['image_size'][0], doc['image_size'][1], doc['calibration_matrix'][0], doc['calibration_matrix'][4], doc['calibration_matrix'][2], doc['calibration_matrix'][5] )

    c = conn.cursor()
    c.execute("INSERT INTO camera_parameters (name, date, image_width, image_height, fx, fy, cx, cy, distortion_model) values (?,datetime('now'), ?, ?, ?, ?, ?, ?, 0)", values)
    camera_id = c.lastrowid

    for i in range(len(doc['distortion_coefficients'])):
        values = (camera_id, i, doc['distortion_coefficients'][i])
        c = conn.cursor()
        c.execute("INSERT INTO distortion_coefficients (camera_id, rank, value) values (?,?,?)", values)

    print(camera_id)
    return camera_id

def import_pose(pose):
    values = tuple(pose)
    c = conn.cursor()
    c.execute("INSERT INTO poses (qx, qy, qz, qw, x, y, z) values (?,?,?,?,?,?,?)", values)
    return c.lastrowid

def import_rig_camera(rig_id, rank, pose, camera_id):
    pose_id = import_pose(pose)

    values = (rig_id, rank, pose_id, camera_id)
    c = conn.cursor()
    c.execute("INSERT INTO rig_cameras (rig_id, rank, camera_to_rig, camera_id) values (?,?,?,?)", values)

def import_rig(left, right):

    f = open(os.path.join(input_recording_dir, "rig.json"), "r")
    doc = json.load(f)
    f.close()

    values = ("imported calibration", 2)
    c = conn.cursor()
    c.execute("INSERT INTO rig_parameters (name, date, number_of_cameras) values (?,datetime('now'), ?)", values)
    rig_id = c.lastrowid

    import_rig_camera(rig_id, 0, doc['left_camera_to_rig'], left)
    import_rig_camera(rig_id, 1, doc['right_camera_to_rig'], right)

left_cam = import_camera('left_camera.json')
right_cam = import_camera('right_camera.json')
import_rig(left_cam, right_cam)

conn.commit()
conn.close()

