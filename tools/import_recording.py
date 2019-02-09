import sqlite3
import shutil
import os
import sys

def generate_directory(path):
    assert( os.path.exists(path) )
    go_on = True
    i = 0
    while go_on:
        dirname = 'rec_' + str(i)
        ret = os.path.join(path, dirname)
        go_on = os.path.exists(ret)
        i += 1
    return ret, dirname

assert( len(sys.argv) == 3 )

input_recording_dir = sys.argv[1]
input_recording_csv = os.path.join(input_recording_dir, 'recording.csv')
output_project = sys.argv[2]
output_db = os.path.join(output_project, 'db.sqlite')

output_rec_path, output_rec_dir = generate_directory(output_project)
os.mkdir(output_rec_path)

csv_file = open(input_recording_csv, 'r')
conn = sqlite3.connect(output_db)

c = conn.cursor()
c.execute("INSERT INTO recordings (name, date, directory, number_of_views) values (?,datetime('now'), ?, 2)", ("imported recording", output_rec_dir))
recording_id = c.lastrowid

rank = 0
for line in csv_file:

    tok = line.strip().split(' ')

    if len(tok) == 4:

        timestamp = float(tok[1])

        c = conn.cursor()
        c.execute("INSERT INTO recording_frames (recording_id,rank,time) values (?,?,?)", (recording_id, rank, timestamp))
        frame_id = c.lastrowid

        viewrank = 0
        for viewfile in ( tok[2], tok[3] ):
            shutil.move(os.path.join(input_recording_dir, viewfile), output_rec_path)
            #print(os.path.join(input_recording_dir, viewfile), output_rec_path)

            c = conn.cursor()
            c.execute("INSERT INTO recording_views (frame_id,view,filename) values (?,?,?)", (frame_id, viewrank, viewfile))

            viewrank += 1

        rank += 1

csv_file.close()
conn.commit()
#conn.rollback()
conn.close()

