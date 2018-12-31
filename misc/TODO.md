* on startup, print some nice ascii logo on stdout.
* make sure that a project has been opened before we do any operation!
* saveCamera must update the model by calling CameraCalibraitonModel::refresh. Same for the others (rig, recording, reconstruction).
* before rig calibration, make sure that there exist some camera calibration.
* before reconstruction, make sure there is some recording and rig calibration.
* add a toolbar in which to indicate current project.
* add CSS stylesheet to improve appearance of QTextEdit.
* save last user input into QSettings and recover it on dialog display.
* raise Project::changed after Project::endTransaction rather than after save{Camera,Rig,Recording,Reconstruction}.

