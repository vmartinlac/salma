#include <QFileInfo>
#include "Project.h"

Project::Project(QObject* parent) : QObject(parent)
{
    mCameraCalibrationModel = new CameraCalibrationModel(this);
    mRigCalibrationModel = new RigCalibrationModel(this);
    mRecordingModel = new RecordingModel(this);
    mReconstructionModel = new ReconstructionModel(this);

    connect(this, SIGNAL(cameraCalibrationModelChanged()), mCameraCalibrationModel, SLOT(refresh()));
    connect(this, SIGNAL(rigCalibrationModelChanged()), mRigCalibrationModel, SLOT(refresh()));
    connect(this, SIGNAL(recordingModelChanged()), mRecordingModel, SLOT(refresh()));
    connect(this, SIGNAL(reconstructionModelChanged()), mReconstructionModel, SLOT(refresh()));

    connect(this, SIGNAL(changed()), this, SIGNAL(cameraCalibrationModelChanged()));
    connect(this, SIGNAL(changed()), this, SIGNAL(rigCalibrationModelChanged()));
    connect(this, SIGNAL(changed()), this, SIGNAL(recordingModelChanged()));
    connect(this, SIGNAL(changed()), this, SIGNAL(reconstructionModelChanged()));
}

bool Project::open(const QString& path)
{
    QString db_path;
    bool ok = true;

    if(ok)
    {
        ok = mDir.cd(path);
    }

    if(ok)
    {
        db_path = mDir.absoluteFilePath("db.sqlite");
        ok = QFileInfo::exists(db_path);
    }

    if(ok)
    {
        mDB = QSqlDatabase::database();
        ok = mDB.isValid();
    }

    if(ok)
    {
        mDB.setDatabaseName(db_path);
        ok = mDB.open();
    }

    // TODO check that some table exists.

    changed();

    return ok;
}

void Project::close()
{
    mDir = QDir();
    mDB.close();

    changed();
}

