#include <QLabel>
#include <QMessageBox>
#include <QStackedLayout>
#include <QSettings>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include "OpenReconstructionDialog.h"

OpenReconstructionDialog::OpenReconstructionDialog(VisualizationDataPort* visudata, QWidget* p) : QDialog(p)
{
    mVisualizationData = visudata;

    mProjectPath = new PathWidget(PathWidget::GET_OPEN_FILENAME);

    QPushButton* btn_open_db = new QPushButton("Open database");

    connect(btn_open_db, SIGNAL(clicked()), this, SLOT(openDatabase()));

    mReconstructionList = new QListWidget();

    connect(mReconstructionList, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(accept()));

    QPushButton* btnok = new QPushButton("OK");
    QPushButton* btncancel = new QPushButton("Cancel");

    connect(btnok, SIGNAL(clicked()), this, SLOT(accept()));
    connect(btncancel, SIGNAL(clicked()), this, SLOT(reject()));

    QHBoxLayout* btnlay = new QHBoxLayout();
    btnlay->addWidget(btnok);
    btnlay->addWidget(btncancel);

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(mProjectPath);
    lay->addWidget(btn_open_db);
    lay->addWidget(mReconstructionList);
    lay->addLayout(btnlay);

    setLayout(lay);
    setWindowTitle("Open Reconstruction");

    QSettings s;
    s.beginGroup("open_reconstruction_dialog");
    mProjectPath->setPath(s.value("last_path", "").toString());
    s.endGroup();
}

void OpenReconstructionDialog::accept()
{
    ReconstructionPtr rec(new Reconstruction());
    QListWidgetItem* item = nullptr;

    bool ok = true;
    const char* msg = "";

    if(ok)
    {
        ok = bool(mDB);
        msg = "Please select a database and a reconstruction!";
    }

    if(ok)
    {
        item = mReconstructionList->currentItem();

        ok = bool(item);
        msg = "Please select a database and a reconstruction!";
    }

    if(ok)
    {
        const int i = item->data(Qt::UserRole).toInt();

        ok = mDB->load(i, rec) && bool(rec);
        msg = "Could not load reconstruction!";
    }

    if(ok)
    {
        mVisualizationData->beginWrite();
        mVisualizationData->data().reconstruction = rec;
        mVisualizationData->data().cutListOfFramesIntoSegments();
        mVisualizationData->endWrite();

        QSettings s;
        s.beginGroup("open_reconstruction_dialog");
        s.setValue("last_path", mProjectPath->path());
        s.endGroup();
        s.sync();

        mDB->close();
        QDialog::accept();
    }
    else
    {
        QMessageBox::critical(this, "Error", msg);
    }
}

void OpenReconstructionDialog::openDatabase()
{
    mDB.reset(new SLAMReconstructionDB());

    bool ok = true;

    mReconstructionList->clear();

    if(ok)
    {
        ok = mDB->open(mProjectPath->path().toStdString());
        
        if(ok == false)
        {
            QMessageBox::critical(this, "Error", "Could not open database!");
        }
    }

    if(ok)
    {
        ok = ( mDB->getNumberOfReconstructions() > 0 );

        if(ok == false)
        {
            QMessageBox::critical(this, "Error", "Unexisting or empty database!");
        }
    }

    if(ok)
    {
        for(int i=0; i<mDB->getNumberOfReconstructions(); i++)
        {
            QListWidgetItem* item = new QListWidgetItem( mDB->getReconstructionName(i).c_str() );
            item->setData(Qt::UserRole, i);
            mReconstructionList->addItem(item);
        }
    }
}

