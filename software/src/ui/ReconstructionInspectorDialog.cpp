#include <QToolBar>
#include <QStatusBar>
#include <QMessageBox>
#include <QComboBox>
#include <QVBoxLayout>
#include "ReconstructionInspectorDialog.h"

ReconstructionInspectorDialog::ReconstructionInspectorDialog(SLAMReconstructionPtr reconstr, QWidget* parent) : QDialog(parent)
{
    mReconstruction = reconstr;
    mViewer = new ViewerWidget(reconstr);

    mSegmentsWidget = new QComboBox();
    for(int i=0; i<mReconstruction->segments.size(); i++)
    {
        const QString name = "Segment " + QString::number(i);
        mSegmentsWidget->addItem(name, i);
    }

    QToolBar* tb = new QToolBar();

    QAction* aClose = tb->addAction("Close");
    tb->addSeparator();
    tb->addWidget(mSegmentsWidget);
    QAction* aMapPoints = tb->addAction("MapPoints");
    QAction* aDensePoints = tb->addAction("DensePoints");
    QAction* aRigs = tb->addAction("Rigs");
    QAction* aTrajectory = tb->addAction("Trajectory");
    QAction* aLight = tb->addAction("Light");
    tb->addSeparator();
    QAction* aExport = tb->addAction("Export");
    QAction* aHome = tb->addAction("Home");

    aMapPoints->setCheckable(true);
    aDensePoints->setCheckable(true);
    aRigs->setCheckable(true);
    aTrajectory->setCheckable(true);
    aLight->setCheckable(true);

    aMapPoints->setChecked(true);
    aDensePoints->setChecked(true);
    aRigs->setChecked(true);
    aTrajectory->setChecked(true);
    aLight->setChecked(true);

    connect(aClose, SIGNAL(triggered()), this, SLOT(accept()));
    connect(aTrajectory, SIGNAL(toggled(bool)), mViewer, SLOT(showTrajectory(bool)));
    connect(aRigs, SIGNAL(toggled(bool)), mViewer, SLOT(showRigs(bool)));
    connect(aDensePoints, SIGNAL(toggled(bool)), mViewer, SLOT(showDensePoints(bool)));
    connect(aMapPoints, SIGNAL(toggled(bool)), mViewer, SLOT(showMapPoints(bool)));
    connect(aExport, SIGNAL(triggered()), this, SLOT(onExport()));
    connect(mSegmentsWidget, SIGNAL(currentIndexChanged(int)), this, SLOT(onSelectSegment()));
    connect(aHome, SIGNAL(triggered()), mViewer, SLOT(home()));
    connect(aLight, SIGNAL(toggled(bool)), mViewer, SLOT(setLighting(bool)));

    /*
    mInformation = new QLabel();
    mInformation->setText("N/A");

    QStatusBar* sb = new QStatusBar();
    sb->addPermanentWidget(mInformation);
    */

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(tb, 0);
    lay->addWidget(mViewer, 1);
    //lay->addWidget(sb, 0);

    setWindowTitle("Reconstruction Visualization");
    setLayout(lay);

    mViewer->buildScene();
    onSelectSegment();
}

void ReconstructionInspectorDialog::onSelectSegment()
{
    const QVariant v = mSegmentsWidget->currentData();

    if(v.isValid())
    {
        const int seg = v.toInt();

        /*
        if( 0 <= seg && seg < mReconstruction->segments.size() )
        {
            QString txt;
            txt += QString::number(mReconstruction->segments[seg].frames.size()) + " frames.";
            mInformation->setText(txt);
        }
        else
        {
            mInformation->setText("N/A");
        }
        */

        mViewer->showSegment(seg);
        //mViewer->home();
    }
}

void ReconstructionInspectorDialog::onExport()
{
    QMessageBox::critical(this, "Error", "Not implemented!");
}

