#include <QSlider>
#include <QToolBar>
#include <QVBoxLayout>
#include "RecordingPlayerDialog.h"

RecordingPlayerDialog::RecordingPlayerDialog(RecordingHeaderPtr reader, QWidget* parent)
{
    mReader.reset(new RecordingReader(reader, false));

    mTimer = new QTimer(this);

    QToolBar* tb = new QToolBar();

    QAction* aPrevious = tb->addAction("Previous frame");
    QAction* aPlay = tb->addAction("Play");
    QAction* aNext = tb->addAction("Next frame");

    aPrevious->setIcon(QIcon::fromTheme("media-skip-backward"));
    aPlay->setIcon(QIcon::fromTheme("media-playback-start"));
    aNext->setIcon(QIcon::fromTheme("media-skip-forward"));

    aPlay->setCheckable(true);

    QSlider* slider = new QSlider();
    slider->setOrientation(Qt::Horizontal);

    VideoWidget* video_widget = new VideoWidget();
    mVideo = video_widget->getPort();

    mStatusBar = new QStatusBar();

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(tb);
    lay->addWidget(slider);
    lay->addWidget(video_widget);
    lay->addWidget(mStatusBar);
    
    setLayout(lay);
    setWindowTitle("Play Recording");
}

RecordingPlayerDialog::~RecordingPlayerDialog()
{
}

void RecordingPlayerDialog::onNext()
{
}

void RecordingPlayerDialog::onPrevious()
{
}

void RecordingPlayerDialog::onPlay()
{
}

