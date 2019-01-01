#include <QScrollArea>
#include <QToolBar>
#include <QVBoxLayout>
#include <iostream>
#include "RecordingPlayerDialog.h"

RecordingPlayerDialog::RecordingPlayerDialog(RecordingHeaderPtr header, QWidget* parent)
{
    mCurrentFrame = -1;

    mHeader = header;
    mReader.reset(new RecordingReader(header, false));

    mTimer = new QTimer(this);

    connect(mTimer, SIGNAL(timeout()), this, SLOT(onTimeout()));

    QToolBar* tb = new QToolBar();
    tb->setToolButtonStyle(Qt::ToolButtonIconOnly);

    QAction* aFirst = tb->addAction("First frame");
    QAction* aPrevious = tb->addAction("Previous frame");
    QAction* aPlay = tb->addAction("Play");
    QAction* aNext = tb->addAction("Next frame");
    QAction* aLast = tb->addAction("Last frame");

    connect(aFirst, SIGNAL(triggered()), this, SLOT(onFirst()));
    connect(aPrevious, SIGNAL(triggered()), this, SLOT(onPrevious()));
    connect(aPlay, SIGNAL(toggled(bool)), this, SLOT(onPlay(bool)));
    connect(aNext, SIGNAL(triggered()), this, SLOT(onNext()));
    connect(aLast, SIGNAL(triggered()), this, SLOT(onLast()));

    aFirst->setIcon(QIcon::fromTheme("media-skip-backward"));
    aPrevious->setIcon(QIcon::fromTheme("media-seek-backward"));
    aPlay->setIcon(QIcon::fromTheme("media-playback-start"));
    aNext->setIcon(QIcon::fromTheme("media-seek-forward"));
    aLast->setIcon(QIcon::fromTheme("media-skip-forward"));

    aPlay->setCheckable(true);

    mSlider = new QSlider();
    mSlider->setOrientation(Qt::Horizontal);
    mSlider->setMinimum(0);
    mSlider->setMaximum(mHeader->num_frames-1);
    mSlider->setSingleStep(1);

    connect(mSlider, SIGNAL(valueChanged(int)), this, SLOT(onSliderValueChanged(int)));

    VideoWidget* video_widget = new VideoWidget();
    mVideo = video_widget->getPort();

    QScrollArea* scroll = new QScrollArea();
    scroll->setAlignment(Qt::AlignCenter);
    scroll->setWidget(video_widget);

    mLabelFrame = new QLabel();
    mLabelTimestamp = new QLabel();

    QStatusBar* sb = new QStatusBar();
    sb->addPermanentWidget(mLabelFrame);
    sb->addPermanentWidget(mLabelTimestamp);

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(tb);
    lay->addWidget(mSlider);
    lay->addWidget(scroll);
    lay->addWidget(sb);
    
    setLayout(lay);
    setWindowTitle("Play Recording");

    showFrame(-1);
}

RecordingPlayerDialog::~RecordingPlayerDialog()
{
}

void RecordingPlayerDialog::onNext()
{
    if( 0 <= mCurrentFrame && mCurrentFrame+1 < mHeader->num_frames )
    {
        showFrame(mCurrentFrame+1);
    }
    else
    {
        showFrame(0);
    }

    mSlider->setValue(mCurrentFrame);
}

void RecordingPlayerDialog::onPrevious()
{
    if( 0 <= mCurrentFrame-1 && mCurrentFrame < mHeader->num_frames )
    {
        showFrame(mCurrentFrame-1);
    }
    else
    {
        showFrame(mHeader->num_frames-1);
    }

    mSlider->setValue(mCurrentFrame);
}

void RecordingPlayerDialog::onPlay(bool value)
{
    if(value)
    {
        mTimer->start(333);
    }
    else
    {
        mTimer->stop();
    }
}

void RecordingPlayerDialog::onSliderValueChanged(int value)
{
    showFrame(value);
}

void RecordingPlayerDialog::onTimeout()
{
    showFrame( (mCurrentFrame+1)%mHeader->num_frames );
    mSlider->setValue(mCurrentFrame);
}

void RecordingPlayerDialog::onFirst()
{
    showFrame(0);
    mSlider->setValue(mCurrentFrame);
}

void RecordingPlayerDialog::onLast()
{
    showFrame(mHeader->num_frames-1);
    mSlider->setValue(mCurrentFrame);
}

void RecordingPlayerDialog::showFrame(int frame)
{
    bool ok = false;

    if(0 <= frame && frame < mHeader->num_frames)
    {
        Image image;
        mReader->seek(frame);
        mReader->trigger();
        mReader->read(image);
        mCurrentFrame = frame;

        Image concat;
        image.concatenate(concat);

        if( concat.isValid() )
        {
            mVideo->beginWrite();
            mVideo->data().image = concat.getFrame();
            std::cout << concat.getFrame().cols << " " << concat.getFrame().rows << std::endl;
            mVideo->endWrite();

            ok = true;
        }
    }

    if(ok)
    {
        mLabelFrame->setText("Frame "+QString::number(mCurrentFrame+1) + "/" + QString::number(mHeader->num_frames));
        mLabelTimestamp->setText("t =  " + QString::number(mHeader->frames[mCurrentFrame].timestamp) );
    }
    else
    {
        cv::Mat image(320, 200, CV_8UC3);
        image = cv::Scalar(0,0,0);

        mVideo->beginWrite();
        mVideo->data().image = image;
        mVideo->endWrite();

        mLabelFrame->setText("N/A");
        mLabelTimestamp->setText("N/A");
    }
}

