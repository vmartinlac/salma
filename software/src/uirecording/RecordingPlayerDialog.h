#pragma once

#include <QLabel>
#include <QStatusBar>
#include <QSlider>
#include <QTimer>
#include <QTimer>
#include <QDialog>
#include "RecordingReader.h"
#include "VideoWidget.h"

class RecordingPlayerDialog : public QDialog
{
    Q_OBJECT

public:

    RecordingPlayerDialog(RecordingHeaderPtr reader, QWidget* parent=nullptr);
    ~RecordingPlayerDialog();

protected slots:

    void onFirst();
    void onPrevious();
    void onPlay(bool);
    void onNext();
    void onLast();
    void onSliderValueChanged(int);
    void onTimeout();
    void showFrame(int frame);

protected:

    int mCurrentFrame;
    VideoInputPort* mVideo;
    RecordingReaderPtr mReader;
    RecordingHeaderPtr mHeader;
    QTimer* mTimer;
    QLabel* mLabelFrame;
    QLabel* mLabelTimestamp;
    QSlider* mSlider;
};

