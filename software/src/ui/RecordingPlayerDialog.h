#pragma once

#include <QStatusBar>
#include <QTimer>
#include <QTimer>
#include <QDialog>
#include "RecordingReader.h"
#include "VideoWidget.h"

class RecordingPlayerDialog : public QDialog
{
public:

    RecordingPlayerDialog(RecordingHeaderPtr reader, QWidget* parent=nullptr);
    ~RecordingPlayerDialog();

protected slots:

    void onPrevious();
    void onPlay();
    void onNext();

protected:

    int mCurrentFrame;
    VideoInputPort* mVideo;
    RecordingReaderPtr mReader;
    QTimer* mTimer;
    QStatusBar* mStatusBar;
};

