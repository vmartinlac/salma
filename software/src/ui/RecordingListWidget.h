#pragma once

#include <QComboBox>

class Project;

class RecordingListWidget : public QComboBox
{
    Q_OBJECT
public:
    
    RecordingListWidget(Project* project, QWidget* parent=nullptr);
    ~RecordingListWidget();

    int getRecordingId();

public slots:

    void refreshList();

protected:

    Project* mProject;
};

