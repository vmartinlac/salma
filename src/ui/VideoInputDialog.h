#pragma once

#include <QComboBox>
#include <QSpinBox>
#include <QFrame>
#include <QRadioButton>
#include <QDialog>
#include <memory>
#include "Camera.h"

class VideoInputDialog : public QDialog
{
    Q_OBJECT

public:

    VideoInputDialog(QWidget* parent=nullptr);

    static std::shared_ptr<Camera> ask_video_input(QWidget* parent=nullptr);

protected:

    void create_avt_frame();
    void create_opencv_frame();
    void create_file_frame();

protected slots:

    void choose_file();
    void input_changed();
    void accept() override;

protected:

    QRadioButton* m_avt;
    QRadioButton* m_opencv;
    QRadioButton* m_file;

    bool m_has_avt;
    QFrame* m_avt_frame;
    QComboBox* m_avt_camera;

    QFrame* m_opencv_frame;
    QSpinBox* m_opencv_camera;

    QFrame* m_file_frame;
    QPushButton* m_file_choose;
    QString m_file_path;
};
