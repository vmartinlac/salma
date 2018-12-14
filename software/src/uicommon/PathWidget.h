#pragma once

#include <QLineEdit>
#include <QWidget>

class PathWidget : public QWidget
{
    Q_OBJECT

public:

    enum Mode
    {
        GET_EXISTING_DIRECTORY,
        GET_SAVE_FILENAME,
        GET_OPEN_FILENAME
    };

public:

    PathWidget(Mode mode, QWidget* parent=nullptr);
    QString path();

public slots:

    void setPath(const QString& path);

protected slots:

    void selectFile();

protected:

    QLineEdit* mText;
    Mode mMode;
};

