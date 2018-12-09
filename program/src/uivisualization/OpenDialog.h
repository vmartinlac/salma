#pragma once

#include <QDialog>
#include <memory>
#include "SLAMReconstructionDB.h"

class OpenDialog : public QDialog
{
    Q_OBJECT

public:

    OpenDialog(QWidget* parent=nullptr);

protected:

    std::shared_ptr<SLAMReconstructionDB> mDB;
};

