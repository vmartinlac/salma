#pragma once

#include <QDialog>
#include "Operation.h"
#include "OperationThread.h"

class OperationProgressDialog : public QDialog
{
    Q_OBJECT

public:

    OperationProgressDialog(OperationPtr op, QWidget* parent=nullptr);
    ~OperationProgressDialog() override;

protected:

    OperationThread* mThread;
    OperationPtr mOperation;
};

