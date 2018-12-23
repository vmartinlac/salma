#pragma once

#include <QDialog>
#include "Operation.h"
#include "Project.h"

class NewOperationDialog : public QDialog
{
    Q_OBJECT

public:

    NewOperationDialog(Project* project, QWidget* parent=nullptr);

    OperationPtr getOperation();

    Project* project();

protected:

    void setOperation(OperationPtr op);

private:

    Project* mProject;
    OperationPtr mOperation;
};

