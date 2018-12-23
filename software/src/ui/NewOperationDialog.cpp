#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include "NewOperationDialog.h"

NewOperationDialog::NewOperationDialog(Project* proj, QWidget* parent) : QDialog(parent)
{
    mProject = proj;
}

OperationPtr NewOperationDialog::getOperation()
{
    return mOperation;
}

void NewOperationDialog::setOperation(OperationPtr op)
{
    mOperation = std::move(op);
}

Project* NewOperationDialog::project()
{
    return mProject;
}

