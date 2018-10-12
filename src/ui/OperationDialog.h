#pragma once

#include <QDialog>
#include <QRadioButton>
#include <QStackedWidget>
#include <QButtonGroup>
#include <vector>
#include "Operation.h"
#include "OperationParametersWidget.h"

class OperationDialog : public QDialog
{
    Q_OBJECT

public:

    OperationDialog(QWidget* parent=nullptr);

    OperationPtr getOperation();

    int exec() override;

protected:

    QWidget* createHomeWidget();
    void createOperationParametersWidgets();
    OperationParametersWidget* currentOperationParametersWidget();

protected slots:

    void onOK();
    void onCancel();

protected:

    QButtonGroup* mBtnGroup;

    QStackedWidget* mStackedWidget;

    std::vector<OperationParametersWidget*> mOperationParametersWidgets;

    QWidget* mPageHome;

    OperationPtr mOperation;
};

