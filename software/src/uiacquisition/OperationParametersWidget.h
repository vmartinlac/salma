#pragma once

#include <QWidget>
#include "Operation.h"

class OperationParametersWidget : public QWidget
{
public:

    OperationParametersWidget(QWidget* parent=nullptr);

    virtual OperationPtr getOperation() = 0;

    virtual QString name() = 0;
};

