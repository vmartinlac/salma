#pragma once

#include <QWidget>
#include "Operation.h"

class OperationWidget : public QWidget
{
public:

    OperationWidget(QWidget* parent=nullptr);

    virtual OperationPtr getOperation() = 0;

    virtual QString name() = 0;
};

