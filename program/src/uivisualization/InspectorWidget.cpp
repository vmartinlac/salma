#include <QTreeView>
#include <QTextEdit>
#include "InspectorWidget.h"

InspectorWidget::InspectorWidget(QWidget* parent) : QSplitter(parent)
{
    QTreeView* treeview = new QTreeView();
    QTextEdit* text = new QTextEdit();

    addWidget(treeview);
    addWidget(text);
}

