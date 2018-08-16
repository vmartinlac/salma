#include <QFormLayout>
#include "StatsWidget.h"

StatsWidget::StatsWidget(
    SLAMOutput* slam,
    QWidget* parent) :
    m_slam(slam),
    QWidget(parent)
{
    m_num_landmarks = new QLabel("n/a");

    QFormLayout* lay = new QFormLayout();

    lay->addRow("Number of landmarks:", m_num_landmarks);

    setLayout(lay);
}
