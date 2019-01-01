#include <QLabel>
#include <QTextEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFormLayout>
#include <BuildInfo.h>
#include <iostream>
#include "AboutDialog.h"

AboutDialog::AboutDialog(QWidget* parent) : QDialog(parent)
{
    QString html;

    html += "<html><head>";
    html += "<style type='text/css'>";
    html += "h1 { margin:1em; text-align:center; }";
    html += "th { text-align:left; }";
    html += "p { margin:1em; }";
    html += "table { margin:1em; }";
    html += "</style>";
    html += "</head><body>";
    html += "<h1>SALMA</h1>";
    html += "<p>This software was written by Victor MARTIN LAC between august 2018 and october 2018.</p>";
    html += "<table>";

    html += "<tr><th>Version :</th><td>" +
        QString::number(BuildInfo::getVersionMajor()) + "." +
        QString::number(BuildInfo::getVersionMinor()) + "." +
        QString::number(BuildInfo::getVersionRevision()) + "</td></tr>";

    html += "<tr><th>Compilation date:</th><td>" + QString(BuildInfo::getCompilationDate().c_str()).toHtmlEscaped() + "</td></tr>";

    html += "<tr><th>Compiler:</th><td>" + QString(BuildInfo::getCompilerName().c_str()).toHtmlEscaped() + "</td></tr>";

    html += "<tr><th>Build type:</th><td>" + QString(BuildInfo::getBuildType().c_str()).toHtmlEscaped() + "</td></tr>";

    html += "</table>";
    html += "</body></html>";

    QTextEdit* textedit = new QTextEdit();
    textedit->setReadOnly(true);
    textedit->setHtml(html);

    QPushButton* btn = new QPushButton("Close");

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(textedit);
    lay->addWidget(btn);

    setLayout(lay);
    setWindowTitle("About");
    resize(340, 250);

    QObject::connect(btn, SIGNAL(clicked()), this, SLOT(accept()));
}

