#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ui_mainwindow.h"
#include <QMainWindow>

#include "globalVariables.h"

/* Qt libraries */
#include <qcustomplot.h>
#include <QPair>
#include <QTimer>
#include <QVector>
#include <QWidget>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void setRobotLed(bool on);
    void RobotLedOn();
    void RobotLedOff();

    void setq1Led(bool on);
    void q1LedOn();
    void q1LedOff();

    void setqmLed(bool on);
    void qmLedOn();
    void qmLedOff();

    void setqbLed(bool on);
    void qbLedOn();
    void qbLedOff();

    void updateWindow();
    void createPlot(QCustomPlot *plot);
    void drawPlot(QCustomPlot* plot, double t, double y1, const QString& y1Name, std::optional<double> y2 = std::nullopt, std::optional<QString> y2Name = std::nullopt);

    void readUIParams();

private slots:


    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
    double key;
    double Plot_time_window_POS = 2.0;
    QSharedPointer<QCPAxisTickerTime> timeTicker;
};
#endif // MAINWINDOW_H
