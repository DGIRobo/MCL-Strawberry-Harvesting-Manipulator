/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.9.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionManipulator_Position_Data;
    QWidget *centralwidget;
    QCustomPlot *q1Plot;
    QLabel *label_2;
    QLabel *label_3;
    QCustomPlot *qmPlot;
    QCustomPlot *qbPlot;
    QLabel *label_4;
    QCustomPlot *xposPlot;
    QLabel *label_5;
    QLabel *label_6;
    QCustomPlot *yposPlot;
    QCustomPlot *zposPlot;
    QLabel *label_7;
    QLabel *label_8;
    QCustomPlot *yposIctrlPlot;
    QCustomPlot *zposIctrlPlot;
    QLabel *label_9;
    QLabel *label_10;
    QCustomPlot *xposIctrlPlot;
    QLabel *label_11;
    QCustomPlot *qmCurrentPlot;
    QCustomPlot *qbCurrentPlot;
    QLabel *label_12;
    QLabel *label_13;
    QCustomPlot *q1CurrentPlot;
    QLabel *robot_activation_led;
    QLabel *label_14;
    QLabel *q1_activation_led;
    QLabel *label_15;
    QLabel *label_16;
    QLabel *label_17;
    QLabel *qb_activation_led;
    QLabel *label_18;
    QLabel *qm_activation_led;
    QLabel *label_19;
    QDoubleSpinBox *targetXpos;
    QLabel *label_20;
    QLabel *label_21;
    QLabel *label_22;
    QDoubleSpinBox *targetYpos;
    QDoubleSpinBox *targetZpos;
    QLabel *label_23;
    QDoubleSpinBox *xposPctrlGain;
    QLabel *label_24;
    QLabel *label_25;
    QDoubleSpinBox *xposIctrlGain;
    QDoubleSpinBox *xposDctrlGain;
    QLabel *label_26;
    QLabel *label_27;
    QDoubleSpinBox *xposCutoffFreq;
    QDoubleSpinBox *xposWindupGain;
    QLabel *label_28;
    QLabel *label_29;
    QLabel *label_30;
    QDoubleSpinBox *yposIctrlGain;
    QLabel *label_31;
    QLabel *label_32;
    QDoubleSpinBox *yposDctrlGain;
    QDoubleSpinBox *yposPctrlGain;
    QDoubleSpinBox *yposWindupGain;
    QDoubleSpinBox *yposCutoffFreq;
    QLabel *label_33;
    QLabel *label_34;
    QLabel *label_35;
    QLabel *label_36;
    QDoubleSpinBox *zposIctrlGain;
    QLabel *label_37;
    QLabel *label_38;
    QDoubleSpinBox *zposWindupGain;
    QLabel *label_39;
    QDoubleSpinBox *zposPctrlGain;
    QDoubleSpinBox *zposCutoffFreq;
    QLabel *label_40;
    QDoubleSpinBox *zposDctrlGain;
    QLabel *label_41;
    QStatusBar *statusbar;
    QMenuBar *menubar;
    QMenu *menuStrawberry_Harvesting_Robot;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(1366, 974);
        actionManipulator_Position_Data = new QAction(MainWindow);
        actionManipulator_Position_Data->setObjectName("actionManipulator_Position_Data");
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        q1Plot = new QCustomPlot(centralwidget);
        q1Plot->setObjectName("q1Plot");
        q1Plot->setGeometry(QRect(230, 110, 361, 181));
        label_2 = new QLabel(centralwidget);
        label_2->setObjectName("label_2");
        label_2->setGeometry(QRect(340, 86, 141, 20));
        label_2->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_3 = new QLabel(centralwidget);
        label_3->setObjectName("label_3");
        label_3->setGeometry(QRect(710, 89, 161, 21));
        label_3->setAlignment(Qt::AlignmentFlag::AlignCenter);
        qmPlot = new QCustomPlot(centralwidget);
        qmPlot->setObjectName("qmPlot");
        qmPlot->setGeometry(QRect(610, 110, 361, 181));
        qbPlot = new QCustomPlot(centralwidget);
        qbPlot->setObjectName("qbPlot");
        qbPlot->setGeometry(QRect(990, 110, 361, 181));
        label_4 = new QLabel(centralwidget);
        label_4->setObjectName("label_4");
        label_4->setGeometry(QRect(1100, 86, 141, 20));
        label_4->setAlignment(Qt::AlignmentFlag::AlignCenter);
        xposPlot = new QCustomPlot(centralwidget);
        xposPlot->setObjectName("xposPlot");
        xposPlot->setGeometry(QRect(230, 320, 361, 181));
        label_5 = new QLabel(centralwidget);
        label_5->setObjectName("label_5");
        label_5->setGeometry(QRect(340, 296, 141, 20));
        label_5->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_6 = new QLabel(centralwidget);
        label_6->setObjectName("label_6");
        label_6->setGeometry(QRect(720, 296, 141, 20));
        label_6->setAlignment(Qt::AlignmentFlag::AlignCenter);
        yposPlot = new QCustomPlot(centralwidget);
        yposPlot->setObjectName("yposPlot");
        yposPlot->setGeometry(QRect(610, 320, 361, 181));
        zposPlot = new QCustomPlot(centralwidget);
        zposPlot->setObjectName("zposPlot");
        zposPlot->setGeometry(QRect(990, 320, 361, 181));
        label_7 = new QLabel(centralwidget);
        label_7->setObjectName("label_7");
        label_7->setGeometry(QRect(1090, 299, 161, 21));
        label_7->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_8 = new QLabel(centralwidget);
        label_8->setObjectName("label_8");
        label_8->setGeometry(QRect(1100, 506, 141, 20));
        label_8->setAlignment(Qt::AlignmentFlag::AlignCenter);
        yposIctrlPlot = new QCustomPlot(centralwidget);
        yposIctrlPlot->setObjectName("yposIctrlPlot");
        yposIctrlPlot->setGeometry(QRect(610, 530, 361, 181));
        zposIctrlPlot = new QCustomPlot(centralwidget);
        zposIctrlPlot->setObjectName("zposIctrlPlot");
        zposIctrlPlot->setGeometry(QRect(990, 530, 361, 181));
        label_9 = new QLabel(centralwidget);
        label_9->setObjectName("label_9");
        label_9->setGeometry(QRect(720, 506, 141, 20));
        label_9->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_10 = new QLabel(centralwidget);
        label_10->setObjectName("label_10");
        label_10->setGeometry(QRect(340, 506, 141, 20));
        label_10->setAlignment(Qt::AlignmentFlag::AlignCenter);
        xposIctrlPlot = new QCustomPlot(centralwidget);
        xposIctrlPlot->setObjectName("xposIctrlPlot");
        xposIctrlPlot->setGeometry(QRect(230, 530, 361, 181));
        label_11 = new QLabel(centralwidget);
        label_11->setObjectName("label_11");
        label_11->setGeometry(QRect(1060, 716, 221, 20));
        label_11->setAlignment(Qt::AlignmentFlag::AlignCenter);
        qmCurrentPlot = new QCustomPlot(centralwidget);
        qmCurrentPlot->setObjectName("qmCurrentPlot");
        qmCurrentPlot->setGeometry(QRect(610, 740, 361, 181));
        qbCurrentPlot = new QCustomPlot(centralwidget);
        qbCurrentPlot->setObjectName("qbCurrentPlot");
        qbCurrentPlot->setGeometry(QRect(990, 740, 361, 181));
        label_12 = new QLabel(centralwidget);
        label_12->setObjectName("label_12");
        label_12->setGeometry(QRect(680, 716, 221, 20));
        label_12->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_13 = new QLabel(centralwidget);
        label_13->setObjectName("label_13");
        label_13->setGeometry(QRect(310, 716, 201, 20));
        label_13->setAlignment(Qt::AlignmentFlag::AlignCenter);
        q1CurrentPlot = new QCustomPlot(centralwidget);
        q1CurrentPlot->setObjectName("q1CurrentPlot");
        q1CurrentPlot->setGeometry(QRect(230, 740, 361, 181));
        robot_activation_led = new QLabel(centralwidget);
        robot_activation_led->setObjectName("robot_activation_led");
        robot_activation_led->setGeometry(QRect(70, 30, 100, 100));
        robot_activation_led->setMinimumSize(QSize(100, 100));
        robot_activation_led->setMaximumSize(QSize(100, 100));
        robot_activation_led->setStyleSheet(QString::fromUtf8("QLabel#robot_activation_led {\n"
"  background: #95a5a6;       /* OFF \355\232\214\354\203\211 */\n"
"  border: 1px solid #7f8c8d;\n"
"  border-radius: 50px;         /* \353\260\230\354\247\200\353\246\204 = \355\201\254\352\270\260/2 */\n"
"}\n"
"QLabel#robot_activation_led[on=\"true\"]  { /* ON */\n"
"	background:#2ecc71; \n"
"	border-color:#27ae60; \n"
"} \n"
"QLabel#robot_activation_led[on=\"false\"] { /* OFF */\n"
"	background:#95a5a6; \n"
"	border-color:#7f8c8d; \n"
"}"));
        label_14 = new QLabel(centralwidget);
        label_14->setObjectName("label_14");
        label_14->setGeometry(QRect(40, 10, 161, 20));
        label_14->setAlignment(Qt::AlignmentFlag::AlignCenter);
        q1_activation_led = new QLabel(centralwidget);
        q1_activation_led->setObjectName("q1_activation_led");
        q1_activation_led->setGeometry(QRect(30, 190, 30, 30));
        q1_activation_led->setMinimumSize(QSize(30, 30));
        q1_activation_led->setMaximumSize(QSize(30, 30));
        q1_activation_led->setStyleSheet(QString::fromUtf8("QLabel#q1_activation_led {\n"
"  background: #95a5a6;       /* OFF \355\232\214\354\203\211 */\n"
"  border: 1px solid #7f8c8d;\n"
"  border-radius: 15px;         /* \353\260\230\354\247\200\353\246\204 = \355\201\254\352\270\260/2 */\n"
"}\n"
"QLabel#q1_activation_led[on=\"true\"]  { /* ON */\n"
"	background:#2ecc71; \n"
"	border-color:#27ae60; \n"
"} \n"
"QLabel#q1_activation_led[on=\"false\"] { /* OFF */\n"
"	background:#95a5a6; \n"
"	border-color:#7f8c8d; \n"
"}"));
        label_15 = new QLabel(centralwidget);
        label_15->setObjectName("label_15");
        label_15->setGeometry(QRect(50, 150, 141, 16));
        label_15->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_16 = new QLabel(centralwidget);
        label_16->setObjectName("label_16");
        label_16->setGeometry(QRect(20, 170, 51, 16));
        label_16->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_17 = new QLabel(centralwidget);
        label_17->setObjectName("label_17");
        label_17->setGeometry(QRect(160, 170, 51, 16));
        label_17->setAlignment(Qt::AlignmentFlag::AlignCenter);
        qb_activation_led = new QLabel(centralwidget);
        qb_activation_led->setObjectName("qb_activation_led");
        qb_activation_led->setGeometry(QRect(170, 190, 30, 30));
        qb_activation_led->setMinimumSize(QSize(30, 30));
        qb_activation_led->setMaximumSize(QSize(30, 30));
        qb_activation_led->setStyleSheet(QString::fromUtf8("QLabel#qb_activation_led {\n"
"  background: #95a5a6;       /* OFF \355\232\214\354\203\211 */\n"
"  border: 1px solid #7f8c8d;\n"
"  border-radius: 15px;         /* \353\260\230\354\247\200\353\246\204 = \355\201\254\352\270\260/2 */\n"
"}\n"
"QLabel#qb_activation_led[on=\"true\"]  { /* ON */\n"
"	background:#2ecc71; \n"
"	border-color:#27ae60; \n"
"} \n"
"QLabel#qb_activation_led[on=\"false\"] { /* OFF */\n"
"	background:#95a5a6; \n"
"	border-color:#7f8c8d; \n"
"}"));
        label_18 = new QLabel(centralwidget);
        label_18->setObjectName("label_18");
        label_18->setGeometry(QRect(90, 170, 51, 16));
        label_18->setAlignment(Qt::AlignmentFlag::AlignCenter);
        qm_activation_led = new QLabel(centralwidget);
        qm_activation_led->setObjectName("qm_activation_led");
        qm_activation_led->setGeometry(QRect(100, 190, 30, 30));
        qm_activation_led->setMinimumSize(QSize(30, 30));
        qm_activation_led->setMaximumSize(QSize(30, 30));
        qm_activation_led->setStyleSheet(QString::fromUtf8("QLabel#qm_activation_led {\n"
"  background: #95a5a6;       /* OFF \355\232\214\354\203\211 */\n"
"  border: 1px solid #7f8c8d;\n"
"  border-radius: 15px;         /* \353\260\230\354\247\200\353\246\204 = \355\201\254\352\270\260/2 */\n"
"}\n"
"QLabel#qm_activation_led[on=\"true\"]  { /* ON */\n"
"	background:#2ecc71; \n"
"	border-color:#27ae60; \n"
"} \n"
"QLabel#qm_activation_led[on=\"false\"] { /* OFF */\n"
"	background:#95a5a6; \n"
"	border-color:#7f8c8d; \n"
"}"));
        label_19 = new QLabel(centralwidget);
        label_19->setObjectName("label_19");
        label_19->setGeometry(QRect(40, 240, 161, 20));
        label_19->setAlignment(Qt::AlignmentFlag::AlignCenter);
        targetXpos = new QDoubleSpinBox(centralwidget);
        targetXpos->setObjectName("targetXpos");
        targetXpos->setGeometry(QRect(120, 270, 81, 22));
        label_20 = new QLabel(centralwidget);
        label_20->setObjectName("label_20");
        label_20->setGeometry(QRect(40, 270, 71, 21));
        label_20->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_21 = new QLabel(centralwidget);
        label_21->setObjectName("label_21");
        label_21->setGeometry(QRect(40, 300, 71, 21));
        label_21->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_22 = new QLabel(centralwidget);
        label_22->setObjectName("label_22");
        label_22->setGeometry(QRect(40, 330, 71, 21));
        label_22->setAlignment(Qt::AlignmentFlag::AlignCenter);
        targetYpos = new QDoubleSpinBox(centralwidget);
        targetYpos->setObjectName("targetYpos");
        targetYpos->setGeometry(QRect(120, 300, 81, 22));
        targetZpos = new QDoubleSpinBox(centralwidget);
        targetZpos->setObjectName("targetZpos");
        targetZpos->setGeometry(QRect(120, 330, 81, 22));
        label_23 = new QLabel(centralwidget);
        label_23->setObjectName("label_23");
        label_23->setGeometry(QRect(29, 370, 171, 20));
        label_23->setAlignment(Qt::AlignmentFlag::AlignCenter);
        xposPctrlGain = new QDoubleSpinBox(centralwidget);
        xposPctrlGain->setObjectName("xposPctrlGain");
        xposPctrlGain->setGeometry(QRect(120, 400, 81, 22));
        label_24 = new QLabel(centralwidget);
        label_24->setObjectName("label_24");
        label_24->setGeometry(QRect(39, 400, 71, 21));
        label_24->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_25 = new QLabel(centralwidget);
        label_25->setObjectName("label_25");
        label_25->setGeometry(QRect(39, 430, 71, 21));
        label_25->setAlignment(Qt::AlignmentFlag::AlignCenter);
        xposIctrlGain = new QDoubleSpinBox(centralwidget);
        xposIctrlGain->setObjectName("xposIctrlGain");
        xposIctrlGain->setGeometry(QRect(120, 430, 81, 22));
        xposDctrlGain = new QDoubleSpinBox(centralwidget);
        xposDctrlGain->setObjectName("xposDctrlGain");
        xposDctrlGain->setGeometry(QRect(120, 460, 81, 22));
        label_26 = new QLabel(centralwidget);
        label_26->setObjectName("label_26");
        label_26->setGeometry(QRect(39, 460, 71, 21));
        label_26->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_27 = new QLabel(centralwidget);
        label_27->setObjectName("label_27");
        label_27->setGeometry(QRect(39, 490, 71, 21));
        label_27->setAlignment(Qt::AlignmentFlag::AlignCenter);
        xposCutoffFreq = new QDoubleSpinBox(centralwidget);
        xposCutoffFreq->setObjectName("xposCutoffFreq");
        xposCutoffFreq->setGeometry(QRect(120, 490, 81, 22));
        xposWindupGain = new QDoubleSpinBox(centralwidget);
        xposWindupGain->setObjectName("xposWindupGain");
        xposWindupGain->setGeometry(QRect(120, 520, 81, 22));
        label_28 = new QLabel(centralwidget);
        label_28->setObjectName("label_28");
        label_28->setGeometry(QRect(39, 520, 71, 21));
        label_28->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_29 = new QLabel(centralwidget);
        label_29->setObjectName("label_29");
        label_29->setGeometry(QRect(39, 650, 71, 21));
        label_29->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_30 = new QLabel(centralwidget);
        label_30->setObjectName("label_30");
        label_30->setGeometry(QRect(39, 710, 71, 21));
        label_30->setAlignment(Qt::AlignmentFlag::AlignCenter);
        yposIctrlGain = new QDoubleSpinBox(centralwidget);
        yposIctrlGain->setObjectName("yposIctrlGain");
        yposIctrlGain->setGeometry(QRect(120, 620, 81, 22));
        label_31 = new QLabel(centralwidget);
        label_31->setObjectName("label_31");
        label_31->setGeometry(QRect(39, 590, 71, 21));
        label_31->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_32 = new QLabel(centralwidget);
        label_32->setObjectName("label_32");
        label_32->setGeometry(QRect(39, 620, 71, 21));
        label_32->setAlignment(Qt::AlignmentFlag::AlignCenter);
        yposDctrlGain = new QDoubleSpinBox(centralwidget);
        yposDctrlGain->setObjectName("yposDctrlGain");
        yposDctrlGain->setGeometry(QRect(120, 650, 81, 22));
        yposPctrlGain = new QDoubleSpinBox(centralwidget);
        yposPctrlGain->setObjectName("yposPctrlGain");
        yposPctrlGain->setGeometry(QRect(120, 590, 81, 22));
        yposWindupGain = new QDoubleSpinBox(centralwidget);
        yposWindupGain->setObjectName("yposWindupGain");
        yposWindupGain->setGeometry(QRect(120, 710, 81, 22));
        yposCutoffFreq = new QDoubleSpinBox(centralwidget);
        yposCutoffFreq->setObjectName("yposCutoffFreq");
        yposCutoffFreq->setGeometry(QRect(120, 680, 81, 22));
        label_33 = new QLabel(centralwidget);
        label_33->setObjectName("label_33");
        label_33->setGeometry(QRect(29, 560, 171, 20));
        label_33->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_34 = new QLabel(centralwidget);
        label_34->setObjectName("label_34");
        label_34->setGeometry(QRect(39, 680, 71, 21));
        label_34->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_35 = new QLabel(centralwidget);
        label_35->setObjectName("label_35");
        label_35->setGeometry(QRect(29, 750, 171, 20));
        label_35->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_36 = new QLabel(centralwidget);
        label_36->setObjectName("label_36");
        label_36->setGeometry(QRect(39, 840, 71, 21));
        label_36->setAlignment(Qt::AlignmentFlag::AlignCenter);
        zposIctrlGain = new QDoubleSpinBox(centralwidget);
        zposIctrlGain->setObjectName("zposIctrlGain");
        zposIctrlGain->setGeometry(QRect(120, 810, 81, 22));
        label_37 = new QLabel(centralwidget);
        label_37->setObjectName("label_37");
        label_37->setGeometry(QRect(39, 810, 71, 21));
        label_37->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_38 = new QLabel(centralwidget);
        label_38->setObjectName("label_38");
        label_38->setGeometry(QRect(39, 900, 71, 21));
        label_38->setAlignment(Qt::AlignmentFlag::AlignCenter);
        zposWindupGain = new QDoubleSpinBox(centralwidget);
        zposWindupGain->setObjectName("zposWindupGain");
        zposWindupGain->setGeometry(QRect(120, 900, 81, 22));
        label_39 = new QLabel(centralwidget);
        label_39->setObjectName("label_39");
        label_39->setGeometry(QRect(39, 780, 71, 21));
        label_39->setAlignment(Qt::AlignmentFlag::AlignCenter);
        zposPctrlGain = new QDoubleSpinBox(centralwidget);
        zposPctrlGain->setObjectName("zposPctrlGain");
        zposPctrlGain->setGeometry(QRect(120, 780, 81, 22));
        zposCutoffFreq = new QDoubleSpinBox(centralwidget);
        zposCutoffFreq->setObjectName("zposCutoffFreq");
        zposCutoffFreq->setGeometry(QRect(120, 870, 81, 22));
        label_40 = new QLabel(centralwidget);
        label_40->setObjectName("label_40");
        label_40->setGeometry(QRect(39, 870, 71, 21));
        label_40->setAlignment(Qt::AlignmentFlag::AlignCenter);
        zposDctrlGain = new QDoubleSpinBox(centralwidget);
        zposDctrlGain->setObjectName("zposDctrlGain");
        zposDctrlGain->setGeometry(QRect(120, 840, 81, 22));
        label_41 = new QLabel(centralwidget);
        label_41->setObjectName("label_41");
        label_41->setGeometry(QRect(820, 10, 531, 41));
        label_41->setScaledContents(false);
        label_41->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);
        MainWindow->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 1366, 22));
        menuStrawberry_Harvesting_Robot = new QMenu(menubar);
        menuStrawberry_Harvesting_Robot->setObjectName("menuStrawberry_Harvesting_Robot");
        MainWindow->setMenuBar(menubar);

        menubar->addAction(menuStrawberry_Harvesting_Robot->menuAction());

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        actionManipulator_Position_Data->setText(QCoreApplication::translate("MainWindow", "Manipulator Position Data", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:700;\">Yaw Joint Position</span></p></body></html>", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:700;\">Mono Joint Position</span></p></body></html>", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:700;\">Bi Joint Position</span></p></body></html>", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:700;\">End Effector Xpos</span></p></body></html>", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:700;\">End Effector Ypos</span></p></body></html>", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:700;\">End Effector Zpos</span></p></body></html>", nullptr));
        label_8->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:700;\">I Controller Zpos</span></p></body></html>", nullptr));
        label_9->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:700;\">I Controller Ypos</span></p></body></html>", nullptr));
        label_10->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:700;\">I Controller Xpos</span></p></body></html>", nullptr));
        label_11->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:700;\">Bi Joint Input Current</span></p></body></html>", nullptr));
        label_12->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:700;\">Mono Joint Input Current</span></p></body></html>", nullptr));
        label_13->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:700;\">Yaw Joint Input Current</span></p></body></html>", nullptr));
        robot_activation_led->setText(QString());
        label_14->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:10pt; font-weight:700;\">Manipulator Activation</span></p></body></html>", nullptr));
        q1_activation_led->setText(QString());
        label_15->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:10pt; font-weight:700;\">Joint Activation</span></p></body></html>", nullptr));
        label_16->setText(QCoreApplication::translate("MainWindow", "Yaw", nullptr));
        label_17->setText(QCoreApplication::translate("MainWindow", "Bi", nullptr));
        qb_activation_led->setText(QString());
        label_18->setText(QCoreApplication::translate("MainWindow", "Mono", nullptr));
        qm_activation_led->setText(QString());
        label_19->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:10pt; font-weight:700;\">Target Position Reference</span></p></body></html>", nullptr));
        label_20->setText(QCoreApplication::translate("MainWindow", "EE Xpos ref", nullptr));
        label_21->setText(QCoreApplication::translate("MainWindow", "EE Ypos ref", nullptr));
        label_22->setText(QCoreApplication::translate("MainWindow", "EE Zpos ref", nullptr));
        label_23->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:10pt; font-weight:700;\">Xpos Control Gain Tunning</span></p></body></html>", nullptr));
        label_24->setText(QCoreApplication::translate("MainWindow", "P gain", nullptr));
        label_25->setText(QCoreApplication::translate("MainWindow", "I gain", nullptr));
        label_26->setText(QCoreApplication::translate("MainWindow", "D gain", nullptr));
        label_27->setText(QCoreApplication::translate("MainWindow", "Cutoff freq", nullptr));
        label_28->setText(QCoreApplication::translate("MainWindow", "Anti-windup", nullptr));
        label_29->setText(QCoreApplication::translate("MainWindow", "D gain", nullptr));
        label_30->setText(QCoreApplication::translate("MainWindow", "Anti-windup", nullptr));
        label_31->setText(QCoreApplication::translate("MainWindow", "P gain", nullptr));
        label_32->setText(QCoreApplication::translate("MainWindow", "I gain", nullptr));
        label_33->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:10pt; font-weight:700;\">Ypos Control Gain Tunning</span></p></body></html>", nullptr));
        label_34->setText(QCoreApplication::translate("MainWindow", "Cutoff freq", nullptr));
        label_35->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:10pt; font-weight:700;\">Zpos Control Gain Tunning</span></p></body></html>", nullptr));
        label_36->setText(QCoreApplication::translate("MainWindow", "D gain", nullptr));
        label_37->setText(QCoreApplication::translate("MainWindow", "I gain", nullptr));
        label_38->setText(QCoreApplication::translate("MainWindow", "Anti-windup", nullptr));
        label_39->setText(QCoreApplication::translate("MainWindow", "P gain", nullptr));
        label_40->setText(QCoreApplication::translate("MainWindow", "Cutoff freq", nullptr));
        label_41->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:20pt; font-weight:700;\">Strawberry Harvesting Robot GUI</span><span style=\" font-family:'Roboto','sans-serif'; font-size:20pt; color:#000000; background-color:#ffffff;\">\360\237\215\223</span><span style=\" font-family:'Apple Color Emoji','Segoe UI Emoji','Noto Color Emoji','Android Emoji','EmojiSymbols','EmojiOne Mozilla','Twemoji Mozilla','Segoe UI Symbol','Noto Color Emoji Compat','emoji','notoEmoji','notoEmoji Fallback'; font-size:20pt; color:#101114; background-color:#f0f3f7;\">\360\237\244\226</span></p></body></html>", nullptr));
        menuStrawberry_Harvesting_Robot->setTitle(QCoreApplication::translate("MainWindow", "Manipulator Position Data", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
