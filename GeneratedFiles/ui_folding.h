/********************************************************************************
** Form generated from reading UI file 'folding.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FOLDING_H
#define UI_FOLDING_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_FoldingClass
{
public:
    QAction *actionFoldiding;
    QAction *actionOutlines;
    QAction *actionInitialize;
    QAction *actionSegmentation;
    QAction *actionFolding_output;
    QAction *actionObj_output;
    QAction *actionWhole;
    QAction *actionFoldingData;
    QAction *actionFolding_import;
    QAction *actionObj_import;
    QWidget *centralWidget;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_inputFille2;
    QLabel *label_inputFile;
    QSpacerItem *verticalSpacer;
    QVBoxLayout *verticalLayout_5;
    QLabel *objFileName;
    QLabel *foldFileName;
    QSpacerItem *horizontalSpacer;
    QSpacerItem *verticalSpacer_2;
    QTableWidget *tableWidget;
    QMenuBar *menuBar;
    QMenu *menuStart;
    QMenu *menuImport;
    QMenu *menuOutput;
    QMenu *menuTool;
    QMenu *menuOptimization;
    QMenu *menuDebug;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *FoldingClass)
    {
        if (FoldingClass->objectName().isEmpty())
            FoldingClass->setObjectName(QStringLiteral("FoldingClass"));
        FoldingClass->resize(591, 576);
        actionFoldiding = new QAction(FoldingClass);
        actionFoldiding->setObjectName(QStringLiteral("actionFoldiding"));
        actionOutlines = new QAction(FoldingClass);
        actionOutlines->setObjectName(QStringLiteral("actionOutlines"));
        actionInitialize = new QAction(FoldingClass);
        actionInitialize->setObjectName(QStringLiteral("actionInitialize"));
        actionSegmentation = new QAction(FoldingClass);
        actionSegmentation->setObjectName(QStringLiteral("actionSegmentation"));
        actionFolding_output = new QAction(FoldingClass);
        actionFolding_output->setObjectName(QStringLiteral("actionFolding_output"));
        actionObj_output = new QAction(FoldingClass);
        actionObj_output->setObjectName(QStringLiteral("actionObj_output"));
        actionWhole = new QAction(FoldingClass);
        actionWhole->setObjectName(QStringLiteral("actionWhole"));
        actionFoldingData = new QAction(FoldingClass);
        actionFoldingData->setObjectName(QStringLiteral("actionFoldingData"));
        actionFolding_import = new QAction(FoldingClass);
        actionFolding_import->setObjectName(QStringLiteral("actionFolding_import"));
        actionObj_import = new QAction(FoldingClass);
        actionObj_import->setObjectName(QStringLiteral("actionObj_import"));
        centralWidget = new QWidget(FoldingClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayoutWidget = new QWidget(centralWidget);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(0, 0, 581, 511));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, -1, -1, -1);
        label_inputFille2 = new QLabel(verticalLayoutWidget);
        label_inputFille2->setObjectName(QStringLiteral("label_inputFille2"));

        verticalLayout_3->addWidget(label_inputFille2);

        label_inputFile = new QLabel(verticalLayoutWidget);
        label_inputFile->setObjectName(QStringLiteral("label_inputFile"));

        verticalLayout_3->addWidget(label_inputFile);

        verticalSpacer = new QSpacerItem(20, 60, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer);


        horizontalLayout_2->addLayout(verticalLayout_3);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, -1, -1, -1);
        objFileName = new QLabel(verticalLayoutWidget);
        objFileName->setObjectName(QStringLiteral("objFileName"));
        objFileName->setStyleSheet(QLatin1String("QLabel { \n"
"	background-color : white; \n"
"	color : black;\n"
"	border: 2px solid darkgray;\n"
" }"));

        verticalLayout_5->addWidget(objFileName);

        foldFileName = new QLabel(verticalLayoutWidget);
        foldFileName->setObjectName(QStringLiteral("foldFileName"));
        foldFileName->setStyleSheet(QLatin1String("QLabel { \n"
"	background-color : white; \n"
"	color : black;\n"
"	border: 2px solid darkgray;\n"
" }"));

        verticalLayout_5->addWidget(foldFileName);

        horizontalSpacer = new QSpacerItem(150, 20, QSizePolicy::Preferred, QSizePolicy::Minimum);

        verticalLayout_5->addItem(horizontalSpacer);

        verticalSpacer_2 = new QSpacerItem(9, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_5->addItem(verticalSpacer_2);


        horizontalLayout_2->addLayout(verticalLayout_5);

        tableWidget = new QTableWidget(verticalLayoutWidget);
        tableWidget->setObjectName(QStringLiteral("tableWidget"));

        horizontalLayout_2->addWidget(tableWidget);


        verticalLayout->addLayout(horizontalLayout_2);

        FoldingClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(FoldingClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 591, 24));
        menuStart = new QMenu(menuBar);
        menuStart->setObjectName(QStringLiteral("menuStart"));
        menuImport = new QMenu(menuStart);
        menuImport->setObjectName(QStringLiteral("menuImport"));
        menuOutput = new QMenu(menuStart);
        menuOutput->setObjectName(QStringLiteral("menuOutput"));
        menuTool = new QMenu(menuBar);
        menuTool->setObjectName(QStringLiteral("menuTool"));
        menuOptimization = new QMenu(menuTool);
        menuOptimization->setObjectName(QStringLiteral("menuOptimization"));
        menuDebug = new QMenu(menuBar);
        menuDebug->setObjectName(QStringLiteral("menuDebug"));
        FoldingClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(FoldingClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        FoldingClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(FoldingClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        FoldingClass->setStatusBar(statusBar);

        menuBar->addAction(menuStart->menuAction());
        menuBar->addAction(menuTool->menuAction());
        menuBar->addAction(menuDebug->menuAction());
        menuStart->addAction(menuImport->menuAction());
        menuStart->addAction(menuOutput->menuAction());
        menuImport->addAction(actionFolding_import);
        menuImport->addAction(actionObj_import);
        menuOutput->addAction(actionFolding_output);
        menuOutput->addAction(actionObj_output);
        menuTool->addAction(menuOptimization->menuAction());
        menuTool->addAction(actionInitialize);
        menuTool->addAction(actionSegmentation);
        menuOptimization->addAction(actionOutlines);
        menuOptimization->addAction(actionWhole);
        menuDebug->addAction(actionFoldingData);

        retranslateUi(FoldingClass);
        QObject::connect(actionFolding_import, SIGNAL(triggered(bool)), FoldingClass, SLOT(openFileDialogFold()));
        QObject::connect(actionObj_import, SIGNAL(triggered(bool)), FoldingClass, SLOT(openFileDialogObj()));
       // QObject::connect(tableWidget, SIGNAL(cellClicked(int,int)), FoldingClass, SLOT(openFileDialogObj()));

        QMetaObject::connectSlotsByName(FoldingClass);
    } // setupUi

    void retranslateUi(QMainWindow *FoldingClass)
    {
        FoldingClass->setWindowTitle(QApplication::translate("FoldingClass", "Folding", 0));
        actionFoldiding->setText(QApplication::translate("FoldingClass", "Folding", 0));
        actionOutlines->setText(QApplication::translate("FoldingClass", "outlines", 0));
        actionInitialize->setText(QApplication::translate("FoldingClass", "initialize", 0));
        actionSegmentation->setText(QApplication::translate("FoldingClass", "Segmentation", 0));
        actionFolding_output->setText(QApplication::translate("FoldingClass", "FoldingData", 0));
        actionObj_output->setText(QApplication::translate("FoldingClass", "Obj", 0));
        actionWhole->setText(QApplication::translate("FoldingClass", "whole", 0));
        actionFoldingData->setText(QApplication::translate("FoldingClass", "foldingData", 0));
        actionFolding_import->setText(QApplication::translate("FoldingClass", "Folding", 0));
        actionObj_import->setText(QApplication::translate("FoldingClass", "Obj", 0));
        label_inputFille2->setText(QApplication::translate("FoldingClass", "inputFileFold : ", 0));
        label_inputFile->setText(QApplication::translate("FoldingClass", "inputFileObj : ", 0));
        objFileName->setText(QString());
        foldFileName->setText(QString());
        menuStart->setTitle(QApplication::translate("FoldingClass", "file", 0));
        menuImport->setTitle(QApplication::translate("FoldingClass", "Import", 0));
        menuOutput->setTitle(QApplication::translate("FoldingClass", "Output", 0));
        menuTool->setTitle(QApplication::translate("FoldingClass", "tool", 0));
        menuOptimization->setTitle(QApplication::translate("FoldingClass", "Optimization", 0));
        menuDebug->setTitle(QApplication::translate("FoldingClass", "debug", 0));
    } // retranslateUi

};

namespace Ui {
    class FoldingClass: public Ui_FoldingClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FOLDING_H
