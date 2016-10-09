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
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QOpenGLWidget>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_FoldingClass
{
public:
    QAction *actionStart;
    QWidget *centralWidget;
    QOpenGLWidget *QGLClass;
    QPushButton *pushButton;
    QMenuBar *menuBar;
    QMenu *menuStart;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *FoldingClass)
    {
        if (FoldingClass->objectName().isEmpty())
            FoldingClass->setObjectName(QStringLiteral("FoldingClass"));
        FoldingClass->resize(600, 400);
        actionStart = new QAction(FoldingClass);
        actionStart->setObjectName(QStringLiteral("actionStart"));
        centralWidget = new QWidget(FoldingClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        QGLClass = new QOpenGLWidget(centralWidget);
        QGLClass->setObjectName(QStringLiteral("QGLClass"));
        QGLClass->setGeometry(QRect(-20, 0, 601, 341));
        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(0, 0, 75, 23));
        FoldingClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(FoldingClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 600, 24));
        menuStart = new QMenu(menuBar);
        menuStart->setObjectName(QStringLiteral("menuStart"));
        FoldingClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(FoldingClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        FoldingClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(FoldingClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        FoldingClass->setStatusBar(statusBar);

        menuBar->addAction(menuStart->menuAction());
        menuStart->addAction(actionStart);

        retranslateUi(FoldingClass);

        QMetaObject::connectSlotsByName(FoldingClass);
    } // setupUi

    void retranslateUi(QMainWindow *FoldingClass)
    {
        FoldingClass->setWindowTitle(QApplication::translate("FoldingClass", "Folding", 0));
        actionStart->setText(QApplication::translate("FoldingClass", "start", 0));
        pushButton->setText(QApplication::translate("FoldingClass", "start", 0));
        menuStart->setTitle(QApplication::translate("FoldingClass", "file", 0));
    } // retranslateUi

};

namespace Ui {
    class FoldingClass: public Ui_FoldingClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FOLDING_H
