#include "folding.h"
#include "ui_folding.h"
#include <QFileDialog>



Folding::Folding(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this); 
	QGLClass *gLwidget = new QGLClass();
	ui.verticalLayout->addWidget(gLwidget);
	QObject::connect(ui.tableWidget, SIGNAL(cellHa(i, int)), ui.tableWidget, SLOT(removeCell(int,int)));
	connect(ui.actionFolding_import, SIGNAL(triggered(bool)), ui.tableWidget, SLOT(openFileDialogFold()));
}

Folding::~Folding()
{
	
}

void Folding::openFileDialogFold() {//	折りたたみファイルを読み込む
	QString foldfile = QFileDialog::getOpenFileName(this,
		tr("Open Folding Data"), "C:/Users/miyato/Desktop/Folding/Folding", tr("Image Files (*.txt)"));
	QStringList stList = foldfile.split("/");
	ui.foldFileName->setText(stList[stList.length() - 1]);
	std::string filePath = stList[stList.length() - 1].toLocal8Bit().constData();
	std::string fileName = foldfile.toUtf8().constData();
	//折りたたみデータを変換
	foldmethod f = InputFold(fileName);
	//テーブルにデータを表示する
	int num = f.outlinepoints.size();
	for (int i = 0; i < f.outlinepoints.size(); i++) {
		num += f.outlinepoints[i]->points.size();
	}
	int rowNum = std::max((int)(f.pointPosition.size() + f.betweenPosition.size()), num);
	ui.tableWidget->setRowCount(rowNum);
	ui.tableWidget->setColumnCount(2);
	ui.tableWidget->setItem(0, 0, new QTableWidgetItem("topPosY"));
	ui.tableWidget->setItem(1, 0, new QTableWidgetItem("pointsPosition"));
	ui.tableWidget->setItem(1, 1, new QTableWidgetItem("outlinePoints"));
	ui.tableWidget->setItem(f.pointPosition.size() + 2, 0, new QTableWidgetItem("outlinePoints"));

	ui.tableWidget->setItem(0, 1, new QTableWidgetItem(QString::number(f.topPosY)));

	for (int i = 0; i < f.pointPosition.size(); i++) {
		double x, y;
		x = f.pointPosition[i].x;
		y = f.pointPosition[i].y;
		QString st = "(" + QString::number(x) + "," + QString::number(y) + ")";
		QTableWidgetItem *newItem = new QTableWidgetItem(st);
		ui.tableWidget->setItem(2 + i, 0,  newItem);
	}

	for (int i = 0; i < f.betweenPosition.size(); i++) {
		double x, y;
		x = f.betweenPosition[i].x;
		y = f.betweenPosition[i].y;
		QString st = "(" + QString::number(x) + "," + QString::number(y) + ")";
		QTableWidgetItem *newItem = new QTableWidgetItem(st);
		ui.tableWidget->setItem(3 + i + f.pointPosition.size(), 0, newItem);
	}
	int rowCount = 2;

	for (int i = 0; i < f.outlinepoints.size(); i++) {
		QString st = "line " + QString::number(i);
		QTableWidgetItem *newItem = new QTableWidgetItem(st);
		ui.tableWidget->setItem(rowCount++, 1, newItem);
		for (int j = 0; j < f.outlinepoints[i]->points.size(); j++) {
			double x, y;
			x = f.outlinepoints[i]->points[j].x;
			y = f.outlinepoints[i]->points[j].y;
			QString st_ = "(" + QString::number(x) + "," + QString::number(y) + ")";
			QTableWidgetItem *newItem_ = new QTableWidgetItem(st_);
			ui.tableWidget->setItem(rowCount++, 1, newItem_);
		}
	}
	//--------------------------------
	//     topPosY    |               (0)
	//--------------------------------
	// pointsPosition | outlinePoints (1)
	//--------------------------------
	//                |               (2)
	//--------------------------------
	//  betwennPoints |               
	//--------------------------------
}

void Folding::openFileDialogObj() {//	Obj形式のファイルを読み込む
	QString objfile = QFileDialog::getOpenFileName(this,
		tr("Open Obj"), "C:/Users/miyato/Desktop/Folding/Folding", tr("Image Files (*.obj)"));
	QStringList stList = objfile.split("/");
	ui.objFileName->setText(stList[stList.length() - 1]);


}


