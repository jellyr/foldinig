#include "folding.h"
#include <QtWidgets/QApplication>
#include <iostream>

int main(int argc, char *argv[])
{
	std::cout << "Qt start!!";
	QApplication a(argc, argv);
	QGLClass *w = new QGLClass();
	w->resize(300, 300);
	w->show();
	w->initFold();
	return a.exec();
}
