#include "folding.h"
#include <QtWidgets/QApplication>
#include <iostream>


int main(int argc, char *argv[])
{
	cout << "aaaa\n";
	QApplication a(argc, argv);
	Folding window;
	window.show();
	
	return a.exec();
}
