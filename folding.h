#ifndef FOLDING_H
#define FOLDING_H

#include <QtWidgets/QMainWindow>
#include "ui_folding.h"
#include "Render.h"


namespace Ui {
	class Folding;
}

class Folding : public QMainWindow
{
	Q_OBJECT

public:
	explicit Folding(QWidget *parent = 0);
	~Folding();

private:
	Ui::FoldingClass ui;
};

#endif // FOLDING_H
