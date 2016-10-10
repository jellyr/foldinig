#ifndef MYGLWIDGET_H
#define MYGLWIDGET_H

#include <QGLWidget>
#include "cgal_defineData.h"
#include "foldingMethod.h"


class QGLClass : public QGLWidget
{
	Q_OBJECT
public:
	explicit  QGLClass(QWidget *parent = 0);
	~QGLClass();
signals:
private slots :
	void startAllProcess();

protected:
	void initializeGL();
	void paintGL();
	void resizeGL(int width, int height);

	QSize minimumSizeHint() const;
	QSize sizeHint() const;
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);
public: 

	COpenGL *fObj;
	Cmodel *cgalObj;
	void initFold();

	int vWidth; //	ビューポートの幅
	int vHeight;//	ビューポートのタカサ
	int zoomValue;//	ズーム量

private:
	void draw();

	int xRot;
	int yRot;
	int zRot;

	QPoint lastPos;
};

#endif // MYGLWIDGET_H