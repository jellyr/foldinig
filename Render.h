#ifndef MYGLWIDGET_H
#define MYGLWIDGET_H

#include <QOpenGLWidget>
#include "cgal_defineData.h"
#include "foldingMethod.h"


class QGLClass : public QOpenGLWidget
{
	Q_OBJECT
public:
	explicit  QGLClass(QWidget *parent = 0) : QOpenGLWidget(parent) { };
	~QGLClass();
signals:
private slots :
	void startAllProcess();
	void initFold(int p1, int p2);
	void addData();

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