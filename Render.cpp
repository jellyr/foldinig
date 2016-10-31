#include <QtWidgets>
#include <QtOpenGL>
#include <iostream>

#include "Render.h"
#include "InputFile/InputFile.h"
#include "optimization.h"
using namespace std;

const GLfloat lightDiffuse[] = { 1, 1, 1, 1 };
const GLfloat materialDiffuseFront[] = { 0, 1.0, 0, 1 };
const GLfloat materialDiffuseBack[] = { 1.0, 1.0, 0, 1 };
const GLfloat lightPosition[] = { 10, 10, 10, 0 };

void QGLClass::initFold() {
	fObj = new COpenGL();
	cgalObj = new Cmodel();
	cgalObj->inputM = fObj->readOBJ("bunny\\head.obj", true);//	���ʂ̃��b�V���f�[�^
	cgalObj->inputM->reducepolygon(500);
	cgalObj->cgalPoly = holeFillAndConvertPolyG(cgalObj->inputM);//	�����ӂ�����polyhedron�֕ϊ�
	cgalObj->cgalPoly_Nef = convert_Poly_NefPoly((*cgalObj->cgalPoly));//	polyhedron��Nef_polyhedron�֕ϊ�

	cgalObj->foldM = InputData();//	�Z�p�`�̐܂肽���݃��f�������
	//fObj->optimization(cgalObj->foldM);//	�œK��
	fObj->Trim(cgalObj->foldM);//	�g��������
	cout << "foldingGap(Model *foldM): " << foldingGap(cgalObj->foldM) << "\n";
	fObj->convertFoldingToMesh(cgalObj->foldM);//	�܂肽���݃��f�������b�V���f�[�^�ɕϊ�
	//cgalObj->foldPoly = inputPoly_Gnew(cgalObj->foldM);//	�܂肽���݃��f����cga��Polyhedron�ɕϊ�
	
	convertPolyToModel(cgalObj->inputM);
	//�V���ʂ�����
	fObj->Quickhull(convertTo2D(cgalObj->inputM), cgalObj->inputM);
	reductionTopPolygon(cgalObj->inputM);
	
	//���ʂ�����
	setCluster(cgalObj->inputM);
	fObj->Trim(cgalObj->inputM);
	fObj->convertFoldingToMesh(cgalObj->inputM);
	cgalObj->foldM = cgalObj->inputM;

	//�܂肽���݉\�ɂ���
	fObj->optimization(cgalObj->foldM);
	
	//�K���̌v�Z�����܂�
	cgalObj->foldPoly = inputPoly_Gnew(cgalObj->foldM);
	cgalObj->metroPrepar();

//	(*cgalObj->foldPoly) = Optimization(cgalObj, fObj);

}

void QGLClass::startAllProcess() {
	
}

QGLClass::QGLClass(QWidget *parent)
	: QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
	xRot = 0;
	yRot = 0;
	zRot = 0;
	zoomValue = 100;
}

QGLClass::~QGLClass()
{
	initFold();//	���f������́A�����ӂ���
}

QSize QGLClass::minimumSizeHint() const
{
	return QSize(50, 50);
}

QSize QGLClass::sizeHint() const
{
	return QSize(400, 400);
}

static void qNormalizeAngle(int &angle)
{
	while (angle < 0)
		angle += 360 * 16;
	while (angle > 360)
		angle -= 360 * 16;
}

void QGLClass::initializeGL()
{
	qglClearColor(Qt::white);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glShadeModel(GL_SMOOTH);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
}

void QGLClass::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity(); // reset transformation state
	glTranslatef(0.0, 0.0, -1.0);
	glScalef(zoomValue, zoomValue, zoomValue);
	glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
	glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
	glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);
	draw();
}

void QGLClass::resizeGL(int width, int height)
{
	int side = qMin(width, height);
	glViewport((width - side) / 2, (height - side) / 2, side, side);

	vWidth = width;
	vHeight = height;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
#ifdef QT_OPENGL_ES_1
	glOrthof(-2, +2, -2, +2, 1.0, 15.0);
#else
	glOrtho(-vWidth, +vWidth, -vHeight, +vHeight, -1000.0, 1000.0);
#endif
	glMatrixMode(GL_MODELVIEW);
}

void QGLClass::mousePressEvent(QMouseEvent *event)
{
	lastPos = event->pos();
}

void QGLClass::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - lastPos.x();
	int dy = event->y() - lastPos.y();
	
	if (event->buttons() & Qt::LeftButton) {
		xRot = xRot + 8 * dy;
		yRot = yRot + 8 * dx;
	}
	else if (event->buttons() & Qt::RightButton) {
		xRot = xRot + 8 * dy;
		zRot = zRot + 8 * dx;
	}

	lastPos = event->pos();
	updateGL();//	�����Ȃ���update����Ȃ�
	paintGL();
}

void QGLClass::wheelEvent(QWheelEvent *event) {
	zoomValue += (event->angleDelta().y() / 300);
	updateGL();//	�����Ȃ���update����Ȃ�
	paintGL();
}

void QGLClass::draw()
{
	//	qglColor(Qt::red);
	renderFoldModel(cgalObj->inputM);
	//rendercgalPoly(cgalObj->foldPoly);
	rendercgalPoly(cgalObj->cgalPoly);
	renderModelCluster(cgalObj->inputM);
}