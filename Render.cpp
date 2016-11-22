#include <QtWidgets>
#include <QtOpenGL>
#include <QGLWidget>
#include <iostream>

#include "Render.h"
#include "InputFile/InputFile.h"
#include "optimization.h"
using namespace std;

const GLfloat lightDiffuse[] = { 1, 1, 1, 1 };
const GLfloat materialDiffuseFront[] = { 0, 1.0, 0, 1 };
const GLfloat materialDiffuseBack[] = { 1.0, 1.0, 0, 1 };
const GLfloat lightPosition[] = { 10, 10, 10, 0 };
Model *bunny = new Model();
Polyhedron_G *afterOpt = new Polyhedron_G();
Polyhedron_G *pol = new Polyhedron_G();

void QGLClass::inputata() {

}

void QGLClass::initFold(int p1, int p2) {
	fObj = new COpenGL();
	cgalObj = new Cmodel();
	cgalObj->inputM = fObj->readOBJ("bunny\\body_fixed.obj", true);//	普通のメッシュデータ
	autoScaling(cgalObj->inputM);
	if (cgalObj->inputM->faces.size() > 500) {
		cgalObj->inputM->reducepolygon(500);
	}

	cgalObj->cgalPoly = holeFillAndConvertPolyG(cgalObj->inputM);//	穴をふさいでpolyhedronへ変換
	cgalObj->cgalPoly_Nef = convert_Poly_NefPoly((*cgalObj->cgalPoly));//	polyhedron→Nef_polyhedronへ変換

	cgalObj->foldM = InputData();//	六角形の折りたたみモデルを入力
	fObj->optimization(cgalObj->foldM);//	最適化
	fObj->Trim(cgalObj->foldM);//	トリム処理
	cout << "foldingGap(Model *foldM): " << foldingGapDisp(cgalObj->foldM) << "\n";
	fObj->convertFoldingToMesh(cgalObj->foldM);//	折りたたみモデルをメッシュデータに変換
	cgalObj->foldPoly = inputPoly_Gnew(cgalObj->foldM);//	折りたたみモデルをcgaのPolyhedronに変換
	pol = inputPoly_Gnew(cgalObj->foldM);
	convertPolyToModel(cgalObj->inputM);
	(*bunny) = (*cgalObj->inputM);
	cgalObj->bunny = bunny;
	//天頂面をつくる
	fObj->Quickhull(convertTo2D(cgalObj->inputM), cgalObj->inputM);
	reductionTopPolygon(cgalObj->inputM);
	pseudoVolumeDiff(cgalObj->inputM, cgalObj->foldM);
	//側面をつくる
	Model *m = setCluster(cgalObj->inputM);
	NcurveFitting(cgalObj->inputM);

	m->fold = cgalObj->inputM->fold;
	cout << "Trim??\n";
	fObj->Trim(m);
	fObj->convertFoldingToMesh(m);
	cout << "Trim??\n";
	
	cgalObj->foldM = m;
	
	cgalObj->foldPoly = inputPoly_Gnew(cgalObj->foldM);
	cout << "Trim??\n";
	//double volumeDiffFirst = calculateDiff((*cgalObj->foldPoly), cgalObj->cgalPoly_Nef, cgalObj->cgalPoly);
	//最適化
	//double firstFoldGap = foldingGapDisp(cgalObj->foldM);
	//cout << " before \n";
	//outputFolding(cgalObj->foldM);
	//cout << "firstFap: " << firstFoldGap << "\n";
	//cout << "foldingGap(Model *foldM): " << foldingGapDisp(cgalObj->foldM) << "\n";
	//clock_t start = clock();
	//(*afterOpt) = Optimization(cgalObj, fObj);
	//clock_t end = clock();
	//std::cout << "duration = " << (double)(end - start) / CLOCKS_PER_SEC << "sec.\n";
	//cout << "foldingGap(Model *foldM): " << "fisst " << firstFoldGap << "afterfoldgap: " << foldingGapDisp(cgalObj->foldM) << "\n";
	//fObj->optimization(cgalObj->foldM);
	//fObj->convertFoldingToMesh(cgalObj->foldM);
	//fObj->Trim(cgalObj->foldM);//	トリム処理
	//cgalObj->foldPoly = inputPoly_Gnew(cgalObj->foldM);
	//pseudoVolumeDiff(cgalObj->inputM, cgalObj->foldM);
	//cout << "optimized folding gap: " << foldingGapDisp(cgalObj->foldM) << "\n";
	//double afterVolumeDiff = calculateDiff((*cgalObj->foldPoly), cgalObj->cgalPoly_Nef, cgalObj->cgalPoly);
	//cout << "volumeDiff: first " << volumeDiffFirst << "after: " << afterVolumeDiff << "\n";
	//pseudoVolumeDiff(bunny, cgalObj->foldM);
	//fObj->outputObj(cgalObj->foldM);
	cout << " after \n";
	outputFolding_(cgalObj->foldM);
//	(*cgalObj->foldPoly) = Optimization(cgalObj, fObj);

}

void QGLClass::startAllProcess() {

}

QGLClass::~QGLClass()
{
//	initFold();//	モデルを入力、穴をふさぐ
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
	//qglClearColor(Qt::white);

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
	//updateGL();//	書かないとupdateされない
	paintGL();
}

void QGLClass::wheelEvent(QWheelEvent *event) {
	zoomValue += (event->angleDelta().y() / 300);
	//updateGL();//	書かないとupdateされない
	paintGL();
}

void QGLClass::draw()
{
	//	qglColor(Qt::red);
	//renderFoldModel(cgalObj->foldM);
	//renderFoldModel(bunny);
	//renderPsuedV(cgalObj->inputM);
	//renderPsuedV(bunny);
	//renderFoldModel(cgalObj->inputM);
	//rendercgalPoly(pol);
	//rendercgalPoly(afterOpt);
	//rendercgalPoly(cgalObj->foldPoly);
//	renderModelCluster(bunny);
}