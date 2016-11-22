#include "optimization.h"

#ifndef _INCLUDE_OPTIMIZATION_
#define _INCLUDE_OPTIMIZATION_
#include "Optimization/trim/Trimfunc.h"
#include "Optimization/SteepDescent.h"
#include "Optimization/Animation/Animation.h"
#include "foldingMethod.h"
#include "Triangulation_cliping.h"
#endif

using namespace std;
using namespace Eigen;

void setJacobian(Eigen::MatrixXd &jacobian, Eigen::VectorXd setP, Eigen::VectorXd fx_tmp, int constraintNum, int count, double invDelta) {
	for (int i = 0; i < constraintNum; i++) {
		/*if (invDelta * setP(i) + fx_tmp(i) < 1.0e-10) {
			jacobian(count, i) = 0;
			continue;
		}*/
		/////cout << "set: " << invDelta * setP(i) << "," << fx_tmp(i) << "\n";
		jacobian(count, i) = invDelta * setP(i) + fx_tmp(i);
	}
}
//	数値微分
Eigen::MatrixXd ComputeNumericalJacobian(Cmodel *cm, COpenGL *fObj, int variableNum, int constraintNum) {
	Model *foldM = cm->foldM;
	Eigen::MatrixXd jacobian(variableNum, constraintNum);
	double DELTA = 1.0e-7;
	double invDelta = 1.0 / DELTA;
	Eigen::VectorXd fx_tmp = -invDelta * eachPenalty(cm, fObj);
	int pointLastNum = foldM->fold->pointPosition.size() - 1;
	int count = 0;
	//yの高さ
	foldM->fold->topPosY += DELTA;
	setJacobian(jacobian, eachPenalty(cm, fObj), fx_tmp, constraintNum, count, invDelta);
	//cout << count << ": " << jacobian(count) << "\n";
	foldM->fold->topPosY -= DELTA;
	count++;

	for (int i = 0; i < foldM->fold->pointPosition.size() - 1; i++) {
		//cout << "i in outlineP: " << i << "\n";
		for (int j = 0; j < foldM->fold->outlinepoints[i]->points.size(); j++) {
			if (j == 0) {//	pointsPositionのx,yを変更
				foldM->fold->pointPosition[i].x += DELTA;
				if (i == 0) foldM->fold->pointPosition[pointLastNum].x += DELTA;
				updateBetweenPos(foldM);
				setJacobian(jacobian, eachPenalty(cm, fObj), fx_tmp, constraintNum, count, invDelta);
				//cout << i << "," << j << "x: " << jacobian(count) << "\n";
				count++;
				foldM->fold->pointPosition[i].x -= DELTA;
				if (i == 0) foldM->fold->pointPosition[pointLastNum].x -= DELTA;

				foldM->fold->pointPosition[i].y += DELTA;
				if (i == 0) foldM->fold->pointPosition[pointLastNum].y += DELTA;
				updateBetweenPos(foldM);
				setJacobian(jacobian, eachPenalty(cm, fObj), fx_tmp, constraintNum, count, invDelta);
				//cout << i << "," << j << "y: " << jacobian(count) << "\n";
				count++;
				foldM->fold->pointPosition[i].y -= DELTA;
				if (i == 0) foldM->fold->pointPosition[pointLastNum].y -= DELTA;
				updateBetweenPos(foldM);
				continue;
			}
			foldM->fold->outlinepoints[i]->points[j].x += DELTA;
			setJacobian(jacobian, eachPenalty(cm, fObj), fx_tmp, constraintNum, count, invDelta);
			count++;
			foldM->fold->outlinepoints[i]->points[j].x -= DELTA;
			if (j != foldM->fold->outlinepoints[i]->points.size() - 1) {
				foldM->fold->outlinepoints[i]->points[j].y += DELTA;
				setJacobian(jacobian, eachPenalty(cm, fObj), fx_tmp, constraintNum, count, invDelta);
				//cout << i << "," << j << "y: " << jacobian(count) << "\n";
				count++;
				foldM->fold->outlinepoints[i]->points[j].y -= DELTA;
			}
		}
		if (i == foldM->fold->pointPosition.size() - 2) {//最後にoutlinePointsのy座標全てを変換
			for (int j = 0; j < foldM->fold->pointPosition.size() - 1; j++) {
				int outlinepointsLastNum = foldM->fold->outlinepoints[j]->points.size() - 1;
				foldM->fold->outlinepoints[j]->points[outlinepointsLastNum].y += DELTA;
			}
			setJacobian(jacobian, eachPenalty(cm, fObj), fx_tmp, constraintNum, count, invDelta);
			//cout << count << ": " << jacobian(count) << "\n";
			//cout << i << "," << "y: " << jacobian(count) << "\n";
			for (int j = 0; j < foldM->fold->pointPosition.size() - 1; j++) {
				int outlinepointsLastNum = foldM->fold->outlinepoints[j]->points.size() - 1;
				foldM->fold->outlinepoints[j]->points[outlinepointsLastNum].y -= DELTA;
			}
		}
	}
	cout << "\n";
	//cout << "count in jacobian calculation: " << count << "\n";
	return jacobian;
}
Eigen::MatrixXf ComputeNumericalJacobian(std::vector<float> u, float *x1, float *x2, int num) {

	Eigen::MatrixXf jacobian(u.size(), 21);
	float DELTA = 1.0e-7;
	float invDelta = 1.0 / DELTA;

	for (int i = 0; i < 21; i++) {
		int count = 0;
		float fx_tmp = (-invDelta * ((u[0] * pow((x1[i] - u[1]), 2) - x2[i])));

		u[0] += DELTA;
		jacobian(count, i) = invDelta * ((u[0] * pow((x1[i] - u[1]), 2) - x2[i])) + fx_tmp;
		u[0] -= DELTA;

		count++;

		u[1] += DELTA;
		jacobian(count, i) = invDelta *((u[0] * pow((x1[i] - u[1]), 2) - x2[i])) + fx_tmp;
		u[1] -= DELTA;
	}

	return jacobian;
}
void Step(double *output, double gap, std::vector<Vec2> points, int size, Vec2 *pointsN, bool One){

	double *lIdeal = new double[size];

	double *x1 = new double[(int)points.size()];
	double *y1 = new double[(int)points.size()];
	double *x2 = new double[(int)points.size()];
	double *y2 = new double[(int)points.size()];


	for (int i = 0; i<(int)points.size(); i++){
		x1[i] = points[i].x;
		x2[i] = x1[i];
		y1[i] = points[i].y;
		y2[i] = y1[i];
		if (size > i){
			lIdeal[i] = fabs(output[i]);
		}
	}

	method(x1, y1, x2, y2, lIdeal, (int)points.size());
	
	for (int i = 0; i<(int)points.size(); i++){
		pointsN[i] = Vec2(x1[i], y1[i]);
	}
	delete[] x1;
	delete[] x2;
	delete[] y1;
	delete[] y2;

}

void outputFolding(Model *foldM){
	cout << "\ntopYpos: " << foldM->fold->topPosY << "\n";

	for (int i = 0; i < foldM->fold->outlinepoints.size(); i++){
		cout << "outlinePoint " << i << "\n";
		for (int j = 0; j < foldM->fold->outlinepoints[i]->points.size(); j++){
			cout << foldM->fold->outlinepoints[i]->points[j].x << " " << foldM->fold->outlinepoints[i]->points[j].y << "\n";
		}
	}
	cout << "pointPosition\n";
	for (int i = 0; i < foldM->fold->pointPosition.size(); i++){
		cout << foldM->fold->pointPosition[i].x << " " << foldM->fold->pointPosition[i].y << "\n";
	}

	updateBetweenPos(foldM);
	cout << "betweenPosition\n";
	for (int i = 0; i < foldM->fold->betweenPosition.size(); i++) {
		cout << foldM->fold->betweenPosition[i].x << " " << foldM->fold->betweenPosition[i].y << "\n";
	}
}

double distancepointlineInOpt(Vec2 P, Vec2 A, Vec2 B){//点PからABへの最短距離
	Vec2 ABnor = B - A; ABnor.normalize();
	double Ax = ABnor * (P - A);
	Vec2 x = A + (ABnor*Ax);

	return (P - x).length();
}

bool TriangleIntersectInOpt(Vec3 Orig, Vec3 dir, Faces *face, float *pRetT)
{
	Vec3 v0 = face->halfedge->vertex->p;
	Vec3 v1 = face->halfedge->next->vertex->p;
	Vec3 v2 = face->halfedge->prev->vertex->p;
	Vec3 e1, e2, pvec, tvec, qvec;
	float det;
	float t, u, v;
	float inv_det;

	e1 = v1 - v0;
	e2 = v2 - v0;

	pvec = dir % e2;
	det = e1 * pvec;
	if (det > (1e-3)) {

		tvec = Orig - v0;
		u = tvec * pvec;
		if (u < 0.0f || u > det) return false;

		qvec = tvec % e1;
		v = dir * qvec;
		if (v < 0.0 || u + v > det) return false;
	}
	else if (det < -(1e-3)) {
		tvec = Orig - v0;
		u = tvec * pvec;
		if (u > 0.0 || u < det) return false;

		qvec = tvec % e1;
		v = dir * qvec;
		if (v > 0.0 || u + v < det) return false;

	}
	else {
		return false;
	}

	inv_det = 1.0f / det;

	t = e2 * qvec;
	t *= inv_det;
	u *= inv_det;
	v *= inv_det;

	if (pRetT) *pRetT = t;

	return true;    //hit!!
}

double psuedoVolumeDiffInOpt(Model *m, Model *bunny) {
	std::list<Vertexs*>::iterator it_v;
	std::list<Faces*>::iterator it_f;
	std::list<Vertexs*>::iterator it_v2;
	double allLength = 0;
	double max = 0;

	/*for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		double minDis = 10000000;
		Vertexs *P;
		for (it_v2 = bunny->vertices.begin(); it_v2 != bunny->vertices.end(); it_v2++) {
			double len = ((*it_v)->p - (*it_v2)->p).length();
			if (minDis > len) {
				minDis = len;
				P = (*it_v2);
			}
		}
		(*it_v)->minDis = minDis;
		(*it_v)->dir = P->p;
		if (((*it_v)->p - P->p) * (*it_v)->normal > 0) {
			(*it_v)->minDis *= -1;
		}
		if (max < minDis) {
			max = minDis;
		}
		allLength += minDis;
	}*/

	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {

		//頂点の法線を設定する

		Halfedge  *h1 = (*it_v)->halfedge;
		int count = 0;
		Vec3 verNormal; verNormal.set(0, 0, 0);
		do {
			verNormal += h1->face->normal;
			count++;
			h1 = h1->prev->pair;
		} while (h1 != (*it_v)->halfedge);

		(*it_v)->normal = verNormal / (double)count;
		//cout << (*it_v)->num << ", " << (*it_v)->normal.x << " ";
		//面を持ってくる

		Vec3 verNormal2d; verNormal2d.set((*it_v)->normal.x, (*it_v)->normal.y, (*it_v)->normal.z);
		verNormal2d.normalize();
		//verNormal2d = verNormal2d * 100;//頂点のx-z方向の方向ベクトル
		double distance = 1000000;
		Vec3 nor; nor.set(0, 0, 0);

		for (it_f = bunny->faces.begin(); it_f != bunny->faces.end(); it_f++) {
			//面のあたり判定
			float lengthToFace1 = -10000;
			float lengthToFace2 = -10000;
			Vec3 nor1, nor2;
			if (TriangleIntersectInOpt((*it_v)->p, verNormal2d, (*it_f), &lengthToFace1)) {//ベクトルの法線が外側を向いている
				if (verNormal2d * (*it_f)->normal > 0) {//法線ベクトル調査
					/*if (distance > lengthToFace1) {
					distance = lengthToFace;
					nor = verNormal2d;
					}*/
					nor1 = verNormal2d;
				}
			}
			if (TriangleIntersectInOpt((*it_v)->p, -verNormal2d, (*it_f), &lengthToFace2)) {
				if (-verNormal2d * (*it_f)->normal < 0) {//法線ベクトル調査
					/*if (distance > lengthToFace2) {
					distance = lengthToFace;
					nor = -verNormal2d;
					}*/
					nor2 = -verNormal2d;
				}
			}
			if (abs(lengthToFace1) < abs(lengthToFace2)) {
				if (distance > abs(lengthToFace1)) {
					distance = abs(lengthToFace1);
					nor = nor1;
				}
			}
			else{
				if (distance > abs(lengthToFace2)) {
					distance = abs(lengthToFace2);
					nor = nor2;
				}
			}
		}

		if (distance == 1000000) {
			distance = 0;
		}
		else{
			//cout << "dis: " << distance << " ";
		}
		allLength += abs(distance);
		//cout << (nor*distance).x << ". " << (nor*distance).y << "\n";
		//pseudoVolumeDi.push_back(nor*distance);
	}

	return allLength;
}

/*void determineClustNumForVertex(Model *m) {
	std::list<Vertexs*>::iterator it_v;

	Vec2 centroid; centroid.set(0, 0);

	for (int i = 0; i < m->fold->pointPosition.size() - 1; i++) {
		centroid = centroid + m->fold->pointPosition[i];
	}

	centroid = centroid / (double)(m->fold->pointPosition.size() - 1.0);

	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		for (int i = 0; i < m->fold->pointPosition.size() - 1; i++) {
			Vec2 vertex2D((*it_v)->p);	vertex2D = vertex2D - centroid; vertex2D.normalize();
			Vec2 piTocent; piTocent = m->fold->pointPosition[i] - centroid; piTocent.normalize();
			Vec2 pi_1Tocent; pi_1Tocent = m->fold->pointPosition[i + 1] - centroid; pi_1Tocent.normalize();
			
			if (acos(vertex2D * piTocent) < acos(piTocent * pi_1Tocent) && acos(vertex2D * pi_1Tocent) < acos(piTocent * pi_1Tocent)) {
				//割り当て
				(*it_v)->clusterNum = i;
				break;
			}
		}
	}
}*/

double penalty(Cmodel *cm, COpenGL *fObj) {
	Model *foldM = cm->foldM;
	//Polyhedron_G *inputP = cm->cgalPoly;
	//Nef_polyhedron_3 inputP_ = cm->cgalPoly_Nef;
	////	最初に三次元座標へと変換
	for (int i = 0; i < foldM->fold->outlinepoints.size(); i++) {
		double Ypos = 0;
		for (int j = 1; j < foldM->fold->outlinepoints[i]->points.size(); j++) {
			if (foldM->fold->outlinepoints[i]->points[j].y < Ypos) {
				//cout << "return 100000";
				return 1000000;
			}
			else {
				Ypos = foldM->fold->outlinepoints[i]->points[j].y;
			}
		}
	}
	fObj->Trim(foldM);
	fObj->convertFoldingToMesh(foldM);//	メッシュデータへと変換
	//Polyhedron_G P = inputPoly_G(foldM);//	Polyhedron_Gへと変換utpu
	//outputFolding(cm->foldM);
	double Diff = psuedoVolumeDiffInOpt(cm->inputM, cm->foldM);
	//cout << "diff: " << Diff << "\n";
	//double Diff = metro(foldM, cm->inputC, cm->foldC);//	metroへ変換
	/*if (Diff == 0) {
		for (int i = 0; i < foldM->fold->pointPosition.size(); i++){
		//cout << "pointsPos: " << foldM->fold->pointPosition[i].x << "," << foldM->fold->pointPosition[i].y << "\n";
		}
		//cout << "\n";
		for (int i = 0; i < foldM->fold->pointPosition.size() - 1; i++){
		//cout << "outline: " << i << "\n";
		//cout << "outline size: " << foldM->fold->outlinepoints.size() << "\n";
		//cout << "points size: " << foldM->fold->outlinepoints[i]->points.size() << "\n";
		for (int j = 0; j < foldM->fold->outlinepoints[i]->points.size(); j++){
		//cout << "outliePos" << foldM->fold->outlinepoints[i]->points[j].x << "," << foldM->fold->outlinepoints[i]->points[j].y << "\n";
		}
		//cout << "\n";
		}
		return 1000000;
		}*/
	if (Diff == -1) {
		Diff = 0;
	}
	return Diff;
}

float penalty(std::vector<float> u, float *x1, float *x2, int num) {

	double sum = 0;

	for (int i = 0; i < num; i++) {
		sum += pow((u[0] * pow((x1[i] - u[1]), 2) - x2[i]), 2);
	}
	return sum;
}
float penalty_(std::vector<float> u, float *x1, float *x2, int num) {

	float sum = 0;

	for (int i = 0; i < num; i++) {
		sum += (u[0] * pow((x1[i] - u[1]), 2) - x2[i]);
	}
	//cout << "sum: " << sum << "\n";
	return sum;
}

Eigen::VectorXd eachPenalty(Cmodel *cm, COpenGL *fObj) {
	
	Model *foldM = cm->foldM;

	int constraintsNum = 4;
	Eigen::VectorXd penaltyValue(constraintsNum);
	//	outputFolding(foldM);
	penaltyValue(0) = 0.5*penalty(cm, fObj);
	penaltyValue(1) = topConvex_area(foldM);
	//penaltyValue(2) = topSmoothing(foldM);
	penaltyValue(2) = 10.0*foldingGap(foldM);
	penaltyValue(3) = 0;
	//cout << "penaltyValue: \n" << penaltyValue << "\n";
	return penaltyValue;
}

Eigen::MatrixXd computeLambda(Eigen::MatrixXd A, double lambda, int numOfA){
	for (int i = 0; i < numOfA; i++) {
		A(i, i) = /*A(i, i) +*/ A(i, i) * lambda;
	}
	return A;
}
Eigen::MatrixXf computeLambda(Eigen::MatrixXf A, double lambda, int numOfA){
	for (int i = 0; i < numOfA; i++) {
		A(i, i) = A(i, i) + A(i, i) * lambda;
	}
	return A;
}

Eigen::MatrixXd computeLambdaUnit(Eigen::MatrixXd A, double lambda, int numOfA){
	for (int i = 0; i < numOfA; i++) {
		A(i, i) = A(i, i) + lambda;
		//cout << A(i, i) << "\n";
	}
	return A;
}
Eigen::MatrixXf computeLambdaUnit(Eigen::MatrixXf A, double lambda, int numOfA){
	for (int i = 0; i < numOfA; i++) {
		A(i, i) = A(i, i) + lambda;
		//cout << A(i, i) << "\n";
	}
	return A;
}

Vec2 intersection_l(Vec2 a1, Vec2 a2, Vec2 b1, Vec2 b2) {// 直線との交点計算
	Vec2 a = a2 - a1; Vec2 b = b2 - b1;
	return a1 + a * (b % (b1 - a1)) / (b % a);
}

double topConvex(Model *foldM) {//	凹形状の場合に値が大きくなる関数 長さバージョン
	std::vector<Vec2> pointPosition = foldM->fold->pointPosition;
	Vec2 center; center.set(0, 0);
	for (int i = 0; i < pointPosition.size() - 1; i++) {
		center += pointPosition[i];
	}
	center = center / ((double)pointPosition.size() - 1);

	double penaltySum = 0;
	int pointLastNum = pointPosition.size() - 2;
	for (int i = 0; i < pointPosition.size() - 1; i++) {
		Vec2 crossP;
		if (i == 0){
			crossP = intersection_l(pointPosition[i + 1], pointPosition[pointLastNum], center, pointPosition[i]);//線との垂線との交点
		}
		else {
			crossP = intersection_l(pointPosition[i + 1], pointPosition[i - 1], center, pointPosition[i]);//線との垂線との交点
		}

		double length1 = (crossP - center).length();
		double length2 = (pointPosition[i] - center).length();
		penaltySum += pow(10, (length1 / length2));
	}
	return penaltySum;
}
double topConvex_area(Model *foldM) {//	凹形状の場合に値が大きくなる関数 面積バージョン
	std::vector<Vec2> pointPosition = foldM->fold->pointPosition;
	Vec2 center; center.set(0, 0);
	for (int i = 0; i < pointPosition.size() - 1; i++) {
		center += pointPosition[i];
	}
	center = center / ((double)pointPosition.size() - 1);

	double penaltySum = 0;
	double AreaSum = 0;
	int pointLastNum = pointPosition.size() - 2;
	for (int i = 0; i < pointPosition.size() - 1; i++) {
		Vec2 P1 = pointPosition[i];
		Vec2 P2 = pointPosition[i + 1];
		double L1 = (P1 - center).length();
		double L2 = (P2 - center).length();
		double L3 = (P1 - P2).length();
		double s = (L1 + L2 + L3) / 2;
		double Area = sqrt(s*(s - L1)*(s - L2)*(s - L3));
		Vec2 crossP;
		if (i == 0){
			crossP = intersection_l(pointPosition[i + 1], pointPosition[pointLastNum], center, pointPosition[i]);//線との垂線との交点
		}
		else {
			crossP = intersection_l(pointPosition[i + 1], pointPosition[i - 1], center, pointPosition[i]);//線との垂線との交点
		}
		if ((center - crossP) * (pointPosition[i] - crossP) > 0) {
			penaltySum += Area;
		}
		else{
			AreaSum += Area;
		}
	}
	////cout << "convex penalty sum (Area): " << penaltySum << "\n";
	return penaltySum / AreaSum;
}
//	𝑃𝑖′=1/4𝑃𝑖−1+1/2𝑃𝑖+1/4𝑃𝑖+1 (𝑖=2,…𝑛−1)
double topSmoothing(Model *foldM) {
	std::vector<Vec2> pointPosition = foldM->fold->pointPosition;
	std::vector<Vec2> pIdeal;
	double diffSum = 0;
	double periphery = 0;//	外周

	for (int i = 0; i < pointPosition.size() - 1; i++) {
		periphery += (pointPosition[i] - pointPosition[i + 1]).length();
	}

	for (int i = 0; i < pointPosition.size() - 1; i++) {
		Vec2 P0, P1, P2;
		if (i == 0) {
			P0 = pointPosition[pointPosition.size() - 2];
		}
		else{
			P0 = pointPosition[i - 1];
		}
		P1 = pointPosition[i];
		P2 = pointPosition[i + 1];
		Vec2 IdealPos = 0.25*P0 + 0.5*P1 + 0.25*P2;
		pIdeal.push_back(0.25*P0 + 0.5*P1 + 0.25*P2);
		diffSum += (IdealPos - pointPosition[i]).length() / periphery;
	}
	//	誤差を計測する
	return diffSum;
}


double gapcalc(double *output, std::vector<Vec2> points, int size, int points_size){//outputのサイズ
	double fold_sum = 0.0;
	double sum = 0.0;
	double error = 1.0;

	fold_sum += points[0].x;
	for (int i = 0; i<size; i++){
		double len = sqrt((points[i].x - points[i + 1].x)*(points[i].x - points[i + 1].x) + (points[i].y - points[i + 1].y)*(points[i].y - points[i + 1].y));
		sum += len;
		if (output[i] > 0){
			fold_sum += len;
		}
		else{
			fold_sum -= len;
		}
		if (fold_sum + error < points[i + 1].x){
			////cout << "false\n";
			return -1;
		}
	}
	////cout << "fold_sum: " << fold_sum << ", points[points_size - 1]: " << points[points_size - 1].x << "\n";
	return fabs(fold_sum - points[points_size - 1].x) / sum;
}

double foldingGap(Model *foldM) {

	Model *m = foldM;
	double maxGap = 0;
	double foldgapMin = 0;

	for (int i = 0; i < (int)m->fold->outlinepoints.size(); i++){
		bool flg;
		std::vector<Vec2> points = m->fold->outlinepoints[i]->points;
		std::vector<double> l;
		Vec2 foo;

		l.resize((int)points.size() - 1);
		for (int j = 0; j < (int)points.size() - 1; j++){
			foo = points[j] - points[j + 1];

			double length = foo.length();
			l[j] = length;

		}

		int *sign = new int[(int)l.size()];
		std::vector<double*> output;
		std::vector<double> gap;
		gap.resize((int)pow(2.0, (int)l.size() - 2));
		if (points[0].y > points[(int)points.size() - 1].y){
			for (int j = 0; j < (int)points.size() / 2; j++){
				Vec2 t = points[j];
				points[j] = points[(int)points.size() - j - 1];
				points[(int)points.size() - j - 1] = t;
			}
			for (int j = 0; j < (int)l.size(); j++){
				double t = l[j];
				l[j] = l[(int)l.size() - j - 1];
				l[(int)l.size() - j - 1] = t;
			}
		}

		std::vector<int> output_array_size;
		std::vector<dR> output_dR;
		output_array_size.resize((int)pow(2.0, (int)l.size() - 2));
		output_dR.resize((int)pow(2.0, (int)l.size() - 2));
		uninfo(0, 0, sign, l, output_dR, gap, output_array_size, 0);
		output.resize((int)output_dR.size());
		for (int s = 0; s < (int)output_dR.size(); s++){
			double *tmp = new double[(int)output_dR[s].dArray.size()];
			for (int k = 0; k < output_array_size[s]; k++){
				tmp[k] = output_dR[s].dArray[k];
			}
			output[s] = tmp;
		}

		for (int j = 0; j < (int)gap.size(); j++){
			gap[j] = gap[j] - points[(int)points.size() - 1].x + points[0].x;
		}

		double *evalu1 = new double[(int)output.size()];
		double *evalu2 = new double[(int)output.size()];
		double *evalu3 = new double[(int)gap.size()];

		func1(output, points, evalu1);
		func2(output, points, evalu2);
		func3(gap, output, evalu3, output_array_size);

		double *evalu = new double[(int)output.size()];

		double w1 = 1.0; double w2 = 1.0; double w3 = 1.0;
		for (int j = 0; j < (int)output.size(); j++){
			evalu[j] = w1*evalu1[j] + w2*evalu2[j] - w3*evalu3[j];
		}

		int *eSort = new int[(int)output.size()];
		desIndex(evalu, eSort, (int)output.size());
		double *evaluN = new double[(int)output.size()];

		std::vector<double*> outputN;
		std::vector<int> outputN_array_size;
		std::vector<double> gapN;

		outputN.resize((int)output.size());
		outputN_array_size.resize((int)output.size());
		gapN.resize((int)output.size());
		double gapMin = 100000000;
		for (int j = 0; j < (int)output.size(); j++){
			int index = eSort[j];
			evaluN[j] = evalu[index];
			outputN[j] = output[index];
			outputN_array_size[j] = output_array_size[index];
			gapN[j] = gap[index];
		}

		for (int j = 0; j<(int)outputN.size(); j++){
			double L = 0;
			double alpha;
			for (int k = 0; k<outputN_array_size[j]; k++){
				L += fabs(outputN[j][k]);
			}
			alpha = gapN[j] / L;
			for (int k = 0; k<outputN_array_size[j]; k++){
				double delta = 1.0;
				if (outputN[j][k] > 0){ delta = -1.0; }
				outputN[j][k] = (1 + delta*alpha)*(outputN[j][k]);
			}

			double gapNs = gapcalc(outputN[j], points, outputN_array_size[j], (int)points.size());
			if (gapNs < gapMin && gapNs != -1) {
				gapMin = gapNs;
			}
			/*if (gapMin > abs(alpha)) {
				gapMin = abs(alpha);
			}*/
		}

		foldgapMin += gapMin;
		if (gapMin > maxGap) {
			maxGap = gapMin;
		}
		delete[] sign;
		delete[] evalu;
		delete[] evalu1;
		delete[] evalu2;
		delete[] evalu3;
		delete[] evaluN;
		delete[] eSort;
		for (int k = 0; k < (int)output.size(); k++){
			delete[] output[k];
		}
	}

	/*for(int s=(int)m->fold->outlinepoints.size()-1; s>=0; s--){
	//cout << "optimized outline: " << s << "\n";
	for(int d=0; d<m->fold->outlinepoints[s]->points.size(); d++){
	//cout << m->fold->outlinepoints[s]->points[d].x << "," << m->fold->outlinepoints[s]->points[d].y<< "\n";
	}
	}*/
	//cout << "average Gap: " << foldgapMin / (double)m->fold->betweenPosition.size();
	return foldgapMin;
}

double foldingGapDisp(Model *foldM) {

	Model *m = foldM;
	double maxGap = 0;
	double foldgapMin = 0;
	double maxLS = 0;
	for (int i = 0; i < (int)m->fold->outlinepoints.size(); i++){
		bool flg;
		std::vector<Vec2> points = m->fold->outlinepoints[i]->points;
		std::vector<double> l;
		Vec2 foo;

		l.resize((int)points.size() - 1);
		for (int j = 0; j < (int)points.size() - 1; j++){
			foo = points[j] - points[j + 1];

			double length = foo.length();
			l[j] = length;

		}

		int *sign = new int[(int)l.size()];
		std::vector<double*> output;
		std::vector<double> gap;
		gap.resize((int)pow(2.0, (int)l.size() - 2));
		if (points[0].y > points[(int)points.size() - 1].y){
			for (int j = 0; j < (int)points.size() / 2; j++){
				Vec2 t = points[j];
				points[j] = points[(int)points.size() - j - 1];
				points[(int)points.size() - j - 1] = t;
			}
			for (int j = 0; j < (int)l.size(); j++){
				double t = l[j];
				l[j] = l[(int)l.size() - j - 1];
				l[(int)l.size() - j - 1] = t;
			}
		}

		std::vector<int> output_array_size;
		std::vector<dR> output_dR;
		output_array_size.resize((int)pow(2.0, (int)l.size() - 2));
		output_dR.resize((int)pow(2.0, (int)l.size() - 2));
		uninfo(0, 0, sign, l, output_dR, gap, output_array_size, 0);
		output.resize((int)output_dR.size());
		for (int s = 0; s < (int)output_dR.size(); s++){
			double *tmp = new double[(int)output_dR[s].dArray.size()];
			for (int k = 0; k < output_array_size[s]; k++){
				tmp[k] = output_dR[s].dArray[k];
			}
			output[s] = tmp;
		}

		for (int j = 0; j < (int)gap.size(); j++){
			gap[j] = gap[j] - points[(int)points.size() - 1].x + points[0].x;
		}

		double *evalu1 = new double[(int)output.size()];
		double *evalu2 = new double[(int)output.size()];
		double *evalu3 = new double[(int)gap.size()];

		func1(output, points, evalu1);
		func2(output, points, evalu2);
		func3(gap, output, evalu3, output_array_size);

		double *evalu = new double[(int)output.size()];

		double w1 = 1.0; double w2 = 1.0; double w3 = 1.0;
		for (int j = 0; j < (int)output.size(); j++){
			evalu[j] = w1*evalu1[j] + w2*evalu2[j] - w3*evalu3[j];
		}

		int *eSort = new int[(int)output.size()];
		desIndex(evalu, eSort, (int)output.size());
		double *evaluN = new double[(int)output.size()];

		std::vector<double*> outputN;
		std::vector<int> outputN_array_size;
		std::vector<double> gapN;

		outputN.resize((int)output.size());
		outputN_array_size.resize((int)output.size());
		gapN.resize((int)output.size());
		double gapMin = 100000000;
		for (int j = 0; j < (int)output.size(); j++){
			int index = eSort[j];
			evaluN[j] = evalu[index];
			outputN[j] = output[index];
			outputN_array_size[j] = output_array_size[index];
			gapN[j] = gap[index];
		}
		double LS;
		for (int j = 0; j<(int)outputN.size(); j++){
			double L = 0;
			double alpha;
			for (int k = 0; k<outputN_array_size[j]; k++){
				L += fabs(outputN[j][k]);
			}
			alpha = gapN[j] / L;
			
			for (int k = 0; k<outputN_array_size[j]; k++){
				double delta = 1.0;
				if (outputN[j][k] > 0){ delta = -1.0; }
				outputN[j][k] = (1 + delta*alpha)*(outputN[j][k]);
			}

			double gapNs = gapcalc(outputN[j], points, outputN_array_size[j], (int)points.size());
			if (gapNs < gapMin && gapNs != -1) {
				gapMin = gapNs;
				LS = L;
			}
			/*if (gapMin > abs(alpha)) {
			gapMin = abs(alpha);
			}*/
		}

		foldgapMin += gapMin;
		if (gapMin > maxGap) {
			maxGap = gapMin;
		}
		if (maxLS < (LS / (double)(m->fold->outlinepoints[i]->points.size() - 1)) * 0.001) {
			maxLS = (LS / (double)(m->fold->outlinepoints[i]->points.size() - 1)) * 0.001;
		}
		delete[] sign;
		delete[] evalu;
		delete[] evalu1;
		delete[] evalu2;
		delete[] evalu3;
		delete[] evaluN;
		delete[] eSort;
		for (int k = 0; k < (int)output.size(); k++){
			delete[] output[k];
		}
	}

	/*for(int s=(int)m->fold->outlinepoints.size()-1; s>=0; s--){
	//cout << "optimized outline: " << s << "\n";
	for(int d=0; d<m->fold->outlinepoints[s]->points.size(); d++){
	//cout << m->fold->outlinepoints[s]->points[d].x << "," << m->fold->outlinepoints[s]->points[d].y<< "\n";
	}
	}*/
	//cout << "max Gap: " << maxGap << "\n";
	cout << "OK ?? " << maxLS << "\n";
	return maxGap;
}

double metro(Model *m, CMesh *input, CMesh *fold) {
	for (int i = 0; i < fold->VN(); i++) {
		CMesh::VertexPointer vi = &fold->vert[i];
		////cout << vi->P().X /*<< "," << vi->P().Y << "," << vi->P().Z*/ << "\n";
	}
	openMesh(m, fold);//foldModelの頂点座標を更新する
	//cout << "\n\n";
	for (int i = 0; i < fold->VN(); i++) {
		CMesh::VertexPointer vi = &fold->vert[i];
		////cout << vi->P().X << "," << vi->P().Y << "," << vi->P().Z << "\n";
	}
	setMeshInfo(fold);
	return calcMetro((*input), (*fold));
}

void updateParam(Model *foldM, Eigen::VectorXd dir) {
	int count = 1;
	foldM->fold->topPosY += dir(0);
	int pointLastNum = foldM->fold->pointPosition.size() - 1;
	
	for (int k = 0; k < foldM->fold->pointPosition.size() - 1; k++) {
		for (int l = 0; l < foldM->fold->outlinepoints[k]->points.size(); l++) {
			if (l == 0) {
				Vec2 h;
				double x = dir(count); count++; //cout << "count: " << count-1 << "pointP x : " << dir(count - 1) << "\n";
				double y = dir(count); count++; //cout << "count: " << count-1 << "pointP y : " << dir(count - 1) << "\n";
				h.set(x, y);
				foldM->fold->pointPosition[k] += h;
				if (k == 0) foldM->fold->pointPosition[pointLastNum] = foldM->fold->pointPosition[0];
				continue;
			}
			if (l != foldM->fold->outlinepoints[k]->points.size() - 1) {//最後の点でなければx,y両方を動かせる
				Vec2 h;
				double x = dir(count); count++; //cout << "count: " << count-1 << "out x : " << dir(count - 1) << "\n";
				double y = dir(count); count++; //cout << "count: " << count-1 << "out y : " << dir(count - 1) << "\n";
				h.set(x, y);
				
				foldM->fold->outlinepoints[k]->points[l] += h;
			}
			else {//	最後の点はxだけ動かせる
				Vec2 h;
				double x = dir(count); count++; //cout << "count: " << count-1 << "out last x : " << dir(count - 1) << "\n";
				double y = 0;
				h.set(x, y);
				
				foldM->fold->outlinepoints[k]->points[l] += h;
			}
		}
		if (k == foldM->fold->pointPosition.size() - 2){
			Vec2 h;
			double x = 0;
			double y = dir(count); //cout << "count: " << count-1 << "out last y : " << dir(count) << "\n";
			h.set(x, y);
			for (int m = 0; m < foldM->fold->pointPosition.size() - 1; m++) {
				int outlinepointsLastNum = foldM->fold->outlinepoints[m]->points.size() - 1;
				foldM->fold->outlinepoints[m]->points[outlinepointsLastNum] += h;
			}
		}
	}
	//calc
	//cout << "count in update: " << count << "\n";
	updateBetweenPos(foldM);
}

Polyhedron_G Optimization(Cmodel *cm, COpenGL *fObj) {
	Model *foldM = cm->foldM;
	double lambda = 100000;//	小さいほどガウス：ニュートン法の影響が大きい
	Eigen::VectorXd test = eachPenalty(cm, fObj);
	double prevCost = test.squaredNorm() / 2;
	double firstCost = prevCost;
	double difference;
	int itr = 300;
	int notGoodNum = 0;
	int count = 0;
	int variableNum= 1;
	int constraintNum = 4;//	制約の数 近似項、凸近似項、スムージング項
	double delta = 10.0;
	Polyhedron_G P;
	std::vector<std::vector<Vec2>> outlinepoints_tmp;
	std::vector<Vec2> pointPosition_tmp;

	cout << "first: " << firstCost << "\n";
	outputFolding(foldM);
	//return P;
	variableNum += (foldM->fold->pointPosition.size() - 1) * 2 + 1;
	for (int i = 0; i < foldM->fold->pointPosition.size() - 1; i++) {
		variableNum += (foldM->fold->outlinepoints[i]->points.size() - 1) * 2 - 1;
	}

	int pointLastNum = foldM->fold->pointPosition.size() - 1;

	Eigen::MatrixXd jacobian(variableNum, constraintNum);//	変数の数、制約の数
	Eigen::MatrixXd hessian(variableNum, variableNum);
	Eigen::VectorXd grad(variableNum);

	for (int i = 0; i < itr; i++) {
		Eigen::VectorXd penaltyS;
		penaltyS = eachPenalty(cm, fObj);
		prevCost = penaltyS.squaredNorm() / 2.0;

		Eigen::VectorXd grad = Eigen::VectorXd::Zero(variableNum);
		Eigen::VectorXd gradJ = Eigen::VectorXd::Zero(variableNum);
		jacobian = ComputeNumericalJacobian(cm, fObj, variableNum, constraintNum);
		for (int j = 0; j < constraintNum; j++) {
			//double value = penaltyS[j];
			double value = 1;
			for (int k = 0; k < variableNum; k++) {
				gradJ(k) += value * jacobian(k, j);
			}
		}
		hessian = jacobian * jacobian.transpose();
		double topPos = foldM->fold->topPosY;
		grad = Eigen::VectorXd::Zero(variableNum);
		grad = -gradJ;
		
		double Max = 0;
		for (int ii = 0; ii < variableNum; ii++) {
			if (abs(grad(ii)) > Max) {
				Max = abs(grad(ii));
			}
		}
		double Maxval = Max;
		if (Max < 1.0) {
			Max = pow(10, (int)log10(abs(Max))) * 0.1;
		}
		else{
			Max = pow(10, (int)log10(abs(Max)));
		}

		for (int ii = 0; ii < grad.size(); ii++) {
		//	grad(ii) = grad(ii) / Max;
		}
		while (1) {
			count++;
			//Eigen::MatrixXd A = computeLambda(hessian, lambda, variableNum);
			/*for (int ii = 0; ii < 5; ii++) {
				grad = retunDelta(A, grad, gradJ, variableNum);
			}*/
			//grad = A.colPivHouseholderQr().solve(-1*gradJ);
			//	grad = retunDelta(A, grad, -penalty(foldM, fObj, inputP, inputP_)*jacobian);
			//backUp position
			for (int j = 0; j < foldM->fold->outlinepoints.size(); j++) {
				outlinepoints_tmp.push_back(foldM->fold->outlinepoints[j]->points);
			}
			pointPosition_tmp = foldM->fold->pointPosition;
			updateParam(foldM, delta * grad);
			Eigen::VectorXd penalt = eachPenalty(cm, fObj);
			double cost = penalt.squaredNorm() / 2.0;
			cout << "prevCost: " << delta << " " << prevCost << " ";
			cout << "cost: " << cost << " ";
			if (cost < prevCost) {
				difference = prevCost - cost;
				prevCost = cost;
				outlinepoints_tmp.clear();
				pointPosition_tmp.clear();
				delta *= 10.0;
				cout << "OK ";
				notGoodNum = 0;
			}
			else {
				// 退避したものを戻す
				cout << "not good ";
				//outputFolding(foldM);
				lambda /= 10.0;
				for (int j = 0; j < foldM->fold->outlinepoints.size(); j++) {
					foldM->fold->outlinepoints[j]->points = outlinepoints_tmp[j];
				}
				foldM->fold->pointPosition = pointPosition_tmp;
				foldM->fold->topPosY = topPos;
				outlinepoints_tmp.clear();
				pointPosition_tmp.clear();
				updateBetweenPos(foldM);
				delta *= 0.1;
				notGoodNum++;
				break;
			}
		}
		lambda *= 10;
		//cout << "itr: " << i << "\n";
		if (difference < 1.0e-7 && difference > 0 && itr > 10) {
			//cout << "difference: " << difference << "\n";
		//	break;
		}
		if (notGoodNum > 30) {
			break;
		}
	}

	cout << "first cost: " << firstCost << "\n";

	outputFolding(foldM);
	fObj->convertFoldingToMesh(foldM);//	メッシュデータへと変換
	P = inputPoly_G(foldM);//	Polyhedron_Gへと変換
	return P;
}

void Optimization() {
	float lambda = 0.00001;
	int itr = 30;
	int variableNum = 2;
	int constraintsNum = 21;
	std::vector<float> u;
	u.push_back(-0.8);
	u.push_back(0.8);
	double difference;
	float x2[] = {
		0.502773, 0.1839388, 0.3149675, 0.2122653, 0.2540119, 0.04883509, 0.2105905, 0.4572684, -0.339329, 0.1218154,
		-0.1445506, -0.1208329, -0.3432412, -0.00685782, 0.1469265, -0.1604335, 0.5677256, 0.09309396, -0.1357552, 0.3821898,
		0.9703713
	};

	float x1[] = {
		0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45,
		0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0
	};

	float cost = penalty(u, x1, x2, 21) / 2;
	float prevCost = cost;

	//cout << "first cost: " << cost << "\n";

	Eigen::MatrixXf jacobian(2, constraintsNum);
	Eigen::MatrixXf hessian(constraintsNum, constraintsNum);
	Eigen::VectorXf grad(variableNum);

	for (int i = 0; i < itr; i++) {
		//cout << "i: " << i << "\n";
		//cout << "lambda: " << lambda << "\n";
		prevCost = penalty(u, x1, x2, 21) / 2;
		grad = Eigen::VectorXf::Zero(variableNum);
		jacobian = ComputeNumericalJacobian(u, x1, x2, 21);
		Eigen::VectorXf gradJ = Eigen::VectorXf::Zero(2);
		//cout << "value,u[0],u[1]:\n";
		for (int k = 0; k < 21; k++) {
			float value = ((u[0] * pow((x1[k] - u[1]), 2) - x2[k]));
			gradJ(0) += value * jacobian(0, k);
			gradJ(1) += value * jacobian(1, k);

			//cout << value << "," << jacobian(0, k) << "," << jacobian(1, k) << "\n";
		}
		hessian = jacobian * jacobian.transpose();
		////cout << "jacobian:\n" << jacobian << "\n\n";
		//cout << "gradJ: \n" << gradJ << "\n\n";
		////cout << "hessian: \n" << hessian << "\n\n";
		bool T = true;
		int L = 0;
		while (1) {
			//cout << "while: " << L << "\n";
			Eigen::MatrixXf A;
			if (T) {
				A = computeLambda(hessian, lambda, variableNum);
			}
			else {
				A = computeLambdaUnit(hessian, lambda, variableNum);
			}
			//cout << "A: \n" << A << "\n";
			
			//行列式を解く
			grad = retunDelta(A, grad, -1 * gradJ);
		
			//cout << "deltau: \n" << grad << "\n\n";
			
			//	return;
			//値を保存しておく
			std::vector<float> u_;
			u_.push_back(u[0]);
			u_.push_back(u[1]);

			//	パラメータをアップデート
			u[0] += grad(0);
			u[1] += grad(1);

			float cost = penalty(u, x1, x2, 21) / 2;

			////cout << "cost: " << cost << "\n";
			////cout << "prevCost: " << prevCost << "\n";
			if (cost < prevCost) {
				difference = prevCost - cost;
				break;
			}
			else {
				// 退避したものを戻す
				lambda *= 10.0;
				if (lambda > 10.0) {
					break;
				}
				u[0] = u_[0];
				u[1] = u_[1];
			}
			L++;
		}
		lambda /= 10;
		if (difference < 1.0e-5) {
			break;
		}
	}
	//cout << "u = " << u[0] << "," << u[1] << "\n";

	return;
}

Eigen::VectorXd retunDelta(Eigen::MatrixXd M, Eigen::VectorXd b) {
	Eigen::VectorXd V = M.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
	return V;
}

Eigen::VectorXf retunDelta(Eigen::MatrixXf M, Eigen::VectorXf V, Eigen::VectorXf b) {
	V = M.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
	return V;
}

Eigen::VectorXd retunDelta(Eigen::MatrixXd M, Eigen::VectorXd V, Eigen::VectorXd b, int Mrow) {
	Eigen::VectorXd curr;
	
	double diag;
	for (int j = 0; j < 1; j++) {
		for (int i = 0; i < Mrow; i++) {
			curr = M.row(i);
			diag = M(i, i);
			V(i) += (b(i) - V.dot(curr)) / diag;
			//cout << "V(i):" << V(i) << "\n";
		}
	}
	//cout << "V: " << V << "\n";
	return V;
}


// grad = retunGradient(A, a, jacobian, variableNum * 2);
//	V = Δu
//  M = (Hu + c(lambda)D[Hu])
//	b = jacobian
//	bはベクトル
//
//	grad//	更新する値
// A →　(Hu + c(lambda)D[Hu])
// Ax = b
// (Hu + c(lambda)D[Hu])・Δu = -▽uJ
// x = ▽u
// b = -▽uJ
//

void updateBetweenPos(Model *foldM) {
	for (int i = 0; i < foldM->fold->pointPosition.size() - 1; i++) {
		foldM->fold->betweenPosition[i] = (foldM->fold->pointPosition[i] + foldM->fold->pointPosition[i + 1]) / 2;
	}
}

void testVolumeCalculation(Model *m) {
	m->outputV();
	//cout << "Model m volume:" << calcVolume(m) << "\n";
}

void testVolumeCalculation(Polyhedron_G m) {
	//cout << "Polyhedron_G m volume:" << calcVolume(m) << "\n";
}
