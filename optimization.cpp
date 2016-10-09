#include "optimization.h"
using namespace std;

//	数値微分
Eigen::VectorXd ComputeNumericalJacobian(Model *foldM, COpenGL *fObj, Polyhedron_G *inputP, Nef_polyhedron_3 inputP_, int variableNum) {
	
	Eigen::VectorXd jacobian(variableNum);
	double DELTA = 1.0e-7;
	double invDelta = 1.0 / DELTA;
	double fx_tmp = -invDelta * penalty(foldM, fObj, inputP, inputP_);
	int pointLastNum = foldM->fold->pointPosition.size() - 1;
	int count = 0;

	//yの高さ
	foldM->fold->topPosY += DELTA;
	jacobian(count) = invDelta * penalty(foldM, fObj, inputP, inputP_) + fx_tmp;
	foldM->fold->topPosY -= DELTA;
	count++;

	for (int i = 0; i < foldM->fold->pointPosition.size() - 1; i++) {
		cout << "i in jacobian: " << i << "\n";
		for (int j = 0; j < foldM->fold->outlinepoints[i]->points.size(); j++) {
			if (j == 0) {//	pointsPositionのx,yを変更
				foldM->fold->pointPosition[i].x += DELTA;
				if (i == 0) foldM->fold->pointPosition[pointLastNum].x += DELTA;
				updateBetweenPos(foldM);
				jacobian(count) = invDelta * penalty(foldM, fObj, inputP, inputP_) + fx_tmp;
				count++;
				foldM->fold->pointPosition[i].x -= DELTA;
				if (i == 0) foldM->fold->pointPosition[pointLastNum].x -= DELTA;

				foldM->fold->pointPosition[i].y += DELTA;
				if (i == 0) foldM->fold->pointPosition[pointLastNum].y += DELTA;
				updateBetweenPos(foldM);
				jacobian(count) = invDelta * penalty(foldM, fObj, inputP, inputP_) + fx_tmp;
				count++;
				foldM->fold->pointPosition[i].y -= DELTA;
				if (i == 0) foldM->fold->pointPosition[pointLastNum].y -= DELTA;
				updateBetweenPos(foldM);
				continue;
			}
			foldM->fold->outlinepoints[i]->points[j].x += DELTA;
			jacobian(count) = invDelta * penalty(foldM, fObj, inputP, inputP_) + fx_tmp;
			count++;
			foldM->fold->outlinepoints[i]->points[j].x -= DELTA;
			if (j != foldM->fold->outlinepoints[i]->points.size() - 1) {
				foldM->fold->outlinepoints[i]->points[j].y += DELTA;
				jacobian(count) = invDelta * penalty(foldM, fObj, inputP, inputP_) + fx_tmp;
				count++;
				foldM->fold->outlinepoints[i]->points[j].y -= DELTA;
			}
		}
		if (i == foldM->fold->pointPosition.size() - 2) {//最後にoutlinePointsのy座標全てを変換
			for (int j = 0; j < foldM->fold->pointPosition.size() - 1; j++) {
				int outlinepointsLastNum = foldM->fold->outlinepoints[j]->points.size() - 1;
				foldM->fold->outlinepoints[j]->points[outlinepointsLastNum].y += DELTA;
			}
			jacobian(count) = invDelta * penalty(foldM, fObj, inputP, inputP_) + fx_tmp;
			count++;
			for (int j = 0; j < foldM->fold->pointPosition.size() - 1; j++) {
				int outlinepointsLastNum = foldM->fold->outlinepoints[j]->points.size() - 1;
				foldM->fold->outlinepoints[j]->points[outlinepointsLastNum].y -= DELTA;
			}
		}
	} 
	cout << "count in jacobian calculation: " << count << "\n";
	cout << count;
	return jacobian;
}

double penalty(Model *foldM, COpenGL *fObj, Polyhedron_G * inputP, Nef_polyhedron_3 inputP_) {
	//	最初に三次元座標へと変換
	fObj->convertFoldingToMesh(foldM);//	メッシュデータへと変換
	Polyhedron_G P = inputPoly_G(foldM);//	Polyhedron_Gへと変換
	double Diff = calculateDiff(P, inputP_, inputP);
	if (Diff == 0) {
		for (int i = 0; i < foldM->fold->pointPosition.size(); i++){
			cout << "pointsPos: " << foldM->fold->pointPosition[i].x << "," << foldM->fold->pointPosition[i].y << "\n";
		}
		cout << "\n";
		for (int i = 0; i < foldM->fold->pointPosition.size() - 1; i++){
			cout << "outline: " << i << "\n";
			cout << "outline size: " << foldM->fold->outlinepoints.size() << "\n";
			cout << "points size: " << foldM->fold->outlinepoints[i]->points.size() << "\n";
			for (int j = 0; j < foldM->fold->outlinepoints[i]->points.size(); j++){
				cout << "outliePos" << foldM->fold->outlinepoints[i]->points[j].x << "," << foldM->fold->outlinepoints[i]->points[j].y << "\n";
			}
			cout << "\n";
		}
		return 1000000;
	}
	return Diff;
}

Eigen::MatrixXd computeLambda(Eigen::MatrixXd A, double lambda, int numOfA){
	for (int i = 0; i < numOfA; i++) {
		A(i, i) = A(i, i) + lambda;
	}
	return A;
}

void Optimization(Model *foldM, COpenGL *fObj, Polyhedron_G *inputP, Nef_polyhedron_3 inputP_) {
	double lambda = 1.0;
	double cost = penalty(foldM, fObj, inputP, inputP_);
	double prevCost = cost;
	double difference;
	int itr = 1;
	int variableNum = 0;

	std::vector<std::vector<Vec2>> outlinepoints_tmp;
	std::vector<Vec2> pointPosition_tmp;

	variableNum += (foldM->fold->pointPosition.size() - 1) * 2 + 1;
	for (int i = 0; i < foldM->fold->pointPosition.size() - 1; i++) {
		variableNum += (foldM->fold->outlinepoints[i]->points.size() - 1) * 2 - 1;
	}
	cout << "variable Num: " << variableNum << "\n";

	int pointLastNum = foldM->fold->pointPosition.size() - 1;

	Eigen::VectorXd jacobian(variableNum);
	Eigen::MatrixXd hessian(variableNum, variableNum);
	Eigen::VectorXd grad(variableNum);

	for (int i = 0; i < itr; i++) {
		grad = Eigen::VectorXd::Zero(variableNum * 2);
		cout << "iteration: " << i << "\n";
		jacobian = ComputeNumericalJacobian(foldM, fObj, inputP, inputP_, variableNum);
		//	cout << "after calculate jacobian\n";
		//	cout << "jacobian: " << jacobian << "\n";
		hessian = jacobian * jacobian.transpose();
		bool foundBetter = false;
		//	cout << "hessian\n" << hessian << "\n";

		Eigen::MatrixXd A = computeLambda(hessian, lambda, variableNum * 2);
		cost = penalty(foldM, fObj, inputP, inputP_);
		grad = retunDelta(A, grad, jacobian, variableNum * 2);

		//backUp position
		for (int j = 0; j < foldM->fold->outlinepoints.size(); j++) {
			outlinepoints_tmp.push_back(foldM->fold->outlinepoints[j]->points);
		}
		pointPosition_tmp = foldM->fold->pointPosition;

		//calc
		int count = 1;
		foldM->fold->topPosY -= grad(0);
		for (int k = 0; k < foldM->fold->pointPosition.size() - 1; k++) {
			for (int l = 0; l < foldM->fold->outlinepoints[k]->points.size(); l++) {
				if (l == 0) {
					Vec2 h;
					double x = grad(count); count++;
					double y = grad(count); count++;
					h.set(x, y);
					foldM->fold->pointPosition[i] -= h;
					if (k == 0) foldM->fold->pointPosition[pointLastNum] = foldM->fold->pointPosition[0];
					continue;
				}
				if (l != foldM->fold->outlinepoints[k]->points.size()-1) {//最後の点でなければx,y両方を動かせる
					Vec2 h;
					double x = grad(count); count++;
					double y = grad(count); count++;
					h.set(x, y);
					foldM->fold->outlinepoints[k]->points[l] -= h;
				}
				else {//	最後の点はxだけ動かせる
					Vec2 h;
					double x = grad(count); count++;
					double y = 0;
					h.set(x, y);
					foldM->fold->outlinepoints[k]->points[l] -= h;
				}
				if (k == foldM->fold->pointPosition.size() - 2){
					Vec2 h;
					double x = 0;
					double y = grad(count);
					for (int m = 0; m < foldM->fold->pointPosition.size() - 1; m++) {
						int outlinepointsLastNum = foldM->fold->outlinepoints[m]->points.size() - 1;
						foldM->fold->outlinepoints[m]->points[outlinepointsLastNum] -= h;
					}
				}
			}
		}
		cout << "count in optimization: " << count << "\n";
		updateBetweenPos(foldM);
		cost = penalty(foldM, fObj, inputP, inputP_);
		cout << "cost: " << cost << "\n";
		if (cost < prevCost) {
			foundBetter = true;
			prevCost = cost;
			lambda *= 0.1;
			difference = prevCost - cost;
		}
		else {
			// 退避したものを戻す
			lambda *= 10.0;
			for (int j = 0; j < foldM->fold->outlinepoints.size(); j++) {
				foldM->fold->outlinepoints[j]->points = outlinepoints_tmp[j];
			}
			foldM->fold->pointPosition = pointPosition_tmp;
			foldM->fold->outlinepoints.clear();
			foldM->fold->pointPosition.clear();
		}

		if (!foundBetter){
			cout << "Can't find better iteration\n";
			break;
		}
	}
	cout << "Diff: " << penalty(foldM, fObj, inputP, inputP_) << "\n";
	for (int i = 0; i < foldM->fold->pointPosition.size(); i++){
		cout << "pointsPos: " << foldM->fold->pointPosition[i].x << "," << foldM->fold->pointPosition[i].y << "\n";
	}
	cout << "\n";
	for (int i = 0; i < foldM->fold->pointPosition.size() - 1; i++){
		cout << "outline: " << i << "\n";
		for (int j = 0; j < foldM->fold->outlinepoints[i]->points.size(); j++){
			cout << "outliePos" << foldM->fold->outlinepoints[i]->points[j].x << "," << foldM->fold->outlinepoints[i]->points[j].y << "\n";
		}
		cout << "\n";
	}
}


Eigen::VectorXd retunDelta(Eigen::MatrixXd M, Eigen::VectorXd V, Eigen::VectorXd b, int Mrow) {
	Eigen::VectorXd curr;
	V = Eigen::VectorXd::Zero(Mrow);
	double diag;

	for (int i = 0; i < Mrow; i++) {
		curr = M.row(i);
		diag = M(i, i);
		V(i) += (b(i) - V.dot(curr)) / diag;
	}
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
	cout << "Model m volume:" << calcVolume(m) << "\n";
}

void testVolumeCalculation(Polyhedron_G m) {
	cout << "Polyhedron_G m volume:" << calcVolume(m) << "\n";
}