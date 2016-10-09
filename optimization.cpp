#include "optimization.h"

Eigen::VectorXd ComputeNumericalJacobian(Model *foldM, COpenGL *fObj, Polyhedron_G *inputP, Nef_polyhedron_3 inputP_) {
	int variableNum = 0;
	for (int i = 0; i < foldM->fold->outlinepoints.size(); i++) {
		variableNum += foldM->fold->outlinepoints[i]->points.size();
	}
	Eigen::VectorXd jacobian(variableNum*2);
	double DELTA = 1.0e-7;
	double invDelta = 1.0 / DELTA;
	double fx_tmp = -invDelta * penalty(foldM, fObj, inputP, inputP_);
	int pointLastNum = foldM->fold->pointPosition.size() - 1;
	int count = 0;
	for (int i = 0; i < foldM->fold->pointPosition.size()-1; i++) {
		cout << "i in jacobian: " << i << "\n";
		for (int j = 0; j < foldM->fold->outlinepoints[i]->points.size(); j++) {
			if (j == 0) {
				foldM->fold->pointPosition[i].x += DELTA;
				if (i == 0) {
					foldM->fold->pointPosition[pointLastNum].x += DELTA;
				}
				updateBetweenPos(foldM);
				jacobian(count) = invDelta * penalty(foldM, fObj, inputP, inputP_) + fx_tmp;
				count++;
				foldM->fold->pointPosition[i].x -= DELTA;
				if (i == 0) {
					foldM->fold->pointPosition[pointLastNum].x -= DELTA;
				}
				foldM->fold->pointPosition[i].y += DELTA;
				if (i == 0) {
					foldM->fold->pointPosition[pointLastNum].y += DELTA;
				}
				updateBetweenPos(foldM);
				jacobian(count) = invDelta * penalty(foldM, fObj, inputP, inputP_) + fx_tmp;
				count++;
				foldM->fold->pointPosition[i].y -= DELTA;
				if (i == 0) {
					foldM->fold->pointPosition[pointLastNum].y -= DELTA;
				}
				updateBetweenPos(foldM);
				count++;
				continue;
			}
			foldM->fold->outlinepoints[i]->points[j].x += DELTA;
			updateBetweenPos(foldM);
			jacobian(count) = invDelta * penalty(foldM, fObj, inputP, inputP_) + fx_tmp;
			count++;
			foldM->fold->outlinepoints[i]->points[j].x -= DELTA;
			if (j != foldM->fold->outlinepoints[i]->points.size()-1) {
				foldM->fold->outlinepoints[i]->points[j].y += DELTA;
				updateBetweenPos(foldM);
				jacobian(count) = invDelta * penalty(foldM, fObj, inputP, inputP_) + fx_tmp;
				count++;
				foldM->fold->outlinepoints[i]->points[j].y -= DELTA;
				updateBetweenPos(foldM);
			}
		}
	}

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
		for (int i = 0; i < foldM->fold->pointPosition.size()-1; i++){
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
	int iterator = 1;
	int variableNum = 0;

	std::vector<outline*> outlinepoints_tmp;
	std::vector<Vec2> pointPosition_tmp;
	variableNum += (foldM->fold->pointPosition.size() - 1) * 2 + 1;
	for (int i = 0; i < foldM->fold->pointPosition.size() - 1; i++) {
		variableNum += (foldM->fold->outlinepoints[i]->points.size() - 1) * 2 - 1;
	}
	int pointLastNum = foldM->fold->pointPosition.size() - 1;
	Eigen::VectorXd jacobian(variableNum * 2);
	Eigen::MatrixXd hessian(variableNum * 2, variableNum * 2);
	Eigen::VectorXd grad(variableNum * 2);

	for (int i = 0; i < iterator; i++) {
		grad = Eigen::VectorXd::Zero(variableNum * 2);
		cout << "iteration: " << i << "\n";
		jacobian = ComputeNumericalJacobian(foldM, fObj, inputP, inputP_);
		cout << "after calculate jacobian\n";
		cout << "jacobian: " << jacobian << "\n";
		hessian = jacobian * jacobian.transpose();
		bool foundBetter = false;
		//	cout << "hessian\n" << hessian << "\n";

		for (int j = 0; j < 5; j++) {
			cout << "j: " << j << "\n";
			Eigen::MatrixXd A = computeLambda(hessian, lambda, variableNum*2);
			cost = penalty(foldM, fObj, inputP, inputP_);
			grad = retunGradient(A, grad, jacobian, variableNum * 2);

			//backUp position
			outlinepoints_tmp = foldM->fold->outlinepoints;
			pointPosition_tmp = foldM->fold->pointPosition;
			int count = 0;
			for (int k = 0; k < foldM->fold->pointPosition.size() - 1; k++) {
				for (int l = 0; l < foldM->fold->outlinepoints[k]->points.size(); l++) {
					if (l == 0) {
						Vec2 h;
						double x = grad(count);
						double y = grad(count + variableNum);
						h.set(x, y);
						foldM->fold->pointPosition[i] -= h;
						if (k == 0) {
							foldM->fold->pointPosition[pointLastNum] = foldM->fold->pointPosition[0];
						}
						count++;
						continue;
					}
					Vec2 h;
					double x = grad(count);
					double y = grad(count + variableNum);
					h.set(x, y);
					foldM->fold->outlinepoints[k]->points[l] -= h;
					count++;
				}
			}
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
				foldM->fold->outlinepoints = outlinepoints_tmp;
				foldM->fold->pointPosition = pointPosition_tmp;
			}
		}
		if (!foundBetter){
			cout << "Can't find better irection\n";
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


Eigen::VectorXd retunGradient(Eigen::MatrixXd M, Eigen::VectorXd V, Eigen::VectorXd b, int Mrow) {
	Eigen::VectorXd curr;
	V = Eigen::VectorXd::Zero(Mrow);
	double diag;

	for (int i = 0; i < Mrow; i++) {
		curr = M.row(i);
		diag = M(i,i);
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
	for (int i = 0; i < foldM->fold->pointPosition.size()-1; i++) {
		foldM->fold->betweenPosition[i] = (foldM->fold->pointPosition[i] + foldM->fold->pointPosition[i + 1]) / 2;
	}
}

void testVolumeCalculation(Model *m) {
	cout << "Model m volume:" << calcVolume(m) << "\n";
}

void testVolumeCalculation(Polyhedron_G m) {
	cout << "Polyhedron_G m volume:" << calcVolume(m) << "\n";
}