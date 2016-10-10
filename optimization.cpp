#include "optimization.h"

using namespace std;
using namespace Eigen;

//	数値微分
Eigen::VectorXd ComputeNumericalJacobian(Model *foldM, COpenGL *fObj, Polyhedron_G *inputP, Nef_polyhedron_3 inputP_, int variableNum) {
	
	Eigen::VectorXd jacobian(variableNum);
	double DELTA = 1.0e-7;
	double invDelta = 1.0 / DELTA;
	double fx_tmp = -invDelta * penalty(foldM, fObj, inputP, inputP_);
	cout << "fx_tmp: " << fx_tmp << "\n";
	int pointLastNum = foldM->fold->pointPosition.size() - 1;
	int count = 0;

	//yの高さ
	foldM->fold->topPosY += DELTA;
	jacobian(count) = invDelta * penalty(foldM, fObj, inputP, inputP_) + fx_tmp;
	cout << count << ": " << jacobian(count) << "\n";
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
				cout << i << "," << j <<  "x: " << jacobian(count) << "\n";
				count++;
				foldM->fold->pointPosition[i].x -= DELTA;
				if (i == 0) foldM->fold->pointPosition[pointLastNum].x -= DELTA;

				foldM->fold->pointPosition[i].y += DELTA;
				if (i == 0) foldM->fold->pointPosition[pointLastNum].y += DELTA;
				updateBetweenPos(foldM);
				jacobian(count) = invDelta * penalty(foldM, fObj, inputP, inputP_) + fx_tmp;
				cout << i << "," << j << "y: " << jacobian(count) << "\n";
				count++;
				foldM->fold->pointPosition[i].y -= DELTA;
				if (i == 0) foldM->fold->pointPosition[pointLastNum].y -= DELTA;
				updateBetweenPos(foldM);
				continue;
			}
			foldM->fold->outlinepoints[i]->points[j].x += DELTA;
			jacobian(count) = invDelta * penalty(foldM, fObj, inputP, inputP_) + fx_tmp;
			cout << i << "," << j << "x: " << jacobian(count) << "\n";
			count++;
			foldM->fold->outlinepoints[i]->points[j].x -= DELTA;
			if (j != foldM->fold->outlinepoints[i]->points.size() - 1) {
				foldM->fold->outlinepoints[i]->points[j].y += DELTA;
				jacobian(count) = invDelta * penalty(foldM, fObj, inputP, inputP_) + fx_tmp;
				cout << i << "," << j << "y: " << jacobian(count) << "\n";
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
			cout << count << ": " << jacobian(count) << "\n";
			cout << i << ","  << "y: " << jacobian(count) << "\n";
			for (int j = 0; j < foldM->fold->pointPosition.size() - 1; j++) {
				int outlinepointsLastNum = foldM->fold->outlinepoints[j]->points.size() - 1;
				foldM->fold->outlinepoints[j]->points[outlinepointsLastNum].y -= DELTA;
			}
		}
	} 
	cout << "count in jacobian calculation: " << count << "\n";
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


double penalty(Model *foldM, COpenGL *fObj, Polyhedron_G * inputP, Nef_polyhedron_3 inputP_) {
	//	最初に三次元座標へと変換
	for (int i = 0; i < foldM->fold->outlinepoints.size(); i++) {
		double Ypos = 0;
		for (int j = 1; j < foldM->fold->outlinepoints[i]->points.size(); j++) {
			if (foldM->fold->outlinepoints[i]->points[j].y < Ypos) {
				return 1000000;
			}
			else {
				Ypos = foldM->fold->outlinepoints[i]->points[j].y;
			}
		}
	}
	fObj->convertFoldingToMesh(foldM);//	メッシュデータへと変換
	Polyhedron_G P = inputPoly_G(foldM);//	Polyhedron_Gへと変換
	double Diff = calculateDiff(P, inputP_, inputP);
	
	/*if (Diff == 0) {
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
	}*/
	return Diff;
}

float penalty(std::vector<float> u, float *x1, float *x2, int num) {
	
	double sum = 0;

	for (int i = 0; i < num; i++) {
		sum += pow((u[0] * pow((x1[i] - u[1]), 2) - x2[i]),2);
	}
	return sum;
}

float penalty_(std::vector<float> u, float *x1, float *x2, int num) {

	float sum = 0;

	for (int i = 0; i < num; i++) {
		sum += (u[0] * pow((x1[i] - u[1]), 2) - x2[i]);
	}
	cout << "sum: " << sum << "\n";
	return sum;
}

Eigen::MatrixXd computeLambda(Eigen::MatrixXd A, double lambda, int numOfA){
	for (int i = 0; i < numOfA; i++) {
		A(i, i) = A(i, i) + A(i, i) * lambda;
	}
	return A;
}
Eigen::MatrixXf computeLambda(Eigen::MatrixXf A, double lambda, int numOfA){
	for (int i = 0; i < numOfA; i++) {
		A(i, i) = A(i, i) + A(i, i) * lambda;
		cout << A(i, i) << "\n";
	}
	return A;
}

Eigen::MatrixXd computeLambdaUnit(Eigen::MatrixXd A, double lambda, int numOfA){
	for (int i = 0; i < numOfA; i++) {
		A(i, i) = A(i, i) + lambda;
		cout << A(i, i) << "\n";
	}
	return A;
}

Eigen::MatrixXf computeLambdaUnit(Eigen::MatrixXf A, double lambda, int numOfA){
	for (int i = 0; i < numOfA; i++) {
		A(i, i) = A(i, i) + lambda;
		cout << A(i, i) << "\n";
	}
	return A;
}

void updateParam(Model *foldM, Eigen::VectorXd dir) {
	int count = 1;
	foldM->fold->topPosY += dir(0);
	int pointLastNum = foldM->fold->pointPosition.size() - 1;

	for (int k = 0; k < foldM->fold->pointPosition.size() - 1; k++) {
		for (int l = 0; l < foldM->fold->outlinepoints[k]->points.size(); l++) {
			if (l == 0) {
				Vec2 h;
				double x = dir(count); count++;
				double y = dir(count); count++;
				h.set(x, y);
				foldM->fold->pointPosition[k] += h;
				if (k == 0) foldM->fold->pointPosition[pointLastNum] = foldM->fold->pointPosition[0];
				continue;
			}
			if (l != foldM->fold->outlinepoints[k]->points.size() - 1) {//最後の点でなければx,y両方を動かせる
				Vec2 h;
				double x = dir(count); count++;
				double y = dir(count); count++;
				h.set(x, y);
				foldM->fold->outlinepoints[k]->points[l] += h;
			}
			else {//	最後の点はxだけ動かせる
				Vec2 h;
				double x = dir(count); count++;
				double y = 0;
				h.set(x, y);
				foldM->fold->outlinepoints[k]->points[l] += h;
			}
		}
		if (k == foldM->fold->pointPosition.size() - 2){
			Vec2 h;
			double x = 0;
			double y = dir(count);
			h.set(x, y);
			for (int m = 0; m < foldM->fold->pointPosition.size() - 1; m++) {
				int outlinepointsLastNum = foldM->fold->outlinepoints[m]->points.size() - 1;
				foldM->fold->outlinepoints[m]->points[outlinepointsLastNum] += h;
			}
		}
	}
	//calc
	updateBetweenPos(foldM);
}

Polyhedron_G Optimization(Model *foldM, COpenGL *fObj, Polyhedron_G *inputP, Nef_polyhedron_3 inputP_) {
	double lambda = 0.00001;
	double cost_ = penalty(foldM, fObj, inputP, inputP_);
	double prevCost = pow(cost_, 2) / 2;
	double difference;
	int itr = 2;
	int variableNum = 1;
	Polyhedron_G P;
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
		grad = Eigen::VectorXd::Zero(variableNum);
		jacobian = ComputeNumericalJacobian(foldM, fObj, inputP, inputP_, variableNum);
		prevCost = pow(penalty(foldM, fObj, inputP, inputP_), 2) / 2;
		hessian = jacobian * jacobian.transpose();
		Eigen::VectorXd delTas = penalty(foldM, fObj, inputP, inputP_)*jacobian;
		double topPos = foldM->fold->topPosY;
		while (1) {
			Eigen::MatrixXd A = computeLambda(hessian, lambda, variableNum);

			grad = retunDelta(A, grad, -penalty(foldM, fObj, inputP, inputP_)*jacobian);

			//backUp position
			for (int j = 0; j < foldM->fold->outlinepoints.size(); j++) {
				outlinepoints_tmp.push_back(foldM->fold->outlinepoints[j]->points);
			}
			pointPosition_tmp = foldM->fold->pointPosition;
			cout << "updateParam: " << "\n";
			cout << grad << "\n";
			//	パラメータをアップデート
			updateParam(foldM, -grad);
			cout << "updateParam: after" << "\n";
			cout << "prevCost: " << prevCost << "\n";
			double cost = penalty(foldM, fObj, inputP, inputP_);
			cost = (cost*cost) / 2;

			cout << "cost: " << cost << "\n";
			if (cost < prevCost) {
				difference = prevCost - cost;
				break;
			}
			else {
				// 退避したものを戻す
				lambda *= 10.0;
				if (lambda > 10.0) break;
				for (int j = 0; j < foldM->fold->outlinepoints.size(); j++) {
					foldM->fold->outlinepoints[j]->points = outlinepoints_tmp[j];
				}
				foldM->fold->pointPosition = pointPosition_tmp;
				foldM->fold->topPosY = topPos;
				outlinepoints_tmp.clear();
				pointPosition_tmp.clear();
			}
		}
		lambda /= 10;
		/*if (difference < 1.0e-5 && difference > 0) {
			cout << "difference: " << difference << "\n";
			break; 
		}*/
	}
	cout << "topYpos: " << foldM->fold->topPosY << "\n";
	cout << "Diff: " << penalty(foldM, fObj, inputP, inputP_) << "\n";
	for (int i = 0; i < foldM->fold->pointPosition.size(); i++){
		cout << "pointsPos: " << foldM->fold->pointPosition[i].x << "," << foldM->fold->pointPosition[i].y << "\n";
	}
	cout << "\n";
	for (int i = 0; i < foldM->fold->pointPosition.size() - 1; i++){
		cout << "outline: " << i << "\n";
		for (int j = 0; j < foldM->fold->outlinepoints[i]->points.size(); j++){
			cout << "outlinePos" << foldM->fold->outlinepoints[i]->points[j].x << "," << foldM->fold->outlinepoints[i]->points[j].y << "\n";
		}
		cout << "\n";
	}

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

	cout << "first cost: " << cost << "\n";
 
	Eigen::MatrixXf jacobian(2, constraintsNum);
	Eigen::MatrixXf hessian(constraintsNum, constraintsNum);
	Eigen::VectorXf grad(variableNum);

	for (int i = 0; i < itr; i++) {
		cout << "i: " << i << "\n";
		cout << "lambda: " << lambda << "\n";
		prevCost = penalty(u, x1, x2, 21) / 2;
		grad = Eigen::VectorXf::Zero(variableNum);
		jacobian = ComputeNumericalJacobian(u, x1, x2, 21);
		Eigen::VectorXf gradJ = Eigen::VectorXf::Zero(2);
		for (int k = 0; k < 21; k++) {
			float value = ((u[0] * pow((x1[k] - u[1]), 2) - x2[k]));
			gradJ(0) += value * jacobian(0, k);
			gradJ(1) += value * jacobian(1,k);
		}
		hessian = jacobian * jacobian.transpose();
		cout << "jacobian:\n" << jacobian << "\n\n";
		cout << "hessian: \n" << hessian << "\n\n";
		bool T = true;
		int L = 0;
		while (1) {
			Eigen::MatrixXf A;
			if (T) {
				A = computeLambda(hessian, lambda, variableNum);
			}
			else {
				A = computeLambdaUnit(hessian, lambda, variableNum);
			}
			cout << "A: \n" << A << "\n";
			cout << "gradJ: \n" << gradJ << "\n";
			//行列式を解く
			grad = retunDelta(A, grad, -1 * gradJ);
			cout << "deltau: \n" << grad << "\n";
			//	return;
			//値を保存しておく
			std::vector<float> u_;
			u_.push_back(u[0]);
			u_.push_back(u[1]);

			//	パラメータをアップデート
			u[0] += grad(0);
			u[1] += grad(1);
			
			float cost = penalty(u, x1, x2, 21) / 2;

			cout << "cost: " << cost << "\n";
			cout << "prevCost: " << prevCost << "\n";
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
	cout << "u = " << u[0] << "," << u[1] << "\n";
	
	return;
}

Eigen::VectorXd retunDelta(Eigen::MatrixXd M, Eigen::VectorXd V, Eigen::VectorXd b) {
	V =  M.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
	return V;
}

Eigen::VectorXf retunDelta(Eigen::MatrixXf M, Eigen::VectorXf V, Eigen::VectorXf b) {
	V = M.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
	return V;
}

Eigen::VectorXd retunDelta(Eigen::MatrixXd M, Eigen::VectorXd V, Eigen::VectorXd b, int Mrow) {
	Eigen::VectorXd curr;
	V = Eigen::VectorXd::Zero(Mrow);
	double diag;
	for (int j = 0; j < 1; j++) {
		for (int i = 0; i < Mrow; i++) {
			curr = M.row(i);
			diag = M(i, i);
			V(i) += (b(i) - V.dot(curr)) / diag;
			cout << "V(i):" << V(i) << "\n";
		}
	}
	cout << "V: " << V << "\n";
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
