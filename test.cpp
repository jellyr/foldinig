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
 
	Eigen::MatrixXf jacobian(variableNum, constraintsNum);
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