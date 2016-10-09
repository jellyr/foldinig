#include "../common.h"
/****************************/
/* 主成分分析               */
/*      coded by Y.Suganuma */
/****************************/

#include "../defineData.h"

void pr_main(int dim, Model *m)
{
	double **x, *r, **a;
	int i1, i2, n, p, sw;
	std::list<Vertexs*>::iterator it_v;

	//scanf("%d %d", &p, &n);   // 変数の数とデータの数
	p = dim;
	n = (int)m->vertices.size();

	r = new double [p];
	x = new double * [p];
	a = new double * [p];
	for (i1 = 0; i1 < p; i1++) {
		x[i1]  = new double [n];
		a[i1]  = new double [p];
	}

	/*for (i1 = 0; i1 < n; i1++) {   // データ
		for (i2 = 0; i2 < p; i2++)
			//scanf("%lf", &x[i2][i1]);
	}*/

	int i = 0;
	for(it_v=m->vertices.begin();it_v!=m->vertices.end(); it_v++){
		x[0][i] = (*it_v)->p.x;
		x[1][i] = (*it_v)->p.y;
		x[2][i] = (*it_v)->p.z;
	}

	sw = principal(p, n, x, r, a, 1.0e-10, 200);

	if (sw == 0) {
		for (i1 = 0; i1 < p; i1++) {
			printf("主成分 %f", r[i1]);
			printf(" 係数");
			for (i2 = 0; i2 < p; i2++)
				printf(" %f", a[i1][i2]);
			printf("\n");
		}
	}
	else
		printf("***error  解を求めることができませんでした\n");

	for (i1 = 0; i1 < p; i1++) {
		delete [] x[i1];
		delete [] a[i1];
	}
	delete [] x;
	delete [] a;
	delete [] r;

}

/***********************************/
/* 主成分分析                      */
/*      p : 変数の数               */
/*      n : データの数             */
/*      x : データ                 */
/*      r : 分散（主成分）         */
/*      a : 係数                   */
/*      eps : 収束性を判定する規準 */
/*      ct : 最大繰り返し回数      */
/*      return : =0 : 正常         */
/*               =1 : エラー       */
/***********************************/
int principal(int p, int n, double **x, double *r, double **a, double eps, int ct)
{
	double **A1, **A2, **C, mean, **X1, **X2, s2;
	int i1, i2, i3, sw = 0;
					// 領域の確保
	C  = new double * [p];
	A1 = new double * [p];
	A2 = new double * [p];
	X1 = new double * [p];
	X2 = new double * [p];
	for (i1 = 0; i1 < p; i1++) {
		C[i1]  = new double [p];
		A1[i1] = new double [p];
		A2[i1] = new double [p];
		X1[i1] = new double [p];
		X2[i1] = new double [p];
	}
					// データの基準化
	for (i1 = 0; i1 < p; i1++) {
		mean = 0.0;
		s2   = 0.0;
		for (i2 = 0; i2 < n; i2++) {
			mean += x[i1][i2];
			s2   += x[i1][i2] * x[i1][i2];
		}
		mean /= n;
		s2   /= n;
		s2    = n * (s2 - mean * mean) / (n - 1);
		s2    = sqrt(s2);
		for (i2 = 0; i2 < n; i2++)
			x[i1][i2] = (x[i1][i2] - mean) / s2;
	}
					// 分散強分散行列の計算
	for (i1 = 0; i1 < p; i1++) {
		for (i2 = i1; i2 < p; i2++) {
			s2 = 0.0;
			for (i3 = 0; i3 < n; i3++)
				s2 += x[i1][i3] * x[i2][i3];
			s2        /= (n - 1);
			C[i1][i2]  = s2;
			if (i1 != i2)
				C[i2][i1] = s2;
		}
	}
					// 固有値と固有ベクトルの計算（ヤコビ法）
	sw = Jacobi(p, ct, eps, C, A1, A2, X1, X2);

	if (sw == 0) {
		for (i1 = 0; i1 < p; i1++) {
			r[i1] = A1[i1][i1];
			for (i2 = 0; i2 < p; i2++)
				a[i1][i2] = X1[i2][i1];
		}
	}
					// 領域の解放
	for (i1 = 0; i1 < p; i1++) {
		delete [] C[i1];
		delete [] A1[i1];
		delete [] A2[i1];
		delete [] X1[i1];
		delete [] X2[i1];
	}
	delete [] C;
	delete [] A1;
	delete [] A2;
	delete [] X1;
	delete [] X2;

	return sw;
}

/*************************************************************/
/* 実対称行列の固有値・固有ベクトル（ヤコビ法）              */
/*      n : 次数                                             */
/*      ct : 最大繰り返し回数                                */
/*      eps : 収束判定条件                                   */
/*      A : 対象とする行列                                   */
/*      A1, A2 : 作業域（nxnの行列），A1の対角要素が固有値   */
/*      X1, X2 : 作業域（nxnの行列），X1の各列が固有ベクトル */
/*      return : =0 : 正常                                   */
/*               =1 : 収束せず                               */
/*      coded by Y.Suganuma                                  */
/*************************************************************/

int Jacobi(int n, int ct, double eps, double **A, double **A1, double **A2,
           double **X1, double **X2)
{
	double max, s, t, v, sn, cs;
	int i1, i2, k = 0, ind = 1, p = 0, q = 0;
					// 初期設定
	for (i1 = 0; i1 < n; i1++) {
		for (i2 = 0; i2 < n; i2++) {
			A1[i1][i2] = A[i1][i2];
			X1[i1][i2] = 0.0;
		}
		X1[i1][i1] = 1.0;
	}
					// 計算
	while (ind > 0 && k < ct) {
						// 最大要素の探索
		max = 0.0;
		for (i1 = 0; i1 < n; i1++) {
			for (i2 = 0; i2 < n; i2++) {
				if (i2 != i1) {
					if (fabs(A1[i1][i2]) > max) {
						max = fabs(A1[i1][i2]);
						p   = i1;
						q   = i2;
					}
				}
			}
		}
						// 収束判定
							// 収束した
		if (max < eps)
			ind = 0;
							// 収束しない
		else {
								// 準備
			s  = -A1[p][q];
			t  = 0.5 * (A1[p][p] - A1[q][q]);
			v  = fabs(t) / sqrt(s * s + t * t);
			sn = sqrt(0.5 * (1.0 - v));
			if (s*t < 0.0)
				sn = -sn;
			cs = sqrt(1.0 - sn * sn);
								// Akの計算
			for (i1 = 0; i1 < n; i1++) {
				if (i1 == p) {
					for (i2 = 0; i2 < n; i2++) {
						if (i2 == p)
							A2[p][p] = A1[p][p] * cs * cs + A1[q][q] * sn * sn -
                                       2.0 * A1[p][q] * sn * cs;
						else if (i2 == q)
							A2[p][q] = 0.0;
						else
							A2[p][i2] = A1[p][i2] * cs - A1[q][i2] * sn;
					}
				}
				else if (i1 == q) {
					for (i2 = 0; i2 < n; i2++) {
						if (i2 == q)
							A2[q][q] = A1[p][p] * sn * sn + A1[q][q] * cs * cs +
                                       2.0 * A1[p][q] * sn * cs;
						else if (i2 == p)
							A2[q][p] = 0.0;
						else
							A2[q][i2] = A1[q][i2] * cs + A1[p][i2] * sn;
					}
				}
				else {
					for (i2 = 0; i2 < n; i2++) {
						if (i2 == p)
							A2[i1][p] = A1[i1][p] * cs - A1[i1][q] * sn;
						else if (i2 == q)
							A2[i1][q] = A1[i1][q] * cs + A1[i1][p] * sn;
						else
							A2[i1][i2] = A1[i1][i2];
					}
				}
			}
								// Xkの計算
			for (i1 = 0; i1 < n; i1++) {
				for (i2 = 0; i2 < n; i2++) {
					if (i2 == p)
						X2[i1][p] = X1[i1][p] * cs - X1[i1][q] * sn;
					else if (i2 == q)
						X2[i1][q] = X1[i1][q] * cs + X1[i1][p] * sn;
					else
						X2[i1][i2] = X1[i1][i2];
				}
			}
								// 次のステップへ
			k++;
			for (i1 = 0; i1 < n; i1++) {
				for (i2 = 0; i2 < n; i2++) {
					A1[i1][i2] = A2[i1][i2];
					X1[i1][i2] = X2[i1][i2];
				}
			}
		}
	}

	for (i1 = 0; i1 < n; i1++) {
		cout << X1[i1][0] << "," << X1[i1][1] << "," << X1[i1][2] << "\n";
	}

	return ind;
}
