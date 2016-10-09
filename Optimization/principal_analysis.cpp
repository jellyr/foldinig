#include "../common.h"
/****************************/
/* �听������               */
/*      coded by Y.Suganuma */
/****************************/

#include "../defineData.h"

void pr_main(int dim, Model *m)
{
	double **x, *r, **a;
	int i1, i2, n, p, sw;
	std::list<Vertexs*>::iterator it_v;

	//scanf("%d %d", &p, &n);   // �ϐ��̐��ƃf�[�^�̐�
	p = dim;
	n = (int)m->vertices.size();

	r = new double [p];
	x = new double * [p];
	a = new double * [p];
	for (i1 = 0; i1 < p; i1++) {
		x[i1]  = new double [n];
		a[i1]  = new double [p];
	}

	/*for (i1 = 0; i1 < n; i1++) {   // �f�[�^
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
			printf("�听�� %f", r[i1]);
			printf(" �W��");
			for (i2 = 0; i2 < p; i2++)
				printf(" %f", a[i1][i2]);
			printf("\n");
		}
	}
	else
		printf("***error  �������߂邱�Ƃ��ł��܂���ł���\n");

	for (i1 = 0; i1 < p; i1++) {
		delete [] x[i1];
		delete [] a[i1];
	}
	delete [] x;
	delete [] a;
	delete [] r;

}

/***********************************/
/* �听������                      */
/*      p : �ϐ��̐�               */
/*      n : �f�[�^�̐�             */
/*      x : �f�[�^                 */
/*      r : ���U�i�听���j         */
/*      a : �W��                   */
/*      eps : �������𔻒肷��K�� */
/*      ct : �ő�J��Ԃ���      */
/*      return : =0 : ����         */
/*               =1 : �G���[       */
/***********************************/
int principal(int p, int n, double **x, double *r, double **a, double eps, int ct)
{
	double **A1, **A2, **C, mean, **X1, **X2, s2;
	int i1, i2, i3, sw = 0;
					// �̈�̊m��
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
					// �f�[�^�̊��
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
					// ���U�����U�s��̌v�Z
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
					// �ŗL�l�ƌŗL�x�N�g���̌v�Z�i���R�r�@�j
	sw = Jacobi(p, ct, eps, C, A1, A2, X1, X2);

	if (sw == 0) {
		for (i1 = 0; i1 < p; i1++) {
			r[i1] = A1[i1][i1];
			for (i2 = 0; i2 < p; i2++)
				a[i1][i2] = X1[i2][i1];
		}
	}
					// �̈�̉��
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
/* ���Ώ̍s��̌ŗL�l�E�ŗL�x�N�g���i���R�r�@�j              */
/*      n : ����                                             */
/*      ct : �ő�J��Ԃ���                                */
/*      eps : �����������                                   */
/*      A : �ΏۂƂ���s��                                   */
/*      A1, A2 : ��ƈ�inxn�̍s��j�CA1�̑Ίp�v�f���ŗL�l   */
/*      X1, X2 : ��ƈ�inxn�̍s��j�CX1�̊e�񂪌ŗL�x�N�g�� */
/*      return : =0 : ����                                   */
/*               =1 : ��������                               */
/*      coded by Y.Suganuma                                  */
/*************************************************************/

int Jacobi(int n, int ct, double eps, double **A, double **A1, double **A2,
           double **X1, double **X2)
{
	double max, s, t, v, sn, cs;
	int i1, i2, k = 0, ind = 1, p = 0, q = 0;
					// �����ݒ�
	for (i1 = 0; i1 < n; i1++) {
		for (i2 = 0; i2 < n; i2++) {
			A1[i1][i2] = A[i1][i2];
			X1[i1][i2] = 0.0;
		}
		X1[i1][i1] = 1.0;
	}
					// �v�Z
	while (ind > 0 && k < ct) {
						// �ő�v�f�̒T��
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
						// ��������
							// ��������
		if (max < eps)
			ind = 0;
							// �������Ȃ�
		else {
								// ����
			s  = -A1[p][q];
			t  = 0.5 * (A1[p][p] - A1[q][q]);
			v  = fabs(t) / sqrt(s * s + t * t);
			sn = sqrt(0.5 * (1.0 - v));
			if (s*t < 0.0)
				sn = -sn;
			cs = sqrt(1.0 - sn * sn);
								// Ak�̌v�Z
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
								// Xk�̌v�Z
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
								// ���̃X�e�b�v��
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
