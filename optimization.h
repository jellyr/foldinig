#ifndef _INCLUDE_OPTIMIZATION_H_
#define _INCLUDE_OPTIMIZATION_H_
#include "defineData.h"
#include "cgal_defineData.h"
#include "foldingMethod.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/LU>


Eigen::MatrixXd ComputeNumericalJacobian(Cmodel *cm, COpenGL *fObj, int variableNum, int constraintNum);
Eigen::MatrixXf ComputeNumericalJacobian(std::vector<float> u, float *x1, float *x2, int num);

double penalty(Cmodel *cm, COpenGL *fObj);
float penalty(std::vector<float> u, float *x1, float *x2, int num);
float penalty_(std::vector<float> u, float *x1, float *x2, int num);

Eigen::MatrixXd computeLambda(Eigen::MatrixXd A, double lambda, int numOfA);
Eigen::MatrixXf computeLambda(Eigen::MatrixXf A, double lambda, int numOfA);
Eigen::MatrixXd computeLambdaUnit(Eigen::MatrixXd A, double lambda, int numOfA);

//	void Optimization(Model *foldM, COpenGL *fObj, Polyhedron_G *inputP, Nef_polyhedron_3 inputP_);
//Polyhedron_G Optimization(Model *foldM, COpenGL *fObj, Polyhedron_G *inputP, Nef_polyhedron_3 inputP_);
Polyhedron_G Optimization(Cmodel *cm, COpenGL *fObj);
void Optimization();

Eigen::VectorXd retunDelta(Eigen::MatrixXd M, Eigen::VectorXd b);
Eigen::VectorXd retunDelta(Eigen::MatrixXd M, Eigen::VectorXd V, Eigen::VectorXd b, int Mrow);
Eigen::VectorXf retunDelta(Eigen::MatrixXf M, Eigen::VectorXf V, Eigen::VectorXf b);
Eigen::MatrixXf computeLambdaUnit(Eigen::MatrixXf A, double lambda, int numOfA);

void updateBetweenPos(Model *foldM);

void testVolumeCalculation(Model *m);
void testVolumeCalculation(Polyhedron_G m);

Vec2 intersection_l(Vec2 a1, Vec2 a2, Vec2 b1, Vec2 b2);
double topConvex(Model *foldM);
double topSmoothing(Model *foldM);
double topConvex_area(Model *foldM);
double metro(Model *m, CMesh *input, CMesh *fold);

double foldingGap(Model *foldM);

Eigen::VectorXd eachPenalty(Cmodel *cm, COpenGL *fObj);
void setJacobian(Eigen::MatrixXd &jacobian, Eigen::VectorXd setP, Eigen::VectorXd fx_tmp, int constraintNum, int count, double invDelta);

void Step(double *output, double gap, std::vector<Vec2> points, int size, Vec2 *pointsN, bool One);
void outputFolding(Model *foldM);
double gapcalc(double *output, Vec2 *points, int size, int points_size);
double foldingGapDisp(Model *foldM);
#endif