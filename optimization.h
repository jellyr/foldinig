#ifndef _INCLUDE_OPTIMIZATION_H_
#define _INCLUDE_OPTIMIZATION_H_
#include "defineData.h"
#include "cgal_defineData.h"
#include "foldingMethod.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>


Eigen::VectorXd ComputeNumericalJacobian(Model *foldM, COpenGL *fObj, Polyhedron_G *inputP, Nef_polyhedron_3 inputP_, int variableNum);
Eigen::MatrixXf ComputeNumericalJacobian(std::vector<float> u, float *x1, float *x2, int num);

double penalty(Model *foldM, COpenGL *fObj, Polyhedron_G * inputP, Nef_polyhedron_3 inputP_);
float penalty(std::vector<float> u, float *x1, float *x2, int num);
float penalty_(std::vector<float> u, float *x1, float *x2, int num);

Eigen::MatrixXd computeLambda(Eigen::MatrixXd A, double lambda, int numOfA);
Eigen::MatrixXf computeLambda(Eigen::MatrixXf A, double lambda, int numOfA);
Eigen::MatrixXd computeLambdaUnit(Eigen::MatrixXd A, double lambda, int numOfA);

//	void Optimization(Model *foldM, COpenGL *fObj, Polyhedron_G *inputP, Nef_polyhedron_3 inputP_);
Polyhedron_G Optimization(Model *foldM, COpenGL *fObj, Polyhedron_G *inputP, Nef_polyhedron_3 inputP_);
void Optimization();

Eigen::VectorXd retunDelta(Eigen::MatrixXd M, Eigen::VectorXd V, Eigen::VectorXd b);
Eigen::VectorXd retunDelta(Eigen::MatrixXd M, Eigen::VectorXd V, Eigen::VectorXd b, int Mrow);
Eigen::VectorXf retunDelta(Eigen::MatrixXf M, Eigen::VectorXf V, Eigen::VectorXf b);
Eigen::MatrixXf computeLambdaUnit(Eigen::MatrixXf A, double lambda, int numOfA);

void updateBetweenPos(Model *foldM);

void testVolumeCalculation(Model *m);
void testVolumeCalculation(Polyhedron_G m);

#endif