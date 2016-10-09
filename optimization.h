#include "defineData.h"
#include "cgal_defineData.h"
#include "foldingMethod.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>


Eigen::VectorXd ComputeNumericalJacobian(Model *foldM, COpenGL *fObj, Polyhedron_G *inputP, Nef_polyhedron_3 inputP_);

double penalty(Model *foldM, COpenGL *fObj, Polyhedron_G * inputP, Nef_polyhedron_3 inputP_);

Eigen::MatrixXd computeLambda(Eigen::MatrixXd A, double lambda, int numOfA);

void Optimization(Model *foldM, COpenGL *fObj, Polyhedron_G *inputP, Nef_polyhedron_3 inputP_);

Eigen::VectorXd retunGradient(Eigen::MatrixXd M, Eigen::VectorXd V, Eigen::VectorXd b, int Mrow);

void updateBetweenPos(Model *foldM);