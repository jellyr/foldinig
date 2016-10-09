#include "../../defineData.h"

std::vector<TrimPoints> getPointsForAnimation(std::vector<Vec2> pointPosition, std::vector<Vec2> betweenPosition, std::vector<outline*> outlinepoints);
double crossSectionLeft(Vec2 trimVecBase, Vec2 sweepVecBase, Vec2 trimRight, Vec2 sweepVecRight);
double crossSectionRight(Vec2 trimVecBase, Vec2 sweepVecBase, Vec2 trimVecRight, Vec2 sweepVecRight);