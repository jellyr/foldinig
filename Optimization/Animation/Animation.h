#include "../../defineData.h"

void Animation_adjustHeightJ(std::vector<Vec2> &points, std::vector<Vec2> p, double height);

void Animation_adjustHeight(std::vector<std::vector<std::vector<Vec2>>> &foldingStates);

void FK(std::vector<Vec2> points, std::vector<double> l,int l_length, int state, std::vector<Vec2> &p);

void adjustOnAxis(std::vector<Vec2> points);

//void setPartlinePoint(std::vector<state> p ,int index);