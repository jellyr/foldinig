#include "../defineData.h"


void method(double *x1, double *y1, double *x2, double *y2, double *lIdeal, int size);
void method_one(double *x1, double *y1, double *x2, double *y2, double *lIdeal, int size, int num);
void getGradF(double *x1, double *y1, double *x2, double *y2, double *lIdeal, int num, int size, double *result);
void getGradF_one(double *x1, double *y1, double *x2, double *y2, double *lIdeal, int num, int size, double *result);
double distance(double *L, int lSize);

void func1(std::vector<double*> output, std::vector<Vec2> points, double *sum);
void func2(std::vector<double*> output, std::vector<Vec2> points, double *sum);
void func3(std::vector<double> gapV, std::vector<double*> output, double *evalu, std::vector<int> output_array_size);

int uninfo(int index, double sum, int *sign, std::vector<double> l, std::vector<dR> &output, std::vector<double> &gap, std::vector<int> &output_array_size,int count);
bool foldableV(double *output, Vec2 *points, int size ,int points_size);
void desIndex(double *a, int *dex, int size);