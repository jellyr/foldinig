#include "defineData.h"
#define PIS 3.141526

/*
void  CalculateWorldCo(int x, int y, float depth,double &wx, double &wy,double &wz){
	// 4*4行列を二つ用意する
	GLdouble mvMatrix[16],pjMatrix[16];
	// ビューポート用配列
	GLint viewport[4];
	// パラメータを取得する．
	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, mvMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, pjMatrix);
	//世界座標を取得する
	gluUnProject((double)x,(double)viewport[3]-y,depth,mvMatrix,pjMatrix,viewport,&wx,&wy,&wz);
}
void  CalculateWindow(double x, double y, double z,double &wx, double &wy,double &wz){
	// 4*4行列を二つ用意する
	GLdouble mvMatrix[16],pjMatrix[16];
	// ビューポート用配列
	GLint viewport[4];
	// パラメータを取得する．
	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, mvMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, pjMatrix);
	//ウィンドウ座標を取得する
	gluProject(x,y,z,mvMatrix,pjMatrix,viewport,&wx,&wy,&wz);
}
*/
double Length(double ax,double ay,double bx,double by){
	return sqrt((ax-bx)*(ax-bx) + (ay-by)*(ay-by));

}

double cross(double ax,double ay,double bx,double by){
	return ax*by - ay*bx;
}

Vec3 calcmatrix(Vec3 tmp, double matrix[3][3]){
	Vec3 tmp2;

	tmp2.x = matrix[0][0]*tmp.x + matrix[0][1]*tmp.y + matrix[0][2]*tmp.z;
	tmp2.y = matrix[1][0]*tmp.x + matrix[1][1]*tmp.y + matrix[1][2]*tmp.z;
	tmp2.z = matrix[2][0]*tmp.x + matrix[2][1]*tmp.y + matrix[2][2]*tmp.z;
			
	return tmp2;
}

void setmatrix(double radian,double (*matrix)[3],Vec3 cross){//回転行列をセットする

	matrix[0][0]=cross.x*cross.x*(1-cos(radian))+cos(radian);
	matrix[0][1]=cross.x*cross.y*(1-cos(radian))-cross.z*sin(radian);
	matrix[0][2]=cross.z*cross.x*(1-cos(radian))+cross.y*sin(radian);
	matrix[1][0]=cross.x*cross.y*(1-cos(radian))+cross.z*sin(radian);
	matrix[1][1]=cross.y*cross.y*(1-cos(radian))+cos(radian);
	matrix[1][2]=cross.y*cross.z*(1-cos(radian))-cross.x*sin(radian);
	matrix[2][0]=cross.z*cross.x*(1-cos(radian)) - cross.y*sin(radian);
	matrix[2][1]=cross.y*cross.z*(1-cos(radian)) + cross.x*sin(radian);
	matrix[2][2]=cross.z*cross.z*(1-cos(radian)) + cos(radian);

}

Vec3 Angles(Vec3 nor1, Vec3 nor2){
	double matrix[3][3]; Vec3 re;
	Vec2 zero; zero.set(0,0);
	double xRotate,yRotate,zRotate;

	Vec2 xRotate1; xRotate1.set(nor1.y, nor1.z);
	Vec2 xRotate2; xRotate2.set(nor2.y, nor2.z);
	if(xRotate1.eq(zero) || xRotate2.eq(zero)){
		xRotate = 0;
	}else{
		xRotate = (xRotate1*xRotate2) /(xRotate1.length() * xRotate2.length());
		xRotate = acos(xRotate);//回転角
	}
	setmatrix(xRotate,matrix,Vec3(1.0,0,0));
	nor1 = calcmatrix(nor1,matrix);


	Vec2 yRotate1; yRotate1.set(nor1.x, nor1.z);
	Vec2 yRotate2; yRotate2.set(nor2.x, nor2.z);
	if(yRotate1.eq(zero) || yRotate2.eq(zero)){
		yRotate = 0;
	}else{
		yRotate = (yRotate1*yRotate2) /(yRotate1.length() * yRotate2.length());
		yRotate = acos(yRotate);//回転角
	}
	setmatrix(yRotate,matrix,Vec3(0.0,1.0,0));
	nor1 = calcmatrix(nor1,matrix);

	Vec2 zRotate1; zRotate1.set(nor1.x, nor1.y);
	Vec2 zRotate2; zRotate2.set(nor2.x, nor2.y);
	if(zRotate1.eq(zero) || zRotate2.eq(zero)){
		zRotate = 0;
	}else{
		zRotate = (zRotate1*zRotate2) /(zRotate1.length() * zRotate2.length());
		zRotate = acos(zRotate);//回転角
	}
	/*setmatrix(zRotate,matrix,Vec3(0.0,0,1.0));
	nor1 = calcmatrix(nor1,matrix);*/
	return Vec3(xRotate,yRotate,zRotate);
}

Vec3 Rotates(Vec3 angles ,Vec3 nor1){
	double matrix[3][3];

	setmatrix(angles.x,matrix,Vec3(1.0,0,0));
	nor1 = calcmatrix(nor1,matrix);

	setmatrix(angles.y,matrix,Vec3(0.0,1.0,0));
	nor1 = calcmatrix(nor1,matrix);

	
	setmatrix(angles.z,matrix,Vec3(0.0,0,1.0));
	nor1 = calcmatrix(nor1,matrix);

	return nor1;
}




