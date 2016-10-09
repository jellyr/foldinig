#pragma once
#include "common.h"
#include "Optimization/path/simplifypath.h"

using namespace std;

class Vertexs;
class Halfedge;
class Faces;
class Col;
class Model;
class GLData;
class outline;
class foldmethod;
class TrimPoint;
class TrimPoints;
class optimize_data;
class Vec2i;
class Vec2D;
class line;


class Vec3 {
public:
	double x, y, z;
	Vec3( void );
	Vec3( const double& _x, const double& _y, const double& _z );
	Vec3& operator = ( const Vec3& );
	Vec3& operator+=( const Vec3& );
	Vec3& operator-=( const Vec3& );
  
	double length();
	double length2();
	double angle(const Vec3& v);
	void set( const double& _x, const double& _y, const double& _z );
	void normalize();
	void debugout();
	Vec3(const Vec3& v);
	void set(const Vec3& v);
	void sub( const Vec3& v0, const Vec3& v1);
	void sub( const Vec3& v);
	void add( const Vec3& v);
	void scale( const double s);
	void cross( const Vec3& v);
	void cross(const double& xi, const double& yi, const double& zi);
	double dot( const Vec3& v);

	bool eq(Vec3 a){
		if(a.x == x && a.y == y && a.z == z){
			return true;
		}else{
			return false;
		}
	}

	//X中心にソート
	static bool compareVec3PredicateY(const Vec3 right, const Vec3 left){
		if(right.y <= left.y){
			return true;
		}else if(right.y > left.y){
			return false;
		}else{
			return true;
		}

	}
	
};

class Vec2{
public:
	double x,y;
	int xi,yi;
	Vec2(void);
	Vec2(const double& _x, const double& _y);
	Vec2(const int& _x, const int& _y);
	void set(const double& _x, const double& _y);
	void set(const int& _x, const int& _y);


	Vec2(const Vec2& _vec);
	Vec2& operator = (const Vec2&);
	Vec2& operator += (const Vec2&);
	Vec2& operator -= (const Vec2&);

	Vec2& operator *= (const double&);
	Vec2& operator /= (const double&);

	double length();
	double length2();
	void normalize();
	void rotate(const double theta);
	void projection(const Vec2& v);
	double exterior(const Vec2& v);
	void setLength(const double l);

	bool eq(Vec2 a){
		if(a.x == x && a.y == y){
			return true;
		}else{
			return false;
		}
	}

	//X中心にソート
	static bool compareVec2PredicateX(const Vec2 right, const Vec2 left){
		if(right.x <= left.x){
			return true;
		}else if(right.x > left.x){
			return false;
		}else{
			return true;
		}

	}

	//X中心にソート
	static bool compareVec2PredicateX_it(const Vec2* right, const Vec2* left){
		if(right->x <= left->x){
			return true;
		}else if(right->x > left->x){
			return false;
		}else{
			return true;
		}

	}

	static bool compareVec2SameX_it(const Vec2* right, const Vec2* left){
		return right->x == left->x;
	}
	//Y中心にソート
	static bool compareVec2PredicateY(const Vec2 right, const Vec2 left){
		if(right.y <= left.y){
			return true;
		}else if(right.y > left.y){
			return false;
		}else{
			return true;
		}
	}
	static bool compareVec2SameY(const Vec2 right, const Vec2 left){
		return right.y == left.y;
	}
};

inline Vec3::Vec3( const Vec3& v ) { x = v.x; y = v.y; z = v.z;}
inline Vec3::Vec3( void ) { x = 0; y = 0; z = 0;}
inline Vec3::Vec3( const double& _x, const double& _y, const double& _z ) { x = _x; y = _y; z = _z; }
inline Vec3& Vec3::operator =  ( const Vec3& v ){ x = v.x; y = v.y; z = v.z; return *this; }
inline Vec3 operator - ( const Vec3& v ) { return( Vec3( -v.x, -v.y, -v.z ) ); }
inline Vec3& Vec3::operator+=( const Vec3& v ) { x += v.x; y += v.y; z += v.z; return( *this ); }
inline Vec3& Vec3::operator-=( const Vec3& v ) { x -= v.x; y -= v.y; z -= v.z; return( *this ); }
inline Vec3 operator+( const Vec3& v1, const Vec3& v2 ) { return( Vec3( v1.x+v2.x, v1.y+v2.y, v1.z+v2.z ) );}
inline Vec3 operator-( const Vec3& v1, const Vec3& v2 ) { return( Vec3( v1.x-v2.x, v1.y-v2.y, v1.z-v2.z ) );}
inline Vec3 operator*( const double& k, const Vec3& v ) { return( Vec3( k*v.x, k*v.y, k*v.z ) );}
inline Vec3 operator*( const Vec3& v, const double& k ) { return( Vec3( v.x*k, v.y*k, v.z*k ) );}
inline Vec3 operator/( const Vec3& v, const double& k ) { return( Vec3( v.x/k, v.y/k, v.z/k ) );}
inline bool operator==(const Vec3& v1, const Vec3& v2) {if(v1.x == v2.x && v1.y == v2.y && v1.z == v2.z){return true;}else{return false;} }
//----  内積の定義
inline double operator*( const Vec3& v1, const Vec3& v2 ) { return( v1.x*v2.x + v1.y*v2.y + v1.z*v2.z );}
//----  外積の定義
inline Vec3  operator%( const Vec3& v1, const Vec3& v2 ) { return( Vec3( v1.y*v2.z - v1.z*v2.y,  v1.z*v2.x - v1.x*v2.z,  v1.x*v2.y - v1.y*v2.x ) );}
inline double Vec3::length() { return( sqrt( x*x + y*y + z*z ) );}
inline double Vec3::length2() { return( x*x + y*y + z*z  );}
inline void Vec3::normalize() { double l = length(); x/=l; y/=l; z/=l;}
inline void Vec3::debugout() { printf("Vec3(%f %f %f)\n", x, y, z );}
inline void Vec3::set( const double& _x, const double& _y, const double& _z ) { x = _x; y = _y; z = _z; }
inline double Vec3::angle(const Vec3& v) {
        Vec3 c = (*this)%v;
        return atan2(c.length(), (*this)*v);
}
inline void Vec3::cross(const Vec3& v){
	double x_,y_,z_;
	x_ = y*v.z - z*v.y;
	y_ = z*v.x - x*v.z;
	z_ = x*v.y - y*v.x;

	x = x_;
	y = y_;
	z = z_;
}

inline void Vec3::cross(const double& xi, const double& yi, const double& zi){
	double x_,y_,z_;
	x_ = y*zi - z*yi;
	y_ = z*xi - x*zi;
	z_ = x*yi - y*xi;

	x = x_;
	y = y_;
	z = z_;
}

inline Vec2::Vec2(const Vec2& v) {x = v.x; y = v.y;}
inline Vec2::Vec2(void) {x = 0; y = 0;}
inline Vec2::Vec2(const double& _x, const double& _y) {x = _x; y = _y;}
inline Vec2& Vec2::operator = (const Vec2& v){x = v.x; y = v.y; return *this;}
inline Vec2 operator - (const Vec2& v){ return (Vec2(-v.x, -v.y));}
inline Vec2& Vec2::operator+=(const Vec2& v){ x += v.x; y += v.y; return (*this);}
inline Vec2& Vec2::operator-=(const Vec2& v){ x -= v.x; y -= v.y; return (*this);}
inline Vec2 operator+(const Vec2& v1, const Vec2& v2 ){ return( Vec2(v1.x + v2.x, v1.y + v2.y));}
inline Vec2& Vec2::operator*=(const double& k){ x *= k; y *= k; return(*this);}
inline Vec2 operator-(const Vec2& v1, const Vec2& v2 ){ return( Vec2(v1.x - v2.x, v1.y - v2.y));}
inline Vec2& Vec2::operator/=(const double& k){ x /= k, y /= k; return(*this);}
inline Vec2 operator*(const double& k, const Vec2& v1 ){ return( Vec2(k*v1.x, k*v1.y)) ;}
inline Vec2 operator*(const Vec2& v1, const double& k ){ return( Vec2(k*v1.x, k*v1.y)) ;}
inline Vec2 operator/(const Vec2& v1, const double& k ){ return( Vec2(v1.x/k, v1.y/k)) ;}
inline bool operator==(const Vec2& v1, const Vec2& v2){if(v1.x == v2.x && v1.y == v2.y){return true;}else{return false;}}
//inline bool operator!=(const Vec2& v1, const Vec2& v2){if(v1.x != v2.x || v1.y != v2.y){return true;}else{return false;}}
//------内積-----------------
inline double operator*(const Vec2& v1, const Vec2& v2){return(v1.x*v2.x + v1.y*v2.y);}
inline double Vec2::exterior(const Vec2& v){return (x*v.y - y*v.x);};
//------外積-----------------
inline double operator%(const Vec2& v1, const Vec2& v2 ) { return(v1.x*v2.y-v1.y*v2.x);}
inline double Vec2::length() {return sqrt(x*x + y*y);}
inline double Vec2::length2() {return (x*x + y*y);}
inline void Vec2::normalize(){double l = length(); x/=l;y/=l;}
inline void Vec2::set(const double &_x,const double &_y){x = _x; y = _y;}
inline void Vec2::set(const int &_x,const int &_y){x = (double)_x; y = (double)_y;}
inline void Vec2::rotate(const double theta){double x1,y1; x1 = x; y1 = y; x = x1*cos(theta) - y1*sin(theta);y = x1*sin(theta) + y1*cos(theta);}
inline void Vec2::projection(const Vec2& v){double length,inner;  length=(v.x*x+v.y*y); inner=(*this)*v; (*this)=(*this)*(inner/length); }
inline void Vec2::setLength(const double l){normalize(); x *= l; y *= l;}

class TrimPoints{
public:
	std::vector<TrimPoint> trims;
};

class TrimPoint{
public: 
		int id;
		double alpha ;
		double l;
		int sameLayer;

		TrimPoint(){
			id = 0;
			alpha = 0.0;
			l = 0.0;
			sameLayer = 0;
		}

		TrimPoint(int id_, double alpha_, double l_){
			id = id_;
			alpha = alpha_;
			l = l_;
			sameLayer = 0;
		}
};

class outline{
public: 
	double color[3];
	std::vector<Vec2> points;
	std::vector<Vec2> first_points;
	double maxX,maxY,scaling;
	std::vector<double> foldingPattern;
	int foldingPattern_size;
	Vec3 firstP;
	Vec2 move_start;
	int color_num;

	Vec2 center(){
		Vec2 cen; cen.set(0,0);
		for(int i=0; i<(int)points.size(); i++){
			cen += points[i];
		}
		return cen / (double)points.size();
	}
};

class optimize_data{
public:
	std::vector<Vec2> optmized_points;
	double *optimized_pattern;
};

class state1{
	std::vector<Vec2> state1_p;
};

class state2{
	std::vector<std::vector<Vec2>> state2_p;
};

class foldmethod{//
public:
	std::vector<outline*> outlinepoints;
	std::vector<Vec2> pointPosition;
	std::vector<Vec2> betweenPosition;
	std::vector<TrimPoints> trimPoint;
	std::vector<std::vector<PointH>> vec_point;
	std::vector<outline*> outline_first;

	Vec2 Topcent;
	
	std::vector<std::vector<std::vector<Vec2>>> part_state;

	void move_zero(){
		Vec2 cent;
		for(int i=0; i<(int)pointPosition.size(); i++){
			cent -= pointPosition[i];
		}
		cent /= (double)pointPosition.size();

		for(int i=0; i<(int)pointPosition.size(); i++){
			pointPosition[i] -= cent;
		}
	}
};

class dR{
public:
	std::vector<double> dArray;
};

class Vertexs {
public:
	Vec3 p;
	Halfedge *halfedge;
	Vec3 normal;
	unsigned int num;

	std::vector<Halfedge*> v_half;

	Vertexs(double _x, double _y, double _z,unsigned int _num) {
		p.x = _x;
		p.y = _y;
		p.z = _z;
		halfedge = NULL;
		num = _num;
	}

};

class Halfedge {
public:
	Vertexs *vertex;
	Faces *face;
	Halfedge *pair;
	Halfedge *next;
	Halfedge *prev;
	Halfedge(Vertexs *v) {
		vertex = v;
		pair = nullptr;
		if(v->halfedge == NULL) {
			v->halfedge = this;
		}
	}
};

class Faces {
public:
	Halfedge *halfedge;
	Vec3 normal;
	int num;
	float bbox[2][3];
	Vec3 bary;

	Faces(Halfedge *he,int _num) {
		halfedge = he;
		num = _num;
		//cout << num << " ";
	}

	void setNormal() {
		Vec3 p0 = halfedge->vertex->p;
		Vec3 p1 = halfedge->next->vertex->p;
		Vec3 p2 = halfedge->next->next->vertex->p;

		normal = (p1 - p0) % (p2 - p0);
		normal.normalize();
	}

	float center(int axis){//(0:x, 1:y, 2:z)
		//中心座標を返す
		float cent[3];
		cent[0] = (bbox[0][0] + bbox[1][0])/2;
		cent[1] = (bbox[0][1] + bbox[1][1])/2;
		cent[2] = (bbox[0][2] + bbox[1][2])/2;

		if(axis == 0){//x,y,z座標で比較
			return cent[0];
		}else if(axis == 1){
			return cent[1];
		}else{
			return cent[2];
		}
	}

};

class Vec2D{
public: 
	Vec2 p;
	line *l1;//自分が視点
	line *l2;//自分が終点
	Vec2D(Vec2 p_){
		p = p_;
		l1 = nullptr;
		l2 = nullptr;
	}
};

class line{
public: 
	int num;
	Vec2D *start;
	Vec2D *end;
	Vec2 normal;
	Vec2 cross;

	line(int num_, Vec2D *start_, Vec2D *end_){
		num = num_;
		start = start_;
		end = end_;
		start_->l1 = this;
		end_->l2 = this;
	}

	void setNormal(Vec2 P){//点PからABへの最短距離
		Vec2 ABnor = end->p-start->p; ABnor.normalize();
		double Ax = ABnor * (P-start->p);
		Vec2 x = start->p + (ABnor*Ax);
		normal = x-P; normal.normalize();
		cross = x;
	}
	
};

class Vec2i{
public:
	Vec2 p;
	int index;

	Vec2i(Vec2 p_, int index_){
		p = p_;
		index = index_;
	}
};

class Model {
public:
	std::list<Faces*> faces;
	std::list<Vertexs*> vertices;
	std::list<Halfedge*> halfs;

	std::vector<Vec3> nor;
	std::vector<Vec3> face_cen;

	float color[3];
	float bbox[2][3];
	Vec3 cen;
	bool select;

	foldmethod *fold;

	Vec3 cent;
	Vec3 cent_move;
	Vec3 scale;
	Vec3 angle;

	double rotateAngleV_deg;
	double rotateAngleH_deg;
	int preMousePositionX;   // マウスカーソルの位置を記憶しておく変数
	int preMousePositionY;   // マウスカーソルの位置を記憶しておく変数

	std::vector<Vec3> plane_point;
	std::vector<Vec2> plane_point_2D;

	std::vector<Vec3> plane_point_bottom;
	std::vector<Vec2> plane_point_2D_bottom;
	
	std::vector<line*> convex_line;
	std::vector<line*> convex_line_top;

	std::vector<Vec2> cross_p;
	std::vector<Vec2> all_p;

	std::vector<Vec2> poly_p;//これがTopの座標

	Vec2 convex_cent;

	std::vector<std::vector<Vec3>> cut_point;
	std::vector<std::vector<Vec2>> cut_point_2D;
	std::vector<std::vector<Vec2>> Ideal;
	
	std::vector<std::vector<double>> length;
	std::vector<std::vector<Halfedge*>> null_edges;
private:
	void setHalfedgePair( Halfedge *he );

	// 二つのハーフエッジを互いに pair とする
	void setHalfedgePair(Halfedge *he0, Halfedge *he1);

public:

	// Faces のリストから face を削除する
	void deleteFace( Faces* face );

	// Vertexs のリストから vertex を削除する
	void deleteVertex( Vertexs *vertex );

	void init_model();//全ての要素を削除する

	void sizeNormalize(double m_size,Vec3 center);

	void makeEdge(unsigned int i,Halfedge *he);

	void addsetH();

	void addFace( Vertexs *v0, Vertexs *v1, Vertexs *v2 ,int f);

	void pointNor();

	void vertexColor(unsigned int select,double x, double y,int height,int del_flg);
	
	void resetColor(unsigned char key, int *select_flg);

	void move(Vec3 move);
	
	void rotate(double radian,Vec3 cross,Vec3 cen,Vec3 *d);
	
	void setAABB();
	
	void rotates(double radian,Vec3 cross,Vec3 cen,int flg,Vec3 *d);

	Model* makeShape(double r);
	
	void reducepolygon(int num);

	void Scaling(double value, double scale);


};

class GLData{
public:	//モデルなどの情報

	std::vector<Model*> parts;

	Vec3 camerapos;
	Vec3 cen;
	bool start;

	Vec3 plane_normal;
	Vec3 plane_cent;
	Vec3 plane_normal_Top;
	Vec3 plane_cent_Top;
	

	GLData(){
		start = false;
		plane_normal = Vec3(0,1,0);
		plane_cent = Vec3(0,0,0);
		plane_normal_Top = Vec3(0,1,0);
		plane_cent_Top = Vec3(0,0,0);
	}

};

float returnDepth(int x, int y);
void  CalculateWorldCo(int x, int y, float depth,double &wx, double &wy,double &wz);
void  CalculateWindow(double x, double y, double z,double &wx, double &wy,double &wz);
double Length(double ax,double ay,double bx,double by);
double cross(double ax,double ay,double bx,double by);
Vec3 calcmatrix(Vec3 tmp, double matrix[3][3]);
void setmatrix(double radian,double (*matrix)[3],Vec3 cross);
Vec3 Angles(Vec3 nor1, Vec3 nor2);
Vec3 Rotates(Vec3 angles ,Vec3 nor1);
