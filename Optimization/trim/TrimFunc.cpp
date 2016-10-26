#include "../../common.h"
#include "Trimfunc.h"

#define PI 3.14159265359

std::vector<TrimPoints> getPointsForAnimation(std::vector<Vec2> pointPosition, std::vector<Vec2> betweenPosition, std::vector<outline*> outlinepoints){
	std::vector<TrimPoints> trimPoint;

	int size = (int)outlinepoints.size();
	cout << "size: " << size << "\n";
	for(int i=0; i<size; i++){

		int iRight = (i+1)%size;
		int iRight1 = (i+2)%size;
		int iLeft = (i+size-1)%size;

		int id = 0;
		double alpha = 0.0;
		double l = 0.0;

		Vec2 trimVecBase = pointPosition[iRight]-pointPosition[i];
		Vec2 trimVecRight = pointPosition[iRight1]-pointPosition[iRight];
		Vec2 trimVecLeft = pointPosition[i]-pointPosition[iLeft];
		Vec2 sweepVecBase = pointPosition[iRight]-pointPosition[i];//注目する（基準となる）断面線のスイープ方向
		Vec2 sweepVecRight = pointPosition[iRight1]-pointPosition[iRight];//右隣の断面線のスイープ方向
		Vec2 sweepVecLeft = pointPosition[i]-pointPosition[iLeft];//左隣の断面線のスイープ方向
		
		trimVecBase.rotate(PI/2.0);
		trimVecRight.rotate(PI/2.0);
		trimVecLeft.rotate(PI/2.0);

		Vec2 bar,baz;
		double foo;

		TrimPoints trimList;
		//右側のトリム処理
		for(int j=0; j<(int)outlinepoints[i]->points.size()-1; j++){
			for(int k=0; k<(int)outlinepoints[iRight]->points.size(); k++){

				if(((outlinepoints[i]->points[j].y <= outlinepoints[iRight]->points[k].y && outlinepoints[iRight]->points[k].y <= outlinepoints[i]->points[j+1].y)) || 
					((outlinepoints[i]->points[j].y > outlinepoints[iRight]->points[k].y && outlinepoints[iRight]->points[k].y > outlinepoints[i]->points[j+1].y))){
					id = j+1;
					alpha = fabs((outlinepoints[iRight]->points[k].y-outlinepoints[i]->points[j].y))/fabs(outlinepoints[i]->points[j+1].y-outlinepoints[i]->points[j].y);
					
					foo = alpha*(outlinepoints[i]->points[j+1].x - outlinepoints[i]->points[j].x) + outlinepoints[i]->points[j].x;
					bar = trimVecBase;
					bar.setLength(foo);
					bar += betweenPosition[i];

					baz = trimVecRight;
					baz.setLength(outlinepoints[iRight]->points[k].x);
					baz += betweenPosition[iRight];
					
					l=crossSectionRight(bar,sweepVecBase,baz,sweepVecRight);
					trimList.trims.push_back(TrimPoint(id,alpha,l));
				}

				if(j>0 && k<(int)outlinepoints[iRight]->points.size()-1){
					if(((outlinepoints[iRight]->points[k].y <= outlinepoints[i]->points[j].y && outlinepoints[i]->points[j].y<=outlinepoints[iRight]->points[k+1].y)) ||
						((outlinepoints[iRight]->points[k].y>outlinepoints[i]->points[j].y&& outlinepoints[i]->points[j].y>outlinepoints[iRight]->points[k+1].y))){
						id = j+1;
						alpha = fabs(outlinepoints[i]->points[j].y-outlinepoints[iRight]->points[k].y)/fabs(outlinepoints[iRight]->points[k+1].y-outlinepoints[iRight]->points[k].y);
						foo = alpha*(outlinepoints[iRight]->points[k+1].x-outlinepoints[iRight]->points[k].x) + outlinepoints[iRight]->points[k].x ;
						bar = trimVecRight;
						bar.setLength(foo);
						bar += betweenPosition[iRight];

						baz = trimVecBase ;
						baz.setLength(outlinepoints[i]->points[j].x);
						baz += betweenPosition[i];

						l = crossSectionRight(baz,sweepVecBase,bar,sweepVecRight);
						trimList.trims.push_back(TrimPoint(id-1,1.0,l));
						trimList.trims.push_back(TrimPoint(id,0.0,l));
					}
				}
			}
		}

		//左側のトリム処理
		for(int j=0; j<(int)outlinepoints[i]->points.size()-1; j++){
			for(int k=0; k<(int)outlinepoints[iLeft]->points.size(); k++){
				if(((outlinepoints[i]->points[j].y <= outlinepoints[iLeft]->points[k].y && outlinepoints[iLeft]->points[k].y <= outlinepoints[i]->points[j+1].y)) || 
					((outlinepoints[i]->points[j].y > outlinepoints[iLeft]->points[k].y && outlinepoints[iLeft]->points[k].y > outlinepoints[i]->points[j+1].y))){
					id = -(j+1);
					alpha = fabs((outlinepoints[iLeft]->points[k].y-outlinepoints[i]->points[j].y))/fabs(outlinepoints[i]->points[j+1].y-outlinepoints[i]->points[j].y);
					
					foo = alpha*(outlinepoints[i]->points[j+1].x - outlinepoints[i]->points[j].x) + outlinepoints[i]->points[j].x;
					bar = trimVecBase;
					bar.setLength(foo);
					bar += betweenPosition[i];

					baz = trimVecLeft;
					baz.setLength(outlinepoints[iLeft]->points[k].x);
					baz += betweenPosition[iLeft];
					
					l = crossSectionLeft(bar,sweepVecBase,baz,sweepVecLeft);
					trimList.trims.push_back(TrimPoint(id,alpha,l));
				}

				if(j>0 && k<(int)outlinepoints[iLeft]->points.size()-1){
					if(((outlinepoints[iLeft]->points[k].y <= outlinepoints[i]->points[j].y && outlinepoints[i]->points[j].y<=outlinepoints[iLeft]->points[k+1].y)) ||
						((outlinepoints[iLeft]->points[k].y>outlinepoints[i]->points[j].y&& outlinepoints[i]->points[j].y>outlinepoints[iLeft]->points[k+1].y))){
						id = -(j+1);
						alpha = fabs(outlinepoints[i]->points[j].y-outlinepoints[iLeft]->points[k].y)/fabs(outlinepoints[iLeft]->points[k+1].y-outlinepoints[iLeft]->points[k].y);
						foo = alpha*(outlinepoints[iLeft]->points[k+1].x-outlinepoints[iLeft]->points[k].x) + outlinepoints[iLeft]->points[k].x ;
						bar = trimVecLeft;
						bar.setLength(foo);
						bar += betweenPosition[iLeft];

						baz = trimVecBase ;
						baz.setLength(outlinepoints[i]->points[j].x);
						baz += betweenPosition[i];

						l = crossSectionLeft(baz,sweepVecBase,bar,sweepVecLeft);
						trimList.trims.push_back(TrimPoint(id+1,1.0,l));
						trimList.trims.push_back(TrimPoint(id,0.0,l));
					}
				}
			}
		}
		trimPoint.push_back(trimList);
	}

	return trimPoint;
}

double crossSectionLeft(Vec2 trimVecBase, Vec2 sweepVecBase, Vec2 trimVecRight, Vec2 sweepVecRight){
	double s1,s2;
	Vec2 point;

	Vec2 p0 = trimVecBase;
	Vec2 p1 = trimVecRight;
	Vec2 p2 = trimVecBase + sweepVecBase;
	Vec2 p3 = trimVecRight + sweepVecRight;

	s1 = -((p0.x-p1.x)*(p3.y-p1.y)-(p0.y-p1.y)*(p3.x-p1.x))/2.0;//p0-p1とp3-p1の外積
	s2 = ((p3.x-p1.x)*(p1.y-p2.y)-(p3.y-p1.y)*(p1.x-p2.x))/2.0;//p3-p1とp1-p2の外積

	if(abs(s1+s2) < 0.01){
		point.x = p0.x;
		point.y = p0.y;
	}else{
		point.x = p0.x+(p2.x-p0.x)*s1/(s1+s2);
		point.y = p0.y+(p2.y-p0.y)*s1/(s1+s2); 
	}

	p0 -= point;
	if(p0 * sweepVecBase >= 0){
		return p0.length();
	}else{
		return -p0.length();
	}
}

double crossSectionRight(Vec2 trimVecBase, Vec2 sweepVecBase, Vec2 trimVecRight, Vec2 sweepVecRight){
	double s1,s2;
	Vec2 point;
	Vec2 p0 = trimVecBase;
	Vec2 p1 = trimVecRight;
	Vec2 p2 = trimVecBase + sweepVecBase;
	Vec2 p3 = trimVecRight + sweepVecRight;

	s1 = -((p0.x-p1.x)*(p3.y-p1.y)-(p0.y-p1.y)*(p3.x-p1.x))/2.0;//p0-p1とp3-p1の外積
	s2 = ((p3.x-p1.x)*(p1.y-p2.y)-(p3.y-p1.y)*(p1.x-p2.x))/2.;//p3-p1とp1-p2の外積

	if(abs(s1+s2) < 0.01){
		point.x = p0.x;
		point.y = p0.y;
	}else{
		point.x = p0.x+(p2.x-p0.x)*s1/(s1+s2);
		point.y = p0.y+(p2.y-p0.y)*s1/(s1+s2);
	}

	p0 -= point;
	if(p0 * sweepVecBase <= 0){
		return p0.length();
	}else{
		return -p0.length();
	}

}
