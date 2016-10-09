#include "../../defineData.h"
#include "Animation.h"
#define PI 3.14159265359


double gapThr = 0.1;

void Animation_adjustHeightJ(std::vector<Vec2> &points, std::vector<Vec2> p, double height){
	//points[0]が底面上の点
	//points[MAX-1]が天頂面上の点（軸状になるように修正）
	//ArrayList<Vec2> points = new ArrayList<Vec2>();
	
	points.resize((int)p.size());
	for(int i=0;i<(int)p.size();i++){
		points[i] = Vec2(p[i].x, p[i].y);
	}
	int LOOP = 100;
	double pointOnAxis_x = 0.0;
	int init = points.size()-1;
	int end = 0;

	Vec2 pointOnAxis = Vec2(pointOnAxis_x, height);//スケルトンの先端(目標点)
	double differ=0.0;//Math.sqrt((pointOnAxis.x-points.get(init).x)*(pointOnAxis.x-points.get(init).x)+(pointOnAxis.y-points.get(init).y)*(pointOnAxis.y-points.get(init).y));
	for(int i=0;i<LOOP;i++){
		for(int j=init;j>=end+1;j--){
			Vec2 v1 = Vec2(points[init].x-points[j-1].x, points[init].y-points[j-1].y);//リンクの始端から終端へ向かう単位ベクトル
			Vec2 v2 = Vec2(pointOnAxis.x-points[j-1].x, pointOnAxis.y-points[j-1].y);//リンクの始端から目標位置へ向かう単位ベクトル
			double cos = (v1*v2)/(v1.length()*v2.length());
			if(cos>1.0) cos=1.0;
			double theta = acos(cos);
			if(v1.exterior(v2)<0){theta *= -1.0;}

			for(int k=init;k>=j;k--){
				Vec2 r = Vec2(points[k].x-points[j-1].x, points[k].y-points[j-1].y);
				r.rotate(theta);
				Vec2 pa = Vec2(points[j-1].x+r.x, points[j-1].y+r.y);
				points[k] = pa;
			}
			differ = sqrt((pointOnAxis.x-points[init].x)*(pointOnAxis.x-points[init].x)+(pointOnAxis.y-points[init].y)*(pointOnAxis.y-points[init].y));
			if(differ<gapThr) break;
		}

		if(differ<gapThr) break;
	}
}

void Animation_adjustHeight(std::vector<std::vector<std::vector<Vec2>>> &foldingStates){
	int STEP_DIV = 20;
	int STEP = 100;
	int INIT = 0;
	int interval = STEP/STEP_DIV;

	//cout << "foldingStates: " << foldingStates.size() << "\n";
	std::vector<std::vector<std::vector<Vec2>>> foldingStatesModified;
	
	double height;
	foldingStatesModified.resize(STEP_DIV+1);

	for(int i=0; i<=STEP_DIV ;i++){
		//cout << i << "\n";
		//cout << "(int)foldingStates[i].size(): " << (int)foldingStates[i].size() << "\n";

		foldingStatesModified[i].resize((int)foldingStates[i].size());
		foldingStatesModified[i][0] = foldingStates[i*interval][0];
		height = foldingStates[i*interval][0][foldingStates[i*interval][0].size()-1].y;//基準となる高さ

		for(int j=1;j<(int)foldingStates[i].size();j++){
			int stepSelected = 0;
			int numM = foldingStates[0][j].size();
			double gapHeight = abs(height-foldingStates[0][j][numM-1].y);
			for(int k=0;k<(int)foldingStates.size();k++){
				if(abs(height-foldingStates[k][j][numM-1].y)<0.01){
					stepSelected = k;
					break;
				}else if(abs(height-foldingStates[k][j][numM-1].y)<gapHeight){
					gapHeight = abs(height-foldingStates[k][j][numM-1].y);
					stepSelected = k;
				}
			}
			std::vector<Vec2> points;
			Animation_adjustHeightJ(points, foldingStates[stepSelected][j], height);
			for(int k=0; k<(int)points.size(); k++){
				//cout <<"points: " <<  points[k].x << "," << points[k].y << "\n";
			}
			foldingStatesModified[i][j] = points;
		}
	}
	foldingStates = foldingStatesModified;
}

void FK(std::vector<Vec2> points, std::vector<double> l,int l_length, int state, std::vector<Vec2> &p){
	double STEP = 100;
//	std::vector<Vec2> p;
	Vec2 pN;
	pN = points[(int)points.size()-1];
	p.push_back(pN);
	//cout << "l_length: " << l_length << "\n";
	double *theta = new double[(int)l_length];
	Vec2 *lvecN_2 = new Vec2[(int)l_length];
	Vec2 *lvecN_1 = new Vec2[(int)l_length];

	theta[0] = 0.0;
	/*for(int i=0; i<(int)l.size(); i++){
		cout << "l: " << l[i] << "\n";
	}*/
	for(int i=0; i<(int)l_length; i++){
		lvecN_2[i] = Vec2(points[points.size()-i-2].x-points[points.size()-i-1].x, points[points.size()-i-2].y-points[points.size()-i-1].y);
		if(i>0){
			lvecN_1[i] = Vec2(points[points.size()-i-1].x-points[points.size()-i].x, points[points.size()-i-1].y-points[points.size()-i].y);
		}else{
			lvecN_1[i] = Vec2(1.0, 0.0);
		}
		double cos = (lvecN_2[i]*lvecN_1[i])/(lvecN_2[i].length()*lvecN_1[i].length());
		theta[i] = acos(cos);
	}

	double thetaSum = 0.0;
	for(int i=0;i<(int)l_length;i++){
		double lvecX = lvecN_2[i].x;
		double lvecY = lvecN_2[i].y;
		//cout << "theta[i]1: " << theta[i] << "\n";
		if(i>0){
			if(lvecN_2[i].exterior(lvecN_1[i])<0){		
				theta[i] *=-1.0;
			}
			//cout << "l: " << l[(int)l_length-i] << "," << l[(int)l_length-i-1] << "\n";
			if(l[(int)l_length-i]>=0 && l[(int)l_length-i-1]>=0){
				//cout << "1\n";
				theta[i] *= 1.0*state/STEP;
			}else if(l[(int)l_length-i]>=0 && l[(int)l_length-i-1]<0){
				//cout << "2\n";
				theta[i] = (PI+theta[i])*state/STEP;
			}else if(l[(int)l_length-i]<0 && l[(int)l_length-i-1]>=0){
				//cout << "3\n";
				theta[i] = -(PI-theta[i])*state/STEP;
			}else if(l[(int)l_length-i]<0 && l[(int)l_length-i-1]<0){
				//cout << "4\n";
				theta[i] *= 1.0*state/STEP;
			}
		}else{
			theta[i] *= 1.0*state/STEP;
		}
		lvecN_2[i].x = lvecX*cos(theta[i]+thetaSum)-lvecY*sin(theta[i]+thetaSum);
		lvecN_2[i].y = lvecX*sin(theta[i]+thetaSum)+lvecY*cos(theta[i]+thetaSum);
		Vec2 a = Vec2(p[i].x, p[i].y);
		a.x += lvecN_2[i].x;
		a.y += lvecN_2[i].y;
		
		p.push_back(a);
		//cout << "p[i].x: " << p[i].x << "," << p[i].y << "\n";
		/*cout << "a.x: " << a.x << ", a.y: " << a.y << "\n";
		cout << "lvecN_2[i]: " << lvecN_2[i].x << "," << lvecN_2[i].y << "\n";
		cout << "theta[i]2: " << theta[i] << "\n";*/
		thetaSum += theta[i];
	}
	if(abs(p[p.size()-1].x)>gapThr){
		adjustOnAxis(p);
	}
}

void adjustOnAxis(std::vector<Vec2> points){
	int LOOP = 100;
	double pointOnAxis_x = 0.0;
	int init = (int)points.size()-1;
	int end = 0;
	Vec2 pointOnAxis = Vec2(pointOnAxis_x, points[init].y);//スケルトンの先端(目標点)
	double differ=0.0;//Math.sqrt((pointOnAxis.x-points.get(init).x)*(pointOnAxis.x-points.get(init).x)+(pointOnAxis.y-points.get(init).y)*(pointOnAxis.y-points.get(init).y));
	for(int i=0;i<LOOP;i++){
		for(int j=init;j>=end+1;j--){
			Vec2 v1 = Vec2(points[init].x-points[j-1].x, points[init].y-points[j-1].y);//リンクの始端から終端へ向かう単位ベクトル
			Vec2 v2 = Vec2(pointOnAxis.x-points[j-1].x, pointOnAxis.y-points[j-1].y);//リンクの始端から目標位置へ向かう単位ベクトル
			double cos = (v1*v2)/(v1.length()*v2.length());
			if(cos>1.0) cos=1.0;
			double theta = acos(cos);
			if(v1.exterior(v2)<0){theta *= -1.0;}

			for(int k=init;k>=j;k--){
				Vec2 r = Vec2(points[k].x-points[j-1].x, points[k].y-points[j-1].y);
				r.rotate(theta);
				Vec2 pa = Vec2(points[j-1].x+r.x, points[j-1].y+r.y);
				points[k] = pa;
			}
			differ = sqrt((pointOnAxis.x-points[init].x)*(pointOnAxis.x-points[init].x)+(pointOnAxis.y-points[init].y)*(pointOnAxis.y-points[init].y));
			if(differ<gapThr) break;
		}

		if(differ<gapThr) break;
	}
}
