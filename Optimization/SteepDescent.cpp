#include "SteepDescent.h"

#define PI 3.14159263589

void method(double *x1, double *y1, double *x2, double *y2, double *lIdeal, int size){
	double *xt = new double[size];
	double *yt = new double[size];
	//cout << "size In method: " << size << "\n";
	int i=0;
	for(i=0; i<10000; i++){
		double *thr = new double[2*size];
		int iThr=0;
		for(int j=0; j<size; j++){
			double gradF[2];
			getGradF(x1,y1,x2,y2,lIdeal,j,size,gradF);
			double t=0.05;
			thr[iThr++] = gradF[0];
			thr[iThr++] = gradF[1];
			xt[j] = x1[j] - t*gradF[0];
			yt[j] = y1[j] - t*gradF[1];
			x1[j] = xt[j];
			y1[j] = yt[j];
		}
		double e = 0.01;

		if(distance(thr,2*size) < e){
			for(int j=0; j<size; j++){
				if(j<size-1){
					double l = sqrt((xt[j]-xt[j+1])*(xt[j]-xt[j+1]) + (yt[j]-yt[j+1])*(yt[j]-yt[j+1]));
				}
			}

			break;
		}

		for(int j=0; j<size; j++){
			x1[j] = xt[j];
			y1[j] = yt[j];
		}
	}

	delete [] xt;
	delete [] yt;
}

void getGradF(double *x1, double *y1, double *x2, double *y2, double *lIdeal, int num, int size, double *result){//勾配を求める
	double l=0;
	double dx=0,dy=0;
	double w1 = 0.001;
	double w2 = 1.0;

	if(num<=0 || num>=(size-1)){
		dx = 0;
		dy = 0;
		//cout << "\n\ndx ==0 dy == 0\n\n";
	}else{
		if(num == 1){
			l = sqrt((x1[1]-x1[0])*(x1[1]-x1[0]) + (y1[1]-y1[0])*(y1[1]-y1[0]));
			dx += w2*(2*(x1[1]-x1[0])-2*lIdeal[0]*(x1[1]-x1[0])/l);
			dy += w2*(2*(y1[1]-y1[0])-2*lIdeal[0]*(y1[1]-y1[0])/l);

			dx += w1*2*(x1[num]-x2[num]);
			dy += w1*2*(y1[num]-y2[num]);
		}
		l = sqrt((x1[num]-x1[num+1])*(x1[num]-x1[num+1]) + (y1[num]-y1[num+1])*(y1[num]-y1[num+1]));
		dx += w2*(2*(x1[num]-x1[num+1])-2*lIdeal[num]*(x1[num]-x1[num+1])/l);
		dy += w2*(2*(y1[num]-y1[num+1])-2*lIdeal[num]*(y1[num]-y1[num+1])/l);
		dx += w1*2*(x1[num]-x2[num]);
		dy += w1*2*(y1[num]-y2[num]);
		
	}

	result[0] = dx;
	result[1] = dy;

}


void getGradF_one(double *x1, double *y1, double *x2, double *y2, double *lIdeal, int num, int size, double *result){//勾配を求める
	double l=0;
	double dx=0,dy=0;
	double w1 = 0.001;
	double w2 = 1.0;

	if(num<=0 || num>=(size-1)){
		dx = 0;
		dy = 0;
		//cout << "\n\ndx ==0 dy == 0\n\n";
	}else{
		if(num == 1){
			l = sqrt((x1[1]-x1[0])*(x1[1]-x1[0]) + (y1[1]-y1[0])*(y1[1]-y1[0]));
			dx += w2*(2*(x1[1]-x1[0])-2*lIdeal[0]*(x1[1]-x1[0])/l);
			dy += w2*(2*(y1[1]-y1[0])-2*lIdeal[0]*(y1[1]-y1[0])/l);

			//dx += w1*2*(x1[num]-x2[num]);
			//dy += w1*2*(y1[num]-y2[num]);
		}
		l = sqrt((x1[num]-x1[num+1])*(x1[num]-x1[num+1]) + (y1[num]-y1[num+1])*(y1[num]-y1[num+1]));
		dx += w2*(2*(x1[num]-x1[num+1])-2*lIdeal[num]*(x1[num]-x1[num+1])/l);
		dy += w2*(2*(y1[num]-y1[num+1])-2*lIdeal[num]*(y1[num]-y1[num+1])/l);
		//dx += w1*2*(x1[num]-x2[num]);
		//dy += w1*2*(y1[num]-y2[num]);
		
	}

	result[0] = dx;
	result[1] = dy;

}

double distance(double *L, int lSize){

	double dist = 0;
	for(int i=0; i<lSize; i++){
		dist += L[i]*L[i];
	}
	dist = sqrt(dist);
	return dist;

}


void method_one(double *x1, double *y1, double *x2, double *y2, double *lIdeal, int size, int num){
	double xt;
	double yt;
	
	for(int i=0; i<10000; i++){
		double *thr = new double[2];
		int iThr=0;
		
		double gradF[2];
		getGradF_one(x1,y1,x2,y2,lIdeal,num,size,gradF);
		double t=0.05;
		thr[iThr++] = gradF[0];
		thr[iThr++] = gradF[1];
		xt = x1[num] - t*gradF[0];
		yt = y1[num] - t*gradF[1];
		x1[num] = xt;
		y1[num] = yt;
		//}
		double e = 0.01;

		if(distance(thr,2) < e){
			/*for(int j=0; j<size; j++){
				if(j<size-1){
					double l = sqrt((xt[j]-xt[j+1])*(xt[j]-xt[j+1]) + (yt[j]-yt[j+1])*(yt[j]-yt[j+1]));
				}
			}*/
			break;
		}
		x1[num] = xt;
		y1[num] = yt;
		/*for(int j=0; j<size; j++){
			x1[j] = xt[j];
			y1[j] = yt[j];
		}*/
	}

}

//Evaluaion values
void func1(std::vector<double*> output, std::vector<Vec2> points, double *sum){

	for(int j=0; j<(int)output.size(); j++){
		sum[j] = 0;
		for(int i=0; i<(int)points.size()-1; i++){
			Vec2 n;
			n = points[i+1]-points[i];
			n.rotate(PI/2);
			n.normalize();

			Vec2 n1;
			if(output[j][i]>0){
				n1 = Vec2(0.0,1.0);
			}else{
				n1 = Vec2(0.0,-1.0);
			}
			sum[j] += n*n1;
		}
	}
}

void func2(std::vector<double*> output, std::vector<Vec2> points, double *sum){

	double ext_bfr,ext_afr;
	for(int j=0; j<(int)output.size(); j++){
		sum[j] = 0;
		for(int i=0; i<(int)points.size()-2; i++){
			Vec2 n0;
			n0 = points[i+1]-points[i];
			n0.rotate(PI/2);
			n0.normalize();

			Vec2 n1;
			n1 = points[i+2]-points[i+1];
			n1.rotate(PI/2);
			n1.normalize();

			ext_bfr = n0.exterior(n1);
			double tmp = output[j][i] * output[j][i+1];
			if(tmp > 0){
				ext_afr = 0;
			}else{
				if(output[j][i]>0){
					ext_afr = 1.;
				}else{ 
					ext_afr = -1.;
				}
			}
			sum[j] += ext_bfr*ext_afr;
		}
	}
}

void func3(std::vector<double> gapV, std::vector<double*> output, double *evalu, std::vector<int> output_array_size){
	
	for(int i=0; i<(int)gapV.size(); i++){
		double sum = 0;
		int size = output_array_size[i];
		for(int j=0; j<(int)size; j++){
			sum += fabs(output[i][j]);
		}
		sum /= (double)size;
		evalu [i] = fabs(gapV[i]/sum);
	}
}

int uninfo(int index, double sum, int *sign, std::vector<double> l, std::vector<dR> &output, std::vector<double> &gap, std::vector<int> &output_array_size,int count){
	//cout << "index: " << index <<" : " << l.size() << "\n";
	int next = count;
	if(index == (int)l.size()){
		if(sign[0] > 0 && sign[(int)l.size()-1] < 0){
			//cout << "count : " << count << " ";
			dR tmp;
			tmp.dArray.resize((int)l.size());
			for(int i=0; i<(int)l.size(); i++){
				tmp.dArray[i] =  sign[i]*(double)l[i];
			}

			gap[count]=sum;
			output_array_size[count] = (int)l.size();
			output[count] = tmp;
			//count++;
			return count+1;
		}
	}else{
		sign[index] = 1;
		next = uninfo(index+1,sum+l[index],sign,l,output,gap,output_array_size,next);
		sign[index] = -1;
		next = uninfo(index+1,sum-l[index],sign,l,output,gap,output_array_size,next);
	}

	return next;
	
}

bool foldableV(double *output, Vec2 *points, int size, int points_size){//outputのサイズ
	double fold_sum = 0.0;
	double error = 1.0;

	fold_sum += points[0].x;
	for(int i=0; i<size; i++){
		double len = sqrt((points[i].x-points[i+1].x)*(points[i].x-points[i+1].x) + (points[i].y-points[i+1].y)*(points[i].y-points[i+1].y));
		if(output[i] > 0){
			fold_sum += len;
		}else{
			fold_sum -= len;
		}
		if(fold_sum+error < points[i+1].x){
			//cout << "false\n";
			return false;
		}
	}

	if(fabs(fold_sum-points[points_size-1].x) < error){
		return true;
	}else{
		return false;
	}

}

void desIndex(double *a, int *des, int size){
	int *tmp = new int[size];

	std::map<double, int> mp;
	for(int i=0; i<size; i++){
		mp[a[i]] = i;
	}

	//ソート

	std::map<double, int>::iterator it;
	int i=0;
	it = mp.begin();
	//cout << "\n";
	while(it != mp.end()){
		tmp[i] = it->second;
		i++;
		it++;
		//cout << "first" <<  it->first << "\n"; 
		//cout << "second" << it->second << "\n"; 
	}
	//cout << "\n";
	for(int j=0; j<size; j++){
		des[j] = tmp[size-1-j];
	}

	delete [] tmp;
}