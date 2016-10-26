#include "InputFile.h"
#include <iostream>

using namespace std;

void makeRandumColor(double *col){
	//RGBのうち1つが255,1つが0,1つが0～255
	int randumV = rand() % 256;
	double getColor[6][3] = {
		{1.0, randumV/255.0,0},
		{1.0, 0, randumV/255.0},
		{0, 1.0, randumV/255.0},
		{0, randumV/255.0, 1.0},
		{randumV/255.0, 1.0, 0},
		{randumV/255.0, 0, 1.0}
	};
	int getV = rand() % 6;
	col[0] = getColor[getV][0];
	col[1] = getColor[getV][1];
	col[2] = getColor[getV][2];
}

void InputData(std::string filename, GLData *data){

	ifstream openfile(filename);
	string token(BUFSIZ, '\0');//サイズはBUFSIZE、すべてを\0で初期化

	Model *now_m;
	outline *now_o;
	int readDataKind = -1;
	srand((unsigned int)time(NULL));//初期化

	Vec2 cent;

	while(! (openfile >> token).fail()){
		if(token == "Part"){//新しいモデルを生成
			Model *m = new Model();
			now_m = m;
			now_m->fold = new foldmethod();
			data->parts.push_back(m);
			openfile.ignore(BUFSIZ,'\n');
			//cout << "\nPart\n";
		}else if(token == "outlinePoint"){//新しいアウトラインを生成
			outline *out = new outline();
			makeRandumColor(out->color);
			//cout << out->color[0] << " " << out->color[1] << " " << out->color[2] << "\n";
			//色を自動的に生成
			now_m->fold->outlinepoints.push_back(out);
			now_o = out;
			openfile.ignore(BUFSIZ,'\n');
		}else if(token == "pointPosition"){
			now_o = nullptr;
			readDataKind = 0;
		}else if(token == "betweenPosition"){
			now_o = nullptr;
			readDataKind = 1;
		}else{
			if(now_o != nullptr){
				Vec2 addPoints;
				double x,y;
				openfile >> y;
				std::stringstream ss(token);
				ss >> x;
				addPoints.set(x,y);
				now_o->points.push_back(addPoints);
				now_o->first_points.push_back(addPoints);
				//cout << "points: " << x << "," << y << "\n";
			}else{
				if(readDataKind == 0){
					Vec2 posV;
					std::stringstream ss(token);
					ss >> posV.x;
					openfile >> posV.y;
					now_m->fold->pointPosition.push_back(posV);
					//cout << "pointPosition: " << posV.x << "," << posV.y << "\n";
				}else if(readDataKind == 1){
					Vec2 posV;
					std::stringstream ss(token);
					ss >> posV.x;
					openfile >> posV.y;
					now_m->fold->betweenPosition.push_back(posV);
					//cout << "betweenPosition: " << posV.x << "," << posV.y << "\n";
				}
			}
		}
	}
	
}

Model *InputData(){

	ifstream openfile("final1.txt");
	string token(BUFSIZ, '\0');//サイズはBUFSIZE、すべてを\0で初期化

	Model *now_m;
	outline *now_o;
	int readDataKind = -1;
	srand((unsigned int)time(NULL));//初期化

	Vec2 cent;

	while(! (openfile >> token).fail()){
		if(token == "Part"){//新しいモデルを生成
			Model *m = new Model();
			now_m = m;
			now_m->scale.set(1,1,1);
			now_m->fold = new foldmethod();
			openfile.ignore(BUFSIZ,'\n');
		}else if(token == "outlinePoint"){//新しいアウトラインを生成
			outline *out = new outline();
			makeRandumColor(out->color);
			//色を自動的に生成
			now_m->fold->outlinepoints.push_back(out);
			now_o = out;
			openfile.ignore(BUFSIZ,'\n');
		}else if(token == "pointPosition"){
			now_o = nullptr;
			readDataKind = 0;
		}else if(token == "betweenPosition"){
			now_o = nullptr;
			readDataKind = 1;
		}else{
			if(now_o != nullptr){
				Vec2 addPoints;
				double x,y;
				openfile >> y;
				std::stringstream ss(token);
				ss >> x;
				addPoints.set(x,y);
				now_o->points.push_back(addPoints);
				now_o->first_points.push_back(addPoints);
			}else{
				if(readDataKind == 0){
					Vec2 posV;
					std::stringstream ss(token);
					ss >> posV.x;
					openfile >> posV.y;
					now_m->fold->pointPosition.push_back(posV);
				}else if(readDataKind == 1){
					Vec2 posV;
					std::stringstream ss(token);
					ss >> posV.x;
					openfile >> posV.y;
					now_m->fold->betweenPosition.push_back(posV);
				}
			}
		}
	}
	
	foldmethod *fold;
	now_m->fold->topPosY = 171.975;
	fold = now_m->fold;
	double lines = (double)fold->outlinepoints.size();//全部あわせたやつを出力する
	int row_num = (int)sqrt(lines);

	fold->Topcent.set(0,0);
	for(int j=0; j<(int)fold->pointPosition.size(); j++){
		fold->Topcent += fold->pointPosition[j];
	}
	fold->Topcent /= (double)fold->pointPosition.size();

	
	//最大と最小を比べる
	for(int j=0; j<(int)fold->outlinepoints.size(); j++){
		Vec2 cen; cen.set(0,0);
		std::vector<Vec2> outline = fold->outlinepoints[j]->points;
		std::sort(outline.begin(),outline.end(),Vec2::compareVec2PredicateX);
		fold->outlinepoints[j]->maxX = abs(outline[0].x - outline[(int)outline.size()-1].x);
		
		std::sort(outline.begin(),outline.end(),Vec2::compareVec2PredicateY);
		fold->outlinepoints[j]->maxY = abs(outline[0].y - outline[(int)outline.size()-1].y);
	}
	now_m->angle.set(0,0,0);
	cout << "end\n";
	return now_m;
	
}

void InputData(Model *m){

	ifstream openfile("inputData_2.txt");
	string token(BUFSIZ, '\0');//サイズはBUFSIZE、すべてを\0で初期化

	Model *now_m;
	outline *now_o;
	int readDataKind = -1;
	srand((unsigned int)time(NULL));//初期化

	Vec2 cent;

	while (!(openfile >> token).fail()){
		if (token == "Part"){//新しいモデルを生成
			now_m = m;
			now_m->scale.set(1, 1, 1);
			now_m->fold = new foldmethod();
			openfile.ignore(BUFSIZ, '\n');
		}
		else if (token == "outlinePoint"){//新しいアウトラインを生成
			outline *out = new outline();
			makeRandumColor(out->color);
			//色を自動的に生成
			now_m->fold->outlinepoints.push_back(out);
			now_o = out;
			openfile.ignore(BUFSIZ, '\n');
		}
		else if (token == "pointPosition"){
			now_o = nullptr;
			readDataKind = 0;
		}
		else if (token == "betweenPosition"){
			now_o = nullptr;
			readDataKind = 1;
		}
		else{
			if (now_o != nullptr){
				Vec2 addPoints;
				double x, y;
				openfile >> y;
				std::stringstream ss(token);
				ss >> x;
				addPoints.set(x, y);
				now_o->points.push_back(addPoints);
				now_o->first_points.push_back(addPoints);
			}
			else{
				if (readDataKind == 0){
					Vec2 posV;
					std::stringstream ss(token);
					ss >> posV.x;
					openfile >> posV.y;
					now_m->fold->pointPosition.push_back(posV);
				}
				else if (readDataKind == 1){
					Vec2 posV;
					std::stringstream ss(token);
					ss >> posV.x;
					openfile >> posV.y;
					now_m->fold->betweenPosition.push_back(posV);
				}
			}
		}
	}

	foldmethod *fold;
	now_m->fold->topPosY = 141.0;
	fold = now_m->fold;
	double lines = (double)fold->outlinepoints.size();//全部あわせたやつを出力する
	int row_num = (int)sqrt(lines);

	fold->Topcent.set(0, 0);
	for (int j = 0; j<(int)fold->pointPosition.size(); j++){
		fold->Topcent += fold->pointPosition[j];
	}
	fold->Topcent /= (double)fold->pointPosition.size();


	//最大と最小を比べる
	for (int j = 0; j<(int)fold->outlinepoints.size(); j++){
		Vec2 cen; cen.set(0, 0);
		std::vector<Vec2> outline = fold->outlinepoints[j]->points;
		std::sort(outline.begin(), outline.end(), Vec2::compareVec2PredicateX);
		fold->outlinepoints[j]->maxX = abs(outline[0].x - outline[(int)outline.size() - 1].x);

		std::sort(outline.begin(), outline.end(), Vec2::compareVec2PredicateY);
		fold->outlinepoints[j]->maxY = abs(outline[0].y - outline[(int)outline.size() - 1].y);
	}
	now_m->angle.set(0, 0, 0);
}

void clusterOptimization(Model *m) {

}