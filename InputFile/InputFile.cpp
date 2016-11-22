#include "InputFile.h"
#include <iostream>

using namespace std;

void makeRandumColor(double *col){
	//RGB�̂���1��255,1��0,1��0�`255
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
	string token(BUFSIZ, '\0');//�T�C�Y��BUFSIZE�A���ׂĂ�\0�ŏ�����

	Model *now_m;
	outline *now_o;
	int readDataKind = -1;
	srand((unsigned int)time(NULL));//������

	Vec2 cent;

	while(! (openfile >> token).fail()){
		if(token == "Part"){//�V�������f���𐶐�
			Model *m = new Model();
			now_m = m;
			now_m->fold = new foldmethod();
			data->parts.push_back(m);
			openfile.ignore(BUFSIZ,'\n');
			//cout << "\nPart\n";
		}else if(token == "outlinePoint"){//�V�����A�E�g���C���𐶐�
			outline *out = new outline();
			makeRandumColor(out->color);
			//cout << out->color[0] << " " << out->color[1] << " " << out->color[2] << "\n";
			//�F�������I�ɐ���
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

foldmethod InputFold(std::string path) {
	ifstream openfile(path.c_str());

	string token(BUFSIZ, '\0');//�T�C�Y��BUFSIZE�A���ׂĂ�\0�ŏ�����

	foldmethod fold;
	fold.topPosY = 0;
	outline *now_o;
	int readDataKind = -1;
	srand((unsigned int)time(NULL));//������

	Vec2 cent;
	while (!(openfile >> token).fail()){
		if (token == "Part"){//�V�������f���𐶐�
			openfile.ignore(BUFSIZ, '\n');
		}
		else if (token == "TopposY:"){
			double posY;
			openfile >> posY;
			fold.topPosY = posY;
			openfile.ignore(BUFSIZ, '\n');
		}
		else if (token == "outlinePoint"){//�V�����A�E�g���C���𐶐�
			outline *out = new outline();
			makeRandumColor(out->color);
			//�F�������I�ɐ���
			fold.outlinepoints.push_back(out);
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
					fold.pointPosition.push_back(posV);
				}
				else if (readDataKind == 1){
					Vec2 posV;
					std::stringstream ss(token);
					ss >> posV.x;
					openfile >> posV.y;
					fold.betweenPosition.push_back(posV);
				}
			}
		}
	}

	return fold;
}

Model *InputData(){

	ifstream openfile("bunny\\bunny_first\\ear_.txt");
	cout << "??\n";
	string token(BUFSIZ, '\0');//�T�C�Y��BUFSIZE�A���ׂĂ�\0�ŏ�����

	Model *now_m;
	outline *now_o;
	int readDataKind = -1;
	srand((unsigned int)time(NULL));//������

	Vec2 cent;

	while(! (openfile >> token).fail()){
		if(token == "Part"){//�V�������f���𐶐�
			cout << "Part\n";
			Model *m = new Model();
			now_m = m;
			now_m->scale.set(1,1,1);
			now_m->fold = new foldmethod();
			openfile.ignore(BUFSIZ,'\n');
		}
		else if (token == "TopposY"){
			cout << "topPosy\n";
			double posY;
			openfile >> posY;
			now_m->fold->topPosY = posY;
			cout << "topPosy " << posY << "\n";
			openfile.ignore(BUFSIZ, '\n');
		}
		else if (token == "outlinePoint"){//�V�����A�E�g���C���𐶐�
			cout << "out\n";
			outline *out = new outline();
			makeRandumColor(out->color);
			//�F�������I�ɐ���
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
	//now_m->fold->topPosY = 18.5368;
	fold = now_m->fold;
	double lines = (double)fold->outlinepoints.size();//�S�����킹������o�͂���
	int row_num = (int)sqrt(lines);

	fold->Topcent.set(0,0);
	for(int j=0; j<(int)fold->pointPosition.size(); j++){
		fold->Topcent += fold->pointPosition[j];
	}
	fold->Topcent /= (double)fold->pointPosition.size();

	
	//�ő�ƍŏ����ׂ�
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
	string token(BUFSIZ, '\0');//�T�C�Y��BUFSIZE�A���ׂĂ�\0�ŏ�����

	Model *now_m;
	outline *now_o;
	int readDataKind = -1;
	srand((unsigned int)time(NULL));//������

	Vec2 cent;

	while (!(openfile >> token).fail()){
		if (token == "Part"){//�V�������f���𐶐�
			now_m = m;
			now_m->scale.set(1, 1, 1);
			now_m->fold = new foldmethod();
			openfile.ignore(BUFSIZ, '\n');
		}
		else if (token == "outlinePoint"){//�V�����A�E�g���C���𐶐�
			outline *out = new outline();
			makeRandumColor(out->color);
			//�F�������I�ɐ���
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
	double lines = (double)fold->outlinepoints.size();//�S�����킹������o�͂���
	int row_num = (int)sqrt(lines);

	fold->Topcent.set(0, 0);
	for (int j = 0; j<(int)fold->pointPosition.size(); j++){
		fold->Topcent += fold->pointPosition[j];
	}
	fold->Topcent /= (double)fold->pointPosition.size();


	//�ő�ƍŏ����ׂ�
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