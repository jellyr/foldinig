#include <QtOpenGL>
#ifndef _INCLUDE_OPTIMIZATION_
#define _INCLUDE_OPTIMIZATION_
#include "Optimization/trim/Trimfunc.h"
#include "Optimization/SteepDescent.h"
#include "Optimization/Animation/Animation.h"
#include "foldingMethod.h"
#include "Triangulation_cliping.h"
#endif

#define PI 3.14159265359

Model* COpenGL::readOBJ(std::string filename,bool IsNormalize){
	////cout << "\nread OBJ file\n";
	std::ifstream ifs(filename);
	std::string token(BUFSIZ, '\0');//サイズはBUFSIZE、すべてを\0で初期化
	std::list<Faces*>::iterator fa;
	Model *model = new Model();
	std::vector<Vertexs*> vertices; // インデックスで頂点を参照するためのvector
	Vertexs *v;
	unsigned int i=0;
	int f=0;
	bool vend = false;

	while (!(ifs >> token).fail()) {
		if (token[0] == '#') {
			ifs.ignore(BUFSIZ, '\n');
			continue;
		}

		if (token == "v") {
			double x, y, z;
			ifs >> x >> y >> z;
			ifs.ignore(BUFSIZ, '\n');
			Vertexs *v = new Vertexs(x, y, z,i);
			i++;
			model->vertices.push_back(v);
			vertices.push_back(v);	
		}
		else if (token == "f"){
			if(vend == false){
				vend = true;
			}
			int index0, index1, index2;
			ifs >> index0 >> index1 >> index2;
			ifs.ignore(BUFSIZ, '\n');
		
			model->addFace( vertices[index0-1], vertices[index1-1], vertices[index2-1] ,f);
			fa = model->faces.end();
			fa--;
			 v = vertices[index0-1];//点法線を計算
			 v->normal = v->normal + (*fa)->normal;
			 v = vertices[index1-1];
			 v->normal = v->normal + (*fa)->normal;
			 v = vertices[index2-1];
			 v->normal = v->normal + (*fa)->normal;
			f++;

		} else {
			ifs.ignore(BUFSIZ, '\n'); 
		}
	}

	if(IsNormalize){
		std::list<Vertexs*>::iterator it_v;
		Vec3 minV(DBL_MAX, DBL_MAX, DBL_MAX);
		Vec3 maxV(-DBL_MAX, -DBL_MAX, -DBL_MAX);
		for( it_v = model->vertices.begin(); it_v != model->vertices.end(); it_v++ ) {
			minV.x = min(minV.x, (*it_v)->p.x);
			minV.y = min(minV.y, (*it_v)->p.y);
			minV.z = min(minV.z, (*it_v)->p.z);
			maxV.x = max(maxV.x, (*it_v)->p.x);
			maxV.y = max(maxV.y, (*it_v)->p.y);
			maxV.z = max(maxV.z, (*it_v)->p.z);
		}

		Vec3 c_c = 0.5*(minV + maxV);
		size = (maxV - minV).length();
		c_x = c_c.x;
		c_y = c_c.y;
		c_z = c_c.z;
		
//		data->c_x = c_c.x;
	//	data->c_y = c_c.y;
		//data->c_z = c_c.z;
		//data->size = size;
	}

	double p = size;
	Vec3 cen;
	cen.x = c_x;
	cen.y = c_y;
	cen.z = c_z;

	model->cent = cen;
	//model->sizeNormalize(p,cen);
	start = true;
	//model->pointNor();//点法線をセット
	model->addsetH();

	std::list<Vertexs*>::iterator it_v;
	for(it_v=model->vertices.begin(); it_v!=model->vertices.end(); it_v++){
		(*it_v)->p = (*it_v)->p - cen;
	}
	
	//cout << "ver size: " << model->vertices.size() << "\n";
	//cout << "face size: " <<model->faces.size() << "\n";
	//cout << "half size: " <<model->halfs.size() << "\n";

	
	return model;
}

void COpenGL::renderScene(void)//描画関数
{	
	//if(start){
	//	//描画
	//	glDisable(GL_CULL_FACE);
	//	float color[4] = {0.0f,0.2f,0.7f,1.f};
	//	glMaterialfv (GL_FRONT,  GL_DIFFUSE, color );
	//	std::list<Vertexs*>::iterator it_v;
	//	for(int i=0; i<(int)TDdata->parts.size(); i++){
	//		renderParts(i,TDdata->parts[i], true,/*-TDdata->parts[i]->cent+*/TDdata->parts[i]->cent_move,TDdata->parts[i]->scale);
	//		Model *m = TDdata->parts[i];
	//		glPointSize(5);
	//		glColor3d(1,0.1,0.2);
	//		glDisable(GL_LIGHTING);
	//		for(it_v=m->vertices.begin(); it_v!=m->vertices.end(); it_v++){
	//			//cout << (*it_v)->num << " ";
	//			Vec3 renderP = (*it_v)->p;
	//			glBegin(GL_POINTS);
	//				glVertex3d(TDdata->parts[i]->scale.x*renderP.x+TDdata->parts[i]->cent_move.x, TDdata->parts[i]->scale.y*renderP.y+TDdata->parts[i]->cent_move.y, TDdata->parts[i]->scale.z*renderP.z+TDdata->parts[i]->cent_move.z);
	//			glEnd();
	//		}
	//	}
	//}
}
void COpenGL::renderParts(int number, Model *m, bool flg, Vec3 move, Vec3 scale){
	foldmethod *f = m->fold;
	double setSize = 0.01;

	glColor3d(0.7,0.3,0);
	glPointSize(8);
	glLineWidth(2);

	glDisable(GL_LIGHTING);
		glBegin(GL_POINTS);
			glVertex3d(0,0,0);
		glEnd();
	glEnable(GL_LIGHTING);

	if(number == now_n &&  face_n == 0){
		float color[4] = {0.7f,0.2f,0.0f,1.f};
		glMaterialfv (GL_FRONT,  GL_DIFFUSE, color );
	}
	glBegin(GL_POLYGON);
	for(int i=0; i<(int)f->pointPosition.size(); i++){
		Vec3 Top;
		Top.set(f->pointPosition[i].x*setSize*scale.x,-f->outlinepoints[0]->points[0].y*setSize*scale.y,f->pointPosition[i].y*setSize*scale.z);
		Top = Rotates(m->angle,Top);
		glVertex3d(Top.x+move.x, Top.y+move.y, Top.z+move.z);
	
	}
	glEnd();

	glColor3d(0.7,0.1,0.2);
	glBegin(GL_POINTS);
	for(int i=0; i<(int)f->pointPosition.size(); i++){
		Vec3 Top;
		Top.set(f->pointPosition[i].x*setSize*scale.x,-f->outlinepoints[0]->points[0].y*setSize*scale.y,f->pointPosition[i].y*setSize*scale.z);
		Top = Rotates(m->angle,Top);
		glVertex3d(Top.x+move.x, Top.y+move.y, Top.z+move.z);
	}
	glEnd();

	if(number == now_n &&  face_n == 0){
		float color[4] = {0.0f,0.2f,0.7f,1.f};
		glMaterialfv (GL_FRONT,  GL_DIFFUSE, color );
		Vec3 vis =  m->face_cen[face_n];
		glDisable(GL_LIGHTING);
		glPointSize(2);
		glBegin(GL_POINTS);
			Vec3 faces;
			faces.set(vis.x*scale.x+move.x ,-vis.z*scale.z+move.y,vis.y*scale.y+move.z);
			glVertex3d(faces.x,faces.y,faces.z);
		glEnd();
		glEnable(GL_LIGHTING);
		glPointSize(2);
	}

	glColor3d(0,0.3,1);
	glPointSize(8);
	glLineWidth(2);


	if(number == now_n &&  face_n == 1){
		float color[4] = {0.7f,0.2f,0.0f,1.f};
		glMaterialfv (GL_FRONT,  GL_DIFFUSE, color );
		Vec3 vis =  m->face_cen[face_n];
		glPointSize(2);
		glDisable(GL_LIGHTING);
		glBegin(GL_POINTS);
			Vec3 faces;
			faces.set(vis.x*scale.x+move.x ,-vis.z*scale.z+move.y,vis.y*scale.y+move.z);
			glVertex3d(faces.x,faces.y,faces.z);
		glEnd();
		glEnable(GL_LIGHTING);
		glPointSize(2);
	}
	glBegin(GL_POLYGON);
	for(int i=(int)f->outlinepoints.size()-1; i>=0; i--){
		int ipp = (i+1)%f->pointPosition.size();
		Vec2 sweepVec = -1*f->pointPosition[ipp] + f->pointPosition[i];
		sweepVec.normalize();
		Vec2 trimVec = f->pointPosition[ipp] - f->pointPosition[i];
		trimVec.rotate(PI/2.0);
		trimVec.normalize();

		double gap = f->outlinepoints[i]->points[(int)f->outlinepoints[i]->points.size()-1].x -  f->outlinepoints[i]->points[0].x;
		double l = f->trimPoint[i].trims[(int)f->trimPoint[i].trims.size()-1].l;
		Vec3 q0 = Vec3(f->betweenPosition[i].x,f->betweenPosition[i].y,0.0);
		q0.x += gap*trimVec.x;
		q0.y += gap*trimVec.y;
		q0.z += f->outlinepoints[0]->points[f->outlinepoints[0]->points.size()-1].y;
		q0.x += l*sweepVec.x;
		q0.y += l*sweepVec.y;

		q0 = q0 * setSize;
		Vec3 q0_;q0_.set(q0.x*scale.x, -q0.z*scale.y, q0.y*scale.z);
		q0_ = Rotates(m->angle, q0_);
		////cout << q0_.x << "," << q0_.y << "," << q0_.z << "\n"; 		
		glVertex3d(q0_.x+move.x, q0_.y+move.y, q0_.z+move.z );
	}
	glEnd();
	if(number == now_n &&  face_n == 1){
		float color[4] = {0.0f,0.2f,0.7f,1.f};
		glMaterialfv (GL_FRONT,  GL_DIFFUSE, color );
	}
	
	int face_nor_count = 2;

	for(int i=0; i<(int)f->outlinepoints.size(); i++){
		int ipp = (i+1)%f->pointPosition.size();
		Vec2 sweepVec = f->pointPosition[ipp] - f->pointPosition[i];
		sweepVec.normalize();
		Vec2 trimVec = f->pointPosition[ipp] - f->pointPosition[i];
		trimVec.rotate(PI/2.0);
		trimVec.normalize();
		bool idSwitch = true;
		int count = 0;
		int nums = 0;
		std::vector<Vec3> betw;
		
		for(int j=0; j<(int)f->trimPoint[i].trims.size()-1; j++){
			int id = f->trimPoint[i].trims[j].id;
			double alpha = f->trimPoint[i].trims[j].alpha;
			double l = f->trimPoint[i].trims[j].l;
			int sameLayer = f->trimPoint[i].trims[j].sameLayer;

			int id1 = f->trimPoint[i].trims[j+1].id;
			double alpha1 = f->trimPoint[i].trims[j+1].alpha;
			double l1 = f->trimPoint[i].trims[j+1].l;
			if(id == id1){
				if(id > 0){
					glColor3d(1.0,0.3,0);
					Vec2 outlinei;
					outlinei = f->outlinepoints[i]->points[id];
					outlinei -= f->outlinepoints[i]->points[id-1];

					Vec2 foo = trimVec;
					foo.setLength(alpha*outlinei.x + f->outlinepoints[i]->points[id-1].x);
					double bar = alpha * outlinei.y + f->outlinepoints[i]->points[id-1].y;
					Vec3 p0;
					p0.set(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y,bar);
					Vec3 p1;
					p1 = p0;
					p1.x += sweepVec.x*l;
					p1.y += sweepVec.y*l;


					Vec3 normalCheck0;
					normalCheck0.set(trimVec.x*outlinei.x,trimVec.y*outlinei.x,outlinei.y);

					normalCheck0.cross(sweepVec.x, sweepVec.y, 0.0);
					normalCheck0.normalize();

					glNormal3d(normalCheck0.x, -normalCheck0.z,normalCheck0.y);

					p0 = p0*setSize;
					
					p0.x *= scale.x;
					p0.y *= scale.z;
					p0.z *= scale.y;

					p1 = p1*setSize;

					p1.x *= scale.x;
					p1.y *= scale.z;
					p1.z *= scale.y;

					if(number == now_n &&  face_n == face_nor_count){
						float color[4] = {0.7f,0.2f,0.0f,1.f};
						glMaterialfv (GL_FRONT,  GL_DIFFUSE, color );
						Vec3 vis =  m->face_cen[face_n];
						glDisable(GL_LIGHTING);
						glPointSize(2);
						glBegin(GL_POINTS);
							Vec3 faces;
							faces.set(vis.x*scale.x+move.x ,-vis.z*scale.z+move.y,vis.y*scale.y+move.z);
							glVertex3d(faces.x,faces.y,faces.z);
						glEnd();
						glEnable(GL_LIGHTING);
						glPointSize(2);
					}
					glEnable(GL_LIGHTING);
					glBegin(GL_QUADS);

						Vec3 p0_,p1_,p2_,p3_;
						p0_.set(p0.x, -p0.z, p0.y);
						p0 = Rotates(m->angle, p0_);
						p1_.set(p1.x, -p1.z, p1.y);
						p1 = Rotates(m->angle, p1_);

						glVertex3d(p1.x+move.x, p1.y+move.y, p1.z+move.z);
						glVertex3d(p0.x+move.x, p0.y+move.y, p0.z+move.z);
						
					
						foo = trimVec;
						foo.setLength(alpha1*outlinei.x+f->outlinepoints[i]->points[id-1].x);
						bar = alpha1*outlinei.y+f->outlinepoints[i]->points[id-1].y;
						Vec3 p2 = Vec3(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y, bar);
						Vec3 p3;
						p3 = p2;
						p3.x += sweepVec.x*l1;
						p3.y += sweepVec.y*l1;

						p2 = p2*setSize;
						p3 = p3*setSize;

						p2.x *= scale.x;
						p2.y *= scale.z;
						p2.z *= scale.y;

						p3.x *= scale.x;
						p3.y *= scale.z;
						p3.z *= scale.y;

						p2_.set(p2.x, -p2.z, p2.y);
						p2 = Rotates(m->angle, p2_);
						p3_.set(p3.x, -p3.z, p3.y);
						p3 = Rotates(m->angle, p3_);

						glVertex3d(p2.x+move.x, p2.y+move.y, p2.z+move.z);
						glVertex3d(p3.x+move.x, p3.y+move.y, p3.z+move.z);
						////cout << p0.x << "," <<  p1.x << "," <<  p2.x << "," <<  p3.x << "," << "\n";
					glEnd();
					
					if(number == now_n &&  face_n == face_nor_count){
						float color[4] = {0.0f,0.2f,0.7f,1.f};
						glMaterialfv (GL_FRONT,  GL_DIFFUSE, color );
					}
					face_nor_count++;

				}else{
					//glColor3d(0,0.3,1.0);
					if(idSwitch){
						idSwitch = false;
					}
					id*=-1;

					Vec2 outlinei;
					outlinei = f->outlinepoints[i]->points[id];
					outlinei -= f->outlinepoints[i]->points[id-1];

					Vec2 foo = trimVec;
					foo.setLength(alpha*outlinei.x + f->outlinepoints[i]->points[id-1].x);
					double bar = alpha * outlinei.y + f->outlinepoints[i]->points[id-1].y;
					Vec3 p0;
					p0.set(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y,bar);
					Vec3 p1;
					p1 = p0;
					p1.x += -sweepVec.x*l;
					p1.y += -sweepVec.y*l;

					Vec3 normalCheck0;
					normalCheck0.set(trimVec.x*outlinei.x,trimVec.y*outlinei.x,outlinei.y);
					normalCheck0.cross(sweepVec.x, sweepVec.y, 0.0);
					normalCheck0.normalize();

					glNormal3d(normalCheck0.x, -normalCheck0.z,normalCheck0.y);
					
					p1 = p1*setSize;
					p0 = p0*setSize;

					p0.x *= scale.x;
					p0.y *= scale.z;
					p0.z *= scale.y;

					p1.x *= scale.x;
					p1.y *= scale.z;
					p1.z *= scale.y;

					//こっちがおかしい
					glEnable(GL_LIGHTING);
					if(number == now_n &&  face_n == face_nor_count){
						float color[4] = {0.7f,0.2f,0.0f,1.f};
						glMaterialfv (GL_FRONT,  GL_DIFFUSE, color );
						Vec3 vis =  m->face_cen[face_n];
						glDisable(GL_LIGHTING);
						glPointSize(2);
						glBegin(GL_POINTS);
							Vec3 faces;
							faces.set(vis.x*scale.x+move.x ,-vis.z*scale.z+move.y,vis.y*scale.y+move.z);
							glVertex3d(faces.x,faces.y,faces.z);
						glEnd();
						glPointSize(10);
						glEnable(GL_LIGHTING);
					}
					glEnable(GL_LIGHTING);
					glBegin(GL_QUADS);
						
						Vec3 p0_,p1_,p2_,p3_;
						p0_.set(p0.x, -p0.z, p0.y);
						p0 = Rotates(m->angle, p0_);
						p1_.set(p1.x, -p1.z, p1.y);
						p1 = Rotates(m->angle, p1_);

						glVertex3d(p1.x+move.x, p1.y+move.y, p1.z+move.z);
						glVertex3d(p0.x+move.x, p0.y+move.y, p0.z+move.z);
						
						/*glVertex3d(p1.x+move.x, -p1.z+move.y, p1.y+move.z);
						glVertex3d(p0.x+move.x, -p0.z+move.y, p0.y+move.z);*/
						foo = trimVec;
						foo.setLength(alpha1*outlinei.x+f->outlinepoints[i]->points[id-1].x);
						bar = alpha1*outlinei.y+f->outlinepoints[i]->points[id-1].y;
						Vec3 p2 = Vec3(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y, bar);
						Vec3 p3;
						p3 = p2;
						p3.x += -sweepVec.x*l1;
						p3.y += -sweepVec.y*l1;

						p2 = p2*setSize;
						p3 = p3*setSize;

						p2.x *= scale.x;
						p2.y *= scale.z;
						p2.z *= scale.y;

						p3.x *= scale.x;
						p3.y *= scale.z;
						p3.z *= scale.y;

						p2_.set(p2.x, -p2.z, p2.y);
						p2 = Rotates(m->angle, p2_);
						p3_.set(p3.x, -p3.z, p3.y);
						p3 = Rotates(m->angle, p3_);

						/*if(i == 0){
							//cout << "rotate before" << p1.x+move.x << "," << p1.y+move.y<< "," <<  p1.z+move.z << "\n";
						}*/

						glVertex3d(p2.x+move.x, p2.y+move.y, p2.z+move.z);
						glVertex3d(p3.x+move.x, p3.y+move.y, p3.z+move.z);

						/*glVertex3d(p2.x+move.x, -p2.z+move.y, p2.y+move.z);
						glVertex3d(p3.x+move.x, -p3.z+move.y, p3.y+move.z);*/
					glEnd();

					if(number == now_n &&  face_n == face_nor_count){
						float color[4] = {0.0f,0.2f,0.7f,1.f};
						glMaterialfv (GL_FRONT,  GL_DIFFUSE, color );
					}
					/*glBegin(GL_LINES);
						Vec3 faces;
						faces.set(m->face_cen[face_nor_count].x*scale.x+move.x ,-m->face_cen[face_nor_count].z*scale.z+move.y,m->face_cen[face_nor_count].y*scale.y+move.z);
						glVertex3d(faces.x,faces.y,faces.z);
						glVertex3d(faces.x+m->nor[face_nor_count].x,faces.y+m->nor[face_nor_count].y,faces.z+m->nor[face_nor_count].z);
					glEnd();
					glEnable(GL_LIGHTING);*/
					face_nor_count++;
				
					if(flg){
						glDisable(GL_LIGHTING);
						Vec3 between,between2;
						if(count == 0){
							between = p0;
							between2 = p3;
						}else{
							Vec3 nor;
							double ax;
							nor = p1-p0; nor.normalize();
							ax = nor*(betw[(int)betw.size()-1]-p0);
							between = p0 + ( nor * ax );
						}

						if(f->outlinepoints[i]->points[count] == f->outlinepoints[i]->points[id-1]){
							if(count == f->outlinepoints[i]->color_num){
								glColor3d(0.0,0,1.0);
							}else{
								glColor3d(1.0,0,0.0);
							}

							glBegin(GL_POINTS);
								glVertex3d(between.x+move.x, between.y+move.y, between.z+move.z);
								betw.push_back(between);
							glEnd();
							count++;
						}
						

						Vec3 nor;
						double ax;
						nor = p3-p2; nor.normalize();
						ax = nor*(between-p2);
						between2 = p2 + ( nor * ax );

						if(f->outlinepoints[i]->points[count-1] == f->outlinepoints[i]->points[id-1] && count-1 < (int)f->outlinepoints[i]->points.size()){
							////cout << "In render count: " << count-1 << " ";
							glColor3d(f->outlinepoints[i]->color[0],f->outlinepoints[i]->color[1],f->outlinepoints[i]->color[2]);
							glBegin(GL_LINES);
								glVertex3d(between.x+move.x, between.y+move.y, between.z+move.z);
								glVertex3d(between2.x+move.x, between2.y+move.y, between2.z+move.z);
							glEnd();
						}

						if(count == f->outlinepoints[i]->points.size()-1 && j == (int)f->trimPoint[i].trims.size()-2){
							if(count == f->outlinepoints[i]->color_num){
								glColor3d(0.0,0,1.0);
							}else{
								glColor3d(1.0,0,0.0);
							}
							glBegin(GL_POINTS);
								glVertex3d(between2.x+move.x, between2.y+move.y, between2.z+move.z);
							glEnd();
							////cout << "count: " << count << "," << j << "\n";
						}

						glEnable(GL_LIGHTING);
					}
				}
			}else{
				//線を引くだけ
			}
		}
		////cout << "\n";
		idSwitch = true;
		
		/*glBegin(GL_QUAD_STRIP);
		for(int j=0; j<(int)f->trimPoint[i].trims.size(); j++){
			int id = f->trimPoint[i].trims[j].id;
			double alpha = f->trimPoint[i].trims[j].alpha;
			double l = f->trimPoint[i].trims[j].l;
			
			if(id > 0){
				Vec2 outlinei = f->outlinepoints[i]->points[id];
				outlinei -= f->outlinepoints[i]->points[id-1];
				Vec2 foo = trimVec;
				foo.setLength(alpha*outlinei.x+f->outlinepoints[i]->points[id-1].x);
				double bar = alpha*outlinei.y + f->outlinepoints[i]->points[id-1].y;
				Vec3 p0;
				p0.set(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y, bar);
				Vec3 p1 = p0;

				p1.x += sweepVec.x*l;
				p1.y += sweepVec.y*l;

				Vec3 normalCheck0;
				normalCheck0.set(trimVec.x*outlinei.x, trimVec.y*outlinei.x, outlinei.y);
				normalCheck0.cross(sweepVec.x, sweepVec.y,0.0);
				normalCheck0.normalize();
				glNormal3d(-normalCheck0.x,normalCheck0.z,-normalCheck0.y);

				p0 = p0 * setSize;
				p1 = p1 * setSize;

			}else{
				if(idSwitch){
					glEnd();
					glBegin(GL_QUAD_STRIP);
					idSwitch = false;
				}
				id *= -1;
				Vec2 outlinei = f->outlinepoints[i]->points[id];
				outlinei -= f->outlinepoints[i]->points[id-1];
				Vec2 foo = trimVec;
				foo.setLength(alpha*outlinei.x+f->outlinepoints[i]->points[id-1].x);
				double bar = alpha*outlinei.y + f->outlinepoints[i]->points[id-1].y;
				Vec3 p0;
				p0.set(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y,bar);
				Vec3 p1;
				p1 = p0;
				p1.x += -sweepVec.x*l;
				p1.y += -sweepVec.y*l;

				Vec3 normalCheck0;
				normalCheck0.set(trimVec.x*outlinei.x, trimVec.y*outlinei.x, outlinei.y);
				normalCheck0.cross(sweepVec.x, sweepVec.y, 0.0);
				normalCheck0.normalize();
				glNormal3d(-normalCheck0.x, normalCheck0.z, -normalCheck0.y);

				p0 = p0 * setSize;
				p1 = p1 * setSize;
			}
		}
		glEnd();*/
	}
	
}
void COpenGL::Trim(GLData *data){

	std::vector<TrimPoints> returnData;
	double setSize = 0.01;


	for(int i=0; i<(int)data->parts.size(); i++){
		data->parts[i]->fold->trimPoint.clear();
		data->parts[i]->face_cen.clear();
		data->parts[i]->nor.clear();

		Model *m = data->parts[i];
		foldmethod *f = m->fold;
		
		m->fold->trimPoint = getPointsForAnimation(m->fold->pointPosition, m->fold->betweenPosition,m->fold->outlinepoints);
		Vec3 cent = Vec3(0,0,0);
		int count_=0;
		int count_1=0;

		///////////////////////////////
		Vec3 cent_; cent.set(0,0,0); double cent_count=0;
		for(int j=0; j<(int)f->pointPosition.size(); j++){//天頂面
			Vec3 ver; ver.set(f->pointPosition[j].x*setSize,-f->outlinepoints[0]->points[0].y*setSize,f->pointPosition[j].y*setSize);
			if(j != 0){
				cent_ += ver;
				cent_count++;
			}
		}

		data->parts[i]->face_cen.push_back(cent_/(double)cent_count);
		data->parts[i]->nor.push_back(Vec3(0,1,0));

		cent.set(0,0,0); cent_count = 0;
		for(int j=(int)f->outlinepoints.size()-1; j>=0; j--){

			int ipp = (j+1)%f->pointPosition.size();
			Vec2 sweepVec = -1*f->pointPosition[ipp] + f->pointPosition[j];
			sweepVec.normalize();
			Vec2 trimVec = f->pointPosition[ipp] - f->pointPosition[j];
			trimVec.rotate(PI/2.0);
			trimVec.normalize();

			double gap = f->outlinepoints[j]->points[(int)f->outlinepoints[j]->points.size()-1].x -  f->outlinepoints[j]->points[0].x;
			double l = f->trimPoint[j].trims[(int)f->trimPoint[j].trims.size()-1].l;
			Vec3 q0 = Vec3(f->betweenPosition[j].x,f->betweenPosition[j].y,0.0);
			q0.x += gap*trimVec.x;
			q0.y += gap*trimVec.y;
			q0.z += f->outlinepoints[0]->points[f->outlinepoints[0]->points.size()-1].y;
			q0.x += l*sweepVec.x;
			q0.y += l*sweepVec.y;

			q0 = q0 * setSize;
			//Vec3 ver; ver.set(q0.x, -q0.z, q0.y);
			if(j != (int)f->outlinepoints.size()-1){
				cent_ += q0;
				cent_count++;
			}
		}

		data->parts[i]->face_cen.push_back(cent_/(double)cent_count);
		data->parts[i]->nor.push_back(Vec3(0,-1,0));
		///////////////////////////////
			
		for(int k=0; k<(int)f->outlinepoints.size(); k++){
			int ipp = (k+1)%f->pointPosition.size();
			Vec2 sweepVec = f->pointPosition[ipp] - f->pointPosition[k];
			sweepVec.normalize();
			Vec2 trimVec = f->pointPosition[ipp] - f->pointPosition[k];
			trimVec.rotate(PI/2.0);
			trimVec.normalize();
			bool idSwitch = true;
			int count = 0;
			int count_1 = 0;
			std::vector<Vec3> betw;
			for(int j=0; j<(int)f->trimPoint[k].trims.size()-1; j++){
				int id = f->trimPoint[k].trims[j].id;
				double alpha = f->trimPoint[k].trims[j].alpha;
				double l = f->trimPoint[k].trims[j].l;
				int sameLayer = f->trimPoint[k].trims[j].sameLayer;

				int id1 = f->trimPoint[k].trims[j+1].id;
				double alpha1 = f->trimPoint[k].trims[j+1].alpha;
				double l1 = f->trimPoint[k].trims[j+1].l;
				if(id == id1){
					if(id > 0){
						Vec2 outlinei;
						outlinei = f->outlinepoints[k]->points[id];
						outlinei -= f->outlinepoints[k]->points[id-1];

						Vec2 foo = trimVec;
						foo.setLength(alpha*outlinei.x + f->outlinepoints[k]->points[id-1].x);
						double bar = alpha * outlinei.y + f->outlinepoints[k]->points[id-1].y;
						Vec3 p0;
						p0.set(foo.x+f->betweenPosition[k].x, foo.y+f->betweenPosition[k].y,bar);
						Vec3 p1;
						p1 = p0;
						p1.x += sweepVec.x*l;
						p1.y += sweepVec.y*l;


						Vec3 normalCheck0;
						normalCheck0.set(trimVec.x*outlinei.x,trimVec.y*outlinei.x,outlinei.y);

						normalCheck0.cross(sweepVec.x, sweepVec.y, 0.0);
						normalCheck0.normalize();

						glNormal3d(normalCheck0.x, -normalCheck0.z,normalCheck0.y);
						p0.x *= setSize;
						p0.y *= setSize;
						p0.z *= setSize;
						p1.x *= setSize;
						p1.y *= setSize;
						p1.z *= setSize;

						foo = trimVec;
						foo.setLength(alpha1*outlinei.x+f->outlinepoints[k]->points[id-1].x);
						bar = alpha1*outlinei.y+f->outlinepoints[k]->points[id-1].y;
						Vec3 p2 = Vec3(foo.x+f->betweenPosition[k].x, foo.y+f->betweenPosition[k].y, bar);
						Vec3 p3;
						p3 = p2;
						p3.x += sweepVec.x*l1;
						p3.y += sweepVec.y*l1;

						p2 = p2*setSize;
						p3 = p3*setSize;
							
						data->parts[i]->face_cen.push_back((p0+p1+p2+p3)/4.0);
						data->parts[i]->nor.push_back(Vec3(normalCheck0.x, -normalCheck0.z,normalCheck0.y));

						
					}else{
						if(idSwitch){
							idSwitch = false;
						}
						id*=-1;

						Vec2 outlinei;
						outlinei = f->outlinepoints[k]->points[id];
						outlinei -= f->outlinepoints[k]->points[id-1];

						Vec2 foo = trimVec;
						foo.setLength(alpha*outlinei.x + f->outlinepoints[k]->points[id-1].x);
						double bar = alpha * outlinei.y + f->outlinepoints[k]->points[id-1].y;
						Vec3 p0;
						p0.set(foo.x+f->betweenPosition[k].x, foo.y+f->betweenPosition[k].y,bar);
						Vec3 p1;
						p1 = p0;
						p1.x += -sweepVec.x*l;
						p1.y += -sweepVec.y*l;

						Vec3 normalCheck0;
						normalCheck0.set(trimVec.x*outlinei.x,trimVec.y*outlinei.x,outlinei.y);
						normalCheck0.cross(sweepVec.x, sweepVec.y, 0.0);
						normalCheck0.normalize();

						glNormal3d(normalCheck0.x, -normalCheck0.z,normalCheck0.y);

						p1 = p1*setSize;
						p0 = p0*setSize;
						
						foo = trimVec;
						foo.setLength(alpha1*outlinei.x+f->outlinepoints[k]->points[id-1].x);
						bar = alpha1*outlinei.y+f->outlinepoints[k]->points[id-1].y;
						Vec3 p2 = Vec3(foo.x+f->betweenPosition[k].x, foo.y+f->betweenPosition[k].y, bar);
						Vec3 p3;
						p3 = p2;
						p3.x += -sweepVec.x*l1;
						p3.y += -sweepVec.y*l1;

						p2 = p2*setSize;
						p3 = p3*setSize;

						data->parts[i]->face_cen.push_back((p0+p1+p2+p3)/4.0);
						data->parts[i]->nor.push_back(Vec3(normalCheck0.x, -normalCheck0.z,normalCheck0.y));

						Vec3 between,between2;
						if(count == 0){
							between = (p0 + p1)/2.0;
							between2 = (p2 + p3)/2.0;
							f->outlinepoints[k]->firstP = between;
						}else{
							Vec3 nor;
							double ax;
							nor = p1-p0; nor.normalize();
							ax = nor*(betw[(int)betw.size()-1]-p0);
							between = p0 + ( nor * ax );
						}

						if(f->outlinepoints[k]->points[count] == f->outlinepoints[k]->points[id-1]){
							betw.push_back(between);
							cent.x += between.x;
							cent.y += -between.z;
							cent.z += between.y;
							count++;
							count_++;
						}else if(j == (int)f->outlinepoints[k]->points.size()-2){
							betw.push_back(between);
							/*cent.x += between.x;
							cent.y += -between.z;
							cent.z += between.y;*/
							cent.x += between2.x;
							cent.y += -between2.z;
							cent.z += between2.y;
							count++;
							count_ ++;
						}
						
					}
				}
			}
		}
		
		////cout << "元の中心座標: " << m->cent.x+ m->cent_move.x<< "," << m->cent.y+m->cent_move.y << "," << m->cent.z+m->cent_move.z << "\n";
		cent = cent / (double)count_;
		if(m->cent.eq(Vec3(0,0,0)) && m->cent_move.eq(Vec3(0,0,0))){//初期
			m->cent_move = -cent;
			m->cent = cent;
		}else{
			/*Vec3 moved; moved = m->cent + m->cent_move;
			m->cent_move.x = (moved.x - cent.x);
			m->cent_move.y = (moved.y - cent.y);
			m->cent_move.z = (moved.z - cent.z);
			m->cent = cent;*/
		}
		
		////cout << "変更の後の中心座標: " << m->cent.x+ m->cent_move.x<< "," << m->cent.y+m->cent_move.y << "," << m->cent.z+m->cent_move.z << "\n";

		//TDdata->parts[i]->fold->part_state.clear();
		////最大と最小を比べる
		//for(int j=0; j<(int)data->parts[i]->fold->outlinepoints.size(); j++){
		//	Vec2 cen; cen.set(0,0);
		//	std::vector<Vec2> outline = data->parts[i]->fold->outlinepoints[j]->points;
		//	std::sort(outline.begin(),outline.end(),Vec2::compareVec2PredicateX);
		//	data->parts[i]->fold->outlinepoints[j]->maxX = abs(outline[0].x - outline[(int)outline.size()-1].x);
		//
		//	std::sort(outline.begin(),outline.end(),Vec2::compareVec2PredicateY);
		//	data->parts[i]->fold->outlinepoints[j]->maxY = abs(outline[0].y - outline[(int)outline.size()-1].y);
		//}
	}

}

void COpenGL::Trim(Model *m){
	std::vector<TrimPoints> returnData;
	//double setSize = 0.01;
	double setSize = 1.0;
	m->fold->trimPoint.clear();
	m->face_cen.clear();
	m->nor.clear();

	foldmethod *f = m->fold;
	/*cout << "pointPosition\n";
	for (int i = 0; i < m->fold->pointPosition.size(); i++) {
		cout << i << ": " << m->fold->pointPosition[i].x << "," << m->fold->pointPosition[i].y << "\n";
	}
	cout << "betweenPosition\n";
	for (int i = 0; i < m->fold->betweenPosition.size(); i++) {
		cout << i << ": " << m->fold->betweenPosition[i].x << "," << m->fold->betweenPosition[i].y << "\n";
	}
	for (int i = 0; i < m->fold->outlinepoints.size(); i++) {
		cout << "outline: " << i << "\n";
		for (int j = 0; j < m->fold->outlinepoints[i]->points.size(); j++) {
			cout << m->fold->outlinepoints[i]->points[j].x << "," << m->fold->outlinepoints[i]->points[j].y << "\n";
		}
	}*/

	/*std::reverse(m->fold->pointPosition.begin(), m->fold->pointPosition.end());
	std::reverse(m->fold->betweenPosition.begin(), m->fold->betweenPosition.end());
	std::reverse(m->fold->outlinepoints.begin(), m->fold->outlinepoints.end());
*/
	m->fold->trimPoint = getPointsForAnimation(m->fold->pointPosition, m->fold->betweenPosition, m->fold->outlinepoints);
	
	Vec3 cent = Vec3(0,0,0);
	int count_=0;
	int count_1=0;
	///////////////////////////////
	Vec3 cent_; cent.set(0,0,0); double cent_count=0;
	for(int j=0; j<(int)f->pointPosition.size(); j++){
		Vec3 ver; ver.set(f->pointPosition[j].x*setSize,-f->outlinepoints[0]->points[0].y*setSize,f->pointPosition[j].y*setSize);
		if(j != 0){
			cent_ += ver;
			cent_count++;
		}

		Vertexs *v = new Vertexs(ver.x, ver.y, ver.z, m->vertices.size());
		m->vertices.push_back(v);
	}

	m->face_cen.push_back(cent_/(double)cent_count);
	m->nor.push_back(Vec3(0,1,0));

	Vertexs *ver_cent = new Vertexs(m->face_cen[0].x,m->face_cen[0].y,m->face_cen[0].z,m->vertices.size());
	m->vertices.push_back(ver_cent);

	cent.set(0,0,0); cent_count = 0;
	for(int j=(int)f->outlinepoints.size()-1; j>=0; j--){

		int ipp = (j+1)%f->pointPosition.size();
		Vec2 sweepVec = -1*f->pointPosition[ipp] + f->pointPosition[j];
		sweepVec.normalize();
		Vec2 trimVec = f->pointPosition[ipp] - f->pointPosition[j];
		trimVec.rotate(PI/2.0);
		trimVec.normalize();

		double gap = f->outlinepoints[j]->points[(int)f->outlinepoints[j]->points.size()-1].x -  f->outlinepoints[j]->points[0].x;
		double l = f->trimPoint[j].trims[(int)f->trimPoint[j].trims.size()-1].l;
		Vec3 q0 = Vec3(f->betweenPosition[j].x,f->betweenPosition[j].y,0.0);
		q0.x += gap*trimVec.x;
		q0.y += gap*trimVec.y;
		q0.z += f->outlinepoints[0]->points[f->outlinepoints[0]->points.size()-1].y;
		q0.x += l*sweepVec.x;
		q0.y += l*sweepVec.y;

		q0 = q0 * setSize;
		//Vec3 ver; ver.set(q0.x, -q0.z, q0.y);
		if(j != (int)f->outlinepoints.size()-1){
			cent_ += q0;
			cent_count++;
		}
	}

	m->face_cen.push_back(cent_/(double)cent_count);
	m->nor.push_back(Vec3(0,-1,0));
	///////////////////////////////
			
	for(int k=0; k<(int)f->outlinepoints.size(); k++){
		int ipp = (k+1)%f->pointPosition.size();
		Vec2 sweepVec = f->pointPosition[ipp] - f->pointPosition[k];
		sweepVec.normalize();
		Vec2 trimVec = f->pointPosition[ipp] - f->pointPosition[k];
		trimVec.rotate(PI/2.0);
		trimVec.normalize();
		bool idSwitch = true;
		int count = 0;
		int count_1 = 0;
		std::vector<Vec3> betw;
		for(int j=0; j<(int)f->trimPoint[k].trims.size()-1; j++){
			int id = f->trimPoint[k].trims[j].id;
			double alpha = f->trimPoint[k].trims[j].alpha;
			double l = f->trimPoint[k].trims[j].l;
			int sameLayer = f->trimPoint[k].trims[j].sameLayer;

			int id1 = f->trimPoint[k].trims[j+1].id;
			double alpha1 = f->trimPoint[k].trims[j+1].alpha;
			double l1 = f->trimPoint[k].trims[j+1].l;
			if(id == id1){
				if(id > 0){
					Vec2 outlinei;
					outlinei = f->outlinepoints[k]->points[id];
					outlinei -= f->outlinepoints[k]->points[id-1];

					Vec2 foo = trimVec;
					foo.setLength(alpha*outlinei.x + f->outlinepoints[k]->points[id-1].x);
					double bar = alpha * outlinei.y + f->outlinepoints[k]->points[id-1].y;
					Vec3 p0;
					p0.set(foo.x+f->betweenPosition[k].x, foo.y+f->betweenPosition[k].y,bar);
					Vec3 p1;
					p1 = p0;
					p1.x += sweepVec.x*l;
					p1.y += sweepVec.y*l;


					Vec3 normalCheck0;
					normalCheck0.set(trimVec.x*outlinei.x,trimVec.y*outlinei.x,outlinei.y);

					normalCheck0.cross(sweepVec.x, sweepVec.y, 0.0);
					normalCheck0.normalize();

					glNormal3d(normalCheck0.x, -normalCheck0.z,normalCheck0.y);
					p0.x *= setSize;
					p0.y *= setSize;
					p0.z *= setSize;
					p1.x *= setSize;
					p1.y *= setSize;
					p1.z *= setSize;

					foo = trimVec;

					Vertexs *v = new Vertexs(p0.x, p0.y, p0.z, m->vertices.size());
					m->vertices.push_back(v);

					foo.setLength(alpha1*outlinei.x+f->outlinepoints[k]->points[id-1].x);
					bar = alpha1*outlinei.y+f->outlinepoints[k]->points[id-1].y;
					Vec3 p2 = Vec3(foo.x+f->betweenPosition[k].x, foo.y+f->betweenPosition[k].y, bar);
					Vec3 p3;
					p3 = p2;
					p3.x += sweepVec.x*l1;
					p3.y += sweepVec.y*l1;

					p2 = p2*setSize;
					p3 = p3*setSize;
							
					m->face_cen.push_back((p0+p1+p2+p3)/4.0);
					m->nor.push_back(Vec3(normalCheck0.x, -normalCheck0.z,normalCheck0.y));

						
				}else{
					if(idSwitch){
						idSwitch = false;
					}
					id*=-1;

					Vec2 outlinei;
					outlinei = f->outlinepoints[k]->points[id];
					outlinei -= f->outlinepoints[k]->points[id-1];

					Vec2 foo = trimVec;
					foo.setLength(alpha*outlinei.x + f->outlinepoints[k]->points[id-1].x);
					double bar = alpha * outlinei.y + f->outlinepoints[k]->points[id-1].y;
					Vec3 p0;
					p0.set(foo.x+f->betweenPosition[k].x, foo.y+f->betweenPosition[k].y,bar);
					Vec3 p1;
					p1 = p0;
					p1.x += -sweepVec.x*l;
					p1.y += -sweepVec.y*l;

					Vec3 normalCheck0;
					normalCheck0.set(trimVec.x*outlinei.x,trimVec.y*outlinei.x,outlinei.y);
					normalCheck0.cross(sweepVec.x, sweepVec.y, 0.0);
					normalCheck0.normalize();

					glNormal3d(normalCheck0.x, -normalCheck0.z,normalCheck0.y);

					p1 = p1*setSize;
					p0 = p0*setSize;
						
					foo = trimVec;
					foo.setLength(alpha1*outlinei.x+f->outlinepoints[k]->points[id-1].x);
					bar = alpha1*outlinei.y+f->outlinepoints[k]->points[id-1].y;
					Vec3 p2 = Vec3(foo.x+f->betweenPosition[k].x, foo.y+f->betweenPosition[k].y, bar);
					Vec3 p3;
					p3 = p2;
					p3.x += -sweepVec.x*l1;
					p3.y += -sweepVec.y*l1;

					p2 = p2*setSize;
					p3 = p3*setSize;

					m->face_cen.push_back((p0+p1+p2+p3)/4.0);
					m->nor.push_back(Vec3(normalCheck0.x, -normalCheck0.z,normalCheck0.y));

					Vec3 between,between2;
					if(count == 0){
						between = (p0 + p1)/2.0;
						between2 = (p2 + p3)/2.0;
						f->outlinepoints[k]->firstP = between;
					}else{
						Vec3 nor;
						double ax;
						nor = p1-p0; nor.normalize();
						ax = nor*(betw[(int)betw.size()-1]-p0);
						between = p0 + ( nor * ax );
					}

					if(f->outlinepoints[k]->points[count] == f->outlinepoints[k]->points[id-1]){
						betw.push_back(between);
						cent.x += between.x;
						cent.y += -between.z;
						cent.z += between.y;
						count++;
						count_++;
					}else if(j == (int)f->outlinepoints[k]->points.size()-2){
						betw.push_back(between);
						cent.x += between2.x;
						cent.y += -between2.z;
						cent.z += between2.y;
						count++;
						count_ ++;
					}
						
				}
			}
		}
	}

}

//最適化
bool COpenGL::optimization_oen_outline(Model *m, int outline_num){

	bool flg = false;
	std::vector<Vec2> points = m->fold->outlinepoints[outline_num]->points;
	std::vector<double> l;
	Vec2 foo;
		
	l.resize((int)points.size()-1);
	for(int j=0; j<(int)points.size()-1; j++){
		foo = points[j] - points[j+1];
		double length = foo.length();
		l[j] = length;
	}

	int *sign = new int[(int)l.size()];
	std::vector<double*> output;
	std::vector<double> gap;
	gap.resize((int)pow(2.0,(int)l.size()-2));
	if(points[0].y > points[(int)points.size()-1].y){
		for(int j=0; j<(int)points.size()/2; j++){
			Vec2 t = points[j];
			points[j] = points[(int)points.size()-j-1];
			points[(int)points.size()-j-1] = t;
		}
		for(int j=0; j<(int)l.size(); j++){
			double t = l[j];
			l[j] = l[(int)l.size()-j-1];
			l[(int)l.size()-j-1] = t;
		}
	}

	std::vector<int> output_array_size;
	std::vector<dR> output_dR;
	output_array_size.resize((int)pow(2.0,(int)l.size()-2));
	output_dR.resize((int)pow(2.0,(int)l.size()-2));
	uninfo(0, 0, sign, l, output_dR, gap, output_array_size,0);
	output.resize((int)output_dR.size());
	for(int s=0; s<(int)output_dR.size(); s++){
		double *tmp = new double[(int)output_dR[s].dArray.size()];
		for(int k=0; k<output_array_size[s]; k++){
			tmp[k] = output_dR[s].dArray[k];
		}
		output[s] = tmp;
	}
		
	for(int j=0; j<(int)gap.size(); j++){
		gap[j] = gap[j]-points[(int)points.size()-1].x + points[0].x;
	}

	double *evalu1 = new double[(int)output.size()];
	double *evalu2 = new double[(int)output.size()];
	double *evalu3 = new double[(int)gap.size()];

	func1(output,points,evalu1);
	func2(output,points,evalu2);
	func3(gap,output,evalu3,output_array_size);
		
	double *evalu = new double[(int)output.size()];

	double w1 = 1.0; double w2 = 1.0; double w3 = 1.0;
	for(int j=0; j<(int)output.size(); j++){
		evalu[j] = w1*evalu1[j]+w2*evalu2[j]-w3*evalu3[j];
	}

	int *eSort = new int[(int)output.size()];
	desIndex(evalu, eSort, (int)output.size());
	double *evaluN = new double[(int)output.size()];

	std::vector<double*> outputN;
	std::vector<int> outputN_array_size;
	std::vector<double> gapN;

	outputN.resize((int)output.size());
	outputN_array_size.resize((int)output.size());
	gapN.resize((int)output.size());
	for(int j=0; j<(int)output.size(); j++){
		int index =eSort[j];
		evaluN[j] = evalu[index];
		outputN[j] = output[index];
		outputN_array_size[j] = output_array_size[index];
		gapN[j] = gap[index];
	}

	for(int j=0; j<(int)outputN.size(); j++){
		double L = 0;
		double alpha;
		for(int k=0; k<outputN_array_size[j]; k++){
			L+=fabs(outputN[j][k]);
		}
		alpha = gapN[j]/L;
		for(int k=0; k<outputN_array_size[j]; k++){
			double delta = 1.0;
			if(outputN[j][k] > 0){ delta = -1.0; }
			outputN[j][k] = (1+delta*alpha)*(outputN[j][k]);
		}
		Vec2 *pointsN = new Vec2[(int)points.size()];
		Step(outputN[j], gapN[j], points, outputN_array_size[j],pointsN,false);
		if(foldableV(outputN[j], pointsN, outputN_array_size[j], (int)points.size())){
			//cout << "optimized true: \n";
			for(int s=0; s<(int)m->fold->outlinepoints[outline_num]->points.size(); s++){
				m->fold->outlinepoints[outline_num]->points[s].x = pointsN[s].x;
				m->fold->outlinepoints[outline_num]->points[s].y = pointsN[s].y;
				flg = true;
			}
			break;
		}else{
			////cout << "optimized error\n";
		}
	}

	delete [] sign;
	delete [] evalu;
	delete [] evalu1;
	delete [] evalu2;
	delete [] evalu3;
	delete [] evaluN;
	delete [] eSort;
	for(int k=0; k<(int)output.size(); k++){
		delete [] output[k];
	}


	return flg;
	
}
void COpenGL::optimization(Model *m){

	//m->fold->outlinepoints->foldingPattern.resize((int)m->fold->outlinepoints.size());

	for(int i=0; i<(int)m->fold->outlinepoints.size(); i++){
		//m->fold->outlinepoints[i]->foldingPattern.resize((int)m->fold->outlinepoints[i].size());
		bool flg;
		std::vector<Vec2> points = m->fold->outlinepoints[i]->points;
		std::vector<double> l;
		Vec2 foo;
		
		l.resize((int)points.size()-1);
		for(int j=0; j<(int)points.size()-1; j++){
			foo = points[j] - points[j+1];
			
			double length = foo.length();
			l[j] = length;

		}

		int *sign = new int[(int)l.size()];
		std::vector<double*> output;
		std::vector<double> gap;
		gap.resize((int)pow(2.0,(int)l.size()-2));
		if(points[0].y > points[(int)points.size()-1].y){
			for(int j=0; j<(int)points.size()/2; j++){
				Vec2 t = points[j];
				points[j] = points[(int)points.size()-j-1];
				points[(int)points.size()-j-1] = t;
			}
			for(int j=0; j<(int)l.size(); j++){
				double t = l[j];
				l[j] = l[(int)l.size()-j-1];
				l[(int)l.size()-j-1] = t;
			}
		}

		std::vector<int> output_array_size;
		std::vector<dR> output_dR;
		output_array_size.resize((int)pow(2.0,(int)l.size()-2));
		output_dR.resize((int)pow(2.0,(int)l.size()-2));
		uninfo(0, 0, sign, l, output_dR, gap, output_array_size,0);
		output.resize((int)output_dR.size());
		for(int s=0; s<(int)output_dR.size(); s++){
			double *tmp = new double[(int)output_dR[s].dArray.size()];
			for(int k=0; k<output_array_size[s]; k++){
				tmp[k] = output_dR[s].dArray[k];
			}
			output[s] = tmp;
		}
		
		for(int j=0; j<(int)gap.size(); j++){
			gap[j] = gap[j]-points[(int)points.size()-1].x + points[0].x;
		}

		double *evalu1 = new double[(int)output.size()];
		double *evalu2 = new double[(int)output.size()];
		double *evalu3 = new double[(int)gap.size()];

		func1(output,points,evalu1);
		func2(output,points,evalu2);
		func3(gap,output,evalu3,output_array_size);
		
		double *evalu = new double[(int)output.size()];

		double w1 = 1.0; double w2 = 1.0; double w3 = 1.0;
		for(int j=0; j<(int)output.size(); j++){
			evalu[j] = w1*evalu1[j]+w2*evalu2[j]-w3*evalu3[j];
		}

		int *eSort = new int[(int)output.size()];
		desIndex(evalu, eSort, (int)output.size());
		double *evaluN = new double[(int)output.size()];

		std::vector<double*> outputN;
		std::vector<int> outputN_array_size;
		std::vector<double> gapN;

		outputN.resize((int)output.size());
		outputN_array_size.resize((int)output.size());
		gapN.resize((int)output.size());
		for(int j=0; j<(int)output.size(); j++){
			int index =eSort[j];
			if (index < 0){
				index *= -1;
			}
			evaluN[j] = evalu[index];
			outputN[j] = output[index];
			outputN_array_size[j] = output_array_size[index];
			gapN[j] = gap[index];
		}

		for(int j=0; j<(int)outputN.size(); j++){
			double L = 0;
			double alpha;
			for(int k=0; k<outputN_array_size[j]; k++){
				L+=fabs(outputN[j][k]);
			}
			alpha = gapN[j]/L;
			for(int k=0; k<outputN_array_size[j]; k++){
				double delta = 1.0;
				if(outputN[j][k] > 0){ delta = -1.0; }
				outputN[j][k] = (1+delta*alpha)*(outputN[j][k]);
			}
			Vec2 *pointsN = new Vec2[(int)points.size()];
			Step(outputN[j], gapN[j], points, outputN_array_size[j],pointsN,false);
			if(foldableV(outputN[j], pointsN, outputN_array_size[j], (int)points.size())){
				//cout << "optimized true: " << i << " and j number is " << j << "\n";
				flg = true;
				//m->fold->outlinepoints[i]->foldingPattern.resize(outputN_array_size[j]);;
				m->fold->outlinepoints[i]->foldingPattern_size = outputN_array_size[j];
				m->fold->outlinepoints[i]->foldingPattern.resize(outputN_array_size[j]);
		
				for(int s=0; s<outputN_array_size[j]; s++){
					m->fold->outlinepoints[i]->foldingPattern[s] =outputN[j][s];
					////cout << "outputN: " << m->fold->outlinepoints[i]->foldingPattern[s] << "\n"; 
				}
				for(int s=0; s<(int)m->fold->outlinepoints[i]->points.size(); s++){
					m->fold->outlinepoints[i]->points[s].x = pointsN[s].x;
					m->fold->outlinepoints[i]->points[s].y = pointsN[s].y;
					////cout << "points: " << s << ": " << pointsN[s].x << "," << pointsN[s].y << "\n";
				}
				
			//	//cout << "\n";
				break;
			}else{
				flg = false;
				if(j == 0){
					/*//cout << "error\n";
					//cout << "outputN_array_size[j] : " << outputN_array_size[j] << "\n";
					for(int s=0; s<outputN_array_size[j]; s++){
						//cout << "outputN: " << outputN[j][s] << "\n"; 
					}
					for(int s=0; s<(int)m->fold->outlinepoints[i]->points.size(); s++){
						//cout << "points: " << s << ": " << pointsN[s].x << "," << pointsN[s].y << "\n";
					}*/
				
				}
				////cout << "optimized error\n";
			}
		}

		delete [] sign;
		delete [] evalu;
		delete [] evalu1;
		delete [] evalu2;
		delete [] evalu3;
		delete [] evaluN;
		delete [] eSort;
		for(int k=0; k<(int)output.size(); k++){
			delete [] output[k];
		}
	}

	/*for(int s=(int)m->fold->outlinepoints.size()-1; s>=0; s--){
		//cout << "optimized outline: " << s << "\n";
		for(int d=0; d<m->fold->outlinepoints[s]->points.size(); d++){
			//cout << m->fold->outlinepoints[s]->points[d].x << "," << m->fold->outlinepoints[s]->points[d].y<< "\n";
		}
	}*/

	
}
void COpenGL::optimization_one(Model *m){

		Vec2 first = m->fold->outlinepoints[outline_n]->move_start;
		bool success_flg = false;

		outline *line = m->fold->outlinepoints[outline_n];
		bool flg;
		std::vector<Vec2> points = m->fold->outlinepoints[outline_n]->points;
		std::vector<double> l;
		Vec2 foo;
		
		l.resize((int)points.size()-1);
		for(int j=0; j<(int)points.size()-1; j++){
			foo = points[j] - points[j+1];
			
			double length = foo.length();
			l[j] = length;

		}

		int *sign = new int[(int)l.size()];
		std::vector<double*> output;
		std::vector<double> gap;
		gap.resize((int)pow(2.0,(int)l.size()-2));
		if(points[0].y > points[(int)points.size()-1].y){
			for(int j=0; j<(int)points.size()/2; j++){
				Vec2 t = points[j];
				points[j] = points[(int)points.size()-j-1];
				points[(int)points.size()-j-1] = t;
			}
			for(int j=0; j<(int)l.size(); j++){
				double t = l[j];
				l[j] = l[(int)l.size()-j-1];
				l[(int)l.size()-j-1] = t;
			}
		}

		std::vector<int> output_array_size;
		std::vector<dR> output_dR;
		output_array_size.resize((int)pow(2.0,(int)l.size()-2));
		output_dR.resize((int)pow(2.0,(int)l.size()-2));
		uninfo(0, 0, sign, l, output_dR, gap, output_array_size,0);
		output.resize((int)output_dR.size());
		for(int s=0; s<(int)output_dR.size(); s++){
			double *tmp = new double[(int)output_dR[s].dArray.size()];
			for(int k=0; k<output_array_size[s]; k++){
				tmp[k] = output_dR[s].dArray[k];
			}
			output[s] = tmp;
		}
		
		for(int j=0; j<(int)gap.size(); j++){
			gap[j] = gap[j]-points[(int)points.size()-1].x + points[0].x;
		}

		double *evalu1 = new double[(int)output.size()];
		double *evalu2 = new double[(int)output.size()];
		double *evalu3 = new double[(int)gap.size()];

		func1(output,points,evalu1);
		func2(output,points,evalu2);
		func3(gap,output,evalu3,output_array_size);
		
		double *evalu = new double[(int)output.size()];

		double w1 = 1.0; double w2 = 1.0; double w3 = 1.0;
		for(int j=0; j<(int)output.size(); j++){
			evalu[j] = w1*evalu1[j]+w2*evalu2[j]-w3*evalu3[j];
		}

		int *eSort = new int[(int)output.size()];
		desIndex(evalu, eSort, (int)output.size());
		double *evaluN = new double[(int)output.size()];

		std::vector<double*> outputN;
		std::vector<int> outputN_array_size;
		std::vector<double> gapN;

		outputN.resize((int)output.size());
		outputN_array_size.resize((int)output.size());
		gapN.resize((int)output.size());
		for(int j=0; j<(int)output.size(); j++){
			int index =eSort[j];
			evaluN[j] = evalu[index];
			outputN[j] = output[index];
			outputN_array_size[j] = output_array_size[index];
			gapN[j] = gap[index];
		}

		for(int j=0; j<(int)outputN.size(); j++){
			double L = 0;
			double alpha;
			for(int k=0; k<outputN_array_size[j]; k++){
				L+=fabs(outputN[j][k]);
			}
			alpha = gapN[j]/L;
			for(int k=0; k<outputN_array_size[j]; k++){
				double delta = 1.0;
				if(outputN[j][k] > 0){ delta = -1.0; }
				outputN[j][k] = (1+delta*alpha)*(outputN[j][k]);
			}
			Vec2 *pointsN = new Vec2[(int)points.size()];
			Step(outputN[j], gapN[j], points, outputN_array_size[j],pointsN,true);
			if(foldableV(outputN[j], pointsN, outputN_array_size[j], (int)points.size())){
				//cout << "optimized true: " << "\n";
				flg = true;
				//line->foldingPattern.resize(outputN_array_size[j]);;
				line->foldingPattern_size = outputN_array_size[j];
				line->foldingPattern.resize(outputN_array_size[j]);
		
				for(int s=0; s<outputN_array_size[j]; s++){
					line->foldingPattern[s] =outputN[j][s];
					////cout << "outputN: " << line->foldingPattern[s] << "\n"; 
				}
				for(int s=0; s<(int)line->points.size(); s++){
					line->points[s].x = pointsN[s].x;
					line->points[s].y = pointsN[s].y;
					////cout << "points: " << s << ": " << pointsN[s].x << "," << pointsN[s].y << "\n";
				}
				success_flg = true;
			//	//cout << "\n";
				break;
			}else{
				//cout << "optimized error: " << "\n";
				success_flg = false;
				if(j == 0){
					/*//cout << "error\n";
					//cout << "outputN_array_size[j] : " << outputN_array_size[j] << "\n";
					for(int s=0; s<outputN_array_size[j]; s++){
						//cout << "outputN: " << outputN[j][s] << "\n"; 
					}
					for(int s=0; s<(int)line->points.size(); s++){
						//cout << "points: " << s << ": " << pointsN[s].x << "," << pointsN[s].y << "\n";
					}*/
				
				}
				////cout << "optimized error\n";
			}
		}

		if(!success_flg){
			m->fold->outlinepoints[outline_n]->points[point_n] = first;
		}

		delete [] sign;
		delete [] evalu;
		delete [] evalu1;
		delete [] evalu2;
		delete [] evalu3;
		delete [] evaluN;
		delete [] eSort;
		for(int k=0; k<(int)output.size(); k++){
			delete [] output[k];
		}
	
}
void COpenGL::optimization_color(Model *m){

		////cout << "outline_n: " << outline_n << "\n";
		outline *line = m->fold->outlinepoints[outline_n];
		bool flg;
		std::vector<Vec2> points = m->fold->outlinepoints[outline_n]->points;
		std::vector<double> l;
		Vec2 foo;
		
		l.resize((int)points.size()-1);
		for(int j=0; j<(int)points.size()-1; j++){
			foo = points[j] - points[j+1];
			
			double length = foo.length();
			l[j] = length;

		}

		int *sign = new int[(int)l.size()];
		std::vector<double*> output;
		std::vector<double> gap;
		gap.resize((int)pow(2.0,(int)l.size()-2));
		if(points[0].y > points[(int)points.size()-1].y){
			for(int j=0; j<(int)points.size()/2; j++){
				Vec2 t = points[j];
				points[j] = points[(int)points.size()-j-1];
				points[(int)points.size()-j-1] = t;
			}
			for(int j=0; j<(int)l.size(); j++){
				double t = l[j];
				l[j] = l[(int)l.size()-j-1];
				l[(int)l.size()-j-1] = t;
			}
		}

		std::vector<int> output_array_size;
		std::vector<dR> output_dR;
		output_array_size.resize((int)pow(2.0,(int)l.size()-2));
		output_dR.resize((int)pow(2.0,(int)l.size()-2));
		uninfo(0, 0, sign, l, output_dR, gap, output_array_size,0);
		output.resize((int)output_dR.size());
		for(int s=0; s<(int)output_dR.size(); s++){
			double *tmp = new double[(int)output_dR[s].dArray.size()];
			for(int k=0; k<output_array_size[s]; k++){
				tmp[k] = output_dR[s].dArray[k];
			}
			output[s] = tmp;
		}
		
		for(int j=0; j<(int)gap.size(); j++){
			gap[j] = gap[j]-points[(int)points.size()-1].x + points[0].x;
		}

		double *evalu1 = new double[(int)output.size()];
		double *evalu2 = new double[(int)output.size()];
		double *evalu3 = new double[(int)gap.size()];

		func1(output,points,evalu1);
		func2(output,points,evalu2);
		func3(gap,output,evalu3,output_array_size);
		
		double *evalu = new double[(int)output.size()];

		double w1 = 1.0; double w2 = 1.0; double w3 = 1.0;
		for(int j=0; j<(int)output.size(); j++){
			evalu[j] = w1*evalu1[j]+w2*evalu2[j]-w3*evalu3[j];
		}

		int *eSort = new int[(int)output.size()];
		desIndex(evalu, eSort, (int)output.size());
		double *evaluN = new double[(int)output.size()];

		std::vector<double*> outputN;
		std::vector<int> outputN_array_size;
		std::vector<double> gapN;

		outputN.resize((int)output.size());
		outputN_array_size.resize((int)output.size());
		gapN.resize((int)output.size());
		for(int j=0; j<(int)output.size(); j++){
			int index =eSort[j];
			evaluN[j] = evalu[index];
			outputN[j] = output[index];
			outputN_array_size[j] = output_array_size[index];
			gapN[j] = gap[index];
		}

		////cout << "(int)outputN.size(): " << (int)outputN.size() << "\n";
		for(int j=0; j<(int)outputN.size(); j++){
			double L = 0;
			double alpha;
			for(int k=0; k<outputN_array_size[j]; k++){
				L+=fabs(outputN[j][k]);
			}
			alpha = gapN[j]/L;
			for(int k=0; k<outputN_array_size[j]; k++){
				double delta = 1.0;
				if(outputN[j][k] > 0){ delta = -1.0; }
				outputN[j][k] = (1+delta*alpha)*(outputN[j][k]);
			}

			Vec2 *pointsN = new Vec2[(int)points.size()];
			Step(outputN[j], gapN[j], points, outputN_array_size[j],pointsN,true);
			if(foldableV(outputN[j], pointsN, outputN_array_size[j], (int)points.size())){
				////cout << "optimized true: " << "\n";
				line->color_num = point_n;
		
				break;
			}else{
				////cout << "optimized error: " << "\n";
				line->color_num = -1;
			}
		}


		delete [] sign;
		delete [] evalu;
		delete [] evalu1;
		delete [] evalu2;
		delete [] evalu3;
		delete [] evaluN;
		delete [] eSort;
		for(int k=0; k<(int)output.size(); k++){
			delete [] output[k];
		}
	
}
void COpenGL::Step(double *output, double gap, std::vector<Vec2> points, int size, Vec2 *pointsN, bool One){
	
	double *lIdeal = new double[size];

	double *x1 = new double[(int)points.size()];
	double *y1 = new double[(int)points.size()];
	double *x2 = new double[(int)points.size()];
	double *y2 = new double[(int)points.size()];


	for(int i=0; i<(int)points.size(); i++){
		x1[i] = points[i].x;
		x2[i] = x1[i];
		y1[i] = points[i].y;
		y2[i] = y1[i];
		if(size > i){
			lIdeal[i] = fabs(output[i]);
		}
	}
	
	if(One){
		method_one(x1, y1, x2, y2, lIdeal, (int)points.size(), point_n);
	}else{
		method(x1, y1, x2, y2, lIdeal, (int)points.size());
	}
	for(int i=0; i<(int)points.size(); i++){
		pointsN[i] = Vec2(x1[i],y1[i]);
	}
	delete [] x1;
	delete [] x2;
	delete [] y1;
	delete [] y2;
	
}

//アニメーション
void COpenGL::AnimationSet(foldmethod *fold){
	
	//foldingStates = (ArrayList<ArrayList<Vec2>>[])new ArrayList<?>[STEP+1];
	int STEP = 100;
	fold->part_state.clear();
	fold->part_state.resize(STEP+1);

	for(int i=0;i<=STEP;i++){
		for(int j=0;j<(int)fold->outlinepoints.size();j++){
			////cout << "STEP: " << i << " FK: " << j << "\n";
			std::vector<Vec2> returns;
			FK(fold->outlinepoints[j]->points, fold->outlinepoints[j]->foldingPattern,fold->outlinepoints[j]->foldingPattern_size, i,returns);
			fold->part_state[i].push_back(returns);
			
		}
	}

	Animation_adjustHeight(fold->part_state);

}
void COpenGL::setPartlinePoint(foldmethod *fold,int state){
	int STEP_DIV = 20;
	int STEP = 100;
	int INIT = 0;
	int interval = STEP/STEP_DIV;
	//値が変更したら
	////cout << "state: " << state << "\n";
	for(int i=0; i<(int)fold->outlinepoints.size(); i++){
		int point_num = (int)fold->outlinepoints[i]->points.size()-1;
		for(int j=0; j<(int)fold->outlinepoints[i]->points.size(); j++){
			fold->outlinepoints[i]->points[j] = fold->part_state[state][i][point_num-j];
			////cout << fold->outlinepoints[i]->points[j].x << "," << fold->outlinepoints[i]->points[j].y << "\n";
			//fold->outlinepoints[i]->points[j].y = p[i][p[i].size()-1-j].y;
		}
		////cout << "//////////\n";
	}
}
void COpenGL::outputFolding(Model *m){
	std::string file = "outputfolding.txt";
	ofstream output(file);
	output << "Part 0\n";
	for(int i=0; i<(int)m->fold->outlinepoints.size(); i++){
		std::string text = "outlinePoint " + std::to_string((long double)i) + "\n";
		output << text;
		for(int k=0; k<(int)m->fold->outlinepoints[i]->points.size(); k++){
			text = std::to_string((long double)m->fold->outlinepoints[i]->points[k].x) + " " + std::to_string((long double)m->fold->outlinepoints[i]->points[k].y) + "\n";
			output << text;
		}
	}
	output << "pointPosition\n";
	for(int i=0; i<(int)m->fold->pointPosition.size(); i++){
		std::string text = std::to_string((long double)m->fold->pointPosition[i].x) + " " + std::to_string((long double)m->fold->pointPosition[i].y) + "\n";
		output << text;
	}

	output << "betweenPosition\n";
	for(int i=0; i<(int)m->fold->betweenPosition.size(); i++){
		std::string text = std::to_string((long double)m->fold->betweenPosition[i].x) + " " + std::to_string((long double)m->fold->betweenPosition[i].y) + "\n";
		output << text;
	}
	output.close();
}

//void COpenGL::pick3D(int x,int y, int flg , Model *m){
//	/*float current_aspect=0;                        //アスペクト比
//	GLuint selectBuf[100];                   //セレクトションバッファ
//	GLuint hits;                                          //ヒットナンバー
//	GLint viewport[4];                                //ビューポート
//	//セレクション開始
//	glGetIntegerv(GL_VIEWPORT,viewport); //現在のビューポートを代入
//	glSelectBuffer(100,selectBuf);         //バッファの選択
//	glRenderMode(GL_SELECT);          //セレクションに移行
//	glInitNames();                                       //nameバッファの初期化
//	glPushName(0);
//	glMatrixMode(GL_PROJECTION);         //プロジェクションモード
//	glPushMatrix();                      //セレクションモード
//	glLoadIdentity();
//	gluPickMatrix(x,viewport[3]-y, 5.0,5.0, viewport);  //ピッキング行列の乗算5.0,5.0, viewport);
//	current_aspect = (float)viewport[2]/(float)viewport[3];
//	gluPerspective(30.0, current_aspect, 0.1, 10000.0);
//	glMatrixMode(GL_MODELVIEW);
//	std::vector<Vec3> FaceData;
//	if(flg == 0){//面またはモデル
//		pick3D_render_all();
//	}else if(flg == 1){//点
//		pick3D_render(m);
//	}else{//線
//		pick3D_render_line(m);
//	}
//	glMatrixMode(GL_PROJECTION);
//	glPopMatrix();
//	hits = glRenderMode(GL_RENDER);      //ヒットレコードの戻り値
//	int n=selectBuf[0];
//	
//	if(hits > 0){
//		////cout << "point hits; " << hits << "\n";
//		unsigned int smallest = UINT_MAX;
//		
//		line_n = -1;
//		point_n = -1;
//		face_n = -1;
//
//		if(flg == 0){//面またはモデル
//			now_n = selectBuf[3];
//			for(int i=0; i<(int)hits; i++){
//				if(smallest >= selectBuf[i*6+1] && selectBuf[i*6+4] == 0){
//					smallest = selectBuf[i*6+1];
//					face_n = selectBuf[i*6+5];
//					now_n = selectBuf[i*6+3];
//				}
//			}
//			////cout << "face_n: " << face_n << "\n";
//			////cout << "now_n: " << now_n << "\n";
//			if(now_n >= (int)TDdata->parts.size()){
//				now_n = -1;
//			}
//		}else if(flg == 1){//点
//			int outline = selectBuf[3];
//			int point = selectBuf[4];
//			for(int i=0; i<(int)hits; i++){
//				if(smallest >= selectBuf[i*5+1]){
//					smallest = selectBuf[i*5+1];
//					outline = selectBuf[i*5+3];
//					point = selectBuf[i*5+4];
//				}
//			}
//			outline_n = outline;
//			point_n = point;
//			////cout << "outline_n: " << outline_n << "\n";
//			////cout << "point_n: " << point_n << "\n";
//		}else{//線
//			int outline = selectBuf[3];
//			int point = selectBuf[4];
//			for(int i=0; i<(int)hits; i++){
//				if(smallest >= selectBuf[i*5+1]){
//					smallest = selectBuf[i*5+1];
//					outline = selectBuf[i*5+3];
//					point = selectBuf[i*5+4];
//				}
//			}
//			outline_n = outline;
//			line_n = point;
//			////cout << "outline_n: " << outline_n << "\n";
//			////cout << "line_n: " << line_n << "\n";
//		}
//	}else{
//		line_n = -1;
//		point_n = -1;
//		face_n = -1;
//	}*/
//}
//void COpenGL::pick3D_render_all(){
//	if(start){
//		//描画
//		glDisable(GL_CULL_FACE);
//		for(int i=0; i<(int)TDdata->parts.size(); i++){
//			glLoadName(i);
//			pick3D_renderParts(TDdata->parts[i], true,TDdata->parts[i]->cent_move,TDdata->parts[i]->scale);
//			pick3D_render_line(TDdata->parts[i]);
//		}
//	}
//}
//
//void COpenGL::pick3D_renderParts(Model *m, bool flg, Vec3 move, Vec3 scale){
//	foldmethod *f = m->fold;
//	double setSize = 0.01;
//
//	int poly_num = 0;
//	int line_num = 0;
//	int ver_num = 0;
//	//0- ポリゴン 1-頂点 2-線
//	glColor3d(0.7,0.3,0);
//
//	glPushName(0);
//	glPushName(0);
//	glBegin(GL_POLYGON);
//	for(int i=0; i<(int)f->pointPosition.size(); i++){
//		Vec3 Top;
//		Top.set(f->pointPosition[i].x*setSize*scale.x,-f->outlinepoints[0]->points[0].y*setSize*scale.y,f->pointPosition[i].y*setSize*scale.z);
//		Top = Rotates(m->angle,Top);
//		glVertex3d(Top.x+move.x, Top.y+move.y, Top.z+move.z);
//	}
//	glEnd();
//
//	glPointSize(8);
//	glLineWidth(2);
//
//	glLoadName(1);
//	glBegin(GL_POLYGON);
//	for(int i=(int)f->outlinepoints.size()-1; i>=0; i--){
//		int ipp = (i+1)%f->pointPosition.size();
//		Vec2 sweepVec = -1*f->pointPosition[ipp] + f->pointPosition[i];
//		sweepVec.normalize();
//		Vec2 trimVec = f->pointPosition[ipp] - f->pointPosition[i];
//		trimVec.rotate(PI/2.0);
//		trimVec.normalize();
//
//		double gap = f->outlinepoints[i]->points[(int)f->outlinepoints[i]->points.size()-1].x -  f->outlinepoints[i]->points[0].x;
//		double l = f->trimPoint[i].trims[(int)f->trimPoint[i].trims.size()-1].l;
//		Vec3 q0 = Vec3(f->betweenPosition[i].x,f->betweenPosition[i].y,0.0);
//		q0.x += gap*trimVec.x;
//		q0.y += gap*trimVec.y;
//		q0.z += f->outlinepoints[0]->points[f->outlinepoints[0]->points.size()-1].y;
//		q0.x += l*sweepVec.x;
//		q0.y += l*sweepVec.y;
//
//		q0 = q0 * setSize;
//
//		glVertex3d(q0.x*scale.x+move.x, -q0.z*scale.y+move.y, q0.y*scale.z+move.z);Vec3 q0_;q0_.set(q0.x*scale.x, -q0.z*scale.y, q0.y*scale.z);
//		q0_ = Rotates(m->angle, q0_);
//		glVertex3d(q0_.x+move.x, q0_.y+move.y, q0_.z+move.z );
//	}
//	glEnd();
//	glPopName();
//	glPopName();
//
//	poly_num = 2;
//	
//	for(int i=0; i<(int)f->outlinepoints.size(); i++){
//		int ipp = (i+1)%f->pointPosition.size();
//		Vec2 sweepVec = f->pointPosition[ipp] - f->pointPosition[i];
//		sweepVec.normalize();
//		Vec2 trimVec = f->pointPosition[ipp] - f->pointPosition[i];
//		trimVec.rotate(PI/2.0);
//		trimVec.normalize();
//		bool idSwitch = true;
//		int count = 0;
//		int count_ = 0;
//		std::vector<Vec3> betw;
//		
//		for(int j=0; j<(int)f->trimPoint[i].trims.size()-1; j++){
//			int id = f->trimPoint[i].trims[j].id;
//			double alpha = f->trimPoint[i].trims[j].alpha;
//			double l = f->trimPoint[i].trims[j].l;
//			int sameLayer = f->trimPoint[i].trims[j].sameLayer;
//
//			int id1 = f->trimPoint[i].trims[j+1].id;
//			double alpha1 = f->trimPoint[i].trims[j+1].alpha;
//			double l1 = f->trimPoint[i].trims[j+1].l;
//	
//			if(id == id1){
//				if(id > 0){
//					Vec2 outlinei;
//					outlinei = f->outlinepoints[i]->points[id];
//					outlinei -= f->outlinepoints[i]->points[id-1];
//
//					Vec2 foo = trimVec;
//					foo.setLength(alpha*outlinei.x + f->outlinepoints[i]->points[id-1].x);
//					double bar = alpha * outlinei.y + f->outlinepoints[i]->points[id-1].y;
//					Vec3 p0;
//					p0.set(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y,bar);
//					Vec3 p1;
//					p1 = p0;
//					p1.x += sweepVec.x*l;
//					p1.y += sweepVec.y*l;
//
//
//					Vec3 normalCheck0;
//					normalCheck0.set(trimVec.x*outlinei.x,trimVec.y*outlinei.x,outlinei.y);
//
//					normalCheck0.cross(sweepVec.x, sweepVec.y, 0.0);
//					normalCheck0.normalize();
//
//					glNormal3d(normalCheck0.x, -normalCheck0.z,normalCheck0.y);
//
//					p0 = p0*setSize;
//				
//					p0.x *=scale.x;
//					p0.y *=scale.z;
//					p0.z *=scale.y;
//
//					p1 = p1*setSize;
//					p1.x *=scale.x;
//					p1.y *=scale.z;
//					p1.z *=scale.y;
//					
//					glPushName(0);
//					glPushName(poly_num);
//					glBegin(GL_QUADS);
//						Vec3 p0_,p1_,p2_,p3_;
//						p0_.set(p0.x, -p0.z, p0.y);
//						p0 = Rotates(m->angle, p0_);
//						p1_.set(p1.x, -p1.z, p1.y);
//						p1 = Rotates(m->angle, p1_);
//						glVertex3d(p1.x+move.x, p1.y+move.y, p1.z+move.z);
//						glVertex3d(p0.x+move.x, p0.y+move.y, p0.z+move.z);
//					
//						foo = trimVec;
//						foo.setLength(alpha1*outlinei.x+f->outlinepoints[i]->points[id-1].x);
//						bar = alpha1*outlinei.y+f->outlinepoints[i]->points[id-1].y;
//						Vec3 p2 = Vec3(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y, bar);
//						Vec3 p3;
//						p3 = p2;
//						p3.x += sweepVec.x*l1;
//						p3.y += sweepVec.y*l1;
//
//						p2 = p2*setSize;
//						p3 = p3*setSize;
//
//						p2.x *=scale.x;
//						p2.y *=scale.z;
//						p2.z *=scale.y;
//
//						p3.x *=scale.x;
//						p3.y *=scale.z;
//						p3.z *=scale.y;
//
//						p2_.set(p2.x, -p2.z, p2.y);
//						p2 = Rotates(m->angle, p2_);
//						p3_.set(p3.x, -p3.z, p3.y);
//						p3 = Rotates(m->angle, p3_);
//
//						glVertex3d(p2.x+move.x, p2.y+move.y, p2.z+move.z);
//						glVertex3d(p3.x+move.x, p3.y+move.y, p3.z+move.z);
//
//					glEnd();
//					poly_num++;
//					glPopName();
//					glPopName();
//
//					glEnable(GL_LIGHTING);
//				}else{
//					glColor3d(0,0.3,1.0);
//					if(idSwitch){
//						idSwitch = false;
//					}
//					id*=-1;
//
//					Vec2 outlinei;
//					outlinei = f->outlinepoints[i]->points[id];
//					outlinei -= f->outlinepoints[i]->points[id-1];
//
//					Vec2 foo = trimVec;
//					foo.setLength(alpha*outlinei.x + f->outlinepoints[i]->points[id-1].x);
//					double bar = alpha * outlinei.y + f->outlinepoints[i]->points[id-1].y;
//					Vec3 p0;
//					p0.set(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y,bar);
//					Vec3 p1;
//					p1 = p0;
//					p1.x += -sweepVec.x*l;
//					p1.y += -sweepVec.y*l;
//
//					p1 = p1*setSize;
//					p0 = p0*setSize;
//
//					p0.x *=scale.x;
//					p0.y *=scale.z;
//					p0.z *=scale.y;
//
//					p1.x *=scale.x;
//					p1.y *=scale.z;
//					p1.z *=scale.y;
//
//
//
//					//こっちがおかしい
//					glEnable(GL_LIGHTING);
//					glPushName(0);
//					glPushName(poly_num);
//					glBegin(GL_QUADS);
//						Vec3 p0_,p1_,p2_,p3_;
//						p0_.set(p0.x, -p0.z, p0.y);
//						p0 = Rotates(m->angle, p0_);
//						p1_.set(p1.x, -p1.z, p1.y);
//						p1 = Rotates(m->angle, p1_);
//
//						glVertex3d(p1.x+move.x, p1.y+move.y, p1.z+move.z);
//						glVertex3d(p0.x+move.x, p0.y+move.y, p0.z+move.z);
//				
//						foo = trimVec;
//						foo.setLength(alpha1*outlinei.x+f->outlinepoints[i]->points[id-1].x);
//						bar = alpha1*outlinei.y+f->outlinepoints[i]->points[id-1].y;
//						Vec3 p2 = Vec3(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y, bar);
//						Vec3 p3;
//						p3 = p2;
//						p3.x += -sweepVec.x*l1;
//						p3.y += -sweepVec.y*l1;
//
//						p2 = p2*setSize;
//						p3 = p3*setSize;
//
//						p2.x *=scale.x;
//						p2.y *=scale.z;
//						p2.z *=scale.y;
//
//						p3.x *=scale.x;
//						p3.y *=scale.z;
//						p3.z *=scale.y;
//					
//						p2_.set(p2.x, -p2.z, p2.y);
//						p2 = Rotates(m->angle, p2_);
//						p3_.set(p3.x, -p3.z, p3.y);
//						p3 = Rotates(m->angle, p3_);
//
//						glVertex3d(p2.x+move.x, p2.y+move.y, p2.z+move.z);
//						glVertex3d(p3.x+move.x, p3.y+move.y, p3.z+move.z);
//					glEnd();
//					glPopName();
//					glPopName();
//					poly_num++;
//				
//					if(flg){
//						Vec3 between,between2;
//						if(count == 0){
//							between = (p0 + p1)/2.0;
//							between2 = (p2 + p3)/2.0;
//						}else{
//							Vec3 nor;
//							double ax;
//							nor = p1-p0; 
//							nor.normalize();
//							ax = nor*(betw[(int)betw.size()-1]-p0);
//							between = p0 + ( nor * ax );
//						}
//
//						if(f->outlinepoints[i]->points[count] == f->outlinepoints[i]->points[id-1] && count != 0){
//							if(count == f->outlinepoints[i]->color_num){
//								glColor3d(0.0,0,1.0);
//							}else{
//								glColor3d(1.0,0,0.0);
//							}
//							glPushName(1);
//							glPushName(ver_num);
//							glBegin(GL_POINTS);
//								glVertex3d(between.x+move.x, between.y+move.y, between.z+move.z);
//								betw.push_back(between);
//							glEnd();
//							ver_num++;
//							glPopName();
//							glPopName();
//							count++;
//						}
//
//						if(count == 0){
//							betw.push_back(between);
//							count++;
//						}
//
//						Vec3 nor;
//						double ax;
//						nor = p3-p2; nor.normalize();
//						ax = nor*(between-p2);
//						between2 = p2 + ( nor * ax );
//						if(f->outlinepoints[i]->points[count_] == f->outlinepoints[i]->points[id-1] && count_ < (int)f->outlinepoints[i]->points.size()){
//							////cout << "In pick line_num: " << line_num << "\n";
//							line_num++;
//							count_++;
//						}
//						if(f->outlinepoints[i]->points[count-1] == f->outlinepoints[i]->points[id-1] && count-1 < (int)f->outlinepoints[i]->points.size()){
//							////cout << "In pick line_num: " << line_num << "\n";
//							glPushName(2);
//							glPushName(line_num-1);
//							glBegin(GL_LINES);
//								glVertex3d(between.x+move.x, between.y+move.y, between.z+move.z);
//								glVertex3d(between2.x+move.x, between2.y+move.y, between2.z+move.z);
//							glEnd();
//							glPopName();
//							glPopName();
//							//line_num++;
//						}
//					}
//				}
//			}else{
//				//線を引くだけ
//			}
//		}
//		////cout << "\n";
//		idSwitch = true;
//	}
//
//	
//}
//
//void COpenGL::pick3D_render(Model *m){
//	foldmethod *f = m->fold;
//	Vec3 move = m->cent_move;
//	double setSize = 0.01;
//
//	glPointSize(10);
//
//	for(int i=0; i<(int)f->outlinepoints.size(); i++){
//		int ipp = (i+1)%f->pointPosition.size();
//		Vec2 sweepVec = f->pointPosition[ipp] - f->pointPosition[i];
//		sweepVec.normalize();
//		Vec2 trimVec = f->pointPosition[ipp] - f->pointPosition[i];
//		trimVec.rotate(PI/2.0);
//		trimVec.normalize();
//		bool idSwitch = true;
//		int before_num = 0;
//		int count = 0;
//		int count_ = 0;
//		std::vector<Vec3> betw;
//		glLoadName(i);
//		glPushName(0);
//		for(int j=0; j<(int)f->trimPoint[i].trims.size()-1; j++){
//			int id = f->trimPoint[i].trims[j].id;
//			double alpha = f->trimPoint[i].trims[j].alpha;
//			double l = f->trimPoint[i].trims[j].l;
//			int sameLayer = f->trimPoint[i].trims[j].sameLayer;
//
//			int id1 = f->trimPoint[i].trims[j+1].id;
//			double alpha1 = f->trimPoint[i].trims[j+1].alpha;
//			double l1 = f->trimPoint[i].trims[j+1].l;
//			if(id == id1){
//				if(id > 0){
//					Vec2 outlinei;
//					outlinei = f->outlinepoints[i]->points[id];
//					outlinei -= f->outlinepoints[i]->points[id-1];
//
//					Vec2 foo = trimVec;
//					foo.setLength(alpha*outlinei.x + f->outlinepoints[i]->points[id-1].x);
//					double bar = alpha * outlinei.y + f->outlinepoints[i]->points[id-1].y;
//
//					Vec3 p0;
//					p0.set(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y,bar);
//					
//					Vec3 p1;
//					p1 = p0;
//					p1.x += sweepVec.x*l;
//					p1.y += sweepVec.y*l;
//
//					p0 = p0*setSize;
//					p1 = p1*setSize;
//
//					p0.x *=m->scale.x;
//					p0.y *=m->scale.z;
//					p0.z *=m->scale.y;
//
//					p1.x *=m->scale.x;
//					p1.y *=m->scale.z;
//					p1.z *=m->scale.y;
//
//					Vec3 p0_,p1_,p2_,p3_;
//					p0_.set(p0.x, -p0.z, p0.y);
//					p0 = Rotates(m->angle, p0_);
//					p1_.set(p1.x, -p1.z, p1.y);
//					p1 = Rotates(m->angle, p1_);
//
//					foo = trimVec;
//					foo.setLength(alpha1*outlinei.x+f->outlinepoints[i]->points[id-1].x);
//					bar = alpha1*outlinei.y+f->outlinepoints[i]->points[id-1].y;
//					Vec3 p2 = Vec3(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y, bar);
//					Vec3 p3;
//					p3 = p2;
//					p3.x += sweepVec.x*l1;
//					p3.y += sweepVec.y*l1;
//
//					p2 = p2*setSize;
//					p3 = p3*setSize;
//
//					p2.x *=m->scale.x;
//					p2.y *=m->scale.z;
//					p2.z *=m->scale.y;
//
//					p3.x *=m->scale.x;
//					p3.y *=m->scale.z;
//					p3.z *=m->scale.y;
//
//					p2_.set(p2.x, -p2.z, p2.y);
//					p2 = Rotates(m->angle, p2_);
//					p3_.set(p3.x, -p3.z, p3.y);
//					p3 = Rotates(m->angle, p3_);
//
//				}else{
//					if(idSwitch){
//						idSwitch = false;
//					}
//					id*=-1;
//
//					Vec2 outlinei;
//					outlinei = f->outlinepoints[i]->points[id];
//					outlinei -= f->outlinepoints[i]->points[id-1];
//
//					Vec2 foo = trimVec;
//					foo.setLength(alpha*outlinei.x + f->outlinepoints[i]->points[id-1].x);
//					double bar = alpha * outlinei.y + f->outlinepoints[i]->points[id-1].y;
//					
//					Vec3 p0;
//					p0.set(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y,bar);
//					Vec3 p1;
//					p1 = p0;
//					p1.x += -sweepVec.x*l;
//					p1.y += -sweepVec.y*l;
//
//					p1 = p1*setSize;
//					p0 = p0*setSize;
//
//					p0.x *=m->scale.x;
//					p0.y *=m->scale.z;
//					p0.z *=m->scale.y;
//
//					p1.x *=m->scale.x;
//					p1.y *=m->scale.z;
//					p1.z *=m->scale.y;
//
//					Vec3 p0_,p1_,p2_,p3_;
//					p0_.set(p0.x, -p0.z, p0.y);
//					p0 = Rotates(m->angle, p0_);
//					p1_.set(p1.x, -p1.z, p1.y);
//					p1 = Rotates(m->angle, p1_);
//					
//					foo = trimVec;
//					foo.setLength(alpha1*outlinei.x+f->outlinepoints[i]->points[id-1].x);
//					bar = alpha1*outlinei.y+f->outlinepoints[i]->points[id-1].y;
//					Vec3 p2 = Vec3(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y, bar);
//					Vec3 p3;
//					p3 = p2;
//					p3.x += -sweepVec.x*l1;
//					p3.y += -sweepVec.y*l1;
//
//					p2 = p2*setSize;
//					p3 = p3*setSize;
//
//					p2.x *=m->scale.x;
//					p2.y *=m->scale.z;
//					p2.z *=m->scale.y;
//
//					p3.x *=m->scale.x;
//					p3.y *=m->scale.z;
//					p3.z *=m->scale.y;
//
//					p2_.set(p2.x, -p2.z, p2.y);
//					p2 = Rotates(m->angle, p2_);
//					p3_.set(p3.x, -p3.z, p3.y);
//					p3 = Rotates(m->angle, p3_);
//
//					Vec3 between,between2;
//
//					//////
//					if(count == 0){
//						between = p0;
//						between2 = p3;
//					}else{
//						Vec3 nor;
//						double ax;
//						nor = p1-p0; nor.normalize();
//						ax = nor*(betw[(int)betw.size()-1]-p0);
//						between = p0 + ( nor * ax );
//					}
//
//					if(f->outlinepoints[i]->points[count] == f->outlinepoints[i]->points[id-1]){
//						glLoadName(count);
//						glBegin(GL_POINTS);
//							glVertex3d(between.x+move.x, between.y+move.y, between.z+move.z);
//							betw.push_back(between);
//						glEnd();
//						count++;
//					}
//
//					if(count == f->outlinepoints[i]->points.size()-1 && j == (int)f->trimPoint[i].trims.size()-2){
//						Vec3 nor;
//						double ax;
//						nor = p3-p2; nor.normalize();
//						ax = nor*(between-p2);
//						between2 = p2 + ( nor * ax );
//
//						glLoadName(count);
//						glBegin(GL_POINTS);
//							glVertex3d(between2.x+move.x, between2.y+move.y, between2.z+move.z);
//						glEnd();
//						////cout << "count: " << count << "," << j << "\n";
//					}
//				}
//			}
//		}
//		glPopName();
//		idSwitch = true;
//	}
//	
//}
//void COpenGL::pick3D_render_line(Model *m){
//	Vec3 move = m->cent_move;
//	foldmethod *f = m->fold;
//	double setSize = 0.01;
//
//	//glPointSize(10);
//	glLineWidth(10);
//
//	for(int i=0; i<(int)f->outlinepoints.size(); i++){
//		int ipp = (i+1)%f->pointPosition.size();
//		Vec2 sweepVec = f->pointPosition[ipp] - f->pointPosition[i];
//		sweepVec.normalize();
//		Vec2 trimVec = f->pointPosition[ipp] - f->pointPosition[i];
//		trimVec.rotate(PI/2.0);
//		trimVec.normalize();
//		bool idSwitch = true;
//		int before_num = 0;
//		int count = 0;
//		int count_ = 0;
//		std::vector<Vec3> betw;
//		glLoadName(i);
//		glPushName(0);
//		for(int j=0; j<(int)f->trimPoint[i].trims.size()-1; j++){
//			int id = f->trimPoint[i].trims[j].id;
//			double alpha = f->trimPoint[i].trims[j].alpha;
//			double l = f->trimPoint[i].trims[j].l;
//			int sameLayer = f->trimPoint[i].trims[j].sameLayer;
//
//			int id1 = f->trimPoint[i].trims[j+1].id;
//			double alpha1 = f->trimPoint[i].trims[j+1].alpha;
//			double l1 = f->trimPoint[i].trims[j+1].l;
//			if(id == id1){
//				if(id > 0){
//					/*Vec2 outlinei;
//					outlinei = f->outlinepoints[i]->points[id];
//					outlinei -= f->outlinepoints[i]->points[id-1];
//
//					Vec2 foo = trimVec;
//					foo.setLength(alpha*outlinei.x + f->outlinepoints[i]->points[id-1].x);
//					double bar = alpha * outlinei.y + f->outlinepoints[i]->points[id-1].y;
//
//					Vec3 p0;
//					p0.set(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y,bar);
//					
//					Vec3 p1;
//					p1 = p0;
//					p1.x += sweepVec.x*l;
//					p1.y += sweepVec.y*l;
//
//					p0.x *= setSize;
//					p0.y *= setSize;
//					p0.z *= setSize;
//					p1.x *= setSize;
//					p1.y *= setSize;
//					p1.z *= setSize;
//
//					foo = trimVec;
//					foo.setLength(alpha1*outlinei.x+f->outlinepoints[i]->points[id-1].x);
//					bar = alpha1*outlinei.y+f->outlinepoints[i]->points[id-1].y;
//					Vec3 p2 = Vec3(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y, bar);
//					Vec3 p3;
//					p3 = p2;
//					p3.x += sweepVec.x*l1;
//					p3.y += sweepVec.y*l1;
//
//					p2 = p2*setSize;
//					p3 = p3*setSize;*/
//
//				}else{
//					if(idSwitch){
//						idSwitch = false;
//					}
//					id*=-1;
//
//					Vec2 outlinei;
//					outlinei = f->outlinepoints[i]->points[id];
//					outlinei -= f->outlinepoints[i]->points[id-1];
//
//					Vec2 foo = trimVec;
//					foo.setLength(alpha*outlinei.x + f->outlinepoints[i]->points[id-1].x);
//					double bar = alpha * outlinei.y + f->outlinepoints[i]->points[id-1].y;
//					
//					Vec3 p0;
//					p0.set(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y,bar);
//					Vec3 p1;
//					p1 = p0;
//					p1.x += -sweepVec.x*l;
//					p1.y += -sweepVec.y*l;
//
//					if(i == 0){
//						////cout << "after: " << p1.x+m->cent_move.x << "," <<  p1.y+m->cent_move.y<< "," << p1.z+m->cent_move.z << "\n";
//						////cout << "after: " << p1.x << "," <<  p1.y<< "," << p1.z << "\n";
//						////cout << "after: " << m->cent_move.x << "," <<  m->cent_move.y<< "," << m->cent_move.z << "\n";
//					}
//
//					p1 = p1*setSize;
//					p0 = p0*setSize;
//
//					p0.x *=m->scale.x;
//					p0.y *=m->scale.z;
//					p0.z *=m->scale.y;
//
//					p1.x *=m->scale.x;
//					p1.y *=m->scale.z;
//					p1.z *=m->scale.y;
//					
//					foo = trimVec;
//					foo.setLength(alpha1*outlinei.x+f->outlinepoints[i]->points[id-1].x);
//					bar = alpha1*outlinei.y+f->outlinepoints[i]->points[id-1].y;
//					Vec3 p2 = Vec3(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y, bar);
//					Vec3 p3;
//					p3 = p2;
//					p3.x += -sweepVec.x*l1;
//					p3.y += -sweepVec.y*l1;
//
//					p2 = p2*setSize;
//					p3 = p3*setSize;
//
//					p2.x *=m->scale.x;
//					p2.y *=m->scale.z;
//					p2.z *=m->scale.y;
//
//					p3.x *=m->scale.x;
//					p3.y *=m->scale.z;
//					p3.z *=m->scale.y;
//
//					Vec3 p0_,p1_,p2_,p3_;
//					p0_.set(p0.x, -p0.z, p0.y);
//					p0 = Rotates(m->angle, p0_);
//					p1_.set(p1.x, -p1.z, p1.y);
//					p1 = Rotates(m->angle, p1_);
//
//					p2_.set(p2.x, -p2.z, p2.y);
//					p2 = Rotates(m->angle, p2_);
//					p3_.set(p3.x, -p3.z, p3.y);
//					p3 = Rotates(m->angle, p3_);
//					
//					Vec3 between,between2;
//
//					if(count_ == 0){
//						between = p0;
//						between2 = p3;
//					}else{
//						Vec3 nor;
//						double ax;
//						nor = p1-p0; nor.normalize();
//						ax = nor*(betw[(int)betw.size()-1]-p0);
//						between = p0 + ( nor * ax );
//					}
//
//					if(f->outlinepoints[i]->points[count_] == f->outlinepoints[i]->points[id-1]){
//						betw.push_back(between);
//						count_++;
//					}
//
//					Vec3 nor;
//					double ax;
//					nor = p3-p2; nor.normalize();
//					ax = nor*(between-p2);
//					between2 = p2 + ( nor * ax );
//
//					if(f->outlinepoints[i]->points[count] == f->outlinepoints[i]->points[id-1] && count < (int)f->outlinepoints[i]->points.size()){
//						glLoadName(count);
//						count++;
//					}
//
//					glDisable(GL_LIGHTING);
//					glBegin(GL_LINES);
//						glVertex3d(between.x+move.x, between.y+move.y, between.z+move.z);
//						glVertex3d(between2.x+move.x, between2.y+move.y, between2.z+move.z);
//					glEnd();
//					glEnable(GL_LIGHTING);
//				}
//			}
//		}
//		glPopName();
//		idSwitch = true;
//	}
//	
//}
//void COpenGL::pick3D_render_face(Model *m, std::vector<Vec3> &FaceData){
//	foldmethod *f = m->fold;
//	double setSize = 0.01;
//	int count=0;
//	Vec3 Normals;
//	Vec3 Face_cen; Face_cen.set(0,0,0);
//
//	glLoadName(count);
//	glBegin(GL_POLYGON);
//	for(int i=0; i<(int)f->pointPosition.size(); i++){
//		glVertex3d(f->pointPosition[i].x*setSize,-f->outlinepoints[0]->points[0].y*setSize,f->pointPosition[i].y*setSize);
//		 Face_cen.x = f->pointPosition[i].x*setSize;
//		 Face_cen.y = -f->outlinepoints[0]->points[0].y*setSize;
//		 Face_cen.z = f->pointPosition[i].y*setSize;
//	}
//	glEnd();
//	FaceData.push_back(Vec3(0,1,0));
//	Face_cen = Face_cen / (double)f->pointPosition.size();
//	FaceData.push_back(Face_cen);
//	count++;
//
//	Face_cen.set(0,0,0);
//	FaceData.push_back(Vec3(0,-1,0));
//
//	glLoadName(count);
//	glBegin(GL_POLYGON);
//	for(int i=(int)f->outlinepoints.size()-1; i>=0; i--){
//		int ipp = (i+1)%f->pointPosition.size();
//		Vec2 sweepVec = -1*f->pointPosition[ipp] + f->pointPosition[i];
//		sweepVec.normalize();
//		Vec2 trimVec = f->pointPosition[ipp] - f->pointPosition[i];
//		trimVec.rotate(PI/2.0);
//		trimVec.normalize();
//
//		double gap = f->outlinepoints[i]->points[(int)f->outlinepoints[i]->points.size()-1].x -  f->outlinepoints[i]->points[0].x;
//		double l = f->trimPoint[i].trims[(int)f->trimPoint[i].trims.size()-1].l;
//		Vec3 q0 = Vec3(f->betweenPosition[i].x,f->betweenPosition[i].y,0.0);
//		q0.x += gap*trimVec.x;
//		q0.y += gap*trimVec.y;
//		q0.z += f->outlinepoints[0]->points[f->outlinepoints[0]->points.size()-1].y;
//		q0.x += l*sweepVec.x;
//		q0.y += l*sweepVec.y;
//		q0 = q0 * setSize;
//		glVertex3d(q0.x, -q0.z, q0.y);
//
//		Face_cen.x += q0.x;
//		Face_cen.y += -q0.z;
//		Face_cen.z += q0.y;
//	}
//	Face_cen = Face_cen / (double)f->outlinepoints.size();
//	FaceData.push_back(Face_cen);
//
//	glEnd();
//	count++;
//
//	for(int i=0; i<(int)f->outlinepoints.size(); i++){
//		int ipp = (i+1)%f->pointPosition.size();
//		Vec2 sweepVec = f->pointPosition[ipp] - f->pointPosition[i];
//		sweepVec.normalize();
//		Vec2 trimVec = f->pointPosition[ipp] - f->pointPosition[i];
//		trimVec.rotate(PI/2.0);
//		trimVec.normalize();
//		bool idSwitch = true;
//		
//		std::vector<Vec3> betw;
//		for(int j=0; j<(int)f->trimPoint[i].trims.size()-1; j++){
//			int id = f->trimPoint[i].trims[j].id;
//			double alpha = f->trimPoint[i].trims[j].alpha;
//			double l = f->trimPoint[i].trims[j].l;
//			int sameLayer = f->trimPoint[i].trims[j].sameLayer;
//
//			int id1 = f->trimPoint[i].trims[j+1].id;
//			double alpha1 = f->trimPoint[i].trims[j+1].alpha;
//			double l1 = f->trimPoint[i].trims[j+1].l;
//			if(id == id1){
//				if(id > 0){
//					Vec2 outlinei;
//					outlinei = f->outlinepoints[i]->points[id];
//					outlinei -= f->outlinepoints[i]->points[id-1];
//
//					Vec2 foo = trimVec;
//					foo.setLength(alpha*outlinei.x + f->outlinepoints[i]->points[id-1].x);
//					double bar = alpha * outlinei.y + f->outlinepoints[i]->points[id-1].y;
//					Vec3 p0;
//					p0.set(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y,bar);
//					Vec3 p1;
//					p1 = p0;
//					p1.x += sweepVec.x*l;
//					p1.y += sweepVec.y*l;
//
//					p0.x *= setSize;
//					p0.y *= setSize;
//					p0.z *= setSize;
//					p1.x *= setSize;
//					p1.y *= setSize;
//					p1.z *= setSize;
//
//					Vec3 Pos; Pos.set(0,0,0);
//					Vec3 normalCheck0;
//					normalCheck0.set(trimVec.x*outlinei.x,trimVec.y*outlinei.x,outlinei.y);
//					normalCheck0.cross(sweepVec.x, sweepVec.y, 0.0);
//					normalCheck0.normalize();
//					FaceData.push_back(normalCheck0);
//
//					glLoadName(count);
//					glBegin(GL_QUADS);
//						glVertex3d(p1.x, -p1.z, p1.y);
//						glVertex3d(p0.x, -p0.z, p0.y);
//						Pos.x += p1.x; Pos.y += -p1.z; Pos.z = p1.y;
//						Pos.x += p0.x; Pos.y += -p0.z; Pos.z = p0.y;
//					
//						foo = trimVec;
//						foo.setLength(alpha1*outlinei.x+f->outlinepoints[i]->points[id-1].x);
//						bar = alpha1*outlinei.y+f->outlinepoints[i]->points[id-1].y;
//						Vec3 p2 = Vec3(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y, bar);
//						Vec3 p3;
//						p3 = p2;
//						p3.x += sweepVec.x*l1;
//						p3.y += sweepVec.y*l1;
//
//						p2 = p2*setSize;
//						p3 = p3*setSize;
//						glVertex3d(p2.x, -p2.z, p2.y);
//						glVertex3d(p3.x, -p3.z, p3.y);
//						Pos.x += p2.x; Pos.y += -p2.z; Pos.z = p2.y;
//						Pos.x += p3.x; Pos.y += -p3.z; Pos.z = p3.y;
//					glEnd();
//
//					Pos = Pos / 4.0; FaceData.push_back(Pos);
//					count++;
//				}else{
//					glColor3d(0,0.3,1.0);
//					if(idSwitch){
//						idSwitch = false;
//					}
//					id*=-1;
//
//					Vec2 outlinei;
//					outlinei = f->outlinepoints[i]->points[id];
//					outlinei -= f->outlinepoints[i]->points[id-1];
//
//					Vec2 foo = trimVec;
//					foo.setLength(alpha*outlinei.x + f->outlinepoints[i]->points[id-1].x);
//					double bar = alpha * outlinei.y + f->outlinepoints[i]->points[id-1].y;
//					Vec3 p0;
//					p0.set(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y,bar);
//					Vec3 p1;
//					p1 = p0;
//					p1.x += -sweepVec.x*l;
//					p1.y += -sweepVec.y*l;
//
//					p1 = p1*setSize;
//					p0 = p0*setSize;
//					
//					Vec3 Pos; Pos.set(0,0,0);
//					Vec3 normalCheck0;
//					normalCheck0.set(trimVec.x*outlinei.x,trimVec.y*outlinei.x,outlinei.y);
//					normalCheck0.cross(sweepVec.x, sweepVec.y, 0.0);
//					normalCheck0.normalize();
//					FaceData.push_back(normalCheck0);
//
//					glLoadName(count);
//					glBegin(GL_QUADS);
//						glVertex3d(p0.x, -p0.z, p0.y);
//						glVertex3d(p1.x, -p1.z, p1.y);
//						Pos.x += p1.x; Pos.y += -p1.z; Pos.z = p1.y;
//						Pos.x += p0.x; Pos.y += -p0.z; Pos.z = p0.y;
//
//						foo = trimVec;
//						foo.setLength(alpha1*outlinei.x+f->outlinepoints[i]->points[id-1].x);
//						bar = alpha1*outlinei.y+f->outlinepoints[i]->points[id-1].y;
//						Vec3 p2 = Vec3(foo.x+f->betweenPosition[i].x, foo.y+f->betweenPosition[i].y, bar);
//						Vec3 p3;
//						p3 = p2;
//						p3.x += -sweepVec.x*l1;
//						p3.y += -sweepVec.y*l1;
//
//						p2 = p2*setSize;
//						p3 = p3*setSize;
//						glVertex3d(p3.x, -p3.z, p3.y);
//						glVertex3d(p2.x, -p2.z, p2.y);
//						Pos.x += p2.x; Pos.y += -p2.z; Pos.z = p2.y;
//						Pos.x += p3.x; Pos.y += -p3.z; Pos.z = p3.y;
//					glEnd();
//					Pos = Pos / 4.0; FaceData.push_back(Pos);
//					count++;
//				}
//			}
//		}
//		////cout << "\n";
//		idSwitch = true;
//	}
//}
//void render3D_pick_Top(Model *m){
//	foldmethod *f = m->fold;
//	double setSize = 0.01;
//	Vec3 move = m->cent_move;
//	Vec3 scale = m->scale;
//
//	glPointSize(8);
//	
//	for(int i=0; i<(int)f->pointPosition.size()-1; i++){
//		Vec3 Top;
//		glLoadName(i);
//		Top.set(f->pointPosition[i].x*setSize*scale.x,-f->outlinepoints[0]->points[0].y*setSize*scale.y,f->pointPosition[i].y*setSize*scale.z);
//		Top = Rotates(m->angle,Top);
//		glBegin(GL_POINTS);
//		glVertex3d(Top.x+move.x, Top.y+move.y, Top.z+move.z);
//		glEnd();
//	}
//}

void COpenGL::ChangeTop( Model *m ,int x, int y){
	/*foldmethod *fold = m->fold;
	float current_aspect=0;                        //アスペクト比
	GLuint selectBuf[100];                   //セレクトションバッファ
	GLuint hits;                                          //ヒットナンバー
	GLint viewport[4];                                //ビューポート
	//セレクション開始
	glGetIntegerv(GL_VIEWPORT,viewport); //現在のビューポートを代入
	glSelectBuffer(100,selectBuf);         //バッファの選択
	glRenderMode(GL_SELECT);          //セレクションに移行
	glInitNames();                                       //nameバッファの初期化
	glPushName(0);
	glMatrixMode(GL_PROJECTION);         //プロジェクションモード
	glPushMatrix();                      //セレクションモード
	glLoadIdentity();
	gluPickMatrix(x,viewport[3]-y, 5.0,5.0, viewport);  //ピッキング行列の乗算5.0,5.0, viewport);
	current_aspect = (float)viewport[2]/(float)viewport[3];
	gluPerspective(30.0, current_aspect, 0.1, 10000.0);
	glMatrixMode(GL_MODELVIEW);
	
	render3D_pick_Top(m);

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	hits = glRenderMode(GL_RENDER);      //ヒットレコードの戻り値
	int n=selectBuf[3];
	unsigned int smallest = UINT_MAX;
	for(int i=0; i<(int)hits; i++){
		if(smallest > selectBuf[4*i+1]){
			n = selectBuf[4*i+3];
		}
	}

	if(hits == 0){
		Top_point_n = -1;
	}else{
		Top_point_n = n;
	}
	//cout << "Top_point_m: " << Top_point_n << "\n";*/
}

void COpenGL::move_point(int line_num, int point_num, foldmethod *fold, double x,double y){
	//座標変換

	if(point_num == 0){
		return;
	}

	outline *line = fold->outlinepoints[line_num];
	line->points[point_num].x -= x;
	
	if(line->points[point_num].y+y > line->points[point_num-1].y && line->points[point_num].y+y < line->points[point_num+1].y &&  point_num != (int)fold->outlinepoints[line_num]->points.size()-1 && point_num != 0){
		line->points[point_num].y += y;
	}
	
}

//面とのトリム
void COpenGL::CrossSection(Model *m){
	m->plane_point.clear();
	m->plane_point_2D.clear();

	std::list<Halfedge*>::iterator it_h;

	Vec3 P = TDdata->plane_cent_Top;
	Vec3 N = TDdata->plane_normal;
	Vec3 base(0,0,1);
	Vec3 angle = Angles(TDdata->plane_normal,base);
	
	for(it_h=m->halfs.begin(); it_h!=m->halfs.end(); it_h++){ 
		////cout << "halfs\n";
		Vec3 A = (*it_h)->vertex->p;
		Vec3 B = (*it_h)->next->vertex->p;
		if(((P-A)*N >=0 && (P-B)*N <= 0) || ((P-A)*N <=0 && (P-B)*N >= 0)){//交差
			Vec3 crossP;
			crossP = A + (B-A)*(abs((P-A)*N )/(abs((P-A)*N) + abs((P-B)*N)));
			Vec3 rotated;
			rotated = Rotates(angle,crossP);
			m->plane_point_2D.push_back(Vec2(rotated.x,rotated.y));
			m->plane_point.push_back(crossP);
		}
	}
}
void COpenGL::CrossSectionBottom(Model *m){
	m->plane_point_bottom.clear();
	m->plane_point_2D_bottom.clear();

	std::list<Halfedge*>::iterator it_h;

	Vec3 P = TDdata->plane_cent;
	Vec3 N = TDdata->plane_normal;
	Vec3 base(0,0,1);
	Vec3 angle = Angles(TDdata->plane_normal,base);
	
	for(it_h=m->halfs.begin(); it_h!=m->halfs.end(); it_h++){ 
		////cout << "halfs\n";
		Vec3 A = (*it_h)->vertex->p;
		Vec3 B = (*it_h)->next->vertex->p;
		if(((P-A)*N >=0 && (P-B)*N <= 0) || ((P-A)*N <=0 && (P-B)*N >= 0)){//交差
			Vec3 crossP;
			crossP = A + (B-A)*(abs((P-A)*N )/(abs((P-A)*N) + abs((P-B)*N)));
			Vec3 rotated;
			rotated = Rotates(angle,crossP);
			m->plane_point_2D_bottom.push_back(Vec2(rotated.x,rotated.y));
			m->plane_point_bottom.push_back(crossP);
		}
	}
}

void COpenGL::CrossSection(Vec3 P, Vec3 N, Vec3 Top , Vec3 Bottom, Vec3 dir, std::vector<Vec3> &plane_, Model *m){

	std::list<Halfedge*>::iterator it_h;
	Vec3 base(0,0,1);
	Vec3 angle = Angles(TDdata->plane_normal,base);
	
	for(it_h=m->halfs.begin(); it_h!=m->halfs.end(); it_h++){ 
		////cout << "halfs\n";
		Vec3 A = (*it_h)->vertex->p;
		Vec3 B = (*it_h)->next->vertex->p;
		if(((P-A)*N >=0 && (P-B)*N <= 0) || ((P-A)*N <=0 && (P-B)*N >= 0)){//交差
			Vec3 crossP;
			crossP = A + (B-A)*(abs((P-A)*N )/(abs((P-A)*N) + abs((P-B)*N)));
			Vec3 rotated;
			rotated = Rotates(angle,crossP);
			Vec3 dir2 = crossP - Vec3(Top.x , crossP.y , Top.z);
			if(Top.y > crossP.y && Bottom.y < crossP.y && dir2*dir > 0){
				plane_.push_back(crossP);
			}
		}
	}
}
int getN(Vec2 vec, const std::vector<Vec2> points){
	for(int i=0; i<(int)points.size(); i++){
		if(vec.x == points[i].x && vec.y ==points[i].y){
			return i;
		}
	}
	return 0;
}
double distance_point_line(Vec2 P, Vec2 A, Vec2 B){//点PからABへの最短距離
	Vec2 ABnor = B-A; ABnor.normalize();
	double Ax = ABnor * (P-A);
	Vec2 x = A + (ABnor*Ax);

	return (P-x).length();
}

double distance_point_line_3d(Vec3 P, Vec3 A, Vec3 B){//点PからABへの最短距離
	Vec3 ABnor = B-A; ABnor.normalize();
	double Ax = ABnor * (P-A);
	Vec3 x = A + (ABnor*Ax);

	return (P-x).length();
}
Vec2 vec_point_line(Vec2 P, Vec2 A, Vec2 B){//点PからABへの最短距離
	Vec2 ABnor = B-A; ABnor.normalize();
	double Ax = ABnor * (P-A);
	Vec2 x = A + (ABnor*Ax);

	return x;
}

Vec3 vec_point_line_3d(Vec3 P, Vec3 A, Vec3 B){//点PからABへの最短距離
	Vec3 ABnor = B-A; ABnor.normalize();
	double Ax = ABnor * (P-A);
	Vec3 x = A + (ABnor*Ax);

	return x;
}
void COpenGL::Quickhull(const std::vector<Vec2> points, Model *m){
	//最初

	std::vector<Vec2> point_;
	m->all_p.clear();
	m->cross_p.clear();
	m->convex_line.clear();

	m->convex_cent.set(0,0);
	for(int i=0; i<(int)points.size(); i++){
		point_.push_back(points[i]);
		m->all_p.push_back(points[i]);
		m->convex_cent += points[i];
	}

	m->convex_cent /= (double)points.size();

	std::vector<line*> lines;
	std::sort(point_.begin(),point_.end(),Vec2::compareVec2PredicateX);//X軸ソート

	//最初の三角形をつくる
	double dis = 0;
	int far_num = 0;
	
	for(int i=1; i<(int)point_.size()-1; i++){
		double l = distance_point_line(point_[i], point_[0], point_[point_.size()-1]);
		
		if(l > dis){
			dis = l;
			far_num = i;
		}
	}
	//////cout << "far_num; " << far_num << "\n";
	Vec2D *v1 = new Vec2D(point_[0]);
	Vec2D *v2 = new Vec2D(point_[(int)point_.size()-1]);
	Vec2D *v3 = new Vec2D(point_[far_num]);

	line *l1 = new line(0, v1, v2);
	line *l2 = new line(0, v2, v3);
	line *l3 = new line(0, v3, v1);
	
	lines.push_back(l1);//最初の三角形ができました
	lines.push_back(l2);
	lines.push_back(l3);
	
	Vec2 convex_cent = (point_[0] + point_[(int)point_.size()-1] + point_[far_num]) / 3.0;

	l1->setNormal(convex_cent);//線の法線と交点をセット
	l2->setNormal(convex_cent);
	l3->setNormal(convex_cent);

	point_.erase(point_.begin()+((int)point_.size()-1));
	point_.erase(point_.begin()+far_num);
	point_.erase(point_.begin());

	//points_は点が凸包内部にある場合と凸包を構成する場合に削除。0になったら終わり
	while(1){
		
		std::vector<std::vector<int>> point_to_line;
		point_to_line.resize((int)lines.size());

		for(int i=0; i<(int)point_.size(); i++){//三角形内部にある点を削除
			int count = 0;
			for(int j=0; j<(int)lines.size(); j++){
				Vec2 X = vec_point_line(point_[i], lines[j]->start->p, lines[j]->end->p);
				//m->cross_p.push_back(X);
				if((X-convex_cent)*(X-point_[i]) > 0 ){//内側にある
					count++;
				}
			}
			if(count == (int)lines.size()){
				//内側にあるpt
				//////cout << "erased: " << i << "\n";
				point_.erase(point_.begin() + i);
				i--;
				//m->cross_p.push_back(point_[i]);
			}
		}


		for(int i=0; i<(int)point_.size(); i++){//点を線へと割り当てる
			bool flg = false;
			for(int j=0; j<(int)lines.size(); j++){
				//このr1,r2が２つとも正ならOK
				Vec2 X = vec_point_line(point_[i], lines[j]->start->p, lines[j]->end->p);
				
				Vec2 vec1 = (lines[j]->end->p - lines[j]->start->p);  vec1.normalize();
				Vec2 vec2 = (lines[j]->start->p - lines[j]->end->p);  vec2.normalize();

				Vec2 vec3 = X - point_[i];
				
				
				if(vec2*(point_[i]-lines[j]->end->p) >0  && vec1*(point_[i]-lines[j]->start->p) > 0){//
					if((X - point_[i])*(X - convex_cent) < 0){
						point_to_line[j].push_back(i);
						flg = true;
						break;
					}
				}
			}
			if(!flg){
				point_.erase(point_.begin() + i);
				i--;
			}
		}

		if(point_.size() == 0){
			break;
		}

		double max_dis=0; int max_num[2];
		int counts = 0;
		for(int i=0; i<(int)point_to_line.size(); i++){
			for(int j=0; j<(int)point_to_line[i].size(); j++){//一番遠い点を確認する
				double l = distance_point_line(point_[point_to_line[i][j]], lines[i]->start->p, lines[i]->end->p);
				if(l > max_dis){
					//////cout << "l: " << l << "\n";
					max_dis = l;
					max_num[0] = i;
					max_num[1] = point_to_line[i][j];
				}
				counts++;
			}
		}

		////cout << "xcounts: " << counts << " pount_: " << point_.size() << "\n";

		Vec2 max_far_p = point_[max_num[1]];//こいつを凸包に加える
		////cout << "max_num[1]: " << max_num[1] << "\n";
		//point_から削除
		point_.erase(point_.begin()+max_num[1]);

		//一番遠い点が決定した
		//除去する点と線を選ぶ
		std::vector<line*> delete_line;
		std::vector<int> delete_line_num;

		int delete_start = 0;
		for(int i=0; i<(int)lines.size(); i++){
			Vec2 vec = max_far_p - lines[i]->cross; vec.normalize();
			if(vec*lines[i]->normal > 0){//最遠点から見えるので削除する
				delete_line.push_back(lines[i]);
				delete_line_num.push_back(i);
			}
		}
		if(delete_line_num.size() == 0){
			//m->cross_p.push_back(max_far_p);

			break;
		}
		
		Vec2D *far_p_it = new Vec2D(max_far_p);
		//ラインを加える
		line *addLine1 = new line(0,delete_line[0]->start,far_p_it);
		line *addLine2 = new line(0,far_p_it,delete_line[(int)delete_line.size()-1]->end);
		addLine1->setNormal(convex_cent);
		addLine2->setNormal(convex_cent);

		//線を削除
		for(int i=(int)delete_line.size()-1; i>=0; i--){
			lines.erase(lines.begin() + (unsigned int)delete_line_num[i]);
		}

		lines.insert(lines.begin()+delete_line_num[0],addLine1);//線を追加
		lines.insert(lines.begin()+delete_line_num[0]+1,addLine2);
		
		//break;

		if(point_.size() == 0){
			break;
		}
		point_to_line.resize((int)lines.size());
	}
	
	
	////cout << "lines.size(): " << lines.size() << "\n";
	for(int i=0; i<(int)lines.size(); i++){
		m->convex_line.push_back(lines[i]);
		//////cout << "lines[i]: " << lines[i]->start->p.x << "," <<lines[i]->start->p.y << "\n";
	}

	for(int i=0; i<(int)point_.size(); i++){
		//m->cross_p.push_back(point_[i]);
	}

	//m->convex_cent = convex_cent;

}
bool judgeIntersected(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
	double ta = (c.x - d.x) * (a.y - c.y) + (c.y - d.y) * (c.x - a.x);
	double tb = (c.x - d.x) * (b.y - c.y) + (c.y - d.y) * (c.x - b.x);
	double tc = (a.x - b.x) * (c.y - a.y) + (a.y - b.y) * (a.x - c.x);
	double td = (a.x - b.x) * (d.y - a.y) + (a.y - b.y) * (a.x - d.x);
	if(tc * td < 0 && ta * tb < 0){
		return true;
	}else{
		return false;
	}
}
void COpenGL::Top_optimize(Model *m){//天頂面を近似する

	m->poly_p.clear();

	int div_num = 6;
	double scale = 1.3;//半径
	double rad = 2*PI / (double)div_num;
	bool Issmall = false;

	//中心座標
	Vec2 cent; cent.set(0.0,0.0);
	std::vector<Vec2> Top;
	std::vector<Vec2> Top_first;

	//六角形をつくる
	for(int i=0; i<div_num; i++){
		Top.push_back(Vec2(scale*cos(rad*-i)+m->convex_cent.x, scale*sin(rad*-i)+m->convex_cent.y));
		Top_first.push_back(Vec2(scale*cos(rad*-i), scale*sin(rad*-i)));
	}


	//交差している??
	Vec2 p = m->convex_line[0]->start->p;
	for(int i=0; i<div_num; i++){
		Vec2 p1, p2;
		if(i != div_num - 1){
			p1 = Top[i]; p2 = Top[i+1];
		}else{
			p1 = Top[i]; p2 = Top[0];
		}
		if(judgeIntersected(p1,p2,p,cent)){//交差していれば凸包より小さい
			Issmall = true;
			break;
		}
	}

	//スケールを変更する
	bool flg = false;
	double prescale = scale;
	double addValue = 0.1;
	int count;
	while(1){
		count = 0;
		//交点が見つかるまで繰り返す
		for(int i=0; i<div_num; i++){//六角形と交差しているかここをconve
			Vec2 p1 = Top[i];
			for(int j=0; j<(int)m->convex_line.size(); j++){
				if(judgeIntersected(p1,m->convex_cent,m->convex_line[j]->start->p,m->convex_line[j]->end->p)){
					count++;
				}
			}
		}

		if(count == 1){
			break;
		}else{
			

			if(count == 0){
				if(prescale == scale+addValue){
					addValue /= 10.0;
				}
				prescale = scale;
				scale += addValue;
			}else if(count >= 2){
				if(prescale == scale-addValue){
					addValue /= 10.0;
				}
				prescale = scale;
				scale -= addValue;
			}
			for(int i=0; i<div_num; i++){//拡大
				Top[i] = Top_first[i] * scale + m->convex_cent;
			}
		}
	}

	for(int i=0; i<(int)Top.size(); i++){
		m->poly_p.push_back(Top[i]);
		m->fold->pointPosition.push_back(Top[i]);
	}

}
void COpenGL::CutModel(Model *m){//面を切り取る処理
	//平面の三点を設定する
	std::vector<Vec2> betweenpoints;
	for(int i=0; i<(int)m->poly_p.size(); i++){
		if(i != (int)m->poly_p.size()-1){
			betweenpoints.push_back((m->poly_p[i]+m->poly_p[i+1])/2);
		}else{
			betweenpoints.push_back((m->poly_p[i]+m->poly_p[0])/2);
		}
	}

	m->fold->betweenPosition = betweenpoints;

	//中心と一番したの座標を設定する
	Vec3 TopP, bottomP;

	TopP.set(m->convex_cent.x , TDdata->plane_cent_Top.y,m->convex_cent.y);
	//TopP.y += 1;
	bottomP.set(m->convex_cent.x, TDdata->plane_cent.y , m->convex_cent.y);
	//bottomP.y = plane_cent.y-1;


	//平面をそれぞれ設定してトリムする
	//std::vector<std::vector<Vec2>> cut_point;
	m->cut_point.resize((int)betweenpoints.size());
	for(int i=0; i<(int)betweenpoints.size(); i++){
		//面の設定
		//法線ベクトルを設定
		Vec3 bet; bet.set(betweenpoints[i].x , TDdata->plane_cent.y, betweenpoints[i].y);
		
		Vec3 Normal; Normal = (TopP - bottomP)%(bet - bottomP);
		Normal.normalize();
		Vec3 Cent; Cent = (TopP + bottomP + bet)/3.0;
		Vec3 nor; nor = bet - TDdata->plane_cent; nor.normalize();
		////cout << "nor; " << nor.x << "," << nor.y << "," << nor.z << "\n";
		CrossSection(Cent, Normal, TopP,TDdata->plane_cent, nor, m->cut_point[i], m);
	}

	AutoOptimization(m);
	Trim(m);
}

//点Pと線(AB)の距離
double Distance_DotAndLine(Vec3 P, Vec3 A, Vec3 B)
{
	Vec3 AB,AP;

	AB.x = B.x - A.x;
	AB.y = B.y - A.y;
	AB.z = B.z - A.z;
	AP.x = P.x - A.x;
	AP.y = P.y - A.y;
	AP.z = P.z - A.z;

	//AB、APを外積して求められたベクトルの長さが、平行四辺形Dの面積になる
	double D = (AP%AB).length();

	//AB間の距離
	double L = ( A-B ).length();	//ABの長さ

	double H = D / L;
	return H;

}

void COpenGL::AutoOptimization(Model *m){
	//新しく点をつくる
	int max_points = 6;//最大の点数

	m->cut_point_2D.resize(m->cut_point.size());
	Vec3 TopP,bottomP;

	TopP.set(m->convex_cent.x , TDdata->plane_cent_Top.y,m->convex_cent.y);
	bottomP.set(m->convex_cent.x , TDdata->plane_cent.y,m->convex_cent.y);
	int count = 0;

	//3Dの点を二次元に変換
	m->length.resize(m->cut_point.size());
	for(int i=0; i<(int)m->cut_point.size(); i++){
		Vec3 a,b;
		a.set(m->fold->betweenPosition[i].x , 10, m->fold->betweenPosition[i].y);//上
		b.set(m->fold->betweenPosition[i].x , -10, m->fold->betweenPosition[i].y);//下
		Vec2 first; first.set(0,0);
		m->cut_point_2D[i].push_back(first);
		outline *line = new outline();
		line->points.push_back(first);
		m->fold->outlinepoints.push_back(line);

		std::sort(m->cut_point[i].begin(),m->cut_point[i].end(),Vec3::compareVec3PredicateY);

		for(int j=(int)m->cut_point[i].size()-1,k=0; j>=0; j--,k++){//y軸でソートしておく必要がある
			
			//double d = Distance_DotAndLine(m->cut_point[i][j], a, b);
			Vec3 compare; compare.set(a.x,m->cut_point[i][j].y,a.z);
			
			double d = (compare - m->cut_point[i][j]).length();
			Vec2 v; v.set(d,-(m->cut_point[i][j].y-TopP.y));
			
			if(j != (int)m->cut_point[i].size()-1){
				m->cut_point_2D[i].push_back(v);
				line->points.push_back(v);
				m->length[i].push_back(d);
				////cout << "j: " << j << " "<< v.x << "," << v.y << "\n";
			}
			
		}

		//最後に底面を加える
		//底面の中心からbetweenpoints[i]方向に伸ばした直線に一番距離が近い点を選択する
		Vec3 P; P.set(m->fold->betweenPosition[i].x , bottomP.y, m->fold->betweenPosition[i].y);
		Vec3 betdir = P - bottomP;
		Vec3 crossP;
		double smallest = 1000000;
		for(int j=0; j<(int)m->plane_point_bottom.size(); j++){
			if((m->plane_point_bottom[j]-bottomP)*betdir > 0){
				if(distance_point_line_3d(m->plane_point_bottom[j], bottomP, P) < smallest){
					smallest = distance_point_line_3d(m->plane_point_bottom[j], bottomP, P);
					crossP = vec_point_line_3d(m->plane_point_bottom[j], bottomP, P);
				}
			}
		}

		//このcrossPが最後に追加する点
		Vec3 compare; compare.set(a.x,crossP.y,a.z);
		double d = (compare - crossP).length();
		Vec2 v; v.set(d,-(crossP.y-TopP.y));
		m->cut_point[i].push_back(crossP);
		m->cut_point_2D[i].push_back(v);
		line->points.push_back(v);
	}
	m->fold->vec_point.resize(m->fold->outlinepoints.size());
	for(int i=0; i<(int)m->fold->outlinepoints.size(); i++){//それぞれのラインを変換する
		for(int j=0; j<(int)m->fold->outlinepoints[i]->points.size(); j++){
			PointH convert;
			convert.x = m->fold->outlinepoints[i]->points[j].x;
			convert.y = m->fold->outlinepoints[i]->points[j].y;
			m->fold->vec_point[i].push_back(convert);
		}
	}

	simplifyPath reduce_line;

	for(int i=0; i<(int)m->fold->outlinepoints.size(); i++){//それぞれのラインを最適化??
		std::vector<Vec2> stock;
		for(int j=3; j<=max_points; j++){//点の数
			std::vector<PointH> reduced;
			
			double ep = 0.01;
			double reduce_num = 0.001;
			double before_num = 0.001;
			bool flg_ = false;
			//int j=3;
			
			int count = 0;
			int max = j;


			while(1){
				reduced = reduce_line.simplifyWithRDP(m->fold->vec_point[i], ep);
				////cout << "j: " << j << "point size: " << m->fold->vec_point[i].size() << "reduced.size(): " << reduced.size() << " reduce_num: " << reduce_num << " ep: " << ep << " befor_num: " << before_num << "\n";
				if((int)reduced.size() == j){
					break;
				}else if((int)reduced.size() < j){
					if(before_num == ep-reduce_num || ep-reduce_num <= 0){
						reduce_num *= 0.9;
					}
					before_num = ep;
					ep -= reduce_num;
				}else{
					if(before_num == ep+reduce_num || ep-reduce_num == 0){
						reduce_num *= 0.9;
					}
					before_num = ep;
					ep += reduce_num;
				}
				if(max > (int)reduced.size()){
					max = reduced.size();
				}
				if(count > 2000){
					flg_ = true;
					break;
				}
				count++;
			}

			if(flg_){
				////cout << "this point can't create\n";
				continue;
			}
			////cout << "reduced: " << reduced.size() << "\n";
			m->fold->outlinepoints[i]->points.clear();
			for(int k=0; k<(int)reduced.size(); k++){
				Vec2 addP; addP.x = reduced[k].x; addP.y = reduced[k].y;
				m->fold->outlinepoints[i]->points.push_back(addP);
				// << "addP: " << addP.x << "," << addP.y << "\n";
			}
	
			////cout << "m->fold->outlinepoints[i]->points.: " << m->fold->outlinepoints[i]->points.size() << "\n";
			bool flg =  optimization_oen_outline(m, i);
			if(flg){
				//まだ最適化できる
				////cout << "i: " << i << "j: " << j << "\n";  
				stock = m->fold->outlinepoints[i]->points;
			}else{
				//一個前の点に戻す
				m->fold->outlinepoints[i]->points = stock;
				break;
			}
		}
		//cout << "i: " << i << " 点\n";
		for(int j=0; j<(int)m->fold->outlinepoints[i]->points.size(); j++){
			Vec2 p = m->fold->outlinepoints[i]->points[j];
			//cout << p.x << "," << p.y << "\n";
		}
	}
	optimization(m);
	Trim(m);
}
//底面の市場近い線同士を見つけてその交点を取る

Halfedge* checkHalf(std::list<Halfedge*> halfs){

	Halfedge *PairNullEdge = nullptr;
	std::list<Halfedge*> half_list;
	std::list<Halfedge*>::iterator it_h;


	for(it_h=halfs.begin(); it_h!=halfs.end(); it_h++){
		Halfedge *h1 = (*it_h);
		Halfedge *h2 = (*it_h)->next;
		Halfedge *h3 = (*it_h)->prev;

		if(h1->pair == nullptr){
			PairNullEdge = h1;
			break;
		}
		if(h2->pair == nullptr){
			PairNullEdge = h2;
			break;
		}
		if(h3->pair == nullptr){
			PairNullEdge = h3;
			break;
		}
	}

	return PairNullEdge;
}

void COpenGL::SetPlane(Model *m) {

	std::vector<Vec3> tmp_v;
	std::list<Halfedge*>::iterator it_h;
	std::list<Halfedge*> null_half_list;
	std::list<Faces*>::iterator it_f;
	std::vector<Vec3> cent_vector;
	
	Halfedge *Nulledge = nullptr;
	for(it_f=m->faces.begin(); it_f!=m->faces.end(); it_f++){
		Halfedge *h1 = (*it_f)->halfedge;
		Halfedge *h2 = h1->next;
		Halfedge *h3 = h1->prev;

		if(h1->pair == nullptr){
			null_half_list.push_back(h1);
		}
		if(h2->pair == nullptr){
			null_half_list.push_back(h2);
		}
		if(h3->pair == nullptr){
			null_half_list.push_back(h3);
		}
	}

	Nulledge = (*null_half_list.begin());
	Halfedge *edge = Nulledge;
	Halfedge *half = edge;
	int count_while = 0;
	while(1) {
		Vec3 cent; cent.set(0,0,0);
		int point_count = 0;
		std::vector<Halfedge*> null_edge_tmp;
		do{
			int count = 0;
			int num = edge->next->vertex->v_half.size();
			for(int i=0; i<(int)edge->next->vertex->v_half.size(); i++,count++){
				it_h = find(null_half_list.begin(), null_half_list.end(), edge->next->vertex->v_half[i]);
				if(it_h != null_half_list.end()){
					it_h = find(null_half_list.begin(), null_half_list.end(), edge);
					tmp_v.push_back(edge->vertex->p);
					null_edge_tmp.push_back(edge);
					cent = cent + edge->vertex->p;
					point_count++;
					null_half_list.erase(it_h);
					edge = edge->next->vertex->v_half[i];
					break;
				}
			}
			if(count == (int)edge->next->vertex->v_half.size()){
				it_h = find(null_half_list.begin(), null_half_list.end(), edge);
				null_half_list.erase(it_h);
				tmp_v.push_back(edge->vertex->p);
				null_edge_tmp.push_back(edge);
				cent = cent + edge->vertex->p;
				point_count++;
				break;
			}
		}while(half != edge);
		m->null_edges.push_back(null_edge_tmp);
		cent_vector.push_back(cent/(double)point_count);
		if(null_half_list.size() == 0){
			break;
		}else{
			edge = (*null_half_list.begin());
			half = (*null_half_list.begin());
		}
	}

	Vec3 Top = TDdata->plane_cent_Top;
	Vec3 Bottom = TDdata->plane_cent;
	std::vector<std::vector<Vec3>> Cutted_plane_points;
	//交点を調べる
	for(int i=0; i<(int)m->null_edges.size(); i++){// 切断面ごと
		std::vector<Halfedge*> halfs = m->null_edges[i];
		std::vector<Vec3> points;
		Vec3 N = (Top-cent_vector[i])%(Bottom-cent_vector[i]);// 平面の法線ベクトル
		N.normalize();
		Vec3 P = (Top+Bottom+cent_vector[i])/3.0;//平面の中心
		std::vector<Halfedge*>::iterator it_h2;
		//cout << "i: " << halfs.size() << "\n";
		for(it_h2=halfs.begin(); it_h2!=halfs.end(); it_h2++){ 
			Vec3 A = (*it_h2)->vertex->p;
			Vec3 B = (*it_h2)->next->vertex->p;
			if(((P-A)*N >=0 && (P-B)*N <= 0) || ((P-A)*N <=0 && (P-B)*N >= 0)){//交差
				Vec3 crossP;
				crossP = A + (B-A)*(abs((P-A)*N )/(abs((P-A)*N) + abs((P-B)*N)));
				points.push_back(crossP);
			}
		}
		double max = 0;
		double min = 1000000;
		Vec3 max_vec;
		Vec3 min_vec;
		//cout << "points.size(): " << points.size() << "\n";
		for(int j=0; j<(int)points.size(); j++){
			if(max < Distance_DotAndLine(points[j], Top, Bottom)){
				max = Distance_DotAndLine(points[j], Top, Bottom);
				max_vec = points[j];
			}
			if(min > Distance_DotAndLine(points[j], Top, Bottom)){
				min = Distance_DotAndLine(points[j], Top, Bottom);
				min_vec = points[j];
			}
			/*m->test_plane.push_back(max_vec);
			m->test_plane.push_back(min_vec);*/
		}
		/*
		m->test_plane.push_back(max_vec);
		m->test_plane.push_back(min_vec);*/
	}
}

void COpenGL::CalculateVolume(Model *m) {
	//体積を計算する処理
}

void COpenGL::outputObj(){
		double setSize = 0.01;
		Model *m = TDdata->parts[0];
		m->vertices.clear();
		m->faces.clear();
		std::string file = "outputFoldingModel.obj";
		ofstream output(file);
		output << "Part 0\n";

		int part = 0;
		int current = 0;

		std::vector<std::vector<Vec3>> convert2D;
		std::vector<std::vector<Vec2i>> convert2Di;

		std::vector<Vec2> pointPosition = m->fold->pointPosition;
		std::vector<Vec2> betweenPosition = m->fold->betweenPosition;
		std::vector<outline*> outlinePoints = m->fold->outlinepoints;
		std::vector<TrimPoints> trimPoint = m->fold->trimPoint;
		Vec3 top_center; top_center.set(0, 0, 0);
		Vec3 bottom_center; bottom_center.set(0, 0, 0);

		double *c = new double[16];
		c[0] = 1.0; c[1] = 0.0; c[2] = 0.0;
		c[3] = 0.0; c[4] = 0.0; c[5] = 1.0;
		c[6] = 0.0; c[7] = 0.0; c[8] = 0.0;
		c[9] = 0.0; c[10] = 1.0; c[11] = 0.0;
		c[12] = 0.0;
		/*c[13]= 2.0;
		c[14]= -40.0;
		c[15]= 1.0;*/
		c[13] = 0.0; c[14] = 0.0; c[15] = 0.0;
		//天頂面
		output << "#Model\n";
		int count = 1;

		std::string overlap = " ";
		std::string r_overlap = " ";

		std::vector<std::vector<Vertexs*>> addV;
		std::vector<std::vector<int>> right_id;
		std::vector<std::vector<int>> left_id;
		std::vector<Vertexs*> tmp_vec;

		//for(int i=outlinePoints.size()-1;i>=0;i--){//側面
		for (int i = 0; i<(int)outlinePoints.size(); i++){//側面
			int ipp = (i + 1) % pointPosition.size();
			Vec2 sweepVec = pointPosition[ipp] - pointPosition[i];
			sweepVec.normalize();
			Vec2 trimVec = pointPosition[ipp] - pointPosition[i];
			trimVec.rotate(PI / 2.0);
			trimVec.normalize();
			bool idSwitch = true;
			std::vector<Vertexs*> oneP;
			std::vector<int> rId;
			std::vector<int> lId;
			for (int j = 0; j<(int)trimPoint[i].trims.size(); j++){
				int id = trimPoint[i].trims[j].id;
				double alpha = trimPoint[i].trims[j].alpha;
				double l = trimPoint[i].trims[j].l;
				if (id>0){
					////cout << id << " ";
					//	//cout << trimPoint[i].trims[j].outlinePid << " ";
					Vec2 outlinei = outlinePoints[i]->points[id];
					outlinei = outlinei - outlinePoints[i]->points[id - 1];
					Vec2 foo = trimVec;
					foo.setLength(alpha*outlinei.x + outlinePoints[i]->points[id - 1].x);
					double bar = alpha*outlinei.y + outlinePoints[i]->points[id - 1].y;
					Vec3 p0;
					p0.set(foo.x + betweenPosition[i].x, foo.y + betweenPosition[i].y, bar);

					Vec3 p1 = p0;

					p1.x += sweepVec.x*l;
					p1.y += sweepVec.y*l;

					double aa = p1.z;
					p1.z = p1.y;
					p1.y = -aa;
					//モデル座標変換行列
					Vec3 pp = p1;
					std::string now_overlap = to_string((long double)(pp.x)) + "," + to_string((long double)(pp.y)) + "," + to_string((long double)(pp.z));

					//	重複削除
					/*
					if(outlinePoints[i]->points[count] == outlinePoints[i]->points[id-1]){
					count++;
					lId.push_back(0);
					}else{
					continue;
					lId.push_back(id);
					}
					*/

					if (now_overlap == overlap) {
						//cout << "same: ";
						continue;
					}
					else {
						//cout << "dif: ";
						overlap = now_overlap;
					}

					pp.x = p1.x*c[0] + p1.y*c[4] + p1.z*c[8];// + c[12]/setSize;
					pp.y = p1.x*c[1] + p1.y*c[5] + p1.z*c[9];// + c[13]/setSize;
					pp.z = p1.x*c[2] + p1.y*c[6] + p1.z*c[10];// + c[14]/setSize;

					pp = pp*setSize;
					if (j == 0) {
						top_center.x += pp.x + c[12];
						top_center.y += pp.y + c[13];
						top_center.z += pp.z + c[14];
					}

					std::string text = "v " + to_string((long double)(pp.x + c[12])) + " " + to_string((long double)(pp.y + c[13])) + " " + to_string((long double)(pp.z + c[14])) + " #" + to_string((long double)count++) + "\n";
					Vertexs *V = new Vertexs(pp.x + c[12], pp.y + c[13], pp.z + c[14], m->vertices.size());
					lId.push_back(id);
					oneP.push_back(V);
					m->vertices.push_back(V);
					tmp_vec.push_back(V);
					output << text;
				}
			}
			//cout << "\n";
			r_overlap = " ";
			//cout << "rId: ";
			for (int j = 0; j<(int)trimPoint[i].trims.size(); j++){//右側のIdを確かめている
				int id = trimPoint[i].trims[j].id;
				double alpha = trimPoint[i].trims[j].alpha;
				double l = trimPoint[i].trims[j].l;
				if (id < 0) {
					//	//cout << -1*id << " ";
					id *= -1;
					Vec2 outlinei = outlinePoints[i]->points[id];
					outlinei = outlinei - outlinePoints[i]->points[id - 1];
					Vec2 foo = trimVec;
					foo.setLength(alpha*outlinei.x + outlinePoints[i]->points[id - 1].x);
					double bar = alpha*outlinei.y + outlinePoints[i]->points[id - 1].y;
					Vec3 p0;
					p0.set(foo.x + betweenPosition[i].x, foo.y + betweenPosition[i].y, bar);

					Vec3 p1 = p0;
					p1.x += -sweepVec.x*l;
					p1.y += -sweepVec.y*l;

					double aa = p1.z;
					p1.z = p1.y;
					p1.y = -aa;
					Vec3 pp = p1;
					std::string now_overlap = to_string((long double)(pp.x)) + "," + to_string((long double)(pp.y)) + "," + to_string((long double)(pp.z));

					if (now_overlap != r_overlap) {
						r_overlap = now_overlap;
					}
					else {
						continue;
					}

					pp.x = p1.x*c[0] + p1.y*c[4] + p1.z*c[8];// + c[12]/setSize;
					pp.y = p1.x*c[1] + p1.y*c[5] + p1.z*c[9];// + c[13]/setSize;
					pp.z = p1.x*c[2] + p1.y*c[6] + p1.z*c[10];// + c[14]/setSize;

					pp = pp * setSize;
					rId.push_back(id);
				}
			}
			addV.push_back(oneP);
			right_id.push_back(rId);
			left_id.push_back(lId);
		}

		output << "\n";

		for (int i = 0; i<(int)pointPosition.size() - 1; i++) {
			Vec3 bottomP = addV[i][addV[i].size() - 1]->p;
			bottom_center = bottom_center + bottomP;
		}

		top_center = top_center / (double)(pointPosition.size() - 1);
		std::string text_top = "v " + to_string((long double)(top_center.x)) + " " + to_string((long double)(top_center.y)) + " " + to_string((long double)(top_center.z)) + " #" + to_string((long double)count++) + "\n";
		bottom_center = bottom_center / (double)(pointPosition.size() - 1);
		std::string text_bottom = "v " + to_string((long double)(bottom_center.x)) + " " + to_string((long double)(bottom_center.y)) + " " + to_string((long double)(bottom_center.z)) + " #" + to_string((long double)count++) + "\n";
		std::vector<std::vector<int>> right_Ids;

		output << text_top;
		output << text_bottom;

		Vec3 Top_center; Top_center.set(0, 0, 0);
		for (int i = 0; i<(int)pointPosition.size() - 1; i++) {
			Vec3 pP; pP.set(pointPosition[i].x, 0, pointPosition[i].y);
			Top_center = Top_center + pP;
		}
		Top_center = Top_center / ((double)pointPosition.size() - 1.0);

		for (int i = 0; i<(int)pointPosition.size() - 1; i++) {
			std::vector<Vertexs*> left;
			std::vector<Vertexs*> right;
			std::vector<int> lId;
			std::vector<int> rId;
			std::map<Vertexs*, bool> triangledCheck;
			int Llast;
			int Rlast;
			left = addV[i];
			if (i == pointPosition.size() - 2) {
				lId = right_id[0];
				rId = left_id[0];
				right = addV[0];
			}
			else{
				lId = right_id[i + 1];
				rId = left_id[i + 1];
				right = addV[i + 1];
			}
			Llast = 1;
			Rlast = 0;
			//lとr逆や・・・
			std::vector<std::vector<int>> plane_r;
			std::vector<std::vector<int>> plane_l;
			int rightID_now = rId[0];
			int leftID_now = lId[0];
			std::vector<int> nowRid;
			std::vector<int> nowLid;
			for (int k = 0; k<(int)rId.size(); k++) {
				if (rightID_now != rId[k]) {
					plane_r.push_back(nowRid);
					nowRid.clear();
					nowRid.push_back(k - 1);
					nowRid.push_back(k);
				}
				else {
					nowRid.push_back(k);
				}
				if (k == rId.size() - 1 && rightID_now == rId[k]) {
					plane_r.push_back(nowRid);
				}
				else if (k == rId.size() - 1 && rightID_now != rId[k]) {
					plane_r.push_back(nowRid);
				}
				rightID_now = rId[k];
			}
			for (int k = 0; k<(int)lId.size(); k++) {
				if (leftID_now != lId[k]) {
					plane_l.push_back(nowLid);
					nowLid.clear();
					nowLid.push_back(k - 1);
					nowLid.push_back(k);
				}
				else {
					nowLid.push_back(k);
				}
				if (k == lId.size() - 1 && leftID_now == lId[k]) {
					plane_l.push_back(nowLid);
				}
				else if (k == lId.size() - 1 && leftID_now != lId[k]) {
					//	nowLid.push_back(k);
					plane_l.push_back(nowLid);
				}
				leftID_now = lId[k];
			}
			std::vector<Vec3> between;
			for (int k = 0; k<(int)plane_r.size(); k++) {

				//	平面にするための角度を計算する
				int rF = plane_r[k][0];
				int rL = plane_r[k][plane_r[k].size() - 1];
				int lF = plane_l[k][0];
				int lL = plane_l[k][plane_l[k].size() - 1];
				Vec3 left_first = left[lF]->p;
				Vec3 right_first = right[rF]->p;
				Vec3 left_last = left[lL]->p;
				Vec3 right_last = right[rL]->p;
				Vec3 F, L;
				if (k == 0) {
					between.push_back((left_first + right_first) / 2.0);
					F = ((left_first + right_first) / 2.0);
				}
				else{
					Vec3 nor;
					double ax;
					nor = left_first - right_first; nor.normalize();
					ax = nor*(between[(int)between.size() - 1] - right_first);
					between.push_back(right_first + (nor * ax));
					F = right_first + (nor * ax);
				}

				Vec3 nor;
				double ax;
				nor = left_last - right_last; nor.normalize();
				ax = nor*(between[(int)between.size() - 1] - right_last);
				L = right_last + (nor * ax);

				std::vector<Vec3> bet;
				bet.push_back(F);
				bet.push_back(L);

				Vec3 calcAngle1 = L - F;
				Vec3 calcAngle2 = F - Top_center;
				calcAngle2.y = 0;
				calcAngle1.normalize();
				calcAngle2.normalize();

				//	計算した角度
				double angle = acos(calcAngle1 * calcAngle2);
				double matrix[3][3];
				Vec3 axis = left_first - right_first; axis.normalize();
				setmatrix(angle, matrix, axis);


				std::vector<Vertexs*> plane;
				std::vector<Vec3> plane2D;
				std::vector<Vec2i> plane2Di;
				//cout << "angle: " << angle << "\n";
				for (int l = 0; l<(int)plane_r[k].size(); l++) {
					Vec3 rP = right[plane_r[k][l]]->p;
					rP = calcmatrix(rP, matrix);
					plane2D.push_back(rP);
					Vec2 p2; p2.x = rP.x; p2.y = rP.z;
					Vec2i p2i(p2, right[plane_r[k][l]]->num);
					plane2Di.push_back(p2i);
				}
				std::reverse(plane2Di.begin(), plane2Di.end());
				for (int l = 0; l<(int)plane_l[k].size(); l++) {
					Vec3 lP = left[plane_l[k][l]]->p;
					lP = calcmatrix(lP, matrix);
					plane2D.push_back(lP);
					Vec2 p2; p2.x = lP.x; p2.y = lP.z;
					Vec2i p2i(p2, left[plane_l[k][l]]->num);
					plane2Di.push_back(p2i);
				}

				convert2D.push_back(plane2D);
				convert2Di.push_back(plane2Di);

				for (int l = 0; l<(int)plane_r[k].size(); l++) {
					plane.push_back(right[plane_r[k][l]]);
				}
				for (int l = 0; l<(int)plane_l[k].size(); l++) {
					plane.push_back(left[plane_l[k][l]]);
				}
				//	TDdata->plane_vec.push_back(plane);
			}
		}

		for (int i = 0; i<convert2Di.size(); i++) {
			std::vector<Vec2i> points_;
			points_ = triangulate(convert2Di[i]);
			for (int k = 0; k<convert2Di[i].size(); k++) {
				//cout << convert2Di[i][k].index << " ";
			}
			for (int k = 0; k<points_.size(); k += 3) {
				Vertexs *v0 = tmp_vec[points_[k].index];
				Vertexs *v1 = tmp_vec[points_[k + 1].index];
				Vertexs *v2 = tmp_vec[points_[k + 2].index];
				m->addFace(v0, v1, v2, m->faces.size());
			}
		}

		Vertexs *top_v = new Vertexs(top_center.x, top_center.y, top_center.z, m->vertices.size());
		m->vertices.push_back(top_v);
		Vertexs *bottom_v = new Vertexs(bottom_center.x, bottom_center.y, bottom_center.z, m->vertices.size());
		m->vertices.push_back(bottom_v);


		//	天頂面
		for (int i = 0; i<(int)pointPosition.size() - 1; i++) {
			if (i == pointPosition.size() - 2) {
				m->addFace(top_v, addV[i][0], addV[0][0], m->faces.size());
			}
			else{
				m->addFace(top_v, addV[i][0], addV[i + 1][0], m->faces.size());
			}
		}

		//底面
		for (int i = 0; i<(int)pointPosition.size() - 1; i++) {
			int left_last = addV[i].size() - 1;
			if (i == pointPosition.size() - 2) {
				int right_last = addV[0].size() - 1;
				m->addFace(bottom_v, addV[0][right_last], addV[i][left_last], m->faces.size());
			}
			else{
				int right_last = addV[i + 1].size() - 1;
				m->addFace(bottom_v, addV[i + 1][right_last], addV[i][left_last], m->faces.size());
			}
		}

		std::list<Faces*>::iterator it_f;
		for (it_f = m->faces.begin(); it_f != m->faces.end(); it_f++) {
			Halfedge *h = (*it_f)->halfedge;
			Halfedge *h_next = (*it_f)->halfedge->next;
			Halfedge *h_prev = (*it_f)->halfedge->prev;
			std::string text_face = "f " + to_string((long double)h->vertex->num + 1) + " " + to_string((long double)h_next->vertex->num + 1) + " " + to_string((long double)h_prev->vertex->num + 1) + "\n";
			output << text_face;
		}
		output.close();
		//cout << "end\n";
}

void COpenGL::convertFoldingToMesh(Model *m){
	//cout << "convert\n";
	double setSize = 1.0;
	m->vertices.clear();
	m->faces.clear();
	m->halfs.clear();
	
	int part = 0;
	int current = 0;

	std::vector<std::vector<Vec3>> convert2D;
	std::vector<std::vector<Vec2i>> convert2Di;

	std::vector<Vec2> pointPosition = m->fold->pointPosition;
	std::vector<Vec2> betweenPosition = m->fold->betweenPosition;
	std::vector<outline*> outlinePoints = m->fold->outlinepoints;
	std::vector<TrimPoints> trimPoint = m->fold->trimPoint;
	Vec3 top_center; top_center.set(0, 0, 0);
	Vec3 bottom_center; bottom_center.set(0, 0, 0);

	Vec2 pFirst = outlinePoints[0]->points[0];
	Vec2 pLast = outlinePoints[0]->points[outlinePoints[0]->points.size()-1];
	double changeY = abs(pLast.y - pFirst.y)/2;
	//	//cout << "changeY: " << changeY << "\n";
	double *c = new double[16];
	c[0] = 1.0; c[1] = 0.0; c[2] = 0.0;
	c[3] = 0.0; c[4] = 0.0; c[5] = 1.0;
	c[6] = 0.0; c[7] = 0.0; c[8] = 0.0;
	c[9] = 0.0; c[10] = 1.0; c[11] = 0.0;
	c[12] = 0.0;
	c[13] = 0.0; c[14] = 0.0; c[15] = 0.0;
	//天頂面
	int count = 1;

	std::string overlap = " ";
	std::string r_overlap = " ";

	std::vector<std::vector<Vertexs*>> addV;
	std::vector<std::vector<int>> right_id;
	std::vector<std::vector<int>> left_id;
	std::vector<Vertexs*> tmp_vec;
	//for(int i=outlinePoints.size()-1;i>=0;i--){//側面
	for (int i = 0; i<(int)outlinePoints.size(); i++){//側面
		//cout << "outline: " << i << "\n";
		int ipp = (i + 1) % pointPosition.size();
		Vec2 sweepVec = pointPosition[ipp] - pointPosition[i];
		sweepVec.normalize();
		Vec2 trimVec = pointPosition[ipp] - pointPosition[i];
		trimVec.rotate(PI / 2.0);
		trimVec.normalize();
		bool idSwitch = true;
		std::vector<Vertexs*> oneP;
		std::vector<int> rId;
		std::vector<int> lId;
		int pCount = 0;
		for (int j = 0; j<(int)trimPoint[i].trims.size(); j++){
			int id = trimPoint[i].trims[j].id;
			double alpha = trimPoint[i].trims[j].alpha;
			double l = trimPoint[i].trims[j].l;
			if (id>0){
				Vec2 outlinei = outlinePoints[i]->points[id];
				outlinei = outlinei - outlinePoints[i]->points[id - 1];
				Vec2 foo = trimVec;
				foo.setLength(alpha*outlinei.x + outlinePoints[i]->points[id - 1].x);
				double bar = alpha*outlinei.y + outlinePoints[i]->points[id - 1].y;
				Vec3 p0;
				p0.set(foo.x + betweenPosition[i].x, foo.y + betweenPosition[i].y, bar);

				Vec3 p1 = p0;

				p1.x += sweepVec.x*l;
				p1.y += sweepVec.y*l;

				double aa = p1.z;
				p1.z = p1.y;
				p1.y = -aa;
				//モデル座標変換行列
				Vec3 pp = p1;
				std::string now_overlap = to_string((long double)(pp.x)) + "," + to_string((long double)(pp.y)) + "," + to_string((long double)(pp.z));

				if (now_overlap == overlap) {
					continue;
				} else {
					overlap = now_overlap;
				}

				pp.x = p1.x*c[0] + p1.y*c[4] + p1.z*c[8];// + c[12]/setSize;
				pp.y = p1.x*c[1] + p1.y*c[5] + p1.z*c[9];// + c[13]/setSize;
				pp.z = p1.x*c[2] + p1.y*c[6] + p1.z*c[10];// + c[14]/setSize;

				pp = pp*setSize;
				if (j == 0) {
					top_center.x += pp.x + c[12];
					top_center.y += pp.y + c[13] + m->fold->topPosY*setSize;
					top_center.z += pp.z + c[14];
				}
				Vertexs *V;

				if (j == 0) {
					V = new Vertexs(pp.x + c[12], pp.y + c[13] + m->fold->topPosY*setSize, pp.z + c[14], m->vertices.size());
					////cout << "outline: " << outlinePoints[i]->points[pCount].x << "," << outlinePoints[i]->points[pCount].y << "\n";
					//cout << V->p.x << "," << V->p.y << "," << V->p.z << "\n";
					pCount++;
				}
				else {
					V = new Vertexs(pp.x + c[12], pp.y + c[13] + m->fold->topPosY*setSize, pp.z + c[14], m->vertices.size());
					//cout << "outline: " << outlinePoints[i]->points[pCount].x << "," << outlinePoints[i]->points[pCount].y << "\n";
					//cout << V->p.x << "," << V->p.y << "," << V->p.z << "\n";
					pCount++;
				}
				lId.push_back(id);
				oneP.push_back(V);
				m->vertices.push_back(V);
				tmp_vec.push_back(V);
			}
		}
		r_overlap = " ";
		for (int j = 0; j<(int)trimPoint[i].trims.size(); j++){//右側のIdを確かめている
			int id = trimPoint[i].trims[j].id;
			double alpha = trimPoint[i].trims[j].alpha;
			double l = trimPoint[i].trims[j].l;
			if (id < 0) {
				//	//cout << -1*id << " ";
				id *= -1;
				Vec2 outlinei = outlinePoints[i]->points[id];
				outlinei = outlinei - outlinePoints[i]->points[id - 1];
				Vec2 foo = trimVec;
				foo.setLength(alpha*outlinei.x + outlinePoints[i]->points[id - 1].x);
				double bar = alpha*outlinei.y + outlinePoints[i]->points[id - 1].y;
				Vec3 p0;
				p0.set(foo.x + betweenPosition[i].x, foo.y + betweenPosition[i].y, bar);

				Vec3 p1 = p0;
				p1.x += -sweepVec.x*l;
				p1.y += -sweepVec.y*l;

				double aa = p1.z;
				p1.z = p1.y;
				p1.y = -aa;
				Vec3 pp = p1;
				std::string now_overlap = to_string((long double)(pp.x)) + "," + to_string((long double)(pp.y)) + "," + to_string((long double)(pp.z));

				if (now_overlap != r_overlap) {
					r_overlap = now_overlap;
				}
				else {
					continue;
				}

				pp.x = p1.x*c[0] + p1.y*c[4] + p1.z*c[8];// + c[12]/setSize;
				pp.y = p1.x*c[1] + p1.y*c[5] + p1.z*c[9];// + c[13]/setSize;
				pp.z = p1.x*c[2] + p1.y*c[6] + p1.z*c[10];// + c[14]/setSize;

				pp = pp * setSize;
				rId.push_back(id);
			}
		}
		addV.push_back(oneP);
		right_id.push_back(rId);
		left_id.push_back(lId);
	}


	for (int i = 0; i<(int)pointPosition.size() - 1; i++) {
		Vec3 bottomP = addV[i][addV[i].size() - 1]->p;
		bottom_center = bottom_center + bottomP;
	}

	top_center = top_center / (double)(pointPosition.size() - 1);
	std::string text_top = "v " + to_string((long double)(top_center.x)) + " " + to_string((long double)(top_center.y)) + " " + to_string((long double)(top_center.z)) + " #" + to_string((long double)count++) + "\n";
	bottom_center = bottom_center / (double)(pointPosition.size() - 1);
	std::string text_bottom = "v " + to_string((long double)(bottom_center.x)) + " " + to_string((long double)(bottom_center.y)) + " " + to_string((long double)(bottom_center.z)) + " #" + to_string((long double)count++) + "\n";
	std::vector<std::vector<int>> right_Ids;


	Vec3 Top_center; Top_center.set(0, 0, 0);
	for (int i = 0; i<(int)pointPosition.size() - 1; i++) {
		Vec3 pP; pP.set(pointPosition[i].x, 0, pointPosition[i].y);
		Top_center = Top_center + pP;
	}
	Top_center = Top_center / ((double)pointPosition.size() - 1.0);



	for (int i = 0; i<(int)pointPosition.size() - 1; i++) {

		std::vector<Vertexs*> left;
		std::vector<Vertexs*> right;
		std::vector<int> lId;
		std::vector<int> rId;
		std::map<Vertexs*, bool> triangledCheck;
		int Llast;
		int Rlast;
		left = addV[i];
		if (i == pointPosition.size() - 2) {
			lId = right_id[0];
			rId = left_id[0];
			right = addV[0];
		}
		else{
			lId = right_id[i + 1];
			rId = left_id[i + 1];
			right = addV[i + 1];
		}
		Llast = 1;
		Rlast = 0;
		//lとr逆や・・・
		std::vector<std::vector<int>> plane_r;
		std::vector<std::vector<int>> plane_l;
		int rightID_now = rId[0];
		int leftID_now = lId[0];
		std::vector<int> nowRid;
		std::vector<int> nowLid;
		for (int k = 0; k<(int)rId.size(); k++) {
			if (rightID_now != rId[k]) {
				plane_r.push_back(nowRid);
				nowRid.clear();
				nowRid.push_back(k - 1);
				nowRid.push_back(k);
			}
			else {
				nowRid.push_back(k);
			}
			if (k == rId.size() - 1 && rightID_now == rId[k]) {
				plane_r.push_back(nowRid);
			}
			else if (k == rId.size() - 1 && rightID_now != rId[k]) {
				plane_r.push_back(nowRid);
			}
			rightID_now = rId[k];
		}
		for (int k = 0; k<(int)lId.size(); k++) {
			if (leftID_now != lId[k]) {
				plane_l.push_back(nowLid);
				nowLid.clear();
				nowLid.push_back(k - 1);
				nowLid.push_back(k);
			}
			else {
				nowLid.push_back(k);
			}
			if (k == lId.size() - 1 && leftID_now == lId[k]) {
				plane_l.push_back(nowLid);
			}
			else if (k == lId.size() - 1 && leftID_now != lId[k]) {
				//	nowLid.push_back(k);
				plane_l.push_back(nowLid);
			}
			leftID_now = lId[k];
		}
		std::vector<Vec3> between;
		for (int k = 0; k<(int)plane_r.size(); k++) {

			//	平面にするための角度を計算する
			int rF = plane_r[k][0];
			int rL = plane_r[k][plane_r[k].size() - 1];
			int lF = plane_l[k][0];
			int lL = plane_l[k][plane_l[k].size() - 1];
			Vec3 left_first = left[lF]->p;
			Vec3 right_first = right[rF]->p;
			Vec3 left_last = left[lL]->p;
			Vec3 right_last = right[rL]->p;
			Vec3 F, L;
			if (k == 0) {
				between.push_back((left_first + right_first) / 2.0);
				F = ((left_first + right_first) / 2.0);
			}
			else{
				Vec3 nor;
				double ax;
				nor = left_first - right_first; nor.normalize();
				ax = nor*(between[(int)between.size() - 1] - right_first);
				between.push_back(right_first + (nor * ax));
				F = right_first + (nor * ax);
			}

			Vec3 nor;
			double ax;
			nor = left_last - right_last; nor.normalize();
			ax = nor*(between[(int)between.size() - 1] - right_last);
			L = right_last + (nor * ax);

			std::vector<Vec3> bet;
			bet.push_back(F);
			bet.push_back(L);

			Vec3 calcAngle1 = L - F;
			Vec3 calcAngle2 = F - Top_center;
			calcAngle2.y = 0;
			calcAngle1.normalize();
			calcAngle2.normalize();

			//	計算した角度
			double angle = acos(calcAngle1 * calcAngle2);
			double matrix[3][3];
			Vec3 axis = left_first - right_first; axis.normalize();
			setmatrix(angle, matrix, axis);


			std::vector<Vertexs*> plane;
			std::vector<Vec3> plane2D;
			std::vector<Vec2i> plane2Di;
			for (int l = 0; l<(int)plane_r[k].size(); l++) {
				Vec3 rP = right[plane_r[k][l]]->p;
				rP = calcmatrix(rP, matrix);
				plane2D.push_back(rP);
				Vec2 p2; p2.x = rP.x; p2.y = rP.z;
				Vec2i p2i(p2, right[plane_r[k][l]]->num);
				plane2Di.push_back(p2i);
			}
			std::reverse(plane2Di.begin(), plane2Di.end());
			for (int l = 0; l<(int)plane_l[k].size(); l++) {
				Vec3 lP = left[plane_l[k][l]]->p;
				lP = calcmatrix(lP, matrix);
				plane2D.push_back(lP);
				Vec2 p2; p2.x = lP.x; p2.y = lP.z;
				Vec2i p2i(p2, left[plane_l[k][l]]->num);
				plane2Di.push_back(p2i);
			}

			convert2D.push_back(plane2D);
			convert2Di.push_back(plane2Di);

			for (int l = 0; l<(int)plane_r[k].size(); l++) {
				plane.push_back(right[plane_r[k][l]]);
			}
			for (int l = 0; l<(int)plane_l[k].size(); l++) {
				plane.push_back(left[plane_l[k][l]]);
			}
		}
	}



	for (int i = 0; i<convert2Di.size(); i++) {
		std::vector<Vec2i> points_;
		points_ = triangulate(convert2Di[i]);
		for (int k = 0; k<points_.size(); k += 3) {
			Vertexs *v0 = tmp_vec[points_[k].index];
			Vertexs *v1 = tmp_vec[points_[k + 1].index];
			Vertexs *v2 = tmp_vec[points_[k + 2].index];
			m->addFace(v0, v1, v2, m->faces.size());
		}
	}

	Vertexs *top_v = new Vertexs(top_center.x, top_center.y, top_center.z, m->vertices.size());
	m->vertices.push_back(top_v);
	Vertexs *bottom_v = new Vertexs(bottom_center.x, bottom_center.y, bottom_center.z, m->vertices.size());
	m->vertices.push_back(bottom_v);



	//	天頂面
	for (int i = 0; i<(int)pointPosition.size() - 1; i++) {
		if (i == pointPosition.size() - 2) {
			m->addFace(top_v, addV[i][0], addV[0][0], m->faces.size());
		}
		else{
			m->addFace(top_v, addV[i][0], addV[i + 1][0], m->faces.size());
		}
	}

	//底面
	for (int i = 0; i<(int)pointPosition.size() - 1; i++) {
		int left_last = addV[i].size() - 1;
		if (i == pointPosition.size() - 2) {
			int right_last = addV[0].size() - 1;
			m->addFace(bottom_v, addV[0][right_last], addV[i][left_last], m->faces.size());
		}
		else{
			int right_last = addV[i + 1].size() - 1;
			m->addFace(bottom_v, addV[i + 1][right_last], addV[i][left_last], m->faces.size());
		}
	}

	std::list<Faces*>::iterator it_f;
	for (it_f = m->faces.begin(); it_f != m->faces.end(); it_f++) {
		Halfedge *h = (*it_f)->halfedge;
		Halfedge *h_next = (*it_f)->halfedge->next;
		Halfedge *h_prev = (*it_f)->halfedge->prev;
	}


}

void COpenGL::changeVertexPos(Model *m){
	
		double setSize = 0.01;

		std::vector<Vec2> pointPosition = m->fold->pointPosition;
		std::vector<Vec2> betweenPosition = m->fold->betweenPosition;
		std::vector<outline*> outlinePoints = m->fold->outlinepoints;
		std::vector<TrimPoints> trimPoint = m->fold->trimPoint;
		Vec3 top_center; top_center.set(0, 0, 0);
		Vec3 bottom_center; bottom_center.set(0, 0, 0);

		Vec2 pFirst = outlinePoints[0]->points[0];
		Vec2 pLast = outlinePoints[0]->points[outlinePoints[0]->points.size() - 1];
		double changeY = abs(pLast.y - pFirst.y) / 2;
		//	//cout << "changeY: " << changeY << "\n";
		double *c = new double[16];
		c[0] = 1.0; c[1] = 0.0; c[2] = 0.0;
		c[3] = 0.0; c[4] = 0.0; c[5] = 1.0;
		c[6] = 0.0; c[7] = 0.0; c[8] = 0.0;
		c[9] = 0.0; c[10] = 1.0; c[11] = 0.0;
		c[12] = 0.0;
		c[13] = m->fold->topPosY; c[14] = 0.0; c[15] = 0.0;
		//天頂面
		int count = 0;
		std::string overlap = " ";
		std::vector<std::vector<Vec3>> addV;
		std::list<Vertexs*>::iterator it_v;
		it_v = m->vertices.begin();
		for (int i = 0; i<(int)outlinePoints.size(); i++){//側面
			int ipp = (i + 1) % pointPosition.size();
			Vec2 sweepVec = pointPosition[ipp] - pointPosition[i];
			sweepVec.normalize();
			Vec2 trimVec = pointPosition[ipp] - pointPosition[i];
			trimVec.rotate(PI / 2.0);
			trimVec.normalize();
			bool idSwitch = true;
			std::vector<Vec3> oneP;
			
			for (int j = 0; j<(int)trimPoint[i].trims.size(); j++){
				int id = trimPoint[i].trims[j].id;
				double alpha = trimPoint[i].trims[j].alpha;
				double l = trimPoint[i].trims[j].l;
				if (id>0){
					Vec2 outlinei = outlinePoints[i]->points[id];
					outlinei = outlinei - outlinePoints[i]->points[id - 1];
					Vec2 foo = trimVec;
					foo.setLength(alpha*outlinei.x + outlinePoints[i]->points[id - 1].x);
					double bar = alpha*outlinei.y + outlinePoints[i]->points[id - 1].y;
					Vec3 p0;
					p0.set(foo.x + betweenPosition[i].x, foo.y + betweenPosition[i].y, bar);

					Vec3 p1 = p0;

					p1.x += sweepVec.x*l;
					p1.y += sweepVec.y*l;

					double aa = p1.z;
					p1.z = p1.y;
					p1.y = -aa;
					//モデル座標変換行列
					Vec3 pp = p1;
					std::string now_overlap = to_string((long double)(pp.x)) + "," + to_string((long double)(pp.y)) + "," + to_string((long double)(pp.z));

					if (now_overlap == overlap) {
						continue;
					}
					else {
						overlap = now_overlap;
					}

					pp.x = p1.x*c[0] + p1.y*c[4] + p1.z*c[8];// + c[12]/setSize;
					pp.y = p1.x*c[1] + p1.y*c[5] + p1.z*c[9];// + c[13]/setSize;
					pp.z = p1.x*c[2] + p1.y*c[6] + p1.z*c[10];// + c[14]/setSize;

					pp = pp*setSize;
					if (j == 0) {
						top_center.x += pp.x + c[12];
						top_center.y += pp.y + c[13];
						top_center.z += pp.z + c[14];
					}
					Vec3 V;
					V.set(pp.x + c[12], pp.y + c[13], pp.z + c[14]);
					oneP.push_back(V);
					(*it_v)->p = V;
					it_v++;
				}
			}
			addV.push_back(oneP);
		}


		for (int i = 0; i<(int)pointPosition.size() - 1; i++) {
			Vec3 bottomP = addV[i][addV[i].size() - 1];
			bottom_center = bottom_center + bottomP;
		}

		top_center = top_center / (double)(pointPosition.size() - 1);
		bottom_center = bottom_center / (double)(pointPosition.size() - 1);

		(*it_v)->p = top_center; it_v++;
		(*it_v)->p = bottom_center;
}
