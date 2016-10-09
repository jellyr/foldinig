#include "defineData.h"
#include "Optimization/simplify/Simplify.h"
#define M_PI_FUN 3.14159265359

void Model::Scaling(double value, double scale){
	value =   1.0 + value*0.1;
	//cout << "value: " << value << "\n";
	//cout << "scale: " << scale << "\n";
	if(value < 0){
		value = -1/value;
	}
	
	std::list<Vertexs*>::iterator it_v;
	for(it_v=vertices.begin(); it_v!=vertices.end(); it_v++){
		(*it_v)->p = ((*it_v)->p)*value;
	}
}

void Model::outputV() {
	std::list<Vertexs*>::iterator it_v;

	for (it_v = vertices.begin(); it_v != vertices.end(); it_v++) {
		cout << (*it_v)->num << ":" << (*it_v)->p.x << "," << (*it_v)->p.y << "," << (*it_v)->p.z << "\n";

	}
}

void Model::reducepolygon(int num){
	
	Simplify::Vertex v;
	std::list<Vertexs*>::iterator ver;
	std::list<Faces*>::iterator fa;
	Simplify::vertices.clear();
	Simplify::triangles.clear();
	for(ver=vertices.begin();ver!=vertices.end();ver++){
		Simplify::Vertex v;
		v.p.x = (*ver)->p.x;
		v.p.y = (*ver)->p.y;
		v.p.z = (*ver)->p.z;
		Simplify::vertices.push_back(v);
	}
	for(fa=faces.begin();fa!=faces.end();fa++){
		Simplify::Triangle t;
		t.v[0]= (*fa)->halfedge->vertex->num;
		t.v[1]= (*fa)->halfedge->next->vertex->num;
		t.v[2]= (*fa)->halfedge->prev->vertex->num;
		Simplify::triangles.push_back(t);
	}

	Simplify::simplify_mesh((int)num);
	std::vector<Vertexs*> vers;
	std::vector<Simplify::Triangle>::iterator tri;
	std::vector<Simplify::Vertex>::iterator vv;
	std::list<Faces*>::iterator fas;

	vertices.clear();
	faces.clear();
	halfs.clear();

	int i=0;
	for(vv=Simplify::vertices.begin();vv!=Simplify::vertices.end();vv++,i++){
		Vertexs *v = new Vertexs((*vv).p.x, (*vv).p.y, (*vv).p.z,i);
		vertices.push_back(v);
		vers.push_back(v);
	}
	i=0;
	for(tri=Simplify::triangles.begin();tri!=Simplify::triangles.end();tri++,i++){
		addFace( vers[(*tri).v[0]], vers[(*tri).v[1]], vers[(*tri).v[2]] ,i);
	}
	
	addsetH();
}
void Model::setHalfedgePair(Halfedge *he0, Halfedge *he1) {
		he0->pair = he1;
		he1->pair = he0;
}
void Model::deleteVertex( Vertexs *vertex ) {
		vertices.remove(vertex);
		delete vertex;
}
void Model::deleteFace( Faces* face ) {

		faces.remove(face);
		halfs.remove(face->halfedge);
		halfs.remove(face->halfedge->next);
		halfs.remove(face->halfedge->prev);
		delete face->halfedge->next;
		delete face->halfedge->prev;
		delete face->halfedge;
		delete face;
}
void Model::sizeNormalize(double m_size,Vec3 center) {
	std::list<Vertexs*>::iterator it_v;

		for( it_v = vertices.begin(); it_v != vertices.end(); it_v++ ) {
			(*it_v)->p = ((*it_v)->p - center) / (m_size) / 2;
		}

		cout << "m_size: " << m_size << "\n";
	
}
void Model::setAABB(){//ƒ|ƒŠƒSƒ“‚²‚Æ‚ÌAABB‚ðì‚é
	std::list<Faces*>::iterator face;
	double minX,minY,minZ,maxX,maxY,maxZ;
	for(face=faces.begin();face!=faces.end();face++){
		Halfedge *he = (*face)->halfedge;
		minX = he->vertex->p.x;maxX = he->vertex->p.x;
		minY = he->vertex->p.y;maxY = he->vertex->p.y;
		minZ = he->vertex->p.z;maxZ = he->vertex->p.z;

		he = he->next;
		if(he->vertex->p.x <= minX){minX=he->vertex->p.x;}else{maxX = he->vertex->p.x;}
		if(he->vertex->p.y <= minY){minY=he->vertex->p.y;}else{maxY = he->vertex->p.y;}
		if(he->vertex->p.z <= minZ){minZ=he->vertex->p.z;}else{maxZ = he->vertex->p.z;}
		
		he = he->next;
		if(he->vertex->p.x < minX){minX=he->vertex->p.x;}
		if(he->vertex->p.x > maxX){maxX=he->vertex->p.x;}
		if(he->vertex->p.y < minY){minY=he->vertex->p.y;}
		if(he->vertex->p.y > maxY){maxY=he->vertex->p.y;}
		if(he->vertex->p.z < minZ){minZ=he->vertex->p.z;}
		if(he->vertex->p.z > maxZ){maxZ=he->vertex->p.z;}

		(*face)->bbox[0][0] = (float)minX;
		(*face)->bbox[0][1] = (float)minY;
		(*face)->bbox[0][2] = (float)minZ;

		(*face)->bbox[1][0] = (float)maxX;
		(*face)->bbox[1][1] = (float)maxY;
		(*face)->bbox[1][2] = (float)maxZ;
	}
	//cout << "end polygon AABB\n";
}
void Model::addsetH(){
	int num = vertices.size() * vertices.size();
	int vernum = vertices.size();
	
	std::map<string,Halfedge*> Hmap;
	std::list<Faces*>::iterator face;
	std::list<Halfedge*>::iterator half;
//	
	//cout << "halfs.size: " << halfs.size() << "\n";
	for(half=halfs.begin();half!=halfs.end();half++){

		std::string hash = std::to_string((long double)(*half)->vertex->num) + "," + std::to_string((long double)(*half)->next->vertex->num);
		Hmap.insert(map<string,Halfedge*>::value_type(hash,(*half)));
	}

	for(face=faces.begin();face!=faces.end();face++){
		Halfedge *he = (*face)->halfedge;
		 do{
			
			 std::string hash = std::to_string((long double)he->next->vertex->num) + "," + std::to_string((long double)he->vertex->num);
			 he->pair = Hmap[hash];
			he = he->next;
		} while(he != (*face)->halfedge);
	}

	//cout << "end halfedge\n";
	//free(p);
}
void Model::addFace( Vertexs *v0, Vertexs *v1, Vertexs *v2, int f) {
		
		Halfedge* he0 = new Halfedge( v0 );
		Halfedge* he1 = new Halfedge( v1 );
		Halfedge* he2 = new Halfedge( v2 );
		
		he0->next = he1;	he0->prev = he2;
		he1->next = he2;	he1->prev = he0;
		he2->next = he0;	he2->prev = he1;

		v0->v_half.push_back(he0);
		v1->v_half.push_back(he1);
		v2->v_half.push_back(he2);

		Faces *face = new Faces(he0, f);
		faces.push_back( face );
		he0->face = face;
		he1->face = face;
		he2->face = face;

		halfs.push_back(he0);
		halfs.push_back(he1);
		halfs.push_back(he2);

		face->setNormal();
}
void Model::move(Vec3 move){
	std::list<Vertexs*>::iterator it_v;
	
	for(it_v=vertices.begin();it_v!=vertices.end();it_v++){
		
		(*it_v)->p = (*it_v)->p + move;
	}
}
void Model::rotates(double radian,Vec3 cross,Vec3 cen,int flg,Vec3 *d){
	std::list<Vertexs*>::iterator it_v;
	std::vector<Vertexs*> ver;
	Vec3 zz;
	if(flg == 0){
		zz.set(1,0,0);//
	}else if(flg == 1){
		zz.set(0,1,0);//
	}else{
		zz.set(0,0,1);//
	}
	
	double matrix[3][3];
	setmatrix((radian/(double)180)*M_PI_FUN,matrix,zz);//setmatrix(radian,matrix,cen);
	for(it_v=vertices.begin();it_v!=vertices.end();it_v++){
		Vec3 g =  calcmatrix((*it_v)->p-cen,matrix);
		(*it_v)->p = g + cen;
		ver.push_back((*it_v));
	}

	d[0] = (ver[2]->p - ver[1]->p)%(ver[0]->p - ver[1]->p);
	d[1] = -(ver[2]->p - ver[1]->p)%(ver[5]->p - ver[1]->p);//x
	d[2] = (ver[2]->p - ver[3]->p)%(ver[7]->p - ver[3]->p);


	d[0].normalize();
	d[1].normalize();
	d[2].normalize();
}
void Model::rotate(double radian,Vec3 cross,Vec3 cen,Vec3 *d){
	std::list<Vertexs*>::iterator it_v;
	std::vector<Vertexs*> ver;
	Vec3 p,q,zz;
	zz = cen;//
	zz.x = cen.x + 1;//
	zz = zz - cen;//
	zz.normalize();//
	//zz.normalize();
	//p.set(0,0,0);
	double matrix[3][3];
	setmatrix((radian/(double)180)*M_PI_FUN,matrix,zz);//setmatrix(radian,matrix,cen);
	//cout << "matrix" << matrix[0][0] << " " <<matrix[0][1] << " " << matrix[0][2] << " " << matrix[1][0] << "\n";
	for(it_v=vertices.begin();it_v!=vertices.end();it_v++){
	//	cout << "former " << (*it_v)->p.x <<" ";
		Vec3 g =  calcmatrix((*it_v)->p,matrix);
		(*it_v)->p = calcmatrix((*it_v)->p,matrix);
	//	cout << "after " << g.x <<" ";
		p = p + (*it_v)->p;
	}
	p = p/8;
	q = p;
	p = cen - p;
	for(it_v=vertices.begin();it_v!=vertices.end();it_v++){
		(*it_v)->p = (*it_v)->p + p;
		ver.push_back((*it_v));
	}

		d[0] = (ver[2]->p - ver[1]->p)%(ver[0]->p - ver[1]->p);
		d[1] = -(ver[2]->p - ver[1]->p)%(ver[5]->p - ver[1]->p);//x
		d[2] = (ver[2]->p - ver[3]->p)%(ver[7]->p - ver[3]->p);
		//sol->dir2 = (ver[2]->p - ver[3]->p)%(ver[7]->p - ver[3]->p);//z
		//sol->dir3 = -(ver[2]->p - ver[1]->p)%(ver[5]->p - ver[1]->p);//x
		//sol->dir3 = -(ver[2]->p - ver[1]->p)%(ver[0]->p - ver[1]->p);//y


	d[0].normalize();
	d[1].normalize();
	d[2].normalize();
	
}

void Model::pointNor(){
	std::list<Vertexs*>::iterator v;
	for(v=vertices.begin();v!=vertices.end();v++){
		Vec3 normalsum; normalsum.set(0,0,0);
		Halfedge *he = (*v)->halfedge;
		 do{
			normalsum = normalsum + he->next->vertex->p;
			he = he->prev->pair;
		} while(he != (*v)->halfedge);
		normalsum.normalize();
		(*v)->normal = normalsum;
	}
}


