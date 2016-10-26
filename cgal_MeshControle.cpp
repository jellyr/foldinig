#ifndef CGAL_DEFINEDATA_H
#include "cgal_defineData.h"
#endif
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <boost/foreach.hpp>
#include <CGAL/IO/print_wavefront.h>
#include <QtOpenGL>
#include "boolean.h"
typedef Mesh::Vertex_index vertex_descriptor;
typedef Mesh::Face_index face_descriptor;
typedef Polyhedron_C::Vertex_iterator	Vertex_iterator_C;
typedef Polyhedron_C::Facet_iterator	Facet_iterator_C;
typedef Polyhedron_C::Halfedge_around_facet_circulator Halfedge_facet_circulator_C;
typedef Polyhedron_G::Facet_iterator	Facet_iterator;
typedef Polyhedron_G::Vertex_iterator	Vertex_iterator;
typedef Polyhedron_G::Halfedge_around_facet_circulator Halfedge_facet_circulator;


//	addFace�̏�������
Faces *generateFace(Vertexs *v0, Vertexs *v1, Vertexs *v2, int f) {

	Halfedge* he0 = new Halfedge(v0);
	Halfedge* he1 = new Halfedge(v1);
	Halfedge* he2 = new Halfedge(v2);

	he0->next = he1;	he0->prev = he2;
	he1->next = he2;	he1->prev = he0;
	he2->next = he0;	he2->prev = he1;

	Faces *face = new Faces(he0, f);
	he0->face = face;
	he1->face = face;
	he2->face = face;

	face->setNormal();

	return face;
}

// gmp��double���ɕϊ�����
double gmp_to_double(CGAL::Gmpq val) {
	return (val.numerator().to_double() / val.denominator().to_double());
}

//	�ʂ̏W����񂩂�CGAL�p�̃��b�V�����𐶐�
void inputMesh(Model *m) {
	/*
	std::list<Faces*>::iterator it_f;
	std::list<Vertexs*>::iterator it_v;
	std::vector<vertex_descriptor> vContainer;

	//	cgal��mesh�I�u�W�F�N�g�ɒ��_��}��
	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		vertex_descriptor u = m->add_vertex(G_Kernel::Point_3(0, 1, 0));
		vertex_descriptor v = cgalM->add_vertex(G_Kernel::Point_3(0, 0, 0));
		vertex_descriptor w = cgalM->add_vertex(G_Kernel::Point_3(1, 0, 0));
		vContainer.push_back(u);
		vContainer.push_back(v);
		vContainer.push_back(w);
	}

	//	�ʃI�u�W�F�N�g��}��
	for (it_f = m->faces.begin(); it_f != m->faces.end(); it_f++) {
		int p0 = (*it_f)->halfedge->vertex->num;
		int p1 = (*it_f)->halfedge->next->vertex->num;
		int p2 = (*it_f)->halfedge->prev->vertex->num;
		cgalM->add_face(vContainer[p0], vContainer[p1], vContainer[p2]);
	}
	*/
}


//	�ʂ̏W����񂩂�CGAL�p�̑��p�`���𐶐�
Polyhedron_C *inputPoly_C(Model *m){

	std::list<Faces*>::iterator it_f;
	std::list<Vertexs*>::iterator it_v;
	std::vector<Faces*> vecF;
	std::vector<Vertexs*> vecV;
	

	//	cgal��mesh�I�u�W�F�N�g�ɒ��_��}��
	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		vecV.push_back((*it_v));
	}

	//	�ʃI�u�W�F�N�g��}��
	for (it_f = m->faces.begin(); it_f != m->faces.end(); it_f++) {
		vecF.push_back((*it_f));
	}
	
	Polyhedron_C *P = new Polyhedron_C();
	polyhedron_builder<HalfedgeDS_C> builder(vecV, vecF);
	P->delegate(builder);

	return P;
}

//	Simple_cartician��polyhedron����Gmq��polyhedron�֕ϊ�����
Polyhedron_G *PolyC_to_Poly_G(Polyhedron_C *poly_c){

	std::vector<Faces*> vecF;
	std::vector<Vertexs*> vecV;

	for (Vertex_iterator_C v = poly_c->vertices_begin(); v != poly_c->vertices_end(); ++v){
		v->id() = vecV.size();
		Vertexs *ver = new Vertexs(v->point().x(), v->point().y(), v->point().z(), vecV.size());
		vecV.push_back(ver);
	}

	for (Facet_iterator_C i = poly_c->facets_begin(); i != poly_c->facets_end(); ++i) {
		Halfedge_facet_circulator_C j = i->facet_begin();
		// Facets in polyhedral surfaces are at least triangles.
		CGAL_assertion(CGAL::circulator_size(j) >= 3);
		int vertexIndex[3];
		int count = 0;
		do {
			vertexIndex[count] = j->vertex()->id();
			count++;
		} while (++j != i->facet_begin());
		vecF.push_back(
			generateFace(vecV[vertexIndex[0]], vecV[vertexIndex[1]], vecV[vertexIndex[2]], vecF.size())
		);
	}

	Polyhedron_G *P = new Polyhedron_G();

	polyhedron_builder<HalfedgeDS_G> builder(vecV, vecF);
	P->delegate(builder);

	return P;
}

//	�ʂ̏W����񂩂�CGAL�p�̑��p�`���(Gmp)�𐶐�
Polyhedron_G *inputPoly_Gnew(Model *m){

	std::list<Faces*>::iterator it_f;
	std::list<Vertexs*>::iterator it_v;
	std::vector<Faces*> vecF;
	std::vector<Vertexs*> vecV;

	//	cgal��mesh�I�u�W�F�N�g�ɒ��_��}��
	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		vecV.push_back((*it_v));
	}

	//	�ʃI�u�W�F�N�g��}��
	for (it_f = m->faces.begin(); it_f != m->faces.end(); it_f++) {
		vecF.push_back((*it_f));
	}

	Polyhedron_G *P = new Polyhedron_G();
	polyhedron_builder<HalfedgeDS_G> builder(vecV, vecF);
	P->delegate(builder);

	return P;
}

//	�ʂ̏W����񂩂�CGAL�p�̑��p�`���(Gmp)�𐶐�
Polyhedron_G inputPoly_G(Model *m){

	std::list<Faces*>::iterator it_f;
	std::list<Vertexs*>::iterator it_v;
	std::vector<Faces*> vecF;
	std::vector<Vertexs*> vecV;
	//cout << " vertex size: " << m->vertices.size() << "Face size" << m->faces.size() << "\n";
	//	cgal��mesh�I�u�W�F�N�g�ɒ��_��}��
	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		vecV.push_back((*it_v));
	}

	//	�ʃI�u�W�F�N�g��}��
	for (it_f = m->faces.begin(); it_f != m->faces.end(); it_f++) {
		vecF.push_back((*it_f));
	}

	Polyhedron_G P;
	polyhedron_builder<HalfedgeDS_G> builder(vecV, vecF);
	P.delegate(builder);

	return P;
}

typedef Polyhedron_C::Halfedge_handle    Halfedge_handle;
typedef Polyhedron_C::Facet_handle       Facet_handle;
typedef Polyhedron_C::Vertex_handle      Vertex_handle;


//	���̓��f���𕪊�����ƌ����󂭂̂ŁA�̐όv�Z�̂��ߌ����ӂ����ł���
void HoleFill (Polyhedron_C *poly){

	if (poly->empty()) {
		cout << "CGAL::Polyhedron����ł�\n";
		return;
	}
	// Incrementally fill the holes
	
	unsigned int nb_holes = 0;
	
	BOOST_FOREACH(Halfedge_handle h, halfedges((*poly)))
	{
		if (h->is_border())
		{
			std::vector<Facet_handle>  patch_facets;
			std::vector<Vertex_handle> patch_vertices;
			bool success = CGAL::cpp11::get<0>(
				CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(
				(*poly),
				h,
				std::back_inserter(patch_facets),
				std::back_inserter(patch_vertices),
				CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, (*poly))).
				geom_traits(C_Kernel()))
				);
			++nb_holes;
		}
	}
	
}

void renderCgalMesh() {

}
//	Model�N���X�������_�����O
void renderFoldModel(Model *m) {
	std::list<Faces*>::iterator it_f;
	for (it_f = m->faces.begin(); it_f != m->faces.end(); it_f++) {
		Vec3 p0 = (*it_f)->halfedge->vertex->p;
		Vec3 p1 = (*it_f)->halfedge->next->vertex->p;
		Vec3 p2 = (*it_f)->halfedge->prev->vertex->p;
		Vec3 Normal = (p1 - p0) % (p2 - p0); Normal.normalize();
		Normal = Normal * 10;
		glBegin(GL_TRIANGLES);
		glNormal3d(Normal.x, Normal.y, Normal.z);
		glVertex3d(p0.x, p0.y, p0.z);
		glVertex3d(p1.x, p1.y, p1.z);
		glVertex3d(p2.x, p2.y, p2.z);
		glEnd();
	}
}


void makeRandumColorCluster(double *col){
	//RGB�̂���1��255,1��0,1��0�`255
	int randumV = rand() % 256;
	double getColor[6][3] = {
		{ 1.0, randumV / 255.0, 0 },
		{ 1.0, 0, randumV / 255.0 },
		{ 0, 1.0, randumV / 255.0 },
		{ 0, randumV / 255.0, 1.0 },
		{ randumV / 255.0, 1.0, 0 },
		{ randumV / 255.0, 0, 1.0 }
	};
	int getV = rand() % 6;
	col[0] = getColor[getV][0];
	col[1] = getColor[getV][1];
	col[2] = getColor[getV][2];
}

std::vector<double*> color_cluster;
std::vector<Vec3> dirCluster;
std::vector<Vec3> posCluster;
std::vector<double> curvCluster;
std::vector<std::vector<Vec3>> pointCluster;
std::vector<Vec2> pointPosition;
std::vector<Vec2> betweenPosition;
std::vector<std::vector<int>> clustValue;
std::vector<std::vector<Vertexs*>> clustVertex;
std::vector<Vec2> outlineBottomP;
std::vector<int> VertexCluster;

std::vector<int> cluster(std::vector<Vec2> points){
	int clustNum = 3;
	std::vector<int> clustIndex;
	std::vector<Vec2> cent;
	std::vector<Vec2> prev_cent;
	std::vector<int> clustAmount;
	cent.resize(clustNum);
	clustAmount.resize(clustNum);

	for (int i = 0; i < clustNum; i++) {
		clustAmount[i] = 0;
	}
	for (int i = 0; i < points.size(); i++){
		clustIndex.push_back(i % clustNum);
		cent[i % clustNum] += points[i];
		clustAmount[i % clustNum]++;
	}
	for (int i = 0; i < clustNum; i++) {
		cent[i] = cent[i] / (double)clustAmount[i];
	}

	prev_cent = cent;

	while (1) {
		for (int i = 0; i < points.size(); i++){
			double min = 1000000;
			int minNum = clustIndex[i];
			for (int j = 0; j < clustNum; j++) {
				double leng = (cent[j] - points[i]).length();
				if (leng < min) {
					min = leng;
					minNum = j;
				}
			}
			clustIndex[i] = minNum;
		}
		for (int i = 0; i < clustNum; i++) {
			clustAmount[i] = 0;
		}
		for (int i = 0; i < clustNum; i++) {
			cent[i].set(0, 0);
		}
		for (int j = 0; j < clustNum; j++) {
			for (int i = 0; i < points.size(); i++){
				if (j == clustIndex[i]){
					clustAmount[j]++;
					cent[j] += points[i];
				}
			}
		}
		for (int j = 0; j < clustNum; j++) {
			cent[j] = cent[j] / (double)clustAmount[j];
		}
		bool flg = false;
		for (int i = 0; i < clustNum; i++) {
			if ((prev_cent[i]-cent[i]).length() > 0.01) {
				flg = true;
				break;
			}
		}
		if (!flg) {
			break;
		}
		prev_cent = cent;
	}

	return clustIndex;
}


void renderModelCluster(Model *m) {
	std::list<Vertexs*>::iterator it_v;
	glPointSize(5);
	glDisable(GL_LIGHTING);
	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		Vec3 p = (*it_v)->p;
		Vec3 dirCurv = (*it_v)->normal*2*(*it_v)->curvture;
		/*if ((*it_v)->curvture < 0){
			glColor3d(0, 0, 1.0);
			dirCurv = dirCurv * -2;
		}
		else {
			glColor3d(1.0, 0, 0);
		}*/

		/*glBegin(GL_LINES);
		glVertex3d(p.x, p.y, p.z);
		glVertex3d(p.x + dirCurv.x , p.y, p.z + dirCurv.z);
		glEnd();*/
		Vec3 adjP = (*it_v)->adjCenter;
		
		Vec2 dir((*it_v)->normal);
		dir.normalize();
		dir *= abs((*it_v)->curvture);
		if ((*it_v)->clusterNum == -1) {
			glColor3d(0, 0, 0);
			//cout << "-1 ";
		}
		else {
			double *col = color_cluster[(*it_v)->clusterNum];
			if ((*it_v)->clusterNum % 4 == 0) {
				glColor3d(1.0, 0, 0);
			}
			else if ((*it_v)->clusterNum % 4 == 1){
				glColor3d(0.0, 1.0, 0);
			}
			else if ((*it_v)->clusterNum % 4 == 2){
				glColor3d(0.0, 0, 1.0);
			}
			else if ((*it_v)->clusterNum % 4 == 3){
				glColor3d(1.0, 0, 1.0);
			}
			
			/*if (VertexCluster[(*it_v)->num] == 0){
				glColor3d(1.0, 0, 0);
			}
			else if (VertexCluster[(*it_v)->num] == 1){
				glColor3d(0, 1.0, 0);
			}
			else {
				glColor3d(0, 0, 1.0);
			}*/
			
			glBegin(GL_POINTS);
			glVertex3d(p.x, p.y, p.z);
			glEnd();
			
			// ((*it_v)->num == 0 || (*it_v)->num == 1) {
				glBegin(GL_LINES);
				glVertex3d(p.x, p.y, p.z);
				glVertex3d(p.x + dir.x, p.y, p.z + dir.y);
				glEnd();
			//}
		}
	}
	glLineWidth(7);
	for (int i = 0; i < (int)dirCluster.size(); i++) {
		if (i % 4 == 0) {
			glColor3d(1.0, 0, 0);
		}
		else if (i % 4 == 1){
			glColor3d(0.0, 1.0, 0);
		}
		else if (i % 4 == 2){
			glColor3d(0.0, 0, 1.0);
		}
		else if (i % 4 == 3) {
			glColor3d(1.0, 0, 1.0);
		}
		Vec2 dir2D; dir2D.set(dirCluster[i].x, dirCluster[i].z);
		dir2D.normalize();
		/*glBegin(GL_LINES);
		glVertex3d(posCluster[i].x, posCluster[i].y, posCluster[i].z);
		glVertex3d(posCluster[i].x + dir2D.x, posCluster[i].y, posCluster[i].z + dir2D.y);
		glEnd();*/
	}

	glColor3d(1.0, 1.0, 0.0);
	glPointSize(5);
	for (int i = 0; i < pointPosition.size(); i++){
		Vec3 p; p.set(pointPosition[i].x, 0, pointPosition[i].y);
		/*glBegin(GL_POINTS);
		glVertex3d(p.x, p.y, p.z);
		glEnd();*/
	}

	glPointSize(8);
	for (int i = 0; i < clustVertex.size(); i++) {
		for (int j = 0; j < clustVertex[i].size(); j++) {
			if (clustValue[i][j] == 0) {
				glColor3d(0.0, 1.0, 0);
			}
			else if (clustValue[i][j] == 1) {
				glColor3d(1.0, 0.0, 0);
			}
			else{
				glColor3d(0.0, 0.0, 1.0);
			}
			/*glPointSize(2*abs(clustVertex[i][j]->curvture));
			glBegin(GL_POINTS);
			glVertex3d(clustVertex[i][j]->p.x, clustVertex[i][j]->p.y, clustVertex[i][j]->p.z);
			glEnd();*/
		}
	}

	glEnable(GL_LIGHTING);

	/*glBegin(GL_POLYGON);
	for (int i = pointPosition.size()-1; i >= 0; i--){
		Vec3 p; p.set(pointPosition[i].x, 0, pointPosition[i].y);
		glNormal3d(0, 1, 0);
		glVertex3d(p.x, m->fold->topPosY, p.z);
	}
	glEnd();*/

}

//	Polyhedron�N���X�������_�����O
void rendercgalPoly(Polyhedron_G *cgalPoly) {
	for (Facet_iterator i = cgalPoly->facets_begin(); i != cgalPoly->facets_end(); ++i) {
		Halfedge_facet_circulator j = i->facet_begin();
		// Facets in polyhedral surfaces are at least triangles.
		CGAL_assertion(CGAL::circulator_size(j) >= 3);
		Vec3 p[3]; int count = 0;
		do {
			//	cout << "count: " << count << "\n";
			p[count].x = gmp_to_double(j->vertex()->point().x());
			p[count].y = gmp_to_double(j->vertex()->point().y());
			p[count].z = gmp_to_double(j->vertex()->point().z());
			count++;
		} while (++j != i->facet_begin());
		Vec3 Normal = (p[1] - p[0]) % (p[2] - p[0]); Normal.normalize();
		Normal = Normal * 100;
		glBegin(GL_TRIANGLES);
		glNormal3d(Normal.x, Normal.y, Normal.z);
		glVertex3d(p[0].x, p[0].y, p[0].z);
		glVertex3d(p[1].x, p[1].y, p[1].z);
		glVertex3d(p[2].x, p[2].y, p[2].z);
		glEnd();
	}
}

double SignedVolumeOfTriangle(Vec3 p1, Vec3 p2, Vec3 p3) {
	double v321 = p3.x*p2.y*p1.z;
	double v231 = p2.x*p3.y*p1.z;
	double v312 = p3.x*p1.y*p2.z;
	double v132 = p1.x*p3.y*p2.z;
	double v213 = p2.x*p1.y*p3.z;
	double v123 = p1.x*p2.y*p3.z;

	return (1.0 / 6.0)*(-v321 + v231 + v312 - v132 - v213 + v123);
}

//	model�N���X�̑̐ς��v�Z
double calcVolume(Model *m) {
	std::list<Faces*>::iterator it_f;
	double volumeSum = 0;
	for (it_f = m->faces.begin(); it_f != m->faces.end(); it_f++) {
		Halfedge *h1 = (*it_f)->halfedge;
		Vec3 p1 = h1->vertex->p;
		Vec3 p2 = h1->next->vertex->p;
		Vec3 p3 = h1->prev->vertex->p;
		volumeSum += SignedVolumeOfTriangle(p1, p2, p3);
	}
	return volumeSum;
}

//	Polyhedron_G�N���X�̑̐ς��v�Z
double calcVolume(Polyhedron_G cgalPoly) {
	double volumeSum = 0;
	
	for (Facet_iterator i = cgalPoly.facets_begin(); i != cgalPoly.facets_end(); ++i) {
		Halfedge_facet_circulator j = i->facet_begin();
		// Facets in polyhedral surfaces are at least triangles.
		CGAL_assertion(CGAL::circulator_size(j) >= 3);
		Vec3 p[3];
		int vertexCount = 0;
		do {
				p[vertexCount].x = gmp_to_double(j->vertex()->point().x());
				p[vertexCount].y = gmp_to_double(j->vertex()->point().y());
				p[vertexCount].z = gmp_to_double(j->vertex()->point().z());
			vertexCount++;
		} while (++j != i->facet_begin());
		volumeSum += SignedVolumeOfTriangle(p[0], p[1], p[2]);
	}

	return volumeSum;
}


double calculateDiff(Polyhedron_G *P1, Polyhedron_G *P2){
	Polyhedron_G D1;
	Polyhedron_G D2;
	D1 = boolDiff_P1_P2((*P1), (*P2));
	D2 = boolDiff_P1_P2((*P2), (*P1));

	return (calcVolume(D1) + calcVolume(D2));
}

double calculateDiff(Polyhedron_G P1, Nef_polyhedron_3 P2, Polyhedron_G *P2_){
	/*Polyhedron_G D1;
	Polyhedron_G D2;
	D1 = boolDiff_P1_P2(P1, P2, true);
	D2 = boolDiff_P1_P2(P1, P2, false);
	double V1 = calcVolume(D1);
	double V2 = calcVolume(D2);
	double volumeDiff = calcVolume(D1) + calcVolume(D2);
	if (volumeDiff) {
		double P1Vol = calcVolume(P1);
		double P2Vol = calcVolume((*P2_));
		volumeDiff = abs(P1Vol - P2Vol);
	}*/
	Polyhedron_G D1,D2;
	D1 = boolDiff_P1_P2(P1, P2, true); // ����-�E�T�M
	D2 = boolDiff_P1_P2(P1, P2, false); //	�E�T�M-����
	double refeModelVolume = calcVolume((*P2_));
	double V1 = calcVolume(D1);
	double V2 = calcVolume(D2);
	if (V2 > refeModelVolume){//	�E�T�M�����Ƀ��f��������Ƒ̐ς̌v�Z�����������Ȃ�
		V2 = refeModelVolume - calcVolume(P1);
	}
	double V3 = calcVolume((*P2_));//	�E�T�M�S�̂̑̐�
	//cout << "V1:" << V1 << ", V2: " << V2 << ", V3: " << V3 << "\n";
	//cout << "V3 / (V1 - V2): " << V3 / (V1 - V2) << "\n";
	//cout << "�E�T�M: " << refeModelVolume << "����: " << calcVolume(P1) << "\n";
	//cout << "V1(����-�E�T�M): " << V1 << ",V2(�E�T�M-����): " << V2 << "\n";
	double volumeDiff = V1 + V2;
	//cout << "volumeDiff:" << volumeDiff << "\n";
	return volumeDiff / refeModelVolume;
}


Polyhedron_G *holeFillAndConvertPolyG(Model *m){//	�����ӂ�����Polu_G�֕ϊ�����
	Polyhedron_C *holeModel = inputPoly_C(m);
	HoleFill(holeModel);
	return PolyC_to_Poly_G(holeModel);
}

Nef_polyhedron_3 convert_Poly_NefPoly(Polyhedron_G poly) {
	Nef_polyhedron_3 N(poly);
	return N;
}

Polyhedron_G TestMesh(Polyhedron_G *poly1, Nef_polyhedron_3 poly2, bool flg) {
	return boolDiff_P1_P2((*poly1), poly2, flg);
}

void outputAsObj(Polyhedron_G *poly) {
	
	std::ofstream ofs("MeshFile.obj");
	CGAL::print_polyhedron_wavefront(ofs, (*poly));
}

void Cmodel::metroPrepar() {
	inputC = new CMesh();
	foldC = new CMesh();
	openMesh(inputM, inputC);//CMesh�֕ϊ�
	openMesh(foldM, foldC);//CMesh�֕ϊ�
	setMeshInfo(inputC);
	setMeshInfo(foldC);
	double metro = calcMetro((*inputC),(*foldC));
	cout << "first metro is " << metro << "\n";
}

void calculateCurvture(Model *m) {

	std::list<Vertexs*>::iterator it_v;
	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		Halfedge *h = (*it_v)->halfedge;
		Vec3 centroid; centroid.set(0, 0, 0);
		do{
			centroid += h->vertex->p;
			h = h->next->pair;
		} while (h != (*it_v)->halfedge);
	}
}

void calcCurvture(Model *m) {
	std::list<Vertexs*>::iterator it_v;
	std::list<Faces*>::iterator it_f;

	for (it_f = m->faces.begin(); it_f != m->faces.end(); it_f++) {
		Vec3 Normal = (*it_f)->normal;
		Halfedge *h = (*it_f)->halfedge;
		Vec2 n1; n1.set(Normal.x, Normal.z);
		do {
			Vec2 n2;
			n2.set(h->pair->face->normal.x, h->pair->face->normal.z);

			h = h->next->pair;
		} while (h != (*it_f)->halfedge);
	}
	
}

bool judgeIntersected(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy) {
	double ta = (cx - dx) * (ay - cy) + (cy - dy) * (cx - ax);
	double tb = (cx - dx) * (by - cy) + (cy - dy) * (cx - bx);
	double tc = (ax - bx) * (cy - ay) + (ay - by) * (ax - cx);
	double td = (ax - bx) * (dy - ay) + (ay - by) * (ax - dx);

	return (tc * td < 0 && ta * tb < 0);
};

bool judgeIntersectedVec(Vec2 Lv1, Vec2 Lv2, Vec2 Rv1, Vec2 Rv2) {
	
	double ax = Lv1.x;
	double ay = Lv1.y;
	double bx = Lv2.x;
	double by = Lv2.y;
	double cx = Rv1.x;
	double cy = Rv1.y;
	double dx = Rv2.x;
	double dy = Rv2.y;
	
	double ta = (cx - dx) * (ay - cy) + (cy - dy) * (cx - ax);
	double tb = (cx - dx) * (by - cy) + (cy - dy) * (cx - bx);
	double tc = (ax - bx) * (cy - ay) + (ay - by) * (ax - cx);
	double td = (ax - bx) * (dy - ay) + (ay - by) * (ax - dx);

	return (tc * td < 0 && ta * tb < 0);
};

//	�V���ʂ̌`������߂邽�߂ɒ��_�𕪗ނ���
void coloring (Model *m) {
	//	�G�b�W�̕��ς�����Ċp�x�����߂�
	std::list<Halfedge*>::iterator it_h;

	double meanLength = 0;
	double longest = 0;
	for (it_h = m->halfs.begin(); it_h != m->halfs.end(); it_h++) {
		meanLength += ((*it_h)->vertex->p - (*it_h)->next->vertex->p).length();
	}

	meanLength /= (double)m->halfs.size();

	std::list<Vertexs*>::iterator it_v;
	Vec3 centroid; centroid.set(0, 0, 0);
	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		centroid += (*it_v)->p;
	}

	centroid = centroid / (double)m->vertices.size();

	Vec2 cent2D; cent2D.set(centroid.x, centroid.z);
	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		Vec2 P2D; P2D.set((*it_v)->p.x, (*it_v)->p.z);
		double L = (centroid - (*it_v)->p).length();
		if (longest < L) {
			longest = L;
		}
	}
	//	meanLength *= 2;
	double alpha = (360 * meanLength) / (2 * M_PI * longest);
	
	int i = 0;

	for (double rad = alpha; rad <= 360 + alpha; rad += alpha) { 
		Vec3 cluster_dir; cluster_dir.set(0, 0, 0);
		Vec3 cluster_pos; cluster_pos.set(0, 0, 0);
		double color[3];
		makeRandumColorCluster(color);
		color_cluster.push_back(color);
		Vec2 dir1; dir1.set(cos((rad / 180.0) * M_PI), sin((rad / 180.0) * M_PI)); dir1.normalize();
		Vec2 dir2; dir2.set(cos(((rad - alpha) / 180.0) * M_PI), sin(((rad - alpha) / 180.0) * M_PI)); dir2.normalize();
		double clusterNum = 0;
		double curvture = 0;
		for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
			Vec2 ver2D; ver2D.set((*it_v)->p.x, (*it_v)->p.z);
			Vec2 dir2D = ver2D - cent2D; dir2D.normalize();
			if (judgeIntersected(0, 0, dir2D.x, dir2D.y, dir1.x, dir1.y, dir2.x, dir2.y)) {
				(*it_v)->clusterNum = i;
				cluster_dir += (*it_v)->normal;
				cluster_pos += (*it_v)->p;
				clusterNum++;
				curvture += (*it_v)->curvture;
			}
		}
		curvture /= clusterNum;
		cluster_dir = cluster_dir / clusterNum;
		cluster_dir.normalize();
		cluster_pos = cluster_pos / clusterNum;
		dirCluster.push_back(cluster_dir);
		posCluster.push_back(cluster_pos);
		curvCluster.push_back(curvture);
		i++;
	}

}

void removeTopinternalPoint() {//�V���ʂ�xy���ʓ��Ɋ܂܂�钸�_���̂���
	//std::vector<std::vector<int>> clustValue;
	//std::vector<std::vector<Vertexs*>> clustVertex;

	Vec2 topCentroid; topCentroid.set(0, 0);
	for (int i = 0; i < pointPosition.size() - 1; i++) {
		topCentroid += pointPosition[i];
	}

	topCentroid = topCentroid / (double)(pointPosition.size() - 1);

	for (int i = 0; i < clustVertex.size(); i++) {
		for (int j = 0; j < clustVertex[i].size(); j++) {
			bool flg = false;
			for (int k = 0; k < pointPosition.size() - 1; k++) {
				double ax, ay, bx, by;
				double cx, cy, dx, dy;
				ax = pointPosition[k].x;
				ay = pointPosition[k].y;
				bx = pointPosition[k + 1].x;
				by = pointPosition[k + 1].y;
				cx = topCentroid.x;
				cy = topCentroid.y;
				dx = clustVertex[k][j]->p.x;
				dy = clustVertex[k][j]->p.y;
				if (judgeIntersected(ax, ay, bx, by, cx, cy, dx, dy)) {
					flg = true;
					break;
				}
			}
			if (!flg) {
				clustVertex[i].erase(clustVertex[i].begin() + j);
			}
		}
	}
}

void bottomPlaneIntersection(Model *m, Vec3 centroid) {
	Vec3 topCent, bottomCent;
	topCent = centroid; topCent.y = 100;
	bottomCent = centroid; bottomCent.y = -100;
	for (int i = 0; i < clustVertex.size(); i++) {
		Vec3 between3D; between3D.set(betweenPosition[i].x, betweenPosition[i].y, 100);
		Vec3 planeNormal = (between3D - topCent) % (bottomCent - topCent);
		planeNormal.normalize();
		Vec3 planeCent = (between3D + topCent + bottomCent) / 3;
		double minY = 10000;
		Vec2 p;
		for (int j = 0; j < clustVertex[i].size(); j++) {
			Halfedge *h = clustVertex[i][j]->halfedge;
			do {
				Vec3 A = h->vertex->p;
				Vec3 B = h->next->vertex->p;
				if (((A - planeCent)*planeNormal <= 0 && (B - planeCent)*planeNormal >= 0) ||
					((A - planeCent)*planeNormal >= 0 && (B - planeCent)*planeNormal <= 0)) {
					Vec3 crossP = A + (B - A) * (((A - planeCent)*planeNormal) / ((A - planeCent)*planeNormal + (B - planeCent)*planeNormal));
					if (crossP.y < minY) {
						Vec3 betPosbottom;
						p.y = abs(m->fold->topPosY - crossP.y);
						betPosbottom.set(betweenPosition[i].x, betweenPosition[i].y, p.y);
						p.x = (betPosbottom - crossP).length();
					}
				}
				h = h->prev->pair;
			} while (h != clustVertex[i][j]->halfedge);
		}
		outlineBottomP.push_back(p);
	}

	for (int i = 1; i < outlineBottomP.size(); i++) {
		outlineBottomP[0].y += outlineBottomP[i].y;
	}
	outlineBottomP[0].y /= (double)outlineBottomP.size();

	for (int i = 1; i < outlineBottomP.size(); i++) {
		outlineBottomP[i].y = outlineBottomP[0].y;
	}

	return;

}

void setThreeCluster(Model *m) {
	
	cout << "three Cluster\n";

	std::list<Vertexs*>::iterator it_v;
	std::vector<std::vector<Vertexs*>> clustVertex(3);
	std::vector<Vec2> centroid(3);
	std::vector<Vec2> precentroid(3);

	centroid[0].set(0, 0);
	centroid[1].set(0, 0);
	centroid[2].set(0, 0);

	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		VertexCluster.push_back((*it_v)->num % 3);
		clustVertex[(*it_v)->num % 3].push_back((*it_v));
	}

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < clustVertex[i].size(); j++) {
			Vec2 vec; vec.set(clustVertex[i][j]->p.y, 2*clustVertex[i][j]->normal.y);
			centroid[i] += vec;
		}
		centroid[i] = centroid[i] / (double)clustVertex[i].size();
	}
	
	precentroid = centroid;
	while (1) {
		cout << "num\n";
		VertexCluster.clear();
		clustVertex[0].clear();
		clustVertex[1].clear();
		clustVertex[2].clear();
		//�d�S�Ƃ̋������X�V���Ċ��蓖�Ă��Ȃ���
		for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
			Vec2 vec; vec.set(10*(*it_v)->p.y, (*it_v)->normal.y);
			int minN = 0;
			double minL = 1000000;
			for (int i = 0; i < 3; i++) {
				if ((centroid[i] - vec).length() < minL) {
					minL = (centroid[i] - vec).length();
					minN = i;
				}
			}
			VertexCluster.push_back(minN);
			clustVertex[minN].push_back((*it_v));
		}

		centroid[0].set(0, 0);
		centroid[1].set(0, 0);
		centroid[2].set(0, 0);

		//�N���X�^�̏d�S���W���X�V
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < clustVertex[i].size(); j++) {
				Vec2 vec; vec.set(10*clustVertex[i][j]->p.y, clustVertex[i][j]->normal.y);
				centroid[i] += vec;
			}
			centroid[i] = centroid[i] / (double)clustVertex[i].size();
		}

		bool endFlg = false;
		for (int i = 0; i < 3; i++) {
			if ((precentroid[i] - centroid[i]).length() > 0.01) {
				endFlg = true;
			}
		}
		if (!endFlg) break;

		precentroid = centroid;
	}

	cout << "set three end\n";
}

void setCluster(Model *m) {
	std::list<Vertexs*>::iterator it_v;
	Vec3 centroid; centroid.set(0, 0, 0);
	int topEdgeNum = 6;
	std::vector<double> numCluster;
	
	numCluster.resize(dirCluster.size());
	for (int i = 0; i < (int)dirCluster.size(); i++) {
		numCluster[i] = 0;
		for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
			if ((*it_v)->clusterNum == i) {
				numCluster[i]++;
			}
		}
	}
	
	while (1) {
		double minrad = 0;
		int combine[2];
		for (int i = 0; i < dirCluster.size(); i++) {
			//cout << "curvCluster] " << curvCluster[i] << "\n";
			double cos;
			double clusterNums = (1.0 / numCluster[i]) * (1.0 / numCluster[i]);
			if (i == dirCluster.size() - 1) {
				cos = dirCluster[i] * dirCluster[0];
				clusterNums *= (1.0 / numCluster[0]) * (1.0 / numCluster[0]);
				if ((curvCluster[i] < 0 && curvCluster[0] > 0) || 
					(curvCluster[i] > 0 && curvCluster[0] < 0)) {
					clusterNums *= 0.8;
				}
			}
			else {
				cos = dirCluster[i] * dirCluster[i + 1];
				clusterNums *= (1.0 / numCluster[i + 1]) * (1.0 / numCluster[i + 1]);
				if ((curvCluster[i] < 0 && curvCluster[i+1] > 0) ||
					(curvCluster[i] > 0 && curvCluster[i+1] < 0)) {
					clusterNums *= 0.8;
				}
			}
			cos *= clusterNums;
			if (minrad < abs(cos)) {
				if (i == dirCluster.size() - 1) {
					combine[0] = 0;
					combine[1] = i;
				}
				else {
					combine[0] = i;
					combine[1] = i + 1;
				}
				minrad = abs(cos);
			}
		}

		//cout << combine[0] << "," << combine[1] << "\n";
		dirCluster.erase(dirCluster.begin() + combine[1]);
		posCluster.erase(posCluster.begin() + combine[1]);
		curvCluster.erase(curvCluster.begin() + combine[1]);
		for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {//�N���X�^�ԍ����X�V
			if ((*it_v)->clusterNum == combine[1]) {
				(*it_v)->clusterNum = combine[0];
			}
			else if ((*it_v)->clusterNum > combine[1]){
				(*it_v)->clusterNum--;
			}
		}
		
		Vec3 updateDir; updateDir.set(0, 0, 0);
		Vec3 updatePos; updatePos.set(0, 0, 0);
		double updateCurv = 0;
		int count = 0;
		for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {//�N���X�^�ԍ����X�V
			if ((*it_v)->clusterNum == combine[0]) {
				updateDir += (*it_v)->normal;
				updatePos += (*it_v)->p;
				updateCurv += (*it_v)->curvture;
				count++;
			}
		}
		updateDir = updateDir / (double)count;
		updateDir.normalize();
		dirCluster[combine[0]] = updateDir;
		posCluster[combine[0]] = updatePos / (double)count;
		curvCluster[combine[0]] = updateCurv / (double)count;
		numCluster.resize(dirCluster.size());
		for (int i = 0; i < (int)dirCluster.size(); i++) {
			numCluster[i] = 0;
			for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
				if ((*it_v)->clusterNum == i) {
					numCluster[i]++;
				}
			}
		}

		if (dirCluster.size() <= topEdgeNum) {
			break;
		}
	}

	for (int i = 0; i < dirCluster.size(); i++) {
		Vec3 pos;
		int nextNum = i + 1;
		double count = 0;
		if (i == dirCluster.size() - 1){
			nextNum = 0;
		}
		std::vector<Vec2> forClustValue;
		std::vector<Vertexs*> vers;
		for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
			if ((*it_v)->clusterNum != i) {
				continue;
			}
			Halfedge *h = (*it_v)->halfedge;
			do {
				if (h->next->vertex->clusterNum == nextNum) {
					pos += ((*it_v)->p + h->next->vertex->p) / 2;
					count++;
				}
				h = h->prev->pair;
			} while (h != (*it_v)->halfedge);
			Vec2 yCurv; yCurv.set((*it_v)->p.y, (*it_v)->normal.y*2);
			forClustValue.push_back(yCurv);
			vers.push_back((*it_v));
		}

		//�V���ʂ�x-z���ʂɂ���̂͌ォ�珜���Ă���
		clustVertex.push_back(vers);//�c�����̃N���X�^�����O
		clustValue.push_back(cluster(forClustValue));
		Vec2 pos2D; pos2D.set(pos.x, pos.z);
		pos2D = pos2D / count;
		pointPosition.push_back(pos2D);
	}

	cout << "three cluster\n";
	setThreeCluster(m);

	pointPosition.push_back(pointPosition[0]);
	
	for (int i = 0; i < pointPosition.size() - 1; i++) {
		betweenPosition.push_back((pointPosition[i] + pointPosition[i + 1]) / 2);
	}
	//��ԉ��̍��W���v�Z������
}

void convertPolyToModel (Model *m) {

	Polyhedron_C *cgalPoly = inputPoly_C(m);
	HoleFill(cgalPoly);
	cout << "start\n";
	m->vertices.clear();
	m->faces.clear();
	m->halfs.clear();
	std::vector<Vertexs*> stdV;
	std::list<Faces*>::iterator it_f;
	std::list<Vertexs*>::iterator it_v;
	double top = -10000;
	double bottom = 10000;
	for (Vertex_iterator_C i = cgalPoly->vertices_begin(); i != cgalPoly->vertices_end(); i++) {
		double x, y, z;
		x = gmp_to_double(i->point().x());
		y = gmp_to_double(i->point().y());
		z = gmp_to_double(i->point().z());
		i->id() = m->vertices.size();
		Vertexs *V = new Vertexs(x, y, z, m->vertices.size());
		m->vertices.push_back(V);
		stdV.push_back(V);
		if (top < y) {
			top = y;
		}
		if (bottom > y) {
			bottom = y;
		}
	}
	for (Facet_iterator_C i = cgalPoly->facets_begin(); i != cgalPoly->facets_end(); ++i) {
		Halfedge_facet_circulator_C j = i->facet_begin();
		// Facets in polyhedral surfaces are at least triangles.
		CGAL_assertion(CGAL::circulator_size(j) >= 3);
		int index[3];
		int count = 0;
		do {
			index[count] = j->vertex()->id();
			
			count++;
		} while (++j != i->facet_begin());
		Vec3 p[3];
		p[0] = stdV[index[0]]->p;
		p[1] = stdV[index[1]]->p;
		p[2] = stdV[index[2]]->p;

		Vec3 Normal = (p[1] - p[0]) % (p[2] - p[0]); Normal.normalize();
		m->addFace(stdV[index[0]], stdV[index[1]], stdV[index[2]], m->faces.size());
		it_f = m->faces.end(); it_f--;
		(*it_f)->normal = Normal;
		(*it_f)->bary = (p[0] + p[1] + p[2]) / 3;
	}
	
	m->addsetH();
	//���_�̖@��
	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++){
		Halfedge *h = (*it_v)->halfedge;
		int verCount = 0;
		Vec3 vertexNormal;
		vertexNormal.set(0, 0, 0);
		double curv = M_PI;
		std::vector<Vec2> st_v;
		Vec2 st; st.set(0, 0);
		double radD = 0;
		do {
			vertexNormal += h->face->normal;
			(*it_v)->adjCenter += h->next->vertex->p;
			Vec2 ver2D(h->next->vertex->p);
			st_v.push_back(ver2D);
			verCount++;
			Vec2 dir1(h->next->vertex->p - (*it_v)->p); dir1.normalize();
			Vec2 dir2(h->prev->vertex->p - (*it_v)->p); dir2.normalize();
			radD += acos(dir1*dir2);
			st += ver2D;
			//cout << h->next->vertex->p.y << "\n";
			h = h->prev->pair;
		} while (h != (*it_v)->halfedge);
		st = st / (double)verCount;

		(*it_v)->normal = vertexNormal / (double)verCount;
		Vec2 normal2D((*it_v)->normal); normal2D.normalize();
		Vec2 v2D((*it_v)->p);
		std::vector<Vec2> left;
		std::vector<Vec2> right;
		for (int i = 0; i < st_v.size(); i++) {
			if (normal2D % (st_v[i] - v2D) > 0) {//���E�ɕ���
				left.push_back(st_v[i]);
			}
			else{
				right.push_back(st_v[i]);
			}
		}
		if ((*it_v)->num == 0 || (*it_v)->num == 1) {
			cout << "leftSize: " << left.size() << ", rightSize: " << right.size() << "\n";
		}
		double maxRad = 0;
		double radSum;
		int leftMax = 0, rightMax = 0;
		for (int i = 0; i < left.size(); i++) {//left,right�Ő����p���ő�ɐ�����̂����߂�
			Vec2 dir = (left[i] - v2D); dir.normalize();
			double rad = acos(normal2D * dir);
			if (rad > M_PI / 2.0) {
				rad = M_PI - rad;
			}
			if ((*it_v)->num == 0 || (*it_v)->num == 1) {
				cout << "rad: " << (rad * 180) / M_PI << " ";
			}
			if (rad > maxRad) {
				maxRad = rad;
				leftMax = i;
			}
		}
		if ((*it_v)->num == 0 || (*it_v)->num == 1) {
			cout << "Max left rad: " << (maxRad * 180) / M_PI << "\n";
		}
		radSum = maxRad;
		maxRad = 0;
		for (int i = 0; i < right.size(); i++) {//left,right�Ő����p���ő�ɐ�����̂����߂�
			Vec2 dir = (right[i] - v2D); dir.normalize();
			double rad = acos(normal2D * dir);
			if (rad > M_PI/2.0) {
				rad = M_PI - rad;
			}
			if ((*it_v)->num == 0 || (*it_v)->num == 1) {
				cout << "rad: " << (rad * 180) / M_PI << " ";
			}
			if (rad > maxRad) {
				maxRad = rad;
				rightMax = i;
			}
		}
		if ((*it_v)->num == 0 || (*it_v)->num == 1) {
			cout << "Max right rad: " << (maxRad * 180) / M_PI << "\n";
			cout << leftMax << ", " << rightMax << "\n";

		}
		radSum += maxRad;
		Vec2 rightDir, leftDir;
		rightDir = right[rightMax] - (*it_v)->p; rightDir.normalize();
		leftDir =  left[leftMax] - (*it_v)->p;    leftDir.normalize();
		cout << "calc rad: " << (acos(rightDir*leftDir) * 180) / M_PI << "\n";
		Vec2 ver2((*it_v)->p);
		Vec2 normalver2((*it_v)->normal);  normalver2.normalize();
		Vec2 calcDir;
		if ((st - ver2)*normalver2 < 0) {
			calcDir = ver2 - st;
		}
		else{
			calcDir = st - ver2;
		}
		(*it_v)->curvture = sqrt(calcDir*normalver2);
		(*it_v)->curvtureY = 0;

	}
	m->fold = new foldmethod();
	m->fold->topPosY = top;
	m->fold->bottomPosY = bottom;
}