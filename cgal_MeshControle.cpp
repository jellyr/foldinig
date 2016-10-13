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


//	addFaceの書き換え
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

// gmpをdouble方に変換する
double gmp_to_double(CGAL::Gmpq val) {
	return (val.numerator().to_double() / val.denominator().to_double());
}

//	面の集合情報からCGAL用のメッシュ情報を生成
void inputMesh(Model *m) {
	/*
	std::list<Faces*>::iterator it_f;
	std::list<Vertexs*>::iterator it_v;
	std::vector<vertex_descriptor> vContainer;

	//	cgalのmeshオブジェクトに頂点を挿入
	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		vertex_descriptor u = m->add_vertex(G_Kernel::Point_3(0, 1, 0));
		vertex_descriptor v = cgalM->add_vertex(G_Kernel::Point_3(0, 0, 0));
		vertex_descriptor w = cgalM->add_vertex(G_Kernel::Point_3(1, 0, 0));
		vContainer.push_back(u);
		vContainer.push_back(v);
		vContainer.push_back(w);
	}

	//	面オブジェクトを挿入
	for (it_f = m->faces.begin(); it_f != m->faces.end(); it_f++) {
		int p0 = (*it_f)->halfedge->vertex->num;
		int p1 = (*it_f)->halfedge->next->vertex->num;
		int p2 = (*it_f)->halfedge->prev->vertex->num;
		cgalM->add_face(vContainer[p0], vContainer[p1], vContainer[p2]);
	}
	*/
}


//	面の集合情報からCGAL用の多角形情報を生成
Polyhedron_C *inputPoly_C(Model *m){

	std::list<Faces*>::iterator it_f;
	std::list<Vertexs*>::iterator it_v;
	std::vector<Faces*> vecF;
	std::vector<Vertexs*> vecV;
	

	//	cgalのmeshオブジェクトに頂点を挿入
	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		vecV.push_back((*it_v));
	}

	//	面オブジェクトを挿入
	for (it_f = m->faces.begin(); it_f != m->faces.end(); it_f++) {
		vecF.push_back((*it_f));
	}
	
	Polyhedron_C *P = new Polyhedron_C();
	polyhedron_builder<HalfedgeDS_C> builder(vecV, vecF);
	P->delegate(builder);

	return P;
}

//	Simple_carticianのpolyhedronからGmqのpolyhedronへ変換する
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

//	面の集合情報からCGAL用の多角形情報(Gmp)を生成
Polyhedron_G *inputPoly_Gnew(Model *m){

	std::list<Faces*>::iterator it_f;
	std::list<Vertexs*>::iterator it_v;
	std::vector<Faces*> vecF;
	std::vector<Vertexs*> vecV;

	//	cgalのmeshオブジェクトに頂点を挿入
	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		vecV.push_back((*it_v));
	}

	//	面オブジェクトを挿入
	for (it_f = m->faces.begin(); it_f != m->faces.end(); it_f++) {
		vecF.push_back((*it_f));
	}

	Polyhedron_G *P = new Polyhedron_G();
	polyhedron_builder<HalfedgeDS_G> builder(vecV, vecF);
	P->delegate(builder);

	return P;
}

//	面の集合情報からCGAL用の多角形情報(Gmp)を生成
Polyhedron_G inputPoly_G(Model *m){

	std::list<Faces*>::iterator it_f;
	std::list<Vertexs*>::iterator it_v;
	std::vector<Faces*> vecF;
	std::vector<Vertexs*> vecV;
	//cout << " vertex size: " << m->vertices.size() << "Face size" << m->faces.size() << "\n";
	//	cgalのmeshオブジェクトに頂点を挿入
	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		vecV.push_back((*it_v));
	}

	//	面オブジェクトを挿入
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


//	入力モデルを分割すると穴が空くので、体積計算のため穴をふさいでおく
void HoleFill (Polyhedron_C *poly){

	if (poly->empty()) {
		cout << "CGAL::Polyhedronが空です\n";
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
//	Modelクラスをレンダリング
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

//	Polyhedronクラスをレンダリング
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

//	modelクラスの体積を計算
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

//	Polyhedron_Gクラスの体積を計算
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
	D1 = boolDiff_P1_P2(P1, P2, true); // ウサギと立体の共通
	D2 = boolDiff_P1_P2(P1, P2, false); //	ウサギ-立体
	double refeModelVolume = calcVolume((*P2_));
	double V1 = calcVolume(D1);
	double V2 = calcVolume(D2);
	if (V2 > refeModelVolume){//	ウサギ内部にモデルがあると体積の計算がおかしくなる
		V2 = refeModelVolume - calcVolume(P1);
	}
	double V3 = calcVolume((*P2_));//	ウサギ全体の体積
	//cout << "V1:" << V1 << ", V2: " << V2 << ", V3: " << V3 << "\n";
	//cout << "V3 / (V1 - V2): " << V3 / (V1 - V2) << "\n";
	//cout << "ウサギ: " << refeModelVolume << "立体: " << calcVolume(P1) << "\n";
	//cout << "V1(立体-ウサギ): " << V1 << ",V2(ウサギ-立体): " << V2 << "\n";
	double volumeDiff = V1 + V2;
	//cout << "volumeDiff:" << volumeDiff << "\n";
	cout << "return value: " << volumeDiff / (10*refeModelVolume) << "\n";
	return volumeDiff / (10*refeModelVolume);
}


Polyhedron_G *holeFillAndConvertPolyG(Model *m){//	穴をふさいでPolu_Gへ変換する
	Polyhedron_C *holeModel = inputPoly_C(m);
	HoleFill(holeModel);
	return PolyC_to_Poly_G(holeModel);
}

Nef_polyhedron_3 convert_Poly_NefPoly(Polyhedron_G poly) {
	Nef_polyhedron_3 N(poly);
	return N;
}

Polyhedron_G TestMesh(Polyhedron_G *poly1, Nef_polyhedron_3 poly2, bool flg) {
	return boolDiff_P1_P2((*poly1), poly2, false);
}

void outputAsObj(Polyhedron_G *poly) {
	
	std::ofstream ofs("MeshFile.obj");
	CGAL::print_polyhedron_wavefront(ofs, (*poly));
}