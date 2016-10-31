#ifndef CGAL_DEFINEDATA_H
#include "cgal_defineData.h"
#endif
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
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
typedef Polygon_2::Vertex_iterator	Vertex_iterator_2D;
namespace PS = CGAL::Polyline_simplification_2;
using namespace std;

typedef PS::Stop_below_count_threshold Stop;
typedef PS::Squared_distance_cost            Cost;

void convexhull(std::vector<Vec3> ver, Model *model){
	//セグメントを入れる
	std::vector<Vec3>::iterator v;
	std::vector<S_ver*>::iterator it_v;
	std::list<Vertexs*>::iterator it_v2;
	std::list<Faces*>::iterator f;
	std::list<Hullv*> hulv;


	Vec3 Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
	Vec3 initial[6];
	initial[0].set(0, 0, 0); initial[1].set(0, 0, 0); initial[2].set(0, 0, 0);
	initial[3].set(0, 0, 0); initial[4].set(0, 0, 0); initial[5].set(0, 0, 0);
	//ここから初期化を行う
	for (v = ver.begin(); v != ver.end(); v++){
		if (initial[0].x > (*v).x){ initial[0] = (*v); }
		if (initial[1].y > (*v).y){ initial[1] = (*v); }
		if (initial[2].z > (*v).z){ initial[2] = (*v); }
		if (initial[3].x < (*v).x){ initial[3] = (*v); }
		if (initial[4].y < (*v).y){ initial[4] = (*v); }
		if (initial[5].z < (*v).z){ initial[5] = (*v); }
	}//初期化終了

	double leng = 0;
	int num[3];
	num[0] = 0; num[1] = 0; num[2] = 0;
	//この6つの中から一番遠い点２つを選ぶ
	for (int i = 0; i<6; i++){
		for (int j = 0; j<6; j++){
			if (leng < (initial[i] - initial[j]).length()){
				leng = (initial[i] - initial[j]).length();
				num[0] = i;
				num[1] = j;
			}
		}
	}

	leng = 0;
	Vec3 baseline = initial[num[0]] - initial[num[1]];
	//4つの点の中から基準線に一番遠い点を選択する
	for (int i = 0; i<6; i++){
		if (i == num[0] || i == num[1]){ continue; }
		Vec3 comline;
		comline = initial[i] - initial[num[1]];
		double l = (baseline % comline).length() / baseline.length();
		if (l > leng){
			leng = l;
			num[2] = i;
		}
	}

	Vec3 P = (initial[num[0]] + initial[num[1]] + initial[num[2]]) / 3;
	Vec3 N = (initial[num[1]] - initial[num[0]]) % (initial[num[2]] - initial[num[0]]);
	N.normalize();
	leng = 0;
	Vec3 Frd;
	Frd.set(0, 0, 0);
	int i = 0;
	//num[0],num[1],num[2]で三角形ができる
	//この三角形から一番遠い点を全ての中から調べる
	for (v = ver.begin(); v != ver.end(); v++, i++){
		if (leng < abs(((*v) - P)*N)){
			leng = abs(((*v) - P)*N);
			Frd = (*v);
		}
	}

	Vertexs *v0 = new Vertexs(initial[num[0]].x, initial[num[0]].y, initial[num[0]].z, 0);
	v0->p.num = initial[num[0]].num;

	Vertexs *v1 = new Vertexs(initial[num[1]].x, initial[num[1]].y, initial[num[1]].z, 1);
	v0->p.num = initial[num[0]].num;

	Vertexs *v2 = new Vertexs(initial[num[2]].x, initial[num[2]].y, initial[num[2]].z, 2);
	v0->p.num = initial[num[0]].num;

	Vertexs *v3 = new Vertexs(Frd.x, Frd.y, Frd.z, 3);
	v0->p.num = initial[num[0]].num;

	model->vertices.push_back(v0);
	model->vertices.push_back(v1);
	model->vertices.push_back(v2);
	model->vertices.push_back(v3);

	//4つの点から四角錘を生成する
	if ((Frd - P)*N < 0){//同方向
		model->addFace(v0, v1, v2, 0);
		model->addFace(v0, v2, v3, 1);
		model->addFace(v2, v1, v3, 2);
		model->addFace(v1, v0, v3, 3);
	}
	else{//反対方向
		model->addFace(v0, v2, v1, 0);
		model->addFace(v0, v3, v2, 1);
		model->addFace(v2, v3, v1, 2);
		model->addFace(v1, v3, v0, 3);
	}


	for (v = ver.begin(); v != ver.end(); v++){
		if ((*v) == initial[num[0]] || (*v) == initial[num[1]] || (*v) == initial[num[2]] || (*v) == Frd){
			continue;
		}
		Hullv  *hullv = new Hullv();
		hullv->p = (*v);
		hulv.push_back(hullv);

	}
	std::list<Hullv*>::iterator hv_;


	std::list<Hullv*>::iterator hv;
	std::list<Hullf*> Hf;

	model->select = true;

	for (f = model->faces.begin(); f != model->faces.end(); f++){
		Hullf *hf = new Hullf();
		Hf.push_back(hf);
		hf->f = (*f);
	}

	std::list<Hullf*>::iterator hface;
	hv = hulv.begin();
	while (hv != hulv.end()){
		leng = 0; Hullf *HH = NULL;
		for (hface = Hf.begin(); hface != Hf.end(); hface++){
			Vec3 p = ((*hface)->f->halfedge->vertex->p + (*hface)->f->halfedge->prev->vertex->p + (*hface)->f->halfedge->next->vertex->p) / 3;
			if ((float)((p - (*hv)->p)*(*hface)->f->normal) < 0){//表向きなら
				//距離を調べる
				if (leng < abs(((*hv)->p - p)*(*hface)->f->normal)){
					leng = abs(((*hv)->p - p)*(*hface)->f->normal);
					HH = (*hface);
				}
			}
		}
		if (HH != NULL){
			(*hv)->f = HH->f;
			HH->v.push_back((*hv)->p);
		}
		if ((*hv)->f == NULL){
			Hullv *vv = (*hv);
			hv = hulv.erase(hv);
			delete vv;
		}
		else{
			hv++;
		}
	}

	model->addsetH();
	Convex(Hf, model, hulv);
}
void Convex(std::list<Hullf*> ff, Model* model, std::list<Hullv*> hulv){

	std::list<Hullf*>::iterator Hf;
	std::list<Hullf*>::iterator hface;
	std::list<Vec3>::iterator Hv;
	std::list<Faces*> fg;
	std::list<Faces*> deletev;
	std::list<Faces*>::iterator it_f;
	std::list<Faces*>::iterator it_f2;
	std::list<Faces*>::iterator f;
	std::list<Hullv*>::iterator hv_;
	std::list<Vertexs*>::iterator vertex;
	std::list<Vertexs*> Vlist;
	std::list<Vertexs*>::iterator vlist;

	while (hulv.size() != 0){
		Hf = ff.begin(); int ii = 0;
		while (Hf != ff.end()){
			if ((*Hf)->v.size() > 0){
				break;
			}
			else{
				Hf++;
			}
		}

		double leng = 0;
		Vec3 p = ((*Hf)->f->halfedge->vertex->p + (*Hf)->f->halfedge->next->vertex->p + (*Hf)->f->halfedge->prev->vertex->p) / 3;
		Vec3 hv;

		for (Hv = (*Hf)->v.begin(); Hv != (*Hf)->v.end(); Hv++){
			//一番距離が長い点を探索
			if (leng < abs(((*Hv) - p)*((*Hf)->f->normal))){
				leng = abs(((*Hv) - p)*((*Hf)->f->normal));
				hv = (*Hv);
			}
		}


		//頂点リストから削除する
		for (hv_ = hulv.begin(); hv_ != hulv.end(); hv_++){
			if ((*hv_)->p == hv){
				hv_ = hulv.erase(hv_);
				break;
			}
		}

		//頂点オブジェクトを作る
		Vertexs *HV = new Vertexs(hv.x, hv.y, hv.z, model->vertices.size());
		HV->p.num = hv.num;

		model->vertices.push_back(HV);

		//hvから見える面は削除
		Hf = ff.begin();

		bool flg = false;
		while (Hf != ff.end()){
			Vec3 bb;
			Vec3 pp = ((*Hf)->f->halfedge->vertex->p + (*Hf)->f->halfedge->next->vertex->p + (*Hf)->f->halfedge->prev->vertex->p) / 3;
			bb = (hv - pp);
			bb.normalize();
			if (bb*(*Hf)->f->normal > 0){//表向きなら
				fg.push_back((*Hf)->f);
				flg = true;
			}
			Hf++;
		}


		//境界線を探して面を生成す
		it_f = fg.begin();

		while (it_f != fg.end()){
			Halfedge *h = (*it_f)->halfedge;
			bool flg = true;
			do{
				Faces *fa = h->pair->face;
				it_f2 = find(fg.begin(), fg.end(), fa);
				if (it_f2 == fg.end()){//裏向き
					model->addFace(h->vertex, h->next->vertex, HV, 0);//面を追加
					flg = false;
				}
				h = h->next;
			} while (h != (*it_f)->halfedge);
			Faces *df = (*it_f);
			it_f++;
			if (flg){
				deletev.push_back(df);
			}
		}
		//先に頂点を削除

		for (it_f = deletev.begin(); it_f != deletev.end(); it_f++){
			Halfedge *h = (*it_f)->halfedge;
			do{
				bool flg = false;
				Halfedge *h2 = h;
				do{
					Vec3 center = (h2->face->halfedge->vertex->p + h2->face->halfedge->next->vertex->p + h2->face->halfedge->prev->vertex->p) / 3;
					if ((hv - center)*h2->face->normal > 0){
						flg = true;
						break;
					}
					h2 = h2->pair->next;
				} while (h2 != h);
				if (!flg){//点周りが全て削除される面ならば
					vlist = find(Vlist.begin(), Vlist.end(), h->vertex);
					if (vlist == Vlist.end()){
						Vlist.push_back(h->vertex);
					}
				}
				h = h->next;
			} while (h != (*it_f)->halfedge);
		}

		for (vertex = Vlist.begin(); vertex != Vlist.end(); vertex++){
			Vertexs *v = (*vertex);
			model->deleteVertex(v);
		}

		//面を削除
		for (it_f = fg.begin(); it_f != fg.end(); it_f++){
			Faces *f = (*it_f);
			it_f2 = find(model->faces.begin(), model->faces.end(), f);
			model->deleteFace(f);
		}

		int ij = 0;
		for (vertex = Vlist.begin(); vertex != Vlist.end(); vertex++, ij++){
			(*vertex)->num = ij;
		}
		model->addsetH();

		//凸面を更新
		Hf = ff.begin();//インスタンスを削除
		while (Hf != ff.end()){//Hullfを削除する
			Hullf *huf = (*Hf);
			Hf++;
			delete huf;
		}

		//頂点のより分け
		hv_ = hulv.begin();
		while (hv_ != hulv.end()){//初期化
			(*hv_)->f = NULL;
			hv_++;
		}

		ff.clear();

		for (f = model->faces.begin(); f != model->faces.end(); f++){
			Hullf *hf = new Hullf();
			ff.push_back(hf);
			hf->f = (*f);
		}//とりあえず新しくする

		hv_ = hulv.begin();
		while (hv_ != hulv.end()){
			leng = 0; Hullf *HH = NULL;
			for (hface = ff.begin(); hface != ff.end(); hface++){
				Vec3 p = ((*hface)->f->halfedge->vertex->p + (*hface)->f->halfedge->prev->vertex->p + (*hface)->f->halfedge->next->vertex->p) / 3;
				if (((*hv_)->p - p)*(*hface)->f->normal > 0){//表向きなら
					//距離を調べる
					if (leng < abs(((*hv_)->p - p)*(*hface)->f->normal)){
						leng = abs(((*hv_)->p - p)*(*hface)->f->normal);
						HH = (*hface);
					}
				}
			}
			if (HH != NULL){
				(*hv_)->f = HH->f;
				HH->v.push_back((*hv_)->p);
				hv_++;
			}
			else{
				Hullv  *vv = (*hv_);
				hv_ = hulv.erase(hv_);//頂点を除外
				delete vv;
			}
		}

		Vlist.clear();
		deletev.clear();
		fg.clear();

	}

	return;
}


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
	//cout << "m->face.size():" << m->faces.size() << "\n";
	for (it_f = m->faces.begin(); it_f != m->faces.end(); it_f++) {
		Vec3 p0 = (*it_f)->halfedge->vertex->p;
		Vec3 p1 = (*it_f)->halfedge->next->vertex->p;
		Vec3 p2 = (*it_f)->halfedge->prev->vertex->p;
		Vec3 Normal = (p1 - p0) % (p2 - p0); Normal.normalize();
		Normal = Normal * 100;
		glBegin(GL_TRIANGLES);
		glNormal3d(Normal.x, Normal.y, Normal.z);
		glVertex3d(p0.x, p0.y, p0.z);
		glVertex3d(p1.x, p1.y, p1.z);
		glVertex3d(p2.x, p2.y, p2.z);
		glEnd();
	}
}


void makeRandumColorCluster(double *col){
	//RGBのうち1つが255,1つが0,1つが0～255
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
std::vector<Vec2> centVer;
std::vector<Vec2> faceNormalL;
std::vector<Vec2> faceNormalR;
std::vector<std::vector<Vec2>> betweenY;
std::vector<std::vector<Vec2>> outlinePoints;
std::vector<std::vector<Vec3>> outlinePoints3D;
std::vector<Vec3> outlineDir;
std::vector<std::vector<Vec3>> planeS;
std::vector<std::vector<Vec3>> bet3DP;
std::vector<std::vector<Vec3>> normalDir;
Model *convexModel;

std::vector<Vec3> betCluster;

int sideclustnum = 3;

std::vector<int> cluster(std::vector<Vec2> points, int i, std::vector<std::vector<int>> &clusterd){
	int clustNum = sideclustnum;
	std::vector<int> clustIndex;
	std::vector<Vec2> cent;
	std::vector<Vec2> prev_cent;
	std::vector<int> clustAmount;

	cent.resize(clustNum);
	clustAmount.resize(clustNum);
	for (int i = 0; i < clustNum; i++) {
		clustAmount[i] = 0;
	}

	std::vector<Vec2> points_ = points;
	std::vector<int> pointNum;

	for (int i = 0; i < points.size(); i++) {
		pointNum.push_back(i);
	}

	for (int i = 0; i < points.size(); i++) {
		for (int j = 0; j < points.size(); j++) {
			if (points[i].y < points[i].y) {
				Vec2 tmp = points[i];
				points[i] = points[j];
				points[j] = tmp;
				int tmp_ = pointNum[i];
				pointNum[i] = pointNum[j];
				pointNum[j] = tmp_;
			}
		}
	}

	clustIndex.resize(points.size());

	for (int i = 0; i < points.size(); i++){
		int Num;
		if (i < points.size() / 3.0) {
			Num = 0;
		}
		else{
			if (i < 2 * points.size() / 3.0) {
				Num = 1;
			}
			else{
				Num = 2;
			}
		}
		clustIndex[pointNum[i]] = Num;
		cent[Num] += points[i];
		clustAmount[Num]++;
	}
	points = points_;
	for (int i = 0; i < clustNum; i++) {
		cent[i] = cent[i] / (double)clustAmount[i];
	}
	clusterd.resize(sideclustnum);
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
		for (int i = 0; i < points.size(); i++) {
			clusterd[clustIndex[i]].push_back(i);
		}

		if (!flg) {
			break;
		}
		prev_cent = cent;
	}

	int clustSize = 0;
	int count = 0;
	for (int i = 0; i < clustAmount.size(); i++, count++) {
		if (clustAmount[i] == 0) {
			clusterd.erase(clusterd.begin() + i);
			cent.erase(cent.begin() + i);
			count--;
		}
	}
	
	cout << "clusterd.size(): " << clusterd.size() << "\n";

	for (int i = 0; i < cent.size(); i++) {
		for (int j = 0; j < cent.size(); j++) {
			if (cent[i].y > cent[j].y) {
				std::vector<int> tmp;
				tmp = clusterd[j];
				clusterd[j] = clusterd[i];
				clusterd[i] = tmp;
				Vec2 tmp3D;
				tmp3D = cent[i];
				cent[i] = cent[j];
				cent[j] = tmp3D;
			}
		}
	}
	
	clustIndex.clear(); clustIndex.resize(points.size());
	for (int i = 0; i < clusterd.size(); i++) {
		for (int j = 0; j < clusterd[i].size(); j++) {
			clustIndex[clusterd[i][j]] = i;
		}
	}

	return clustIndex;
}


void renderModelCluster(Model *m) {
	std::list<Vertexs*>::iterator it_v;
	std::list<Faces*>::iterator it_f;
	glPointSize(5);
	glDisable(GL_LIGHTING);
	glColor3d(1.0, 0, 1.0);
	glPointSize(7);
	for (int i = 0; i < betCluster.size(); i++) {
		Vec3 p = betCluster[i];
		glPointSize(7);
		glBegin(GL_POINTS);
		glVertex3d(p.x, p.y, p.z);
		glEnd();
		if (i == 2) {
			glColor3d(1.0, 0, 0.0);
		}
		else {
			glColor3d(1.0, 0.5, 0.1);
		}
		glLineWidth(5);
		/*glBegin(GL_LINES);
		glVertex3d(p.x, p.y, p.z);
		glVertex3d(p.x + outlineDir[i].x, p.y + outlineDir[i].y, p.z + outlineDir[i].z);
		glEnd();*/
		/*glBegin(GL_POINTS);
		glVertex3d(p.x, p.y, p.z);
		glEnd();*/
	}

	for (int i = 0; i < planeS.size(); i++) {
		if (i == 2) {
			glColor3d(1.0, 0, 0.0);
		}
		else {
			glColor3d(0.0, 0.5, 0.1);
		}
		glBegin(GL_TRIANGLES);
		glVertex3d(planeS[i][0].x, planeS[i][0].y, planeS[i][0].z);
		glVertex3d(planeS[i][1].x, planeS[i][1].y, planeS[i][1].z);
		glVertex3d(planeS[i][2].x, planeS[i][2].y, planeS[i][2].z);
		glEnd();
		for (int j = 0; j < bet3DP[i].size(); j++) {
			Vec3 p = bet3DP[i][j];
			if (i == 2) {
				glColor3d(1.0, 0, 0.0);
			}
			else {
				glColor3d(0.0, 0.5, 0.1);
			}
			glBegin(GL_POINTS);
			glVertex3d(p.x, p.y, p.z);
			glEnd();
			
			glBegin(GL_LINES);
			glVertex3d(p.x, p.y, p.z);
			glVertex3d(p.x - normalDir[i][j].x, p.y - normalDir[i][j].y, p.z - normalDir[i][j].z);
			glEnd();
		}
	}

	glColor3d(0.0, 0.5, 0.1);
	glLineWidth(1);
	renderFoldModel(convexModel);
	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		Vec3 p = (*it_v)->p;
		Vec3 dirCurv = (*it_v)->normal*2*(*it_v)->curvture;
		

		/*glBegin(GL_LINES);
		glVertex3d(p.x, p.y, p.z);
		glVertex3d(p.x + dirCurv.x , p.y, p.z + dirCurv.z);
		glEnd();*/
		Vec3 adjP = (*it_v)->adjCenter;
		
		Vec2 dir((*it_v)->normal);
		dir.normalize();
		dir *= abs((*it_v)->curvtureY);
		if ((*it_v)->clusterNum == -1) {
			glColor3d(1, 0, 0);
		}
		else {
			double *col = color_cluster[(*it_v)->clusterNum];
			if ((*it_v)->sideClustNum % 5 == 0) {
				glColor3d(1.0, 0, 0);
			}
			else if ((*it_v)->sideClustNum % 5 == 1){
				glColor3d(0.0, 1.0, 0);
			}
			else if ((*it_v)->sideClustNum % 5 == 2){
				glColor3d(0.0, 0, 1.0);
			}
			else if ((*it_v)->sideClustNum % 5 == 3){
				glColor3d(1.0, 0, 1.0);
			}
			else if ((*it_v)->sideClustNum % 5 == 4){
				glColor3d(0.5, 0.1, 1.0);
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
			glPointSize(7);
			glBegin(GL_POINTS);
			glVertex3d(p.x, p.y, p.z);
			glEnd();
		
			/*if ((*it_v)->curvture < 0) {
				glColor3d(0.5, 0.0, 1.0);
			}
			else {
				glColor3d(1.0, 0.0, 0.5);
			}
			glBegin(GL_LINES);
			glVertex3d(p.x, p.y, p.z);
			glVertex3d(p.x + dir.x, p.y, p.z + dir.y);
			glEnd();*/
		
		}
	}


	for (int i = 0; i < (int)dirCluster.size(); i++) {
		if (i % 5 == 0) {
			glColor3d(1.0, 0, 0);
		}
		else if (i % 5 == 1){
			glColor3d(0.0, 1.0, 0);
		}
		else if (i % 5 == 2){
			glColor3d(0.0, 0, 1.0);
		}
		else if (i % 5 == 3) {
			glColor3d(1.0, 0, 1.0);
		}
		else if (i % 5 == 4) {
			glColor3d(0.0, 0.5, 1.0);
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

	glPointSize(5);
	for (int i = 0; i < clustVertex.size(); i++) {
		for (int j = 0; j < clustVertex[i].size(); j++) {
			if (i == 0) {
				glColor3d(0.0, 1.0, 0);
			}
			else if (i == 1) {
				glColor3d(1.0, 0.0, 0);
			}
			else{
				glColor3d(0.0, 0.0, 1.0);
			}
			//glPointSize(2*abs(clustVertex[i][j]->curvture));
			/*glBegin(GL_POINTS);
			glVertex3d(clustVertex[i][j]->p.x, clustVertex[i][j]->p.y, clustVertex[i][j]->p.z);
			glEnd();*/
		}
	}

	glColor3d(0.0, 0.0, 1.0);
	for (int i = 0; i < m->poly_p.size(); i++) {
		Vec3 startP, endP;
		if (i != m->poly_p.size() - 1) {
			startP.set(m->poly_p[i].x, 0, m->poly_p[i].y);
			endP.set(m->poly_p[i + 1].x, 0, m->poly_p[i + 1].y);
		}
		else{
			startP.set(m->poly_p[i].x, 0, m->poly_p[i].y);
			endP.set(m->poly_p[0].x, 0, m->poly_p[0].y);
		}
		glBegin(GL_LINES);
		glVertex3d(startP.x, startP.y, startP.z);
		glVertex3d(endP.x, endP.y, endP.z);
		glEnd();
	}

	glEnable(GL_LIGHTING);

	glBegin(GL_POLYGON);
	for (int i = pointPosition.size()-1; i >= 0; i--){
		Vec3 p; p.set(pointPosition[i].x, 0, pointPosition[i].y);
		glNormal3d(0, 1, 0);
		glVertex3d(p.x, m->fold->topPosY, p.z);
	}


	glEnd();

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
	
	Polyhedron_G D1,D2;
	D1 = boolDiff_P1_P2(P1, P2, true); // 立体-ウサギ
	D2 = boolDiff_P1_P2(P1, P2, false); //	ウサギ-立体
	if (D1.size_of_vertices() == 0 || D2.size_of_vertices() == 0) {
		return -1;
	}
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
	return volumeDiff / refeModelVolume;
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
	return boolDiff_P1_P2((*poly1), poly2, flg);
}

void outputAsObj(Polyhedron_G *poly) {
	
	std::ofstream ofs("MeshFile.obj");
	CGAL::print_polyhedron_wavefront(ofs, (*poly));
}

void Cmodel::metroPrepar() {
	inputC = new CMesh();
	foldC = new CMesh();
	openMesh(inputM, inputC);//CMeshへ変換
	openMesh(foldM, foldC);//CMeshへ変換
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

//	天頂面の形状を決めるために頂点を分類する
void coloring (Model *m) {
	//	エッジの平均を取って角度を決める
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

void removeTopinternalPoint() {//天頂面のxy平面内に含まれる頂点をのぞく
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
	clustVertex.clear();
	cout << "three Cluster\n";

	std::list<Vertexs*>::iterator it_v;
	std::vector<std::vector<Vertexs*>> clustV(3);
	std::vector<Vec2> centroid(3);
	std::vector<Vec2> precentroid(3);
	std::vector<Vertexs*> vertice;
	Vec3 topDir(0, 1, 0); 
	centroid[0].set(0, 0);
	centroid[1].set(0, 0);
	centroid[2].set(0, 0);

	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		vertice.push_back((*it_v));
	}

	for (int i = 0; i < vertice.size(); i++) {
		for (int j = 0; j < vertice.size(); j++) {
			if (vertice[i]->p.y > vertice[j]->p.y) {
				Vertexs* tmp = vertice[i];
				vertice[i] = vertice[j];
				vertice[j] = tmp;
			}
		}
	}

	for (int i = 0; i < vertice.size(); i++) {
		int num;
		if (i < vertice.size() / 3) {
			num = 0;
		}
		else{
			if (i < 2*vertice.size() / 3) {
				num = 1;
			}
			else{
				num = 2;
			}
		}
		vertice[i]->threeClust = num;
		
	}

	vertice.clear();
	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		vertice.push_back((*it_v));
		VertexCluster.push_back((*it_v)->threeClust);
		clustV[(*it_v)->threeClust].push_back((*it_v));
	}
	
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < clustV[i].size(); j++) {
			double rad = acos(topDir * clustV[i][j]->normal);
			Vec2 vec; vec.set(clustV[i][j]->p.y, rad);
			centroid[i] += vec;
		}
		centroid[i] = centroid[i] / (double)clustV[i].size();
		cout << "cent1: " << centroid[i].x << "," << centroid[i].y << "\n";
	}
	
	precentroid = centroid;
	while (1) {
		VertexCluster.clear();
		clustV[0].clear();
		clustV[1].clear();
		clustV[2].clear();
		//重心との距離を更新して割り当てしなおす
		for (int i = 0; i < vertice.size(); i++) {
			double rad = topDir * vertice[i]->normal;
			Vec2 vec; vec.set(vertice[i]->p.y, rad);
			int minN = 0;
			double minL = 1000000;
			for (int j = 0; j < 3; j++) {
				if ((centroid[j] - vec).length() < minL) {
					minL = (centroid[j] - vec).length();
					minN = j;
				}
			}
			VertexCluster.push_back(minN);
			clustV[minN].push_back(vertice[i]);
		}

		centroid[0].set(0, 0);
		centroid[1].set(0, 0);
		centroid[2].set(0, 0);

		//クラスタの重心座標を更新
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < clustV[i].size(); j++) {
				double rad = topDir * clustV[i][j]->normal;
				Vec2 vec; vec.set(clustV[i][j]->p.y, rad);
				centroid[i] += vec;
			}
			centroid[i] = centroid[i] / (double)clustV[i].size();
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
	clustVertex = clustV;
	cout << "set three end\n";
}

void setCluster(Model *m) {

	std::list<Vertexs*>::iterator it_v;
	Vec3 centroid; centroid.set(0, 0, 0);
	int topEdgeNum = 4;////
	std::vector<double> numCluster;
	//m->fold->pointPosition.clear();
	m->fold->betweenPosition.clear();
	m->fold->outlinepoints.clear();
	numCluster.resize(dirCluster.size());

	//std::vector<Vec3> mV;
	//for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
	//	mV.push_back((*it_v)->p);
	//}
	//

	//for (int i = 0; i < (int)dirCluster.size(); i++) {
	//	numCluster[i] = 0;
	//	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
	//		if ((*it_v)->clusterNum == i) {
	//			numCluster[i]++;
	//		}
	//	}
	//}
	//
	//while (1) {
	//	double minrad = 0;
	//	int combine[2];
	//	for (int i = 0; i < dirCluster.size(); i++) {
	//		//cout << "curvCluster] " << curvCluster[i] << "\n";
	//		double cos;
	//		double clusterNums = pow((1.0 / numCluster[i]),0.5);
	//		if (i == dirCluster.size() - 1) {
	//			cos = (dirCluster[i] * dirCluster[0]) *(dirCluster[i] * dirCluster[0]);
	//			clusterNums *= pow((1.0 / numCluster[0]), 0.5);
	//			if ((curvCluster[i] < 0 && curvCluster[0] > 0) ||
	//				(curvCluster[i] > 0 && curvCluster[0] < 0)) {
	//				//clusterNums *= 0.8;
	//			}
	//		}
	//		else {
	//			cos = (dirCluster[i] * dirCluster[i + 1]) * (dirCluster[i] * dirCluster[i + 1]);
	//			clusterNums *= pow((1.0 / numCluster[i + 1]),0.5);
	//			if ((curvCluster[i] < 0 && curvCluster[i + 1] > 0) ||
	//				(curvCluster[i] > 0 && curvCluster[i + 1] < 0)) {
	//				//clusterNums *= 0.8;
	//			}
	//		}
	//		cos *= clusterNums;
	//		if (minrad < abs(cos)) {
	//			if (i == dirCluster.size() - 1) {
	//				combine[0] = 0;
	//				combine[1] = i;
	//			}
	//			else {
	//				combine[0] = i;
	//				combine[1] = i + 1;
	//			}
	//			minrad = abs(cos);
	//		}
	//	}

	//	//cout << combine[0] << "," << combine[1] << "\n";
	//	dirCluster.erase(dirCluster.begin() + combine[1]);
	//	posCluster.erase(posCluster.begin() + combine[1]);
	//	curvCluster.erase(curvCluster.begin() + combine[1]);
	//	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {//クラスタ番号を更新
	//		if ((*it_v)->clusterNum == combine[1]) {
	//			(*it_v)->clusterNum = combine[0];
	//		}
	//		else if ((*it_v)->clusterNum > combine[1]){
	//			(*it_v)->clusterNum--;
	//		}
	//	}
	//	
	//	Vec3 updateDir; updateDir.set(0, 0, 0);
	//	Vec3 updatePos; updatePos.set(0, 0, 0);
	//	double updateCurv = 0;
	//	int count = 0;
	//	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {//クラスタ番号を更新
	//		if ((*it_v)->clusterNum == combine[0]) {
	//			updateDir += (*it_v)->normal;
	//			updatePos += (*it_v)->p;
	//			updateCurv += (*it_v)->curvture;
	//			count++;
	//		}
	//	}
	//	updateDir = updateDir / (double)count;
	//	updateDir.normalize();
	//	dirCluster[combine[0]] = updateDir;
	//	posCluster[combine[0]] = updatePos / (double)count;
	//	curvCluster[combine[0]] = updateCurv / (double)count;
	//	numCluster.resize(dirCluster.size());
	//	for (int i = 0; i < (int)dirCluster.size(); i++) {
	//		numCluster[i] = 0;
	//		for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
	//			if ((*it_v)->clusterNum == i) {
	//				numCluster[i]++;
	//			}
	//		}
	//	}

	//	if (dirCluster.size() <= topEdgeNum) {
	//		break;
	//	}
	//}

	std::vector<std::vector<Vec3>> betweenY3D;
	cout << "dirCluster.size(): " << dirCluster.size() << "\n";
	std::vector<std::vector<Vec2>> forClustValue;
	std::vector<std::vector<Vertexs*>> vers;
	std::vector<std::vector<std::vector<int>>> clusterS;

	forClustValue.resize(dirCluster.size());
	vers.resize(dirCluster.size());
	betweenY.resize(dirCluster.size());
	clusterS.resize(dirCluster.size());

	for (int i = 0; i < dirCluster.size(); i++) {

		Vec3 pos;
		int nextNum = i + 1;
		double count = 0;
		if (i == dirCluster.size() - 1){
			nextNum = 0;
		}
		
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
			Vec2 yCurv; yCurv.set((*it_v)->p.y, 2*(*it_v)->normal.y);
			forClustValue[i].push_back(yCurv);
			vers[i].push_back((*it_v));
		}
		
		//天頂面のx-z平面にあるのは後から除いておく
		clustVertex.push_back(vers[i]);//縦方向のクラスタリング
		clustValue.push_back(cluster(forClustValue[i], i, clusterS[i]));

		std::vector<int> clustside;
		clustside = clustValue[clustValue.size() - 1];
		std::vector<Vec3> bet3D;
		//ここに間を入れる
		cout << "vers[j]->sideClustNum\n";
	
		/*for (int j = 0; j < clusterS.size(); j++) {
			for (int k = 0; k < clusterS[j].size(); k++){
				vers[clusterS[j][k]]->sideClustNum = j;
			}
		}*/

		/*for (int kk = 0; kk < clusterS.size() - 1; kk++) {
			double count = 0;
			double min= 10000000, max = -10000000;
			Vec3 minV, maxV;
			double maxDis = 0;
			Vec3 maxDisP;
			for (int j = 0; j < vers.size(); j++) {
				if (clustside[j] == kk) {
					if (min > vers[j]->p.y) {
						min = vers[j]->p.y;
						minV = vers[j]->p;
					}
				}
				else if (clustside[j] == kk + 1){
					if (max < vers[j]->p.y) {
						max = vers[j]->p.y;
						maxV = vers[j]->p;
					}
				}
				double distance = abs((vers[j]->p - cent) * Normal);
				if (distance > maxDis) {
					maxDis = distance;
					maxDisP = vers[j]->p;
				}
			}
			Vec3 vet3DP = (maxV + minV) / 2;
			vet3DP.x = maxDisP.x;
			vet3DP.z = maxDisP.z;
			bet3D.push_back(vet3DP);
			betCluster.push_back(vet3DP);
		}*/

		/*for (int j = 0; j < vers.size(); j++) {
			vers[j]->sideClustNum = -1;
		}*/
		cout << "i in in : " << i << "\n";
		//betweenY3D.push_back(bet3D);

		Vec2 pos2D; pos2D.set(pos.x, pos.z);
		pos2D = pos2D / count;
		pointPosition.push_back(pos2D);
		//m->fold->pointPosition.push_back(pos2D);
	}

	pointPosition.push_back(pointPosition[0]);
	//m->fold->pointPosition.push_back(pointPosition[0]);
	pointPosition = m->fold->pointPosition;
	for (int i = 0; i < pointPosition.size() - 1; i++) {
		cout << "i: " << i << "\n";
		m->fold->betweenPosition.push_back((m->fold->pointPosition[i] + m->fold->pointPosition[i + 1]) / 2);
	}
	betweenPosition = m->fold->betweenPosition;
	betCluster.resize(dirCluster.size());
	for (int i = 0; i < dirCluster.size(); i++) {
		std::vector<Vec3> dirV;
		std::vector<Vec3> normalV;
		int nextNum = i;
		double count = 0;
		/*if (i == 0){
			nextNum = pointPosition.size() - 2;
		}*/
		cout << "nextNum: " << nextNum << "\n";

		Vec3 p1; p1.set(pointPosition[nextNum].x, m->fold->topPosY, pointPosition[nextNum].y);
		Vec3 p2; p2.set(pointPosition[nextNum + 1].x, m->fold->topPosY, pointPosition[nextNum + 1].y);
		Vec3 p3; p3.set(pointPosition[nextNum].x, m->fold->bottomPosY, pointPosition[nextNum].y);
		Vec3 cent = (p1 + p2 + p3) / 3.0;
		Vec3 Normal = (p2 - p3) % (p1 - p3); Normal.normalize();//平面のていぎ

		Vec3 pos;
		//betCluster.push_back(cent);
		std::vector<Vec3> plane;
		plane.push_back(p1);
		plane.push_back(p2);
		plane.push_back(p3);
		planeS.push_back(plane);

		std::vector<int> clustside;
		clustside = clustValue[i];
		std::vector<Vec3> bet3D;
		//ここに間を入れる
		for (int j = 0; j < clusterS[i].size(); j++) {
			for (int k = 0; k < clusterS[i][j].size(); k++){
				vers[i][clusterS[i][j][k]]->sideClustNum = j;
			}
		}

		for (int kk = 0; kk < clusterS[i].size() - 1; kk++) {
			double count = 0;
			double min = 10000000, max = -10000000;
			Vec3 minV, maxV;
			double maxDis = 0;
			Vec3 maxDisP;
			double meanY = 0;
			for (int j = 0; j < vers[i].size(); j++) {
				if (clustside[j] == kk) {
					if (min > vers[i][j]->p.y) {
						min = vers[i][j]->p.y;
						minV = vers[i][j]->p;
					}
					//if (kk > 0) {
						Halfedge *h = vers[i][j]->halfedge;
						do {
							if (h->next->vertex->clusterNum != i) {
								h = h->prev->pair;
								continue;
							}
							if (h->next->vertex->sideClustNum > kk){
								dirV.push_back(vers[i][j]->p);
								double distance = abs((vers[i][j]->p - cent) * Normal);
								normalV.push_back(Normal * distance);
								meanY += (vers[i][j]->p.y);
								count++;
								if (distance > maxDis) {
									maxDis = distance;
									maxDisP = vers[i][j]->p;
								}
							}
							h = h->prev->pair;
						} while (h != vers[i][j]->halfedge);
					//}
				}
				else if (clustside[j] == kk + 1){
					if (max < vers[i][j]->p.y) {
						max = vers[i][j]->p.y;
						maxV = vers[i][j]->p;
					}
				}

			}
			Vec3 vet3DP = (maxV + minV) / 2;
			vet3DP.x = maxDisP.x;
			vet3DP.z = maxDisP.z;
			vet3DP.y = meanY / count;
			bet3D.push_back(vet3DP);
			betCluster.push_back(vet3DP);
			//dirV.push_back(vet3DP);
		}
		bet3DP.push_back(dirV);
		normalDir.push_back(normalV);
		/*for (int j = 0; j < vers.size(); j++) {
			vers[i][j]->sideClustNum = -1;
		}*/
		betweenY3D.push_back(bet3D);

	}

	//cout << "three cluster\n";
	////setThreeCluster(m);
	for (int i = 0; i < betweenY3D.size(); i++) {
		std::vector<Vec2> outlineOne;
		Vec2 begin; begin.set(0, 0);
		outlineOne.push_back(begin);
		outline *out = new outline();
		out->points.push_back(begin);
		int num = i;


		for (int j = 0; j < betweenY3D[num].size(); j++) {
			Vec3 p1; p1.set(pointPosition[i].x, m->fold->topPosY, pointPosition[i].y);
			Vec3 p2; p2.set(pointPosition[i+1].x, m->fold->topPosY, pointPosition[i+1].y);
			Vec3 p3; p3.set(pointPosition[i].x, m->fold->bottomPosY, pointPosition[i].y);
			Vec3 cent = (p1 + p2 + p3) / 3.0;
			Vec3 Normal = (p2 - p3) % (p1 - p3); Normal.normalize();
			double distance = abs((betweenY3D[num][j] - cent) * Normal);
			double Y = abs(m->fold->topPosY - betweenY3D[num][j].y);
			Vec2 outP; outP.set(distance, Y);
			outlineOne.push_back(outP);
			out->points.push_back(outP);
			Vec3 outp; outp.set(outP.x, betweenY3D[num][j].y, outP.y);
			
			//soutlineDir.push_back(distance * Normal);
		}
		outlinePoints.push_back(outlineOne);
		m->fold->outlinepoints.push_back(out);
	}


	//一番下の座標を計算したい
	std::vector<std::vector<Vec3>> vCluster(dirCluster.size());
	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		vCluster[(*it_v)->clusterNum].push_back((*it_v)->p);
	}

	for (int i = 0; i < m->fold->outlinepoints.size(); i++) {
		for (int j = 0; j < m->fold->outlinepoints[i]->points.size(); j++) {
			cout << m->fold->outlinepoints[i]->points[j].x << "," << m->fold->outlinepoints[i]->points[j].y << "\n";
		}
	}

	std::list<Faces*>::iterator it_f;
	cout << dirCluster.size() << "\n";
	for (int i = 0; i < dirCluster.size(); i++) {
		Model *modelR = new Model();
		int num = i;//+1;
		
		/*if (i == dirCluster.size() - 1) {
			num = 0;
		}*/
		//num = i+1;
		convexhull(vCluster[num], modelR);//凸c包作って交差を調べる
		if (i == 2) {
			convexModel = modelR;
		}
		Vec2 planeDir;
		Vec2 dir = pointPosition[i + 1] - pointPosition[i]; dir.normalize();
		planeDir.x = 1; planeDir.y = -(dir.x / dir.y); planeDir.normalize();
		if ((centroid - betweenPosition[i])*planeDir < 0) planeDir *= -1;//	|→を求める

		Vec3 p1; p1.set(betweenPosition[i].x, m->fold->topPosY, betweenPosition[i].y);
		Vec3 p2; p2.set(betweenPosition[i].x + planeDir.x, m->fold->topPosY, betweenPosition[i].y + planeDir.y);
		Vec3 p3; p3.set(betweenPosition[i].x, m->fold->bottomPosY, betweenPosition[i].y);
		p1 = p1 * 1000;
		p2 = p2 * 1000;
		p3 = p3 * 1000;
		std::vector<Vec3> plane;
		plane.push_back(p1);
		plane.push_back(p2);
		plane.push_back(p3);
		Vec3 cent = (p1 + p2 + p3) / 3.0;
		Vec3 Normal = (p2 - p3) % (p1 - p3); Normal.normalize();
		double minY = 1000000;
		Vec3 bottomPos;
		for (it_f = modelR->faces.begin(); it_f != modelR->faces.end(); it_f++) {
			Halfedge *h = (*it_f)->halfedge;
			do {
				Vec3 A = h->vertex->p;
				Vec3 B = h->next->vertex->p;
				
				Vec3 x = A + (B - A)*(((A - cent)*Normal) / ((A - cent)*Normal + (B - cent)*Normal));
				if (minY > x.y) {
					minY = x.y;
					bottomPos = x;
				}
				h = h->next;
			} while (h != (*it_f)->halfedge);
		}
		betCluster.push_back(bottomPos);
		//if (i == 2) {
			//betCluster.push_back(bottomPos);
		//}
		p1.set(pointPosition[i].x, m->fold->topPosY, pointPosition[i].y);
		p2.set(pointPosition[i+1].x, m->fold->topPosY, pointPosition[i+1].y);
		p3.set(pointPosition[i].x, m->fold->bottomPosY, pointPosition[i].y);
		
		//planeS.push_back(plane);
		cent = (p1 + p2 + p3) / 3.0;
		//betCluster.push_back(cent);
		Normal = (p2 - p3) % (p1 - p3); Normal.normalize();
		double distance = abs((bottomPos - cent) * Normal);
		double Ypos = abs(m->fold->bottomPosY - m->fold->topPosY);
		Vec2 bottomVec2; bottomVec2.set(distance, Ypos);
		if ((m->fold->betweenPosition[i]-bottomVec2) * Normal > 0) {
			bottomVec2.x *= -1;
		}
		outlinePoints[i].push_back(bottomVec2); 
		m->fold->outlinepoints[i]->points.push_back(bottomVec2);
		//if (i == 2) {
			//outlineDir.push_back(distance * Normal);
		//}
	}

	for (int i = 0; i < m->fold->outlinepoints.size(); i++) {
		cout << "after outline: " << i << "\n";
		for (int j = 0; j < m->fold->outlinepoints[i]->points.size(); j++) {
			cout << m->fold->outlinepoints[i]->points[j].x << "," << m->fold->outlinepoints[i]->points[j].y << "\n";
		}
	}
	std::reverse(m->fold->pointPosition.begin(), m->fold->pointPosition.end());
	std::reverse(m->fold->betweenPosition.begin(), m->fold->betweenPosition.end());
	std::reverse(m->fold->outlinepoints.begin(), m->fold->outlinepoints.end());
	
	//cout << "betCluster.size(): " << betCluster.size() << "\n";
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
	//頂点の法線
	Vec3 topDir; topDir.set(0, 1, 0);
	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++){
		Halfedge *h = (*it_v)->halfedge;
		int verCount = 0;
		Vec3 vertexNormal;
		vertexNormal.set(0, 0, 0);
		std::vector<Vec2> faceBary;
		std::vector<Vec2> faceNormal;
		do {
			vertexNormal += h->face->normal;
			(*it_v)->adjCenter += h->next->vertex->p;
			verCount++;
			
			Vec2 bary(h->face->bary);     faceBary.push_back(bary);
			Vec2 normal(h->face->normal); normal.normalize();  faceNormal.push_back(normal);
			h = h->prev->pair;
		} while (h != (*it_v)->halfedge);

		(*it_v)->normal = vertexNormal / (double)verCount;
		Vec2 normal2D((*it_v)->normal); normal2D.normalize();
		Vec2 v2D((*it_v)->p);

		(*it_v)->adjCenter = (*it_v)->adjCenter / (double)verCount;

		std::vector<int> left;
		std::vector<int> right;
		for (int i = 0; i < faceBary.size(); i++) {
			if (normal2D % (faceBary[i] - v2D) > 0) {//左右に分類
				left.push_back(i);
			}
			else{
				right.push_back(i);
			}
		}
		
		Vec2 leftNormal; leftNormal.set(0, 0);
		Vec2 rightNormal; rightNormal.set(0, 0);
		for (int i = 0; i < left.size(); i++) {//left,rightで成す角が最大に成るものを求める
			leftNormal += faceNormal[left[i]];
		}

		leftNormal = leftNormal / (double)left.size();
		leftNormal.normalize();

		for (int i = 0; i < right.size(); i++) {//left,rightで成す角が最大に成るものを求める
			rightNormal += faceNormal[right[i]];
		}

		rightNormal = rightNormal / (double)right.size();
		rightNormal.normalize();

		faceNormalL.push_back(leftNormal);
		faceNormalR.push_back(rightNormal);
		Vec3 dirCent = (*it_v)->adjCenter - (*it_v)->p; dirCent.normalize();
		Vec2 verNormal((*it_v)->normal);
		(*it_v)->curvture = verNormal.length() * verNormal.length() * (2 * acos(rightNormal * leftNormal));

		if (rightNormal % leftNormal < 0) {
			(*it_v)->curvture *= -1;
		}
		

		(*it_v)->curvtureY = (*it_v)->normal.y;

		

		//法線が上の方向の面だけ見る

	}

	m->fold = new foldmethod();
	m->fold->topPosY = top;
	m->fold->bottomPosY = bottom;
	setThreeCluster(m);
}

std::vector<Vec2> convertTo2D(Model *m) {
	std::vector<Vec2> returnValue;
	std::list<Vertexs*>::iterator it_v;
	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++) {
		Vec2 p((*it_v)->p.x, (*it_v)->p.z);
		returnValue.push_back(p);
	}
	return returnValue;
}

double distancepointline(Vec2 P, Vec2 A, Vec2 B){//点PからABへの最短距離
	Vec2 ABnor = B - A; ABnor.normalize();
	double Ax = ABnor * (P - A);
	Vec2 x = A + (ABnor*Ax);

	return (P - x).length();
}

Vec2 vecpointline(Vec2 P, Vec2 A, Vec2 B){//点PからABへの最短距離
	Vec2 ABnor = B - A; ABnor.normalize();
	double Ax = ABnor * (P - A);
	Vec2 x = A + (ABnor*Ax);

	return x;
}

typedef K2D::Point_2 Point;

void Quickhull2D(const std::vector<Vec2> points, Model *m){
	//最初

	std::vector<Vec2> point_;
	m->all_p.clear();
	m->cross_p.clear();
	m->convex_line.clear();

	m->convex_cent.set(0, 0);
	for (int i = 0; i<(int)points.size(); i++){
		point_.push_back(points[i]);
		m->all_p.push_back(points[i]);
		m->convex_cent += points[i];
	}

	m->convex_cent /= (double)points.size();

	std::vector<line*> lines;
	std::sort(point_.begin(), point_.end(), Vec2::compareVec2PredicateX);//X軸ソート

	//最初の三角形をつくる
	double dis = 0;
	int far_num = 0;

	for (int i = 1; i<(int)point_.size() - 1; i++){
		double l = distancepointline(point_[i], point_[0], point_[point_.size() - 1]);

		if (l > dis){
			dis = l;
			far_num = i;
		}
	}
	//////cout << "far_num; " << far_num << "\n";
	Vec2D *v1 = new Vec2D(point_[0]);
	Vec2D *v2 = new Vec2D(point_[(int)point_.size() - 1]);
	Vec2D *v3 = new Vec2D(point_[far_num]);

	line *l1 = new line(0, v1, v2);
	line *l2 = new line(0, v2, v3);
	line *l3 = new line(0, v3, v1);

	lines.push_back(l1);//最初の三角形ができました
	lines.push_back(l2);
	lines.push_back(l3);

	Vec2 convex_cent = (point_[0] + point_[(int)point_.size() - 1] + point_[far_num]) / 3.0;

	l1->setNormal(convex_cent);//線の法線と交点をセット
	l2->setNormal(convex_cent);
	l3->setNormal(convex_cent);

	point_.erase(point_.begin() + ((int)point_.size() - 1));
	point_.erase(point_.begin() + far_num);
	point_.erase(point_.begin());

	//points_は点が凸包内部にある場合と凸包を構成する場合に削除。0になったら終わり
	while (1){

		std::vector<std::vector<int>> point_to_line;
		point_to_line.resize((int)lines.size());

		for (int i = 0; i<(int)point_.size(); i++){//三角形内部にある点を削除
			int count = 0;
			for (int j = 0; j<(int)lines.size(); j++){
				Vec2 X = vecpointline(point_[i], lines[j]->start->p, lines[j]->end->p);
				//m->cross_p.push_back(X);
				if ((X - convex_cent)*(X - point_[i]) > 0){//内側にある
					count++;
				}
			}
			if (count == (int)lines.size()){
				//内側にあるpt
				//////cout << "erased: " << i << "\n";
				point_.erase(point_.begin() + i);
				i--;
				//m->cross_p.push_back(point_[i]);
			}
		}


		for (int i = 0; i<(int)point_.size(); i++){//点を線へと割り当てる
			bool flg = false;
			for (int j = 0; j<(int)lines.size(); j++){
				//このr1,r2が２つとも正ならOK
				Vec2 X = vecpointline(point_[i], lines[j]->start->p, lines[j]->end->p);

				Vec2 vec1 = (lines[j]->end->p - lines[j]->start->p);  vec1.normalize();
				Vec2 vec2 = (lines[j]->start->p - lines[j]->end->p);  vec2.normalize();

				Vec2 vec3 = X - point_[i];


				if (vec2*(point_[i] - lines[j]->end->p) >0 && vec1*(point_[i] - lines[j]->start->p) > 0){//
					if ((X - point_[i])*(X - convex_cent) < 0){
						point_to_line[j].push_back(i);
						flg = true;
						break;
					}
				}
			}
			if (!flg){
				point_.erase(point_.begin() + i);
				i--;
			}
		}

		if (point_.size() == 0){
			break;
		}

		double max_dis = 0; int max_num[2];
		int counts = 0;
		for (int i = 0; i<(int)point_to_line.size(); i++){
			for (int j = 0; j<(int)point_to_line[i].size(); j++){//一番遠い点を確認する
				double l = distancepointline(point_[point_to_line[i][j]], lines[i]->start->p, lines[i]->end->p);
				if (l > max_dis){
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
		point_.erase(point_.begin() + max_num[1]);

		//一番遠い点が決定した
		//除去する点と線を選ぶ
		std::vector<line*> delete_line;
		std::vector<int> delete_line_num;

		int delete_start = 0;
		for (int i = 0; i<(int)lines.size(); i++){
			Vec2 vec = max_far_p - lines[i]->cross; vec.normalize();
			if (vec*lines[i]->normal > 0){//最遠点から見えるので削除する
				delete_line.push_back(lines[i]);
				delete_line_num.push_back(i);
			}
		}
		if (delete_line_num.size() == 0){
			//m->cross_p.push_back(max_far_p);

			break;
		}

		Vec2D *far_p_it = new Vec2D(max_far_p);
		//ラインを加える
		line *addLine1 = new line(0, delete_line[0]->start, far_p_it);
		line *addLine2 = new line(0, far_p_it, delete_line[(int)delete_line.size() - 1]->end);
		addLine1->setNormal(convex_cent);
		addLine2->setNormal(convex_cent);

		//線を削除
		for (int i = (int)delete_line.size() - 1; i >= 0; i--){
			lines.erase(lines.begin() + (unsigned int)delete_line_num[i]);
		}

		lines.insert(lines.begin() + delete_line_num[0], addLine1);//線を追加
		lines.insert(lines.begin() + delete_line_num[0] + 1, addLine2);

		//break;

		if (point_.size() == 0){
			break;
		}
		point_to_line.resize((int)lines.size());
	}


	////cout << "lines.size(): " << lines.size() << "\n";
	for (int i = 0; i<(int)lines.size(); i++){
		m->convex_line.push_back(lines[i]);
		//////cout << "lines[i]: " << lines[i]->start->p.x << "," <<lines[i]->start->p.y << "\n";
	}

	for (int i = 0; i<(int)point_.size(); i++){
		//m->cross_p.push_back(point_[i]);
	}

	//m->convex_cent = convex_cent;

}

bool judgeIntersected2D(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
	double ta = (c.x - d.x) * (a.y - c.y) + (c.y - d.y) * (c.x - a.x);
	double tb = (c.x - d.x) * (b.y - c.y) + (c.y - d.y) * (c.x - b.x);
	double tc = (a.x - b.x) * (c.y - a.y) + (a.y - b.y) * (a.x - c.x);
	double td = (a.x - b.x) * (d.y - a.y) + (a.y - b.y) * (a.x - d.x);
	if (tc * td < 0 && ta * tb < 0){
		return true;
	}
	else{
		return false;
	}
}

void reductionTopPolygon(Model *m) {//2Dポリゴンのsimplification
	cout << "reduction Start\n";
	int Nth = 6;
	std::vector<Vec2> pointPositionFirst;


	while (1) {
		double maxGap = 10000000;
		Vec2D *deleteV;
		int deleteLine[2];
		for (int i = 0; i < m->convex_line.size(); i++){
			int Num[2];
			Vec2 A = m->convex_line[i]->start->p;
			Num[0] = i;
			Vec2 B;
			if (i == m->convex_line.size() - 1) {
				B = m->convex_line[0]->end->p;
				Num[1] = 0;
			}
			else{
				B = m->convex_line[i + 1]->end->p;
				Num[1] = i+1;
			}
			double distance = distancepointline(m->convex_line[i]->end->p, A, B);
			if (maxGap > distance) {
				maxGap = distance;
				deleteV = m->convex_line[Num[1]]->start;
				deleteLine[0] = Num[0];
				deleteLine[1] = Num[1];
			}
		}

		line *l1 = deleteV->l1;//自分が始点
		line *l2 = deleteV->l2;//自分が終点
		cout << "deleteLine[0]: " << deleteLine[0] << ": deleteLine[2]: " << deleteLine[1] << "\n";
		//つなぎなおす
		deleteV->l1->start = deleteV->l2->start->l2->end;
		deleteV->l2->start->l2->end->l1 = l1;
		
		m->convex_line.erase(m->convex_line.begin() + deleteLine[0]);
		
		if (m->convex_line.size() <= Nth) {
			break;
		}
	}

	for (int i = 0; i < m->convex_line.size(); i++) {
		Vec2 p; p.set(m->convex_line[i]->start->p.x, m->convex_line[i]->start->p.y);
		m->poly_p.push_back(p);
		pointPositionFirst.push_back(p);
		m->fold->pointPosition.push_back(p);
	}

	/*Point *points = new Point[m->convex_line.size()];
	///m->fold = new foldmethod();
	for (int i = 0; i < m->convex_line.size(); i++){
		points[i] = Point(m->convex_line[i]->start->p.x, m->convex_line[i]->start->p.y);
	}

	Polygon_2 pgn(points, points + m->convex_line.size());
	Cost cost;
	pgn = PS::simplify(pgn, cost, Stop(Nth));
	Vertex_iterator_2D it_v;
	for (it_v = pgn.vertices_begin(); it_v != pgn.vertices_end(); it_v++) {
		Vec2 p; p.set(it_v->x(), it_v->y());
		
		m->poly_p.push_back(p);
		pointPositionFirst.push_back(p);
		m->fold->pointPosition.push_back(p);
	}*/

	std::list<Vertexs*>::iterator itV;
	std::list<Halfedge*>::iterator it_h;
	double max = -100000;
	double min = 100000;
	for (itV = m->vertices.begin(); itV != m->vertices.end(); itV++) {
		if (max < (*itV)->p.y) {
			max = (*itV)->p.y;
		}
		if (min >(*itV)->p.y) {
			min = (*itV)->p.y;
		}
	}

	double length = abs(max - min) / 10;
	double topPosY = max - length;
	Vec3 N(0, 1, 0);//平面の
	Vec3 planeCent(0, topPosY, 0);
	cout << "topPosy@ " << topPosY << "\n";
	Vec3 centroid; centroid.set(0, 0, 0);
	int count = 0;

	std::vector<Vec2> topConvex;
	cout << "ここ?";
	for (it_h = m->halfs.begin(); it_h != m->halfs.end(); it_h++) {
		Vec3 A = (*it_h)->vertex->p;
		Vec3 B = (*it_h)->next->vertex->p;
		//cout << "bb ";
		if (((planeCent - A)*N >= 0 && (planeCent - B)*N <= 0) || ((planeCent - A)*N <= 0 && (planeCent - B)*N >= 0)){//交差
			cout << "aa ";
			Vec3 crossP;
			crossP = A + (B - A)*(abs((planeCent - A)*N) / (abs((planeCent - A)*N) + abs((planeCent - B)*N)));
			bool flg = false;
			for (int i = 0; i < topConvex.size(); i++) {
				if (crossP == topConvex[i]) {
					flg = true;
					break;
				}
			}
			if (!flg) {
				topConvex.push_back(crossP);
			}
		}
	}
	cout << "topConvex:" << topConvex.size() << "\n";
	Quickhull2D(topConvex, m);	
	cout << "convex end\n";

	Vec2 pointCent; pointCent.set(0, 0);
	for (int i = 0; i < m->fold->pointPosition.size(); i++) {
		pointCent += m->fold->pointPosition[i];
	}

	pointCent = pointCent / (double)m->fold->pointPosition.size();
	Vec2 moveDir = m->convex_cent - pointCent;

	for (int i = 0; i < m->fold->pointPosition.size(); i++) {
		m->fold->pointPosition[i] = m->fold->pointPosition[i] + moveDir;
	}

	//スケールを変更する
	bool flg = false;
	double scale = 1;
	double prescale = scale;
	double addValue = 0.1;
	while (1){
		count = 0;
		//交点が見つかるまで繰り返す
		for (int i = 0; i<m->fold->pointPosition.size(); i++){//六角形と交差しているかここをconve
			Vec2 p1 = m->fold->pointPosition[i];
			for (int j = 0; j<(int)m->convex_line.size(); j++){
				if (judgeIntersected2D(p1, m->convex_cent, m->convex_line[j]->start->p, m->convex_line[j]->end->p)){
					count++;
				}
			}
		}

		if (count == 1){
			break;
		}
		else{
			if (count == 0){
				if (prescale == scale + addValue){
					addValue /= 10.0;
				}
				prescale = scale;
				scale += addValue;
			}
			else if (count >= 2){
				if (prescale == scale - addValue){
					addValue /= 10.0;
				}
				prescale = scale;
				scale -= addValue;
			}
			for (int i = 0; i<m->fold->pointPosition.size(); i++){//拡大
				m->fold->pointPosition[i] = pointPositionFirst[i] * scale + m->convex_cent;
			}
		}
	}
	m->fold->pointPosition.push_back(m->fold->pointPosition[0]);
	m->poly_p.clear();

	for (int i = 0; i < m->fold->pointPosition.size(); i++) {
		m->poly_p.push_back(m->fold->pointPosition[i]);
	}

	dirCluster.resize(m->fold->pointPosition.size() - 1);

	for (itV = m->vertices.begin(); itV != m->vertices.end(); itV++) {
		for (int i = 0; i < m->fold->pointPosition.size()-1; i++) {
			Vec2 dir1 = m->fold->pointPosition[i] - m->convex_cent; dir1.normalize();
			Vec2 dir2 = m->fold->pointPosition[i + 1] - m->convex_cent; dir2.normalize();
			Vec2 posDir; posDir.set((*itV)->p.x, (*itV)->p.z);
			posDir = posDir - m->convex_cent; posDir.normalize();
			double radA = acos(dir1*dir2);
			double radB = acos(dir1*posDir);
			double radC = acos(dir2*posDir);

			if (radB < radA && radC < radA) {
				(*itV)->clusterNum = i;
			}
		}
	}

	
}

