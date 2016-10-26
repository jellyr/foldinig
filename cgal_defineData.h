#ifndef CGAL_DEFINEDATA_H
#define CGAL_DEFINEDATA_H

#include <vector>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Gmpq.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Exact_integer.h>
#include <CGAL/Homogeneous.h>
#include <CGAL/Nef_polyhedron_3.h>
#include "defineData.h"
#include "apps/metro/metro.h"

typedef CGAL::Simple_cartesian<CGAL::Gmpq> G_Kernel;
typedef CGAL::Simple_cartesian<double> C_Kernel;
typedef CGAL::Surface_mesh<G_Kernel::Point_3> Mesh;
typedef CGAL::Polyhedron_3<G_Kernel> Polyhedron_G;
typedef CGAL::Polyhedron_3<C_Kernel, CGAL::Polyhedron_items_with_id_3> Polyhedron_C;//	穴をふさぐ時に使用する
typedef Polyhedron_G::HalfedgeDS HalfedgeDS_G;
typedef Polyhedron_C::HalfedgeDS HalfedgeDS_C;
//	bool演算用の定義
typedef CGAL::Nef_polyhedron_3<G_Kernel>              Nef_polyhedron_3;
typedef G_Kernel::Point_3                             Point_3;
typedef CGAL::Polyhedron_3<G_Kernel>                  Poly_nef;


class Cmodel;

class Cmodel {
public:
	Mesh *cgalM; //	CGALのメッシュ
	Model *foldM;//	折りたたみデータ
	Model *inputM;//　リファレンスモデル
	Polyhedron_G *cgalPoly;//	inputModelをcgalのPolyhedronに変換
	Polyhedron_G *foldPoly;//	foldModelをcgalのPolyhedronに変換
	Polyhedron_G *diffPoly;//	foldPoly - cgalPoly;
	Nef_polyhedron_3 cgalPoly_Nef;
	CMesh *inputC;
	CMesh *foldC;
	Cmodel() {
		cgalM = new Mesh();
		cgalPoly = new Polyhedron_G();
		diffPoly = new Polyhedron_G();
	}
	void metroPrepar();
};

// cgalPolyを生成するためのクラス。Vertexs* Faces*を入れる
template<class HDS>
class polyhedron_builder : public CGAL::Modifier_base<HDS> {
public:

	std::vector<Faces*>   faces;
	std::vector<Vertexs*> vertices;
	polyhedron_builder(std::vector<Vertexs*> _vertices, std::vector<Faces*> _faces) : vertices(_vertices), faces(_faces) {}
	
	void operator()(HDS& hds) {
		typedef typename HDS::Vertex   Vertex;
		typedef typename Vertex::Point Point;

		// create a cgal incremental builder
		CGAL::Polyhedron_incremental_builder_3<HDS> poly(hds, true);
		poly.begin_surface(vertices.size(), faces.size());
		  
		// add the polyhedron vertices
		for (int i = 0; i<(int)vertices.size(); i++){
			poly.add_vertex(Point(vertices[i]->p.x, vertices[i]->p.y, vertices[i]->p.z));
		}

		// add the polyhedron triangles
		for (int i = 0; i<(int)faces.size(); i++){
			int p0 = faces[i]->halfedge->vertex->num;
			int p1 = faces[i]->halfedge->next->vertex->num;
			int p2 = faces[i]->halfedge->prev->vertex->num;

			poly.begin_facet();
			poly.add_vertex_to_facet(p0);
			poly.add_vertex_to_facet(p1);
			poly.add_vertex_to_facet(p2);
			poly.end_facet();
		}

		// finish up the surface
		poly.end_surface();
	}
};

void inputMesh(Model *m);//	foldMからCGALメッシュを作成
void inputPoly();//	foldMからCGAL多角形を作成
Polyhedron_G *inputPoly_Gnew(Model *m);
Polyhedron_G inputPoly_G(Model *m);
Polyhedron_C *inputPoly_C(Model *m);//	入力モデルからPolyhdronを生成
void HoleFill (Polyhedron_C *poly);//	cgalPolyの穴を閉じる作業

void renderCgalMesh();
void renderInputModel();
void renderFoldModel(Model *m);
void rendercgalPoly(Polyhedron_G *cgalPoly);
double calcVolume(Model *m);
double calcVolume(Polyhedron_G cgalPoly);
Polyhedron_G *PolyC_to_Poly_G(Polyhedron_C *poly_c);
Polyhedron_G *holeFillAndConvertPolyG(Model *m);
double calculateDiff(Polyhedron_G *P1, Polyhedron_G *P2);
double calculateDiff(Polyhedron_G P1, Nef_polyhedron_3 P2, Polyhedron_G *P2_);
Nef_polyhedron_3 convert_Poly_NefPoly(Polyhedron_G poly);
Polyhedron_G TestMesh(Polyhedron_G *poly1, Nef_polyhedron_3 poly2, bool flg);
void outputAsObj(Polyhedron_G *poly);
void calcCurvture(Model *m);

void convertPolyToModel(Model *m);
void coloring(Model *m);
void renderModelCluster(Model *m);
void setCluster(Model *m);
std::vector<int> cluster(std::vector<Vec2> points);

void removeTopinternalPoint();
void bottomPlaneIntersection(Model *m, Vec3 centroid);
void setThreeCluster(Model *m);//上、側面、底面に分ける

#endif