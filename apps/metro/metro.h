#pragma once



#include <time.h>


#include <vcg/math/histogram.h>
#include <vcg/complex/complex.h>
#include <vcg/simplex/face/component_ep.h>
#include <vcg/complex/algorithms/update/component_ep.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include "sampling.h"
#include "defineData.h"


#define NO_SAMPLES_PER_FACE             10
#define N_SAMPLES_EDGE_TO_FACE_RATIO    0.1
#define BBOX_FACTOR                     0.1
#define INFLATE_PERCENTAGE			    0.02
#define MIN_SIZE					    125		/* 125 = 5^3 */
#define N_HIST_BINS                     256
#define PRINT_EVERY_N_ELEMENTS          1000


class CFace;
class CVertex;
struct UsedType :public vcg::UsedTypes< vcg::Use<CFace>::AsFaceType, vcg::Use<CVertex>::AsVertexType>{};
class CVertex : public vcg::Vertex<UsedType, vcg::vertex::Coord3d, vcg::vertex::Qualityf, vcg::vertex::Normal3d, vcg::vertex::Color4b, vcg::vertex::BitFlags> {};
class CFace : public vcg::Face< UsedType, vcg::face::VertexRef, vcg::face::Normal3d, vcg::face::EdgePlane, vcg::face::Color4b, vcg::face::Mark, vcg::face::BitFlags> {};
class CMesh : public vcg::tri::TriMesh< std::vector<CVertex>, std::vector<CFace> > {};



CMesh openMesh(Model *m);
double calcMetro(CMesh S1, CMesh S2);
void setMeshInfo(CMesh *S1);