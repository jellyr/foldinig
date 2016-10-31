
// standard libraries
#include "metro.h"

using namespace std;
using namespace vcg;

void openMesh(Model *m, CMesh *S) {
	std::list<Vertexs*>::iterator it_v;
	std::list<Faces*>::iterator it_f;
	CMesh S1;
	CMesh::VertexIterator vi = vcg::tri::Allocator<CMesh>::AddVertices(S1, m->vertices.size());
	CMesh::FaceIterator fi = vcg::tri::Allocator<CMesh>::AddFaces(S1, m->faces.size());
	CMesh::VertexPointer *ivp = new CMesh::VertexPointer[(int)(m->vertices.size())+1];

	int vertexCount = 0;
	for (it_v = m->vertices.begin(); it_v != m->vertices.end(); it_v++, vertexCount++) {
		ivp[vertexCount] = &*vi;
		vi->P() = CMesh::CoordType((*it_v)->p.x, (*it_v)->p.y, (*it_v)->p.z);
		vi++;
	}
	for (it_f = m->faces.begin(); it_f != m->faces.end(); it_f++) {
		Halfedge *h = (*it_f)->halfedge;
		fi->V(0) = ivp[h->vertex->num];
		fi->V(1) = ivp[h->next->vertex->num];
		fi->V(2) = ivp[h->prev->vertex->num];
		fi++;
	};

	vcg::tri::Append<CMesh, CMesh>::MeshCopy((*S), S1);
}

void changeVertexPos(Model *m, CMesh *S) {
	CMesh S1;
	vcg::tri::Append<CMesh, CMesh>::MeshCopy(S1, (*S));
	std::list<Vertexs*>::iterator it_v;
	it_v = m->vertices.begin(); 
	int vertexSize = S1.VN();

	for (int i = 0; i < vertexSize; i++, it_v++) {
		CMesh::VertexPointer vi = &S1.vert[i];
		vi->P() = CMesh::CoordType((*it_v)->p.x, (*it_v)->p.y, (*it_v)->p.z);
	}
	vcg::tri::Append<CMesh, CMesh>::MeshCopy((*S), S1);
}

void setMeshInfo(CMesh *S1) {
	CMesh S;
	vcg::tri::Append<CMesh, CMesh>::MeshCopy(S, (*S1));
	// compute face information
	tri::UpdateComponentEP<CMesh>::Set(S);
	// set bounding boxes for S1 and S2
	tri::UpdateBounding<CMesh>::Box(S);
	vcg::tri::Append<CMesh, CMesh>::MeshCopy((*S1), S);
}


double calcMetro(CMesh &S1, CMesh &S2)
{	
    float                 ColorMin=0, ColorMax=0;
    double                dist1_max, dist2_max;
    unsigned long         n_samples_target, elapsed_time;
    double				  n_samples_per_area_unit;
    int                   flags;
	bool NumberOfSamples = false;
	bool SamplesPerAreaUnit = false;
	bool CleaningFlag = false;

    // default parameters
    flags = SamplingFlags::VERTEX_SAMPLING |
          SamplingFlags::EDGE_SAMPLING |
          SamplingFlags::FACE_SAMPLING |
          SamplingFlags::SIMILAR_SAMPLING;

    // parse command line.
	if(!(flags & SamplingFlags::USE_HASH_GRID) && !(flags & SamplingFlags::USE_AABB_TREE) && !(flags & SamplingFlags::USE_OCTREE))
       flags |= SamplingFlags::USE_STATIC_GRID;

   // load input meshes.
    if(!NumberOfSamples && !SamplesPerAreaUnit)
    {
        NumberOfSamples = true;
        n_samples_target = max(S1.fn,S2.fn);// take 10 samples per face
    }
	//cout << "n_smaples_target: " << n_samples_target << "\n";
    // set Bounding Box.
	Box3<CMesh::ScalarType> bbox, tmp_bbox_M1=S1.bbox, tmp_bbox_M2=S2.bbox;
    bbox.Add(S1.bbox);
    bbox.Add(S2.bbox);
	bbox.Offset(bbox.Diag()*0.02);
	S1.bbox = bbox;
	S2.bbox = bbox;

    // initialize time info.
    int t0=clock();
    Sampling<CMesh> ForwardSampling(S1,S2);
    Sampling<CMesh> BackwardSampling(S2,S1);

    // Forward distance.
    ForwardSampling.SetFlags(flags);
    if(NumberOfSamples)
    {
        ForwardSampling.SetSamplesTarget(n_samples_target);
        n_samples_per_area_unit = ForwardSampling.GetNSamplesPerAreaUnit();
    }
    else
    {
        ForwardSampling.SetSamplesPerAreaUnit(n_samples_per_area_unit);
        n_samples_target = ForwardSampling.GetNSamplesTarget();
    }
    ForwardSampling.Hausdorff();
    dist1_max  = ForwardSampling.GetDistMax();

    // Backward distance.
    BackwardSampling.SetFlags(flags);
    if(NumberOfSamples)
    {
        BackwardSampling.SetSamplesTarget(n_samples_target);
        n_samples_per_area_unit = BackwardSampling.GetNSamplesPerAreaUnit();
    }
    else
    {
        BackwardSampling.SetSamplesPerAreaUnit(n_samples_per_area_unit);
        n_samples_target = BackwardSampling.GetNSamplesTarget();
    }

    BackwardSampling.Hausdorff();
    dist2_max  = BackwardSampling.GetDistMax();
 
    int n_total_sample=ForwardSampling.GetNSamples()+BackwardSampling.GetNSamples();
    double mesh_dist_max  = max(dist1_max , dist2_max);
	//cout << "dist1_max: " << dist1_max << "dist2_max: " << dist2_max << "\n";
    
    return mesh_dist_max;
}