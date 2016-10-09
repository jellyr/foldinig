#include <CGAL/Exact_integer.h>
#include <CGAL/Homogeneous.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/IO/Nef_polyhedron_iostream_3.h>
#include "cgal_defineData.h"
typedef CGAL::Homogeneous<CGAL::Exact_integer> Kernel;
typedef CGAL::Nef_polyhedron_3<Kernel> Nef_polyhedron_3;
typedef Kernel::Point_3 Point_3;

Polyhedron boolDiff_P1_P2(Polyhedron *poly1, Polyhedron *poly2) {//	poly1-poly2‚ðŒvŽZ
	Nef_polyhedron_3 N1((*poly1));
	Nef_polyhedron_3 N2((*poly2));

	Polyhedron P;

	N1 -= N2;//	
	if (N1.is_simple()) {
		N1.convert_to_polyhedron((P));
	}
	else {
		cout << "N1 is not a 2-manifold\n";
	}
	return P;
}