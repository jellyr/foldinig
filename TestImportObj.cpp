
#include "ImportOBJ.h"
#include "CGAL/Simple_cartesian.h"
#include "CGAL/Polyhedron_items_3.h"
#include "CGAL/HalfedgeDS_list.h"
#include "CGAL/Polyhedron_3.h"

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Polyhedron_3<Kernel,
	CGAL::Polyhedron_items_3,
	CGAL::HalfedgeDS_list> CgalPolyhedron;

void testImportOBJ()
{
	CgalPolyhedron _poly;
	SMeshLib::IO::importOBJ("Venus.obj", &_poly);
}