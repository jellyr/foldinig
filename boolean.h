#ifndef _INCLUDE_BOOLEAN_H_
#define _INCLUDE_BOOLEAN_H_

#ifndef CGAL_DEFINEDATA_H
#include "cgal_defineData.h"
#endif


Polyhedron_G boolDiff_P1_P2(Polyhedron_G poly1, Polyhedron_G poly2) {//	poly1-poly2‚ðŒvŽZ
	Nef_polyhedron_3 N1(poly1);
	Nef_polyhedron_3 N2(poly2);

	Polyhedron_G P;

	N1 -= N2;//	
	
	if (N1.is_simple()) {
		N1.convert_to_polyhedron((P));
	}
	else {
		cout << "N1 is not a 2-manifold\n";
	}
	
	return P;
}

Polyhedron_G boolDiff_P1_P2(Polyhedron_G poly1, Nef_polyhedron_3 N2, bool flg) {//	poly1-poly2‚ðŒvŽZ
	if (!poly1.is_valid()) {
		cout << "error\n";
	}
	Nef_polyhedron_3 N1(poly1); //	 ‚±‚±‚ªƒGƒ‰[
	Nef_polyhedron_3 N3;
	Polyhedron_G P;
	if (flg){
		N3 = N1 - N2;
		if (N3.is_simple()) {
			N3.convert_to_polyhedron((P));
		} else {
			cout << "N1 is not a 2-manifold\n";
		}
	}
	else {
		N3 = N2 - N1;
		if (N3.is_simple()) {
			N3.convert_to_polyhedron((P));
		}
		else {
			cout << "N1 is not a 2-manifold\n";
		}
	}	

	return P;
}

Polyhedron_G boolDiff_P1_inter_P2(Polyhedron_G poly1, Nef_polyhedron_3 N2) {//	poly1-poly2‚ðŒvŽZ
	if (!poly1.is_valid()) {
		cout << "error\n";
	}
	Nef_polyhedron_3 N1(poly1);
	Nef_polyhedron_3 N3;
	Polyhedron_G P;

	N3 = N1 * N2;
	if (N3.is_simple()) {
		N3.convert_to_polyhedron((P));
	}
	else {
		cout << "N1 is not a 2-manifold\n";
	}

	return P;
}

#endif