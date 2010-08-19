/*
 * projectiveGeometry.cpp
 * @brief Projective geometry, implemented using tensor library
 * Created on: Feb 12, 2010
 * @author: Frank Dellaert
 */

#include <boost/foreach.hpp>
#include <gtsam/geometry/tensorInterface.h>
#include <gtsam/geometry/projectiveGeometry.h>

namespace gtsam {

	using namespace std;
	using namespace tensors;

	static Eta3 eta;
	static Index<3, 'a'> a, _a;
	static Index<3, 'b'> b, _b;
	static Index<3, 'c'> c, _c;
	static Index<3, 'd'> d, _d;
	static Index<3, 'd'> e, _e;
	static Index<3, 'f'> f, _f;
	static Index<3, 'g'> g, _g;

	/* ************************************************************************* */
	Point2h point2h(double x, double y, double w) {
		double data[3];
		data[0] = x;
		data[1] = y;
		data[2] = w;
		return data;
	}

	/* ************************************************************************* */
	Line2h line2h(double a, double b, double c) {
		double data[3];
		data[0] = a;
		data[1] = b;
		data[2] = c;
		return data;
	}

	/* ************************************************************************* */
	Point3h point3h(double X, double Y, double Z, double W) {
		double data[4];
		data[0] = X;
		data[1] = Y;
		data[2] = Z;
		data[3] = W;
		return data;
	}

	/* ************************************************************************* */
	Plane3h plane3h(double a, double b, double c, double d) {
		double data[4];
		data[0] = a;
		data[1] = b;
		data[2] = c;
		data[3] = d;
		return data;
	}

	/* ************************************************************************* */
	Homography2 estimateHomography2(const list<Correspondence>& correspondences) {
		// Generate entries of A matrix for linear estimation
		Matrix A;
		BOOST_FOREACH(const Correspondence& p, correspondences) {
			Matrix Ap = reshape(p.first(a) * (eta(_b, _c, _d) * p.second(b)),3,9);
			A = stack(2,&A,&Ap);
		}

		// Call DLT and reshape to Homography
		int rank; double error; Vector v;
		boost::tie(rank, error, v) = DLT(A);
		if (rank < 8) throw invalid_argument("estimateHomography2: not enough data");
		return reshape2<3, 3> (v);
	}

	/* ************************************************************************* */
	FundamentalMatrix estimateFundamentalMatrix(const list<Correspondence>& correspondences) {
		// Generate entries of A matrix for linear estimation
		Matrix A;
		BOOST_FOREACH(const Correspondence& p, correspondences) {
			Matrix Ap = reshape(p.first(a) * p.second(b),1,9);
			A = stack(2,&A,&Ap);
		}

		// Call DLT and reshape to Homography
		int rank; double error; Vector v;
		boost::tie(rank, error, v) = DLT(A);
		if (rank < 8) throw invalid_argument("estimateFundamentalMatrix: not enough data");
		return reshape2<3, 3> (v);
	}

	/* ************************************************************************* */
	TrifocalTensor estimateTrifocalTensor(const list<Triplet>& triplets) {
		// Generate entries of A matrix for linear estimation
		Matrix A;
		BOOST_FOREACH(const Triplet& p, triplets) {
			Tensor3<3,3,3> T3 = p.first(a)* (eta(_d,_b,_e) * p.second(d));
			Tensor2<3,3> T2 = eta(_f,_c,_g) * p.third(f);
			// Take outer product of rank 3 and rank 2, then swap indices 3,4 for reshape
			// We get a rank 5 tensor T5(a,_b,_c,_e,_g), where _e and _g index the rows...
			Matrix Ap = reshape((T3(a,_b,_e) * T2(_c,_g)).swap34(), 9, 27);
			A = stack(2,&A,&Ap);
		}

		// Call DLT and reshape to Homography
		int rank; double error; Vector v;
		boost::tie(rank, error, v) = DLT(A);
		if (rank < 26) throw invalid_argument("estimateTrifocalTensor: not enough data");
		return reshape3<3,3,3>(v);
	}

	/* ************************************************************************* */

} // namespace gtsam
