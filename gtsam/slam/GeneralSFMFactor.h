/*
 * GeneralSFMFactor.h
 *
 *   Created on: Dec 15, 2010
 *       Author: nikai
 *  Description: a general SFM factor with an unknown calibration
 */

#pragma once


namespace gtsam {

	/**
	 * Non-linear factor for a constraint derived from a 2D measurement. The calibration is unknown here compared to GenericProjectionFactor
	 */
	template <class Cfg, class CamK, class LmK>
	class GeneralSFMFactor:
	public NonlinearFactor2<Cfg, CamK, LmK> ,
		   Testable<GeneralSFMFactor<Cfg, CamK, LmK> > {
	protected:
		Point2 z_;

	public:

		typedef typename CamK::Value Cam;
		typedef GeneralSFMFactor<Cfg, CamK, LmK> Self ;
		typedef NonlinearFactor2<Cfg, CamK, LmK> Base;

		// shorthand for a smart pointer to a factor
		typedef boost::shared_ptr<GeneralSFMFactor<Cfg, LmK, CamK> > shared_ptr;

		/**
		 * Constructor
		 * @param z is the 2 dimensional location of point in image (the measurement)
		 * @param sigma is the standard deviation
		 * @param cameraFrameNumber is basically the frame number
		 * @param landmarkNumber is the index of the landmark
		 * @param K the constant calibration
		 */

		GeneralSFMFactor():z_(0.0,0.0) {}
		GeneralSFMFactor(const Point2 & p):z_(p) {}
		GeneralSFMFactor(double x, double y):z_(x,y) {}
		GeneralSFMFactor(const Point2& z, const SharedGaussian& model, const CamK& i, const LmK& j) : Base(model, i, j), z_(z) {}

		/**
		 * print
		 * @param s optional string naming the factor
		 */
		void print(const std::string& s = "SFMFactor") const {
			Base::print(s);
			z_.print(s + ".z");
		}

		/**
		 * equals
		 */
		bool equals(const GeneralSFMFactor<Cfg, CamK, LmK> &p, double tol = 1e-9) const	{
			return Base::equals(p, tol) && this->z_.equals(p.z_, tol) ;
		}

		/** h(x)-z */
		Vector evaluateError(
				const Cam& camera,
				const Point3& point,
				boost::optional<Matrix&> H1=boost::none,
				boost::optional<Matrix&> H2=boost::none) const {

			Point2 q(camera.project(point,H1,H2) - z_);
			return q.vector() ;
		}

		/** return the measured */
		inline const Point2 measured() const {
			return z_;
		}

	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(z_);
		}
	};

} //namespace
