/*
 * @file		RefCounted.h
 * @brief		Simple reference-counted base class
 * @author	Frank Dellaert
 * @date	Mar 29, 2011
 */

#include <boost/intrusive_ptr.hpp>

// Forward Declarations
namespace gtsam {
	struct RefCounted;
}

namespace boost {
	void intrusive_ptr_add_ref(const gtsam::RefCounted * p);
	void intrusive_ptr_release(const gtsam::RefCounted * p);
}

namespace gtsam {

	/**
	 *  Simple reference counted class inspired by
	 *  http://www.codeproject.com/KB/stl/boostsmartptr.aspx
	 */
	struct RefCounted {
	private:
		mutable long references_;
		friend void ::boost::intrusive_ptr_add_ref(const RefCounted * p);
		friend void ::boost::intrusive_ptr_release(const RefCounted * p);
	public:
		RefCounted() :
			references_(0) {
		}
		virtual ~RefCounted() {
		}
	};

} // namespace gtsam

// Intrusive Pointer free functions
#ifndef DEBUG_REFCOUNT

namespace boost {

	// increment reference count of object *p
	inline void intrusive_ptr_add_ref(const gtsam::RefCounted * p) {
		++(p->references_);
	}

	// decrement reference count, and delete object when reference count reaches 0
	inline void intrusive_ptr_release(const gtsam::RefCounted * p) {
		if (--(p->references_) == 0)
		delete p;
	}

} // namespace boost

#else

#include <iostream>

	namespace gtsam {
		static long GlobalRefCount = 0;
	}

	namespace boost {
		inline void intrusive_ptr_add_ref(const gtsam::RefCounted * p) {
			++(p->references_);
			gtsam::GlobalRefCount++;
			std::cout << "add_ref " << p << " " << p->references_ << //
					" " << gtsam::GlobalRefCount << std::endl;
		}

		inline void intrusive_ptr_release(const gtsam::RefCounted * p) {
			gtsam::GlobalRefCount--;
			std::cout << "release " << p << " " << (p->references_ - 1) << //
					" " << gtsam::GlobalRefCount << std::endl;
			if (--(p->references_) == 0)
				delete p;
		}

	} // namespace boost

#endif

