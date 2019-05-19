/**
 * \file NearestNeighbor.hpp
 * \brief Header for GeographicLib::NearestNeighbor class
 *
 * Copyright (c) Charles Karney (2016-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_NEARESTNEIGHBOR_HPP)
#define GEOGRAPHICLIB_NEARESTNEIGHBOR_HPP 1

#include <algorithm>            // for nth_element, max_element, etc.
#include <vector>
#include <queue>                // for priority_queue
#include <utility>              // for swap + pair
#include <cstring>
#include <limits>
#include <cmath>
#include <iostream>
#include <sstream>
// Only for GEOGRAPHICLIB_STATIC_ASSERT and GeographicLib::GeographicErr
#include <GeographicLib/Constants.hpp>

#if defined(GEOGRAPHICLIB_HAVE_BOOST_SERIALIZATION) && \
  GEOGRAPHICLIB_HAVE_BOOST_SERIALIZATION
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/vector.hpp>
#endif

#if defined(_MSC_VER)
// Squelch warnings about constant conditional expressions
#  pragma warning (push)
#  pragma warning (disable: 4127)
#endif

namespace GeographicLib {

  /**
   * \brief Nearest-neighbor calculations
   *
   * This class solves the nearest-neighbor problm using a vantage-point tree
   * as described in \ref nearest.
   *
   * This class is templated so that it can handle arbitrary metric spaces as
   * follows:
   *
   * @tparam dist_t the type used for measuring distances; it can be a real or
   *   signed integer type; in typical geodetic applications, \e dist_t might
   *   be <code>double</code>.
   * @tparam pos_t the type for specifying the positions of points; geodetic
   *   application might bundled the latitude and longitude into a
   *   <code>std::pair<dist_t, dist_t></code>.
   * @tparam distfun_t the type of a function object which takes takes two
   *   positions (of type \e pos_t) and returns the distance (of type \e
   *   dist_t); in geodetic applications, this might be a class which is
   *   constructed with a Geodesic object and which implements a member
   *   function with a signature <code>dist_t operator() (const pos_t&, const
   *   pos_t&) const</code>, which returns the geodesic distance between two
   *   points.
   *
   * \note The distance measure must satisfy the triangle inequality, \f$
   * d(a,c) \le d(a,b) + d(b,c) \f$ for all points \e a, \e b, \e c.  The
   * geodesic distance (given by Geodesic::Inverse) does, while the great
   * ellipse distance and the rhumb line distance <i>do not</i>.  If you use
   * the ordinary Euclidean distance, i.e., \f$ \sqrt{(x_a-x_b)^2 +
   * (y_a-y_b)^2} \f$ for two dimensions, don't be tempted to leave out the
   * square root in the interests of "efficiency"; the squared distance does
   * not satisfy the triangle inequality!
   *
   * This is a "header-only" implementation and, as such, depends in a minimal
   * way on the rest of GeographicLib (the only dependency is through the use
   * of GEOGRAPHICLIB_STATIC_ASSERT and GeographicLib::GeographicErr for
   * handling run-time and compile-time exceptions).  Therefore, it is easy to
   * extract this class from the rest of GeographicLib and use it as a
   * stand-alone facility.
   *
   * The \e dist_t type must support numeric_limits queries (specifically:
   * is_signed, is_integer, max(), digits).
   *
   * The NearestNeighbor object is constructed with a vector of points (type \e
   * pos_t) and a distance function (type \e distfun_t).  However the object
   * does \e not store the points.  When querying the object with Search(),
   * it's necessary to supply the same vector of points and the same distance
   * function.
   *
   * There's no capability in this implementation to add or remove points from
   * the set.  Instead Initialize() should be called to re-initialize the
   * object with the modified vector of points.
   *
   * Because of the overhead in constructing a NearestNeighbor object for a
   * large set of points, functions Save() and Load() are provided to save the
   * object to an external file.  operator<<(), operator>>() and <a
   * href="http://www.boost.org/libs/serialization/doc"> Boost
   * serialization</a> can also be used to save and restore a NearestNeighbor
   * object.  This is illustrated in the example.
   *
   * Example of use:
   * \include example-NearestNeighbor.cpp
   **********************************************************************/
  template <typename dist_t, typename pos_t, class distfun_t>
  class NearestNeighbor {
    // For tracking changes to the I/O format
    static const int version = 1;
    // This is what we get "free"; but if sizeof(dist_t) = 1 (unlikely), allow
    // 4 slots (and this accommodates the default value bucket = 4).
    static const int maxbucket =
      (2 + ((4 * sizeof(dist_t)) / sizeof(int) >= 2 ?
            (4 * sizeof(dist_t)) / sizeof(int) : 2));
  public:

    /**
     * Default constructor for NearestNeighbor.
     *
     * This is equivalent to specifying an empty set of points.
     **********************************************************************/
    NearestNeighbor() : _numpoints(0), _bucket(0), _cost(0) {}

    /**
     * Constructor for NearestNeighbor.
     *
     * @param[in] pts a vector of points to include in the set.
     * @param[in] dist the distance function object.
     * @param[in] bucket the size of the buckets at the leaf nodes; this must
     *   lie in [0, 2 + 4*sizeof(dist_t)/sizeof(int)] (default 4).
     * @exception GeographicErr if the value of \e bucket is out of bounds or
     *   the size of \e pts is too big for an int.
     * @exception std::bad_alloc if memory for the tree can't be allocated.
     *
     * \e pts may contain coincident points (i.e., the distance between them
     * vanishes); these are treated as distinct.
     *
     * The choice of \e bucket is a tradeoff between space and efficiency.  A
     * larger \e bucket decreases the size of the NearestNeighbor object which
     * scales as pts.size() / max(1, bucket) and reduces the number of distance
     * calculations to construct the object by log2(bucket) * pts.size().
     * However each search then requires about bucket additional distance
     * calculations.
     *
     * \warning The distances computed by \e dist must satisfy the standard
     * metric conditions.  If not, the results are undefined.  Neither the data
     * in \e pts nor the query points should contain NaNs or infinities because
     * such data violates the metric conditions.
     *
     * \warning The same arguments \e pts and \e dist must be provided
     * to the Search() function.
     **********************************************************************/
    NearestNeighbor(const std::vector<pos_t>& pts, const distfun_t& dist,
                    int bucket = 4) {
      Initialize(pts, dist, bucket);
    }

    /**
     * Initialize or re-initialize NearestNeighbor.
     *
     * @param[in] pts a vector of points to include in the tree.
     * @param[in] dist the distance function object.
     * @param[in] bucket the size of the buckets at the leaf nodes; this must
     *   lie in [0, 2 + 4*sizeof(dist_t)/sizeof(int)] (default 4).
     * @exception GeographicErr if the value of \e bucket is out of bounds or
     *   the size of \e pts is too big for an int.
     * @exception std::bad_alloc if memory for the tree can't be allocated.
     *
     * See also the documentation on the constructor.
     *
     * If an exception is thrown, the state of the NearestNeighbor is
     * unchanged.
     **********************************************************************/
    void Initialize(const std::vector<pos_t>& pts, const distfun_t& dist,
                    int bucket = 4) {
      GEOGRAPHICLIB_STATIC_ASSERT(std::numeric_limits<dist_t>::is_signed,
                                  "dist_t must be a signed type");
      if (!( 0 <= bucket && bucket <= maxbucket ))
        throw GeographicLib::GeographicErr
          ("bucket must lie in [0, 2 + 4*sizeof(dist_t)/sizeof(int)]");
      if (pts.size() > size_t(std::numeric_limits<int>::max()))
        throw GeographicLib::GeographicErr("pts array too big");
      // the pair contains distance+id
      std::vector<item> ids(pts.size());
      for (int k = int(ids.size()); k--;)
        ids[k] = std::make_pair(dist_t(0), k);
      int cost = 0;
      std::vector<Node> tree;
      init(pts, dist, bucket, tree, ids, cost,
           0, int(ids.size()), int(ids.size()/2));
      _tree.swap(tree);
      _numpoints = int(pts.size());
      _bucket = bucket;
      _mc = _sc = 0;
      _cost = cost; _c1 = _k = _cmax = 0;
      _cmin = std::numeric_limits<int>::max();
    }

    /**
     * Search the NearestNeighbor.
     *
     * @param[in] pts the vector of points used for initialization.
     * @param[in] dist the distance function object used for initialization.
     * @param[in] query the query point.
     * @param[out] ind a vector of indices to the closest points found.
     * @param[in] k the number of points to search for (default = 1).
     * @param[in] maxdist only return points with distances of \e maxdist or
     *   less from \e query (default is the maximum \e dist_t).
     * @param[in] mindist only return points with distances of more than
     *   \e mindist from \e query (default = &minus;1).
     * @param[in] exhaustive whether to do an exhaustive search (default true).
     * @param[in] tol the tolerance on the results (default 0).
     * @return the distance to the closest point found (&minus;1 if no points
     *   are found).
     * @exception GeographicErr if \e pts has a different size from that used
     *   to construct the object.
     *
     * The indices returned in \e ind are sorted by distance from \e query
     * (closest first).
     *
     * The simplest invocation is with just the 4 non-optional arguments.  This
     * returns the closest distance and the index to the closest point in
     * <i>ind</i><sub>0</sub>.  If there are several points equally close, then
     * <i>ind</i><sub>0</sub> gives the index of an arbirary one of them.  If
     * there's no closest point (because the set of points is empty), then \e
     * ind is empty and &minus;1 is returned.
     *
     * With \e exhaustive = true and \e tol = 0 (their default values), this
     * finds the indices of \e k closest neighbors to \e query whose distances
     * to \e query are in (\e mindist, \e maxdist].  If \e mindist and \e
     * maxdist have their default values, then these bounds have no effect.  If
     * \e query is one of the points in the tree, then set \e mindist = 0 to
     * prevent this point (and other coincident points) from being returned.
     *
     * If \e exhaustive = false, exit as soon as \e k results satisfying the
     * distance criteria are found.  If less than \e k results are returned
     * then the search was exhaustive even if \e exhaustive = false.
     *
     * If \e tol is positive, do an approximate search; in this case the
     * results are to be interpreted as follows: if the <i>k</i>'th distance is
     * \e dk, then all results with distances less than or equal \e dk &minus;
     * \e tol are correct; all others are suspect &mdash; there may be other
     * closer results with distances greater or equal to \e dk &minus; \e tol.
     * If less than \e k results are found, then the search is exact.
     *
     * \e mindist should be used to exclude a "small" neighborhood of the query
     * point (relative to the average spacing of the data).  If \e mindist is
     * large, the efficiency of the search deteriorates.
     *
     * \note Only the shortest distance is returned (as as the function value).
     * The distances to other points (indexed by <i>ind</i><sub><i>j</i></sub>
     * for \e j > 0) can be found by invoking \e dist again.
     *
     * \warning The arguments \e pts and \e dist must be identical to those
     * used to initialize the NearestNeighbor; if not, this function will
     * return some meaningless result (however, if the size of \e pts is wrong,
     * this function throw an exception).
     *
     * \warning The query point cannot be a NaN or infinite because then the
     * metric conditions are violated.
     **********************************************************************/
    dist_t Search(const std::vector<pos_t>& pts, const distfun_t& dist,
                  const pos_t& query,
                  std::vector<int>& ind,
                  int k = 1,
                  dist_t maxdist = std::numeric_limits<dist_t>::max(),
                  dist_t mindist = -1,
                  bool exhaustive = true,
                  dist_t tol = 0) const {
      if (_numpoints != int(pts.size()))
          throw GeographicLib::GeographicErr("pts array has wrong size");
      std::priority_queue<item> results;
      if (_numpoints > 0 && k > 0 && maxdist > mindist) {
        // distance to the kth closest point so far
        dist_t tau = maxdist;
        // first is negative of how far query is outside boundary of node
        // +1 if on boundary or inside
        // second is node index
        std::priority_queue<item> todo;
        todo.push(std::make_pair(dist_t(1), int(_tree.size()) - 1));
        int c = 0;
        while (!todo.empty()) {
          int n = todo.top().second;
          dist_t d = -todo.top().first;
          todo.pop();
          dist_t tau1 = tau - tol;
          // compare tau and d again since tau may have become smaller.
          if (!( n >= 0 && tau1 >= d )) continue;
          const Node& current = _tree[n];
          dist_t dst = 0;   // to suppress warning about uninitialized variable
          bool exitflag = false, leaf = current.index < 0;
          for (int i = 0; i < (leaf ? _bucket : 1); ++i) {
            int index = leaf ? current.leaves[i] : current.index;
            if (index < 0) break;
            dst = dist(pts[index], query);
            ++c;

            if (dst > mindist && dst <= tau) {
              if (int(results.size()) == k) results.pop();
              results.push(std::make_pair(dst, index));
              if (int(results.size()) == k) {
                if (exhaustive)
                  tau = results.top().first;
                else {
                  exitflag = true;
                  break;
                }
                if (tau <= tol) {
                  exitflag = true;
                  break;
                }
              }
            }
          }
          if (exitflag) break;

          if (current.index < 0) continue;
          tau1 = tau - tol;
          for (int l = 0; l < 2; ++l) {
            if (current.data.child[l] >= 0 &&
                dst + current.data.upper[l] >= mindist) {
              if (dst < current.data.lower[l]) {
                d = current.data.lower[l] - dst;
                if (tau1 >= d)
                  todo.push(std::make_pair(-d, current.data.child[l]));
              } else if (dst > current.data.upper[l]) {
                d = dst - current.data.upper[l];
                if (tau1 >= d)
                  todo.push(std::make_pair(-d, current.data.child[l]));
              } else
                todo.push(std::make_pair(dist_t(1), current.data.child[l]));
            }
          }
        }
        ++_k;
        _c1 += c;
        double omc = _mc;
        _mc += (c - omc) / _k;
        _sc += (c - omc) * (c - _mc);
        if (c > _cmax) _cmax = c;
        if (c < _cmin) _cmin = c;
      }

      dist_t d = -1;
      ind.resize(results.size());

      for (int i = int(ind.size()); i--;) {
        ind[i] = int(results.top().second);
        if (i == 0) d = results.top().first;
        results.pop();
      }
      return d;

    }

    /**
     * @return the total number of points in the set.
     **********************************************************************/
    int NumPoints() const { return _numpoints; }

    /**
     * Write the object to an I/O stream.
     *
     * @param[in,out] os the stream to write to.
     * @param[in] bin if true (the default) save in binary mode.
     * @exception std::bad_alloc if memory for the string representation of the
     *   object can't be allocated.
     *
     * The counters tracking the statistics of searches are not saved; however
     * the initializtion cost is saved.  The format of the binary saves is \e
     * not portable.
     *
     * \note <a href="http://www.boost.org/libs/serialization/doc">
     * Boost serialization</a> can also be used to save and restore a
     * NearestNeighbor object.  This requires that the
     * GEOGRAPHICLIB_HAVE_BOOST_SERIALIZATION macro be defined.
     **********************************************************************/
    void Save(std::ostream& os, bool bin = true) const {
      int realspec = std::numeric_limits<dist_t>::digits *
        (std::numeric_limits<dist_t>::is_integer ? -1 : 1);
      if (bin) {
        char id[] = "NearestNeighbor_";
        os.write(id, 16);
        int buf[6];
        buf[0] = version;
        buf[1] = realspec;
        buf[2] = _bucket;
        buf[3] = _numpoints;
        buf[4] = int(_tree.size());
        buf[5] = _cost;
        os.write(reinterpret_cast<const char *>(buf), 6 * sizeof(int));
        for (int i = 0; i < int(_tree.size()); ++i) {
          const Node& node = _tree[i];
          os.write(reinterpret_cast<const char *>(&node.index), sizeof(int));
          if (node.index >= 0) {
            os.write(reinterpret_cast<const char *>(node.data.lower),
                     2 * sizeof(dist_t));
            os.write(reinterpret_cast<const char *>(node.data.upper),
                     2 * sizeof(dist_t));
            os.write(reinterpret_cast<const char *>(node.data.child),
                     2 * sizeof(int));
          } else {
            os.write(reinterpret_cast<const char *>(node.leaves),
                     _bucket * sizeof(int));
          }
        }
      } else {
        std::stringstream ostring;
          // Ensure enough precision for type dist_t.  With C++11, max_digits10
          // can be used instead.
        if (!std::numeric_limits<dist_t>::is_integer) {
          static const int prec
            = int(std::ceil(std::numeric_limits<dist_t>::digits *
                            std::log10(2.0) + 1));
          ostring.precision(prec);
        }
        ostring << version << " " << realspec << " " << _bucket << " "
                << _numpoints << " " << _tree.size() << " " << _cost;
        for (int i = 0; i < int(_tree.size()); ++i) {
          const Node& node = _tree[i];
          ostring << "\n" << node.index;
          if (node.index >= 0) {
            for (int l = 0; l < 2; ++l)
              ostring << " " << node.data.lower[l] << " " << node.data.upper[l]
                      << " " << node.data.child[l];
          } else {
            for (int l = 0; l < _bucket; ++l)
              ostring << " " << node.leaves[l];
          }
        }
        os << ostring.str();
      }
    }

    /**
     * Read the object from an I/O stream.
     *
     * @param[in,out] is the stream to read from
     * @param[in] bin if true (the default) load in binary mode.
     * @exception GeographicErr if the state read from \e is is illegal.
     * @exception std::bad_alloc if memory for the tree can't be allocated.
     *
     * The counters tracking the statistics of searches are reset by this
     * operation.  Binary data must have been saved on a machine with the same
     * architecture.  If an exception is thrown, the state of the
     * NearestNeighbor is unchanged.
     *
     * \note <a href="http://www.boost.org/libs/serialization/doc">
     * Boost serialization</a> can also be used to save and restore a
     * NearestNeighbor object.  This requires that the
     * GEOGRAPHICLIB_HAVE_BOOST_SERIALIZATION macro be defined.
     *
     * \warning The same arguments \e pts and \e dist used for
     * initialization must be provided to the Search() function.
     **********************************************************************/
    void Load(std::istream& is, bool bin = true) {
      int version1, realspec, bucket, numpoints, treesize, cost;
      if (bin) {
        char id[17];
        is.read(id, 16);
        id[16] = '\0';
        if (!(std::strcmp(id, "NearestNeighbor_") == 0))
          throw GeographicLib::GeographicErr("Bad ID");
        is.read(reinterpret_cast<char *>(&version1), sizeof(int));
        is.read(reinterpret_cast<char *>(&realspec), sizeof(int));
        is.read(reinterpret_cast<char *>(&bucket), sizeof(int));
        is.read(reinterpret_cast<char *>(&numpoints), sizeof(int));
        is.read(reinterpret_cast<char *>(&treesize), sizeof(int));
        is.read(reinterpret_cast<char *>(&cost), sizeof(int));
      } else {
        if (!( is >> version1 >> realspec >> bucket >> numpoints >> treesize
               >> cost ))
          throw GeographicLib::GeographicErr("Bad header");
      }
      if (!( version1 == version ))
        throw GeographicLib::GeographicErr("Incompatible version");
      if (!( realspec == std::numeric_limits<dist_t>::digits *
             (std::numeric_limits<dist_t>::is_integer ? -1 : 1) ))
        throw GeographicLib::GeographicErr("Different dist_t types");
      if (!( 0 <= bucket && bucket <= maxbucket ))
        throw GeographicLib::GeographicErr("Bad bucket size");
      if (!( 0 <= treesize && treesize <= numpoints ))
        throw
          GeographicLib::GeographicErr("Bad number of points or tree size");
      if (!( 0 <= cost ))
        throw GeographicLib::GeographicErr("Bad value for cost");
      std::vector<Node> tree;
      tree.reserve(treesize);
      for (int i = 0; i < treesize; ++i) {
        Node node;
        if (bin) {
          is.read(reinterpret_cast<char *>(&node.index), sizeof(int));
          if (node.index >= 0) {
            is.read(reinterpret_cast<char *>(node.data.lower),
                    2 * sizeof(dist_t));
            is.read(reinterpret_cast<char *>(node.data.upper),
                    2 * sizeof(dist_t));
            is.read(reinterpret_cast<char *>(node.data.child),
                    2 * sizeof(int));
          } else {
            is.read(reinterpret_cast<char *>(node.leaves),
                    bucket * sizeof(int));
            for (int l = bucket; l < maxbucket; ++l)
              node.leaves[l] = 0;
          }
        } else {
          if (!( is >> node.index ))
            throw GeographicLib::GeographicErr("Bad index");
          if (node.index >= 0) {
            for (int l = 0; l < 2; ++l) {
              if (!( is >> node.data.lower[l] >> node.data.upper[l]
                     >> node.data.child[l] ))
                throw GeographicLib::GeographicErr("Bad node data");
            }
          } else {
            // Must be at least one valid leaf followed by a sequence end
            // markers (-1).
            for (int l = 0; l < bucket; ++l) {
              if (!( is >> node.leaves[l] ))
                throw GeographicLib::GeographicErr("Bad leaf data");
            }
            for (int l = bucket; l < maxbucket; ++l)
              node.leaves[l] = 0;
          }
        }
        node.Check(numpoints, treesize, bucket);
        tree.push_back(node);
      }
      _tree.swap(tree);
      _numpoints = numpoints;
      _bucket = bucket;
      _mc = _sc = 0;
      _cost = cost; _c1 = _k = _cmax = 0;
      _cmin = std::numeric_limits<int>::max();
    }

    /**
     * Write the object to stream \e os as text.
     *
     * @param[in,out] os the output stream.
     * @param[in] t the NearestNeighbor object to be saved.
     * @exception std::bad_alloc if memory for the string representation of the
     *   object can't be allocated.
     **********************************************************************/
    friend std::ostream& operator<<(std::ostream& os, const NearestNeighbor& t)
    { t.Save(os, false); return os; }

    /**
     * Read the object from stream \e is as text.
     *
     * @param[in,out] is the input stream.
     * @param[out] t the NearestNeighbor object to be loaded.
     * @exception GeographicErr if the state read from \e is is illegal.
     * @exception std::bad_alloc if memory for the tree can't be allocated.
     **********************************************************************/
    friend std::istream& operator>>(std::istream& is, NearestNeighbor& t)
    { t.Load(is, false); return is; }

    /**
     * Swap with another NearestNeighbor object.
     *
     * @param[in,out] t the NearestNeighbor object to swap with.
     **********************************************************************/
    void swap(NearestNeighbor& t) {
      std::swap(_numpoints, t._numpoints);
      std::swap(_bucket, t._bucket);
      std::swap(_cost, t._cost);
      _tree.swap(t._tree);
      std::swap(_mc, t._mc);
      std::swap(_sc, t._sc);
      std::swap(_c1, t._c1);
      std::swap(_k, t._k);
      std::swap(_cmin, t._cmin);
      std::swap(_cmax, t._cmax);
    }

    /**
     * The accumulated statistics on the searches so far.
     *
     * @param[out] setupcost the cost of initializing the NearestNeighbor.
     * @param[out] numsearches the number of calls to Search().
     * @param[out] searchcost the total cost of the calls to Search().
     * @param[out] mincost the minimum cost of a Search().
     * @param[out] maxcost the maximum cost of a Search().
     * @param[out] mean the mean cost of a Search().
     * @param[out] sd the standard deviation in the cost of a Search().
     *
     * Here "cost" measures the number of distance calculations needed.  Note
     * that the accumulation of statistics is \e not thread safe.
     **********************************************************************/
    void Statistics(int& setupcost, int& numsearches, int& searchcost,
                    int& mincost, int& maxcost,
                    double& mean, double& sd) const {
      setupcost = _cost; numsearches = _k; searchcost = _c1;
      mincost = _cmin; maxcost = _cmax;
      mean = _mc; sd = std::sqrt(_sc / (_k - 1));
    }

    /**
     * Reset the counters for the accumulated statistics on the searches so
     * far.
     **********************************************************************/
    void ResetStatistics() const {
      _mc = _sc = 0;
      _c1 = _k = _cmax = 0;
      _cmin = std::numeric_limits<int>::max();
    }

  private:
    // Package up a dist_t and an int.  We will want to sort on the dist_t so
    // put it first.
    typedef std::pair<dist_t, int> item;
    // \cond SKIP
    class Node {
    public:
      struct bounds {
        dist_t lower[2], upper[2]; // bounds on inner/outer distances
        int child[2];
      };
      union {
        bounds data;
        int leaves[maxbucket];
      };
      int index;

      Node()
        : index(-1)
      {
        for (int i = 0; i < 2; ++i) {
          data.lower[i] = data.upper[i] = 0;
          data.child[i] = -1;
        }
      }

      // Sanity check on a Node
      void Check(int numpoints, int treesize, int bucket) const {
        if (!( -1 <= index && index < numpoints ))
          throw GeographicLib::GeographicErr("Bad index");
        if (index >= 0) {
          if (!( -1 <= data.child[0] && data.child[0] < treesize &&
                 -1 <= data.child[1] && data.child[1] < treesize ))
            throw GeographicLib::GeographicErr("Bad child pointers");
          if (!( 0 <= data.lower[0] && data.lower[0] <= data.upper[0] &&
                 data.upper[0] <= data.lower[1] &&
                 data.lower[1] <= data.upper[1] ))
            throw GeographicLib::GeographicErr("Bad bounds");
        } else {
          // Must be at least one valid leaf followed by a sequence end markers
          // (-1).
          bool start = true;
          for (int l = 0; l < bucket; ++l) {
            if (!( (start ?
                    ((l == 0 ? 0 : -1) <= leaves[l] && leaves[l] < numpoints) :
                    leaves[l] == -1) ))
              throw GeographicLib::GeographicErr("Bad leaf data");
            start = leaves[l] >= 0;
          }
          for (int l = bucket; l < maxbucket; ++l) {
            if (leaves[l] != 0)
              throw GeographicLib::GeographicErr("Bad leaf data");
          }
        }
      }

#if defined(GEOGRAPHICLIB_HAVE_BOOST_SERIALIZATION) && \
  GEOGRAPHICLIB_HAVE_BOOST_SERIALIZATION
      friend class boost::serialization::access;
      template<class Archive>
      void save(Archive& ar, const unsigned int) const {
        ar & boost::serialization::make_nvp("index", index);
        if (index < 0)
          ar & boost::serialization::make_nvp("leaves", leaves);
        else
          ar & boost::serialization::make_nvp("lower", data.lower)
            & boost::serialization::make_nvp("upper", data.upper)
            & boost::serialization::make_nvp("child", data.child);
      }
      template<class Archive>
      void load(Archive& ar, const unsigned int) {
        ar & boost::serialization::make_nvp("index", index);
        if (index < 0)
          ar & boost::serialization::make_nvp("leaves", leaves);
        else
          ar & boost::serialization::make_nvp("lower", data.lower)
            & boost::serialization::make_nvp("upper", data.upper)
            & boost::serialization::make_nvp("child", data.child);
      }
      template<class Archive>
      void serialize(Archive& ar, const unsigned int file_version)
      { boost::serialization::split_member(ar, *this, file_version); }
#endif
    };
    // \endcond
#if defined(GEOGRAPHICLIB_HAVE_BOOST_SERIALIZATION) && \
  GEOGRAPHICLIB_HAVE_BOOST_SERIALIZATION
    friend class boost::serialization::access;
    template<class Archive> void save(Archive& ar, const unsigned) const {
      int realspec = std::numeric_limits<dist_t>::digits *
        (std::numeric_limits<dist_t>::is_integer ? -1 : 1);
      // Need to use version1, otherwise load error in debug mode on Linux:
      // undefined reference to GeographicLib::NearestNeighbor<...>::version.
      int version1 = version;
      ar & boost::serialization::make_nvp("version", version1)
        & boost::serialization::make_nvp("realspec", realspec)
        & boost::serialization::make_nvp("bucket", _bucket)
        & boost::serialization::make_nvp("numpoints", _numpoints)
        & boost::serialization::make_nvp("cost", _cost)
        & boost::serialization::make_nvp("tree", _tree);
    }
    template<class Archive> void load(Archive& ar, const unsigned) {
      int version1, realspec, bucket, numpoints, cost;
      ar & boost::serialization::make_nvp("version", version1);
      if (version1 != version)
        throw GeographicLib::GeographicErr("Incompatible version");
      std::vector<Node> tree;
      ar & boost::serialization::make_nvp("realspec", realspec);
      if (!( realspec == std::numeric_limits<dist_t>::digits *
             (std::numeric_limits<dist_t>::is_integer ? -1 : 1) ))
        throw GeographicLib::GeographicErr("Different dist_t types");
      ar & boost::serialization::make_nvp("bucket", bucket);
      if (!( 0 <= bucket && bucket <= maxbucket ))
        throw GeographicLib::GeographicErr("Bad bucket size");
      ar & boost::serialization::make_nvp("numpoints", numpoints)
        & boost::serialization::make_nvp("cost", cost)
        & boost::serialization::make_nvp("tree", tree);
      if (!( 0 <= int(tree.size()) && int(tree.size()) <= numpoints ))
        throw
          GeographicLib::GeographicErr("Bad number of points or tree size");
      for (int i = 0; i < int(tree.size()); ++i)
        tree[i].Check(numpoints, int(tree.size()), bucket);
      _tree.swap(tree);
      _numpoints = numpoints;
      _bucket = bucket;
      _mc = _sc = 0;
      _cost = cost; _c1 = _k = _cmax = 0;
      _cmin = std::numeric_limits<int>::max();
    }
    template<class Archive>
    void serialize(Archive& ar, const unsigned int file_version)
    { boost::serialization::split_member(ar, *this, file_version); }
#endif

    int _numpoints, _bucket, _cost;
    std::vector<Node> _tree;
    // Counters to track stastistics on the cost of searches
    mutable double _mc, _sc;
    mutable int _c1, _k, _cmin, _cmax;

    int init(const std::vector<pos_t>& pts, const distfun_t& dist, int bucket,
             std::vector<Node>& tree, std::vector<item>& ids, int& cost,
             int l, int u, int vp) {

      if (u == l)
        return -1;
      Node node;

      if (u - l > (bucket == 0 ? 1 : bucket)) {

        // choose a vantage point and move it to the start
        int i = vp;
        std::swap(ids[l], ids[i]);

        int m = (u + l + 1) / 2;

        for (int k = l + 1; k < u; ++k) {
          ids[k].first = dist(pts[ids[l].second], pts[ids[k].second]);
          ++cost;
        }
        // partition around the median distance
        std::nth_element(ids.begin() + l + 1,
                         ids.begin() + m,
                         ids.begin() + u);
        node.index = ids[l].second;
        if (m > l + 1) {        // node.child[0] is possibly empty
          typename std::vector<item>::iterator
            t = std::min_element(ids.begin() + l + 1, ids.begin() + m);
          node.data.lower[0] = t->first;
          t = std::max_element(ids.begin() + l + 1, ids.begin() + m);
          node.data.upper[0] = t->first;
          // Use point with max distance as vantage point; this point act as a
          // "corner" point and leads to a good partition.
          node.data.child[0] = init(pts, dist, bucket, tree, ids, cost,
                                    l + 1, m, int(t - ids.begin()));
        }
        typename std::vector<item>::iterator
          t = std::max_element(ids.begin() + m, ids.begin() + u);
        node.data.lower[1] = ids[m].first;
        node.data.upper[1] = t->first;
        // Use point with max distance as vantage point here too
        node.data.child[1] = init(pts, dist, bucket, tree, ids, cost,
                                  m, u, int(t - ids.begin()));
      } else {
        if (bucket == 0)
          node.index = ids[l].second;
        else {
          node.index = -1;
          // Sort the bucket entries so that the tree is independent of the
          // implementation of nth_element.
          std::sort(ids.begin() + l, ids.begin() + u);
          for (int i = l; i < u; ++i)
            node.leaves[i-l] = ids[i].second;
          for (int i = u - l; i < bucket; ++i)
            node.leaves[i] = -1;
          for (int i = bucket; i < maxbucket; ++i)
            node.leaves[i] = 0;
        }
      }

      tree.push_back(node);
      return int(tree.size()) - 1;
    }

  };

} // namespace GeographicLib

namespace std {

  /**
   * Swap two GeographicLib::NearestNeighbor objects.
   *
   * @tparam dist_t the type used for measuring distances.
   * @tparam pos_t the type for specifying the positions of points.
   * @tparam distfun_t the type for a function object which calculates
   *   distances between points.
   * @param[in,out] a the first GeographicLib::NearestNeighbor to swap.
   * @param[in,out] b the second GeographicLib::NearestNeighbor to swap.
   **********************************************************************/
  template <typename dist_t, typename pos_t, class distfun_t>
  void swap(GeographicLib::NearestNeighbor<dist_t, pos_t, distfun_t>& a,
            GeographicLib::NearestNeighbor<dist_t, pos_t, distfun_t>& b) {
    a.swap(b);
  }

} // namespace std

#if defined(_MSC_VER)
#  pragma warning (pop)
#endif

#endif  // GEOGRAPHICLIB_NEARESTNEIGHBOR_HPP
