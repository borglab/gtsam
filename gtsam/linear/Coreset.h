#include <gtsam/linear/LossFunctions.h>

#include <Eigen/unsupported/Eigen/CX11/Tensor>

namespace gtsam {

Vector FastCaratheodory(Matrix& P, Vector& weights, size_t coreset_size) {
  size_t n = P.rows(), d = P.cols();
  size_t m = 2 * d + 2;

  if (n < d + 1) {
    return weights;
  }

  Vector weights = weights / weights.sum();

  size_t chunk_size = ceil(n / m);
  size_t current_m = ceil(n / chunk_size);

  size_t add_z = chunk_size - size_t(n % chunk_size);
  Matrix u(weights.size(), 1);
  u.col(0) = weights;

  if (add_z != chunk_size) {
    Matrix zeros = Matrix::Zero(add_z, d);
    Matrix P_new = Matrix(P.rows() + zeros.rows(), P.cols() + zeros.cols());
    P_new << P, zeros;
    zeros = Matrix::Zero(add_z, u.cols());
    Matrix u_new(u.rows() + zeros.rows(), u.cols() + zeros.cols());
    u_new << u, zeros;
  }

  Vector idxarray = Vector::LinSpaced(n, 0, n - 1);
  Eigen::Tensor p_groups;

  // p_groups = P.reshape(current_m, chunk_size, P.shape[1])
  // u_groups = u.reshape(current_m, chunk_size)
  // idx_group = idxarray.reshape(current_m, chunk_size)
  // u_nonzero = np.count_nonzero(u)

  //   if not coreset_size:
  //       coreset_size = d+1
  //   while u_nonzero > coreset_size:

  //       groups_means = np.einsum('ijk,ij->ik',p_groups, u_groups)
  //       group_weigts = np.ones(groups_means.shape[0], dtype =
  //       dtype)*1/current_m

  //       Cara_u_idx = Caratheodory(groups_means , group_weigts,dtype = dtype )

  //       IDX = np.nonzero(Cara_u_idx)

  //       new_P = p_groups[IDX].reshape(-1,d)

  //       subset_u = (current_m * u_groups[IDX] * Cara_u_idx[IDX][:,
  //       np.newaxis]).reshape(-1, 1) new_idx_array =
  //       idx_group[IDX].reshape(-1,1)
  //       ##############################################################################3
  //       u_nonzero = np.count_nonzero(subset_u)
  //       chunk_size = math.ceil(new_P.shape[0]/ m)
  //       current_m = math.ceil(new_P.shape[0]/ chunk_size)

  //       add_z = chunk_size - int(new_P.shape[0] % chunk_size)
  //       if add_z != chunk_size:
  //           new_P = np.concatenate((new_P, np.zeros((add_z, new_P.shape[1]),
  //           dtype = dtype))) subset_u = np.concatenate((subset_u,
  //           np.zeros((add_z, subset_u.shape[1]),dtype = dtype)))
  //           new_idx_array = np.concatenate((new_idx_array, np.zeros((add_z,
  //           new_idx_array.shape[1]),dtype = dtype)))
  //       p_groups = new_P.reshape(current_m, chunk_size, new_P.shape[1])
  //       u_groups = subset_u.reshape(current_m, chunk_size)
  //       idx_group = new_idx_array.reshape(current_m , chunk_size)
  //       ###########################################################

  //   new_u = np.zeros(n)
  //   subset_u = subset_u[(new_idx_array < n)]
  //   new_idx_array = new_idx_array[(new_idx_array <
  //   n)].reshape(-1).astype(int) new_u[new_idx_array] = subset_u return u_sum
  //   * new_u
}
}  // namespace gtsam