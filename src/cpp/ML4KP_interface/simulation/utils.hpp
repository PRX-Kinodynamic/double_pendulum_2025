#pragma once

#include <prx/simulation/plant.hpp>
#include <prx/simulation/controllers/lqr_controller.hpp>
// #include "model/src/dp_plant.hpp"
// #include "simulator/src/simulator.hpp"

namespace double_pendulum
{
using LQR = prx::simulation::lqr_controller_t<5, -1>;
using LQRptr = std::shared_ptr<LQR>;

template <Eigen::Index Dim = 4, Eigen::Index Udim = 1>
struct lqr_query_t
{
  using QMat = Eigen::Matrix<double, Dim, Dim>;
  using RMat = Eigen::Matrix<double, Udim, Udim>;
  using State = Eigen::Vector<double, Dim>;
  using Control = Eigen::Vector<double, Udim>;
  lqr_query_t()
    : Q(QMat::Identity())
    , R(Eigen::Matrix<double, Udim, Udim>::Identity())
    , x_goal(State::Zero())
    , u_goal(Control::Zero())
    , normalize(false)
  {
    x_goal[0] = prx::constants::pi;
  }

  QMat Q;
  RMat R;

  State x_goal;
  Control u_goal;
  bool normalize;

  LQR::Diff diff = [](const LQR::VectorX& x, const LQR::VectorX& ref)  // no-lint
  {
    const double th0{ x[0] };
    const double th1{ x[1] };

    const double th0_ref{ ref[0] };
    const double th1_ref{ ref[1] };

    const double dth0{ std::atan2(std::sin(th0 - th0_ref), std::cos(th0 - th0_ref)) };
    const double dth1{ std::atan2(std::sin(th1 - th1_ref), std::cos(th1 - th1_ref)) };
    const double dv0{ x[2] - ref[2] };
    const double dv1{ x[3] - ref[3] };

    State result;
    result[0] = dth0;
    result[1] = dth1;
    result[2] = dv0;
    result[3] = dv1;
    // PRX_DBG_VARS(result.transpose());
    return result;
  };
};

template <typename LqrQuery>
LQRptr create_lqr(std::shared_ptr<prx::plant_t> plant, const LqrQuery lqr_query = lqr_query_t())
{
  const typename LqrQuery::QMat& Q{ lqr_query.Q };
  const typename LqrQuery::RMat& R{ lqr_query.R };
  const typename LqrQuery::State& x_goal{ lqr_query.x_goal };
  const typename LqrQuery::Control& u_goal{ lqr_query.u_goal };
  const typename LQR::Diff diff{ lqr_query.diff };
  const bool normalize{ lqr_query.normalize };

  LQRptr lqr{ std::make_shared<LQR>(plant, "LQR", Q, R, x_goal, u_goal, diff, normalize) };
  prx_assert(lqr != nullptr, "lqr is nullptr!");
  return lqr;
}
}  // namespace double_pendulum