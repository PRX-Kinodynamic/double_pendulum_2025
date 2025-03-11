#pragma once

#include <prx/simulation/plant.hpp>
#include <prx/simulation/controllers/lqr_controller.hpp>
// #include "model/src/dp_plant.hpp"
// #include "simulator/src/simulator.hpp"

namespace double_pendulum
{
using LQR = prx::simulation::lqr_controller_t<5, -1>;
using LQRptr = std::shared_ptr<LQR>;

struct lqr_query_t
{
  lqr_query_t()
    : Q(Eigen::DiagonalMatrix<double, 4>(10.0, 10.0, 1.0, 1.0))
    , R(Eigen::Matrix<double, 1, 1>::Identity())
    , x_goal({ prx::constants::pi, 0.0, 0.0, 0.0 })
    , u_goal(0.0)
  {
  }

  Eigen::Matrix4d Q;
  Eigen::Matrix<double, 1, 1> R;
  Eigen::Vector4d x_goal;
  Eigen::Vector<double, 1> u_goal;
};

LQRptr create_lqr(std::shared_ptr<prx::plant_t> plant, const lqr_query_t lqr_query = lqr_query_t())
{
  const Eigen::Matrix4d& Q{ lqr_query.Q };
  const Eigen::Matrix<double, 1, 1>& R{ lqr_query.R };
  const Eigen::Vector4d& x_goal{ lqr_query.x_goal };
  const Eigen::Vector<double, 1>& u_goal{ lqr_query.u_goal };

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

    return Eigen::Vector4d(dth0, dth1, dv0, dv1);
  };

  LQRptr lqr{ std::make_shared<LQR>(plant, "LQR", Q, R, x_goal, u_goal, diff) };
  prx_assert(lqr != nullptr, "lqr is nullptr!");
  return lqr;
}
}  // namespace double_pendulum