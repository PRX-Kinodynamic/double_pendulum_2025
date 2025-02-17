#include <fstream>
#include <sstream>
#include <prx/utilities/defs.hpp>
#include <prx/utilities/general/param_loader.hpp>

#include <prx/simulation/plants/plants.hpp>
#include <prx/simulation/loaders/obstacle_loader.hpp>
#include <prx/simulation/controllers/lqr_controller.hpp>

#include <prx/planning/world_model.hpp>
#include <prx/planning/planners/aorrt.hpp>
#include <prx/planning/planners/planner.hpp>

#include <prx/visualization/three_js_group.hpp>
#include "ML4KP_interface/simulation/acrobot.hpp"

using LQR = prx::simulation::lqr_controller_t<>;
using LQRptr = std::shared_ptr<LQR>;
using PlantPtr = std::shared_ptr<prx::plant_t>;
using SystemGroupPtr = std::shared_ptr<prx::system_group_t>;

bool inside_convex_poly(const std::vector<Eigen::Vector2d>& poly, const Eigen::Vector2d& pt)
{
  int prev_side{ 0 };
  for (int i = 0; i < poly.size(); ++i)
  {
    const Eigen::Vector2d& P0{ poly[i] };
    const Eigen::Vector2d& P1{ poly[(i + 1) % poly.size()] };

    const double v0{ (pt[1] - P0[1]) * (P1[0] - P0[0]) };
    const double v1{ (pt[0] - P0[0]) * (P1[1] - P0[1]) };
    const double diff{ v0 - v1 };
    const int current_side{ diff < 0.0 ? 1 : -1 };

    if (prev_side != 0 and current_side != prev_side and diff != 0)
    {
      return false;
    }
    if (diff != 0)
    {
      prev_side = current_side;
    }
  }
  return true;
}

double dist(prx::space_point_t xF)
{
  static const Eigen::Vector4d ref(prx::constants::pi, 0.0, 0.0, 0.0);
  Eigen::Vector4d diff{ Vec(xF) - ref };
  diff[0] = std::atan2(std::sin(diff[0]), std::cos(diff[0]));
  diff[1] = std::atan2(std::sin(diff[1]), std::cos(diff[1]));
  // const double dv0{ x[2] - ref[2] };
  // const double dv1{ x[3] - ref[3] };
  return diff.norm();
}

bool propagate_and_check(prx::space_point_t x0, prx::space_point_t xF, PlantPtr plant, LQRptr lqr, SystemGroupPtr sg,
                         prx::condition_check_t& cond_check)
{
  double h;
  prx::space_t* ss{ sg->get_state_space() };
  ss->copy_from(x0);
  do
  {
    lqr->compute_controls();
    sg->propagate_once(nullptr);

    // traj.copy_onto_back(ss);

  } while (!cond_check.check());
  ss->copy_to(xF);
  plant->update_configuration();
  h = plant->configuration("ball").translation()[1];

  // static const Eigen::Vector4d ref(prx::constants::pi, 0.0, 0.0, 0.0);
  // Eigen::Vector4d diff{ Vec(xF) - ref };
  // // above
  // // return h > 0.45;
  // diff[0] = std::atan2(std::sin(diff[0]), std::cos(diff[0]));
  // diff[1] = std::atan2(std::sin(diff[1]), std::cos(diff[1]));
  // const double dv0{ x[2] - ref[2] };
  // const double dv1{ x[3] - ref[3] };
  return dist(xF) < 0.1;
}

std::string print_pt(const prx::space_point_t pt)
{
  std::stringstream strstr;
  strstr << std::fixed << std::setprecision(5);
  // const double th1{ std::atan2(std::sin(pt->at(0)), std::cos(pt->at(0))) };
  // const double th1{ pt->at(0) };
  // const double th2{ std::atan2(std::sin(pt->at(1)), std::cos(pt->at(1))) };
  // strstr << th1 << " ";
  // strstr << th2 << " ";
  // strstr << pt->at(2) << " ";
  // strstr << pt->at(3) << " ";
  strstr << pt << " ";
  return strstr.str();
}

int main(int argc, char* argv[])
{
  prx::param_loader params{ argc, argv };

  prx::simulation_step = 0.002;
  params["/planner/random_seed"].set(112392);

  PlantPtr plant;
  std::string plant_name;

  LQRptr lqr;

  const double pi2{ 2.0 * prx::constants::pi };

  const std::vector<Eigen::Vector2d> p_th1_th2_l{ { 2.45063, 0.605935 },  { 2.45063, 0.845988 },  { 2.50877, 0.891998 },
                                                  { 2.63629, 0.898000 },  { 2.68129, 0.841987 },  { 2.97947, 0.691954 },
                                                  { 3.54393, 0.0158035 }, { 2.74130, 0.00180040 } };
  const std::vector<Eigen::Vector2d> p_th1_th2_r{
    { 3.15, 6.24999 }, { 3.25, 6.24999 }, { 3.79894, 5.83390 }, { 3.80113, 5.40980 }, { 3.55827, 5.40780 }
  };
  // {2.90373,  2.01978}, to
  // {3.80018, -1.07646}, to
  // {3.80018, -3.31696}, to
  // {3.24795, -0.516337}, to
  // {2.93502,  1.38186},
  const std::vector<Eigen::Vector2d> p_th1_dth1_l{};
  // {2.55462,  2.98866}, to
  // {3.35720, -1.49233}, to
  // {2.55278,  1.49500}, to
  // {2.55462,  2.98866}
  const std::vector<Eigen::Vector2d> p_th1_dth1_r{};

  const std::vector<Eigen::Vector2d> p_th1_dth2_l{};
  const std::vector<Eigen::Vector2d> p_th1_dth2_r{};

  const std::vector<Eigen::Vector2d> p_th2_dth1_l{};
  // {5.46215,  0.842409}, to
  // {5.98235,  2.93176}, to
  // {6.27497,  3.50967}, to
  // {6.26413, -3.95866}, to
  // {5.78727, -4.00311}, to
  // {5.46215, -2.53612},
  const std::vector<Eigen::Vector2d> p_th2_dth1_r{};

  // {0.0216752,  3.55412}, to
  // {0.216752,  3.46521}, to
  // {0.791144,  0.975772}, to
  // {0.801982, -2.49166}, to
  // {0.487692, -4.00311}, to
  // {0.0108376, -4.00311},
  const std::vector<Eigen::Vector2d> p_th2_dth2_l{};

  const std::vector<Eigen::Vector2d> p_th2_dth2_r{};
  const std::vector<Eigen::Vector2d> p_dth1_dth2_l{
    { -6.74717, 10.7957 }, { -3.21722, 9.99555 }, { 6.93915, -10.0089 }, { 3.28534, -9.74216 }
  };
  // const std::vector<Eigen::Vector2d> p_dth1_dth2_r{
  //   { -15.8, 30.0 }, { -12.6, 30.0 }, { 13.2, -24.3 }, { 12.2, -28.3 }
  // };

  if (not params.exists("plant"))
  {
    prx_throw("Need plant param: {acrobot, pendubot} ");
  }
  if (params["plant"].as<>() == "acrobot")
  {
    plant_name = "acrobot_dp";
    auto system_aux = prx::system_factory_t::create_system(plant_name, plant_name);
    plant = std::dynamic_pointer_cast<prx::plant_t>(system_aux);
    prx_assert(plant != nullptr, "Plant is nullptr!");

    const Eigen::Matrix4d Q{ Eigen::DiagonalMatrix<double, 4>(1.0, 1.0, 1.0, 1.0) };
    const Eigen::Matrix<double, 1, 1> R{ Eigen::Matrix<double, 1, 1>::Identity() };
    const Eigen::Vector4d x_goal{ { prx::constants::pi, 0.0, 0.0, 0.0 } };
    const Eigen::Vector<double, 1> u_goal{ { 0.0 } };

    PRX_DBG_VARS(Q);
    PRX_DBG_VARS(R);

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

    lqr = std::make_shared<LQR>(plant, "LQR", Q, R, x_goal, u_goal, diff);
    prx_assert(lqr != nullptr, "lqr is nullptr!");
    auto K = lqr->lqr().K();

    PRX_DBG_VARS(K);
  }
  prx::world_model_t world_model({ plant }, {});

  world_model.create_context("context", { plant_name }, {});
  prx::simulation_context context{ world_model.get_context("context") };

  SystemGroupPtr sg{ prx::system_group(context) };
  std::shared_ptr<prx::collision_group_t> cg{ prx::collision_group(context) };

  const double dt{ params["dt"].as<double>() };
  prx::condition_check_t cond_check("sim_time", dt);  // 5 secs

  prx::space_t* ss{ sg->get_state_space() };
  prx::trajectory_t traj{ ss };

  const std::vector<double> start{ params["start"].as<std::vector<double>>() };
  const std::vector<double> finish{ params["finish"].as<std::vector<double>>() };
  prx::space_point_t x0{ ss->make_point(start) };
  prx::space_point_t xF{ ss->make_point(start) };
  // ss->set_bounds(start, finish);

  const std::string file_traj{ params["trajs"].as<>() };
  const std::string file_states{ params["states"].as<>() };
  const std::vector<double> step{ params["step"].as<std::vector<double>>() };

  std::ofstream ofs_trajs(file_traj.c_str());
  std::ofstream ofs_states(file_states.c_str());

  Eigen::Vector2d rod1, ball;

  do
  {
    // traj.clear();
    cond_check.reset();

    ss->copy_from(x0);
    plant->update_configuration();
    rod1 = plant->configuration("rod1").translation().head(2);
    ball = plant->configuration("ball").translation().head(2);

    const Eigen::Vector2d th1_th2(x0->at(0), x0->at(1));
    const Eigen::Vector2d th1_dth1(x0->at(0), x0->at(2));
    const Eigen::Vector2d th1_dth2(x0->at(0), x0->at(3));
    const Eigen::Vector2d th2_dth1(x0->at(1), x0->at(2));
    const Eigen::Vector2d th2_dth2(x0->at(1), x0->at(3));
    const Eigen::Vector2d dth1_dth2(x0->at(2), x0->at(3));
    // ball[1] > 0.45                                       // no-lint
    if (dist(x0) < 0.2                                     // no-lint
        and (inside_convex_poly(p_th1_th2_l, th1_th2)      // no-lint
             or inside_convex_poly(p_th1_th2_r, th1_th2))  // no-lint
        //     and (inside_convex_poly(p_th1_dth1_l, th1_dth1)      // no-lint
        //          or inside_convex_poly(p_th1_dth1_r, th1_dth1))  // no-lint
        //     and (inside_convex_poly(p_th1_dth2_l, th1_dth2)      // no-lint
        //          or inside_convex_poly(p_th1_dth2_r, th1_dth2))  // no-lint
        //     and (inside_convex_poly(p_th2_dth1_l, th2_dth1)      // no-lint
        //          or inside_convex_poly(p_th2_dth1_r, th2_dth1))  // no-lint
        //     and (inside_convex_poly(p_th2_dth2_l, th2_dth2)      // no-lint
        //          or inside_convex_poly(p_th2_dth2_r, th2_dth2))  // no-lint
        //     and
        //     inside_convex_poly(p_dth1_dth2_l, dth1_dth2)  // no-lint
        // or inside_convex_poly(p_dth1_dth2_r, dth1_dth2))  // no-lint
    )
    {
      ofs_states << print_pt(x0) << " ";
      ofs_states << rod1.transpose() << " " << ball.transpose() << " ";

      const bool success{ propagate_and_check(x0, xF, plant, lqr, sg, cond_check) };

      // sg->propagate(pt, lqr, cond_check, traj);

      ss->copy_from(xF);
      plant->update_configuration();
      rod1 = plant->configuration("rod1").translation().head(2);
      ball = plant->configuration("ball").translation().head(2);
      // xf->at(0) = std::atan2
      // ofs_states << xF << " ";
      ofs_states << print_pt(xF) << " ";
      ofs_states << rod1.transpose() << " " << ball.transpose() << " ";
      ofs_states << (success ? "1" : "0") << " ";
      ofs_states << "\n";
    }
    // ofs_trajs << traj;
  } while (x0->step(step, start, finish));
  // bool step(const double step_inc, const LowerBound& lower_bound, const UpperBound& upper_bound)

  // prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, {});

  // const std::string body_name{ plant_name + "/ball" };

  // vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, traj, body_name, ss);
  // vis_group->add_animation(traj, ss, pt);
  // vis_group->output_html("double_pendulum_lqr.html");

  // delete vis_group;
  return 0;
}
