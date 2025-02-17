#include <fstream>
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
#include "ML4KP_interface/simulation/utils.hpp"

using LQR = prx::simulation::lqr_controller_t<5, -1>;
using LQRptr = std::shared_ptr<LQR>;

int main(int argc, char* argv[])
{
  prx::param_loader params{ argc, argv };

  prx::simulation_step = 0.002;
  params["/planner/random_seed"].set(112392);

  std::shared_ptr<prx::plant_t> plant;
  std::string plant_name;

  // LQRptr lqr;
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

    // const Eigen::Matrix4d Q{ Eigen::DiagonalMatrix<double, 4>(1.0, 1.0, 1.0, 1.0) };
    // const Eigen::Matrix<double, 1, 1> R{ Eigen::Matrix<double, 1, 1>::Identity() };
    // const Eigen::Vector4d x_goal{ { prx::constants::pi, 0.0, 0.0, 0.0 } };
    // const Eigen::Vector<double, 1> u_goal{ { 0.0 } };

    // LQR::Diff diff = [](const LQR::VectorX& x, const LQR::VectorX& ref)  // no-lint
    // {
    //   const double th0{ x[0] };
    //   const double th1{ x[1] };

    //   const double th0_ref{ ref[0] };
    //   const double th1_ref{ ref[1] };

    //   const double dth0{ std::atan2(std::sin(th0 - th0_ref), std::cos(th0 - th0_ref)) };
    //   const double dth1{ std::atan2(std::sin(th1 - th1_ref), std::cos(th1 - th1_ref)) };
    //   const double dv0{ x[2] - ref[2] };
    //   const double dv1{ x[3] - ref[3] };

    //   return Eigen::Vector4d(dth0, dth1, dv0, dv1);
    // };

    // lqr = std::make_shared<LQR>(plant, "LQR", Q, R, x_goal, u_goal, diff);
    // prx_assert(lqr != nullptr, "lqr is nullptr!");
    // auto K = lqr->lqr().K();

    // PRX_DBG_VARS(K);
  }
  LQRptr lqr{ double_pendulum::create_lqr(plant) };

  prx::world_model_t world_model({ plant }, {});

  world_model.create_context("context", { plant_name }, {});
  prx::simulation_context context{ world_model.get_context("context") };

  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };
  std::shared_ptr<prx::collision_group_t> cg{ prx::collision_group(context) };

  prx::condition_check_t cond_check("sim_time", 5.0);  // 5 secs

  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };
  prx::plan_t plan{ cs };
  prx::trajectory_t traj{ ss };

  const std::vector<double> start{ params["start"].as<std::vector<double>>() };
  // const std::vector<double> finish{ params["finish"].as<std::vector<double>>() };
  prx::space_point_t pt{ ss->make_point(start) };
  prx::space_point_t ctrl{ cs->make_point() };
  // ss->set_bounds(start, finish);

  const std::string traj_filename{ params["traj"].as<>() };
  const std::string plan_filename{ params["plan"].as<>() };
  // const double step{ params["step"].as<double>() };

  std::ofstream ofs_traj(traj_filename.c_str());
  std::ofstream ofs_plan(plan_filename.c_str());

  // do
  // {
  traj.clear();
  cond_check.reset();
  ss->copy_from(start);
  Eigen::Vector2d rod1, ball;
  traj.copy_onto_back(ss);
  double t{ 0.0 };
  do
  {
    lqr->compute_controls();
    sg->propagate_once(nullptr);
    traj.copy_onto_back(ss);

    plant->update_configuration();
    ball = plant->configuration("ball").translation().head(2);
    cs->copy_to(ctrl);
    PRX_DBG_VARS(t, ball.transpose());
    plan.copy_onto_back(ctrl, prx::simulation_step);
    t += prx::simulation_step;
  } while (!cond_check.check());
  ofs_traj << traj;
  ofs_plan << plan;

  ofs_traj.close();
  ofs_plan.close();
  // } while (pt->step(step));

  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, {});

  const std::string body_name{ plant_name + "/ball" };

  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, traj, body_name, ss);
  vis_group->add_animation(traj, ss, pt);
  vis_group->output_html("double_pendulum_lqr.html");

  delete vis_group;
  return 0;
}
