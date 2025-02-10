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

using LQR = prx::simulation::lqr_controller_t<>;
using LQRptr = std::shared_ptr<LQR>;

int main(int argc, char* argv[])
{
  prx::param_loader params{ argc, argv };

  prx::simulation_step = 0.002;
  params["/planner/random_seed"].set(112392);

  std::shared_ptr<prx::plant_t> plant;
  std::string plant_name;

  LQRptr lqr;

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

    const Eigen::Matrix4d Q{ Eigen::DiagonalMatrix<double, 4>(10.0, 10.0, 1.0, 1.0) };
    const Eigen::Matrix<double, 1, 1> R{ Eigen::Matrix<double, 1, 1>::Identity() };
    const Eigen::Vector4d x_goal{ { prx::constants::pi, 0.0, 0.0, 0.0 } };
    const Eigen::Vector<double, 1> u_goal{ { 0.0 } };

    PRX_DBG_VARS(Q);
    PRX_DBG_VARS(R);

    LQR::Diff diff = [](const LQR::VectorX& a, const LQR::VectorX& b)  // no-lint
    {
      LQR::VectorX res{ a - b };
      res[0] = std::atan2(-std::sin(res[0]), std::cos(res[0]));
      // res[1] = std::atan2(-std::sin(res[1]), std::cos(res[1]));
      // PRX_DBG_VARS(res);
      return res;
    };

    lqr = std::make_shared<LQR>(plant, "LQR", Q, R, x_goal, u_goal, diff);
    prx_assert(lqr != nullptr, "lqr is nullptr!");
    auto K = lqr->lqr().K();

    PRX_DBG_VARS(K);
  }
  prx::world_model_t world_model({ plant }, {});

  world_model.create_context("context", { plant_name }, {});
  prx::simulation_context context{ world_model.get_context("context") };

  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };
  std::shared_ptr<prx::collision_group_t> cg{ prx::collision_group(context) };

  prx::condition_check_t cond_check("sim_time", 5.0);  // 5 secs

  prx::space_t* ss{ sg->get_state_space() };
  prx::trajectory_t traj{ ss };

  const std::vector<double> start{ params["start"].as<std::vector<double>>() };
  const std::vector<double> finish{ params["finish"].as<std::vector<double>>() };
  prx::space_point_t pt{ ss->make_point(start) };
  // ss->set_bounds(start, finish);

  const std::string file_traj{ params["trajs"].as<>() };
  const std::string file_states{ params["states"].as<>() };
  const std::vector<double> step{ params["step"].as<std::vector<double>>() };

  std::ofstream ofs_trajs(file_traj.c_str());
  std::ofstream ofs_states(file_states.c_str());

  Eigen::Vector2d rod1, ball;

  do
  {
    traj.clear();
    cond_check.reset();

    ss->copy_from(pt);
    plant->update_configuration();
    rod1 = plant->configuration("rod1").translation().head(2);
    ball = plant->configuration("ball").translation().head(2);

    if (ball[1] >= 0.45)
    {
      ofs_states << pt << " ";
      ofs_states << rod1.transpose() << " " << ball.transpose() << " ";

      sg->propagate(pt, lqr, cond_check, traj);

      ss->copy_from(traj.back());
      plant->update_configuration();
      rod1 = plant->configuration("rod1").translation().head(2);
      ball = plant->configuration("ball").translation().head(2);
      ofs_states << traj.back() << " ";
      ofs_states << rod1.transpose() << " " << ball.transpose() << "\n";
    }
    // ofs_trajs << traj;
  } while (pt->step(step, start, finish));
  // bool step(const double step_inc, const LowerBound& lower_bound, const UpperBound& upper_bound)

  // prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, {});

  // const std::string body_name{ plant_name + "/ball" };

  // vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, traj, body_name, ss);
  // vis_group->add_animation(traj, ss, pt);
  // vis_group->output_html("double_pendulum_lqr.html");

  // delete vis_group;
  return 0;
}
