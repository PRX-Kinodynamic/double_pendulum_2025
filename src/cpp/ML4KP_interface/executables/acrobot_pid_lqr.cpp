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
#include "ML4KP_interface/simulation/pendubot.hpp"
#include "ML4KP_interface/simulation/utils.hpp"
#include "ML4KP_interface/simulation/acrobot_pid.hpp"

using LQR = prx::simulation::lqr_controller_t<5, -1>;
using LQRptr = std::shared_ptr<LQR>;

int main(int argc, char* argv[])
{
  prx::param_loader params{ argc, argv };

  prx::simulation_step = 0.002;
  params["/planner/random_seed"].set(112392);

  std::shared_ptr<prx::plant_t> plant;
  std::string plant_in{};

  // LQRptr lqr;
  if (params.exists("plant_file"))
  {
    plant_in = params["plant_file"].as<>();
    params["plant"].add_file(plant_in);
  }
  else
  {
    prx_throw("No plant file set");
  }
  const std::string plant_name{ params["/plant/name"].as<>() };
  auto system_aux = prx::system_factory_t::create_system(plant_name, plant_name);
  plant = std::dynamic_pointer_cast<prx::plant_t>(system_aux);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  prx::world_model_t world_model({ plant }, {});

  world_model.create_context("context", { plant_name }, {});
  prx::simulation_context context{ world_model.get_context("context") };

  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };
  std::shared_ptr<prx::collision_group_t> cg{ prx::collision_group(context) };

  prx::condition_check_t cond_check("sim_time", params["sim_time"].as<double>());  // 5 secs

  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };
  prx::space_t* ps{ sg->get_parameter_space() };
  prx::space_point_t goal_state{ ss->make_point() };
  prx::plan_t plan{ cs };
  prx::trajectory_t traj{ ss };
  ps->copy_from({ -1.0 });

  Eigen::RowVector<double, 6> lqrK;
  ss->copy(goal_state, params["goal_state"].as<std::vector<double>>());
  ss->copy(lqrK, params["/plant/lqr/K"].as<std::vector<double>>());

  double_pendulum::lqr_query_t<6> lqr_query;
  double_pendulum::LQRptr lqr{ std::make_shared<double_pendulum::LQR>(plant, plant_name, lqrK, Vec(goal_state),
                                                                      lqr_query.diff) };

  const std::vector<double> start{ params["start"].as<std::vector<double>>() };
  // const std::vector<double> finish{ params["finish"].as<std::vector<double>>() };
  prx::space_point_t pt{ ss->make_point(start) };
  prx::space_point_t ctrl{ cs->make_point() };
  // ss->set_bounds(start, finish);

  // const double step{ params["step"].as<double>() };
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
    // PRX_DBG_VARS(t, ball.transpose(), ctrl);
    plan.copy_onto_back(ctrl, prx::simulation_step);
    t += prx::simulation_step;
  } while (!cond_check.check());

  if (params.exists("traj") and params.exists("plan"))
  {
    const std::string traj_filename{ params["traj"].as<>() };
    const std::string plan_filename{ params["plan"].as<>() };

    std::ofstream ofs_traj(traj_filename.c_str());
    std::ofstream ofs_plan(plan_filename.c_str());

    ofs_traj << traj;
    ofs_plan << plan;

    ofs_traj.close();
    ofs_plan.close();
  }
  // } while (pt->step(step));

  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, {});

  const std::string body_name{ plant_name + "/ball" };

  PRX_DBG_VARS(traj.back());
  // vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, traj, body_name, ss);
  vis_group->add_vis_infos(prx::info_geometry_t::FULL_LINE, traj, body_name, ss);
  vis_group->add_animation(traj, ss, pt);
  vis_group->output_html(plant_name + "_lqr.html");

  delete vis_group;
  return 0;
}
