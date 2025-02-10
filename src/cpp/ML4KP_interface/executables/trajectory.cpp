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

  prx::system_ptr_t plant;
  std::string plant_name;

  LQRptr lqr;
  if (not params.exists("plant"))
  {
    prx_throw("Need plant param: {acrobot, pendubot} ");
  }
  if (params["plant"].as<>() == "acrobot")
  {
    plant_name = "acrobot_dp";
    plant = prx::system_factory_t::create_system(plant_name, plant_name);
    prx_assert(plant != nullptr, "Plant is nullptr!");
  }
  prx::world_model_t world_model({ plant }, {});

  world_model.create_context("context", { plant_name }, {});
  prx::simulation_context context{ world_model.get_context("context") };

  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };
  std::shared_ptr<prx::collision_group_t> cg{ prx::collision_group(context) };

  prx::condition_check_t cond_check("sim_time", 0.5);  // 0.5 secs

  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };
  prx::trajectory_t traj{ ss };

  const std::vector<double> start{ params["start"].as<std::vector<double>>() };
  prx::space_point_t pt{ ss->make_point(start) };
  // ss->set_bounds(start, finish);

  const std::string filename{ params["filename"].as<>() };

  std::ofstream ofs_map(filename.c_str());

  prx::plan_t plan(cs);

  if (params.exists("plan"))
  {
    const std::string plan_file{ params["plan"].as<>() };
    plan.from_file(plan_file);
  }
  else if (params.exists("time"))
  {
    const double tf{ params["time"].as<double>() };
    plan.copy_onto_back(std::vector<double>({ 0.0 }), tf);
  }

  traj.clear();
  cond_check.reset();
  sg->propagate(pt, plan, traj);
  ofs_map << std::setw(6) << traj;

  ofs_map.close();

  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, {});

  const std::string body_name{ plant_name + "/ball" };

  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, traj, body_name, ss);
  vis_group->add_animation(traj, ss, pt);
  vis_group->output_html("double_pendulum_traj.html");

  delete vis_group;
  return 0;
}
