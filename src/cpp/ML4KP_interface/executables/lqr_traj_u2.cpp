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
#include "ML4KP_interface/simulation/acrobot_u2.hpp"
#include "ML4KP_interface/simulation/pendubot.hpp"
#include "ML4KP_interface/simulation/utils.hpp"

using LQR = prx::simulation::lqr_controller_t<5, -1>;
using LQRptr = std::shared_ptr<LQR>;

int main(int argc, char* argv[])
{
  prx::param_loader params{ argc, argv };

  prx::simulation_step = 0.002;
  params["/planner/random_seed"].set(112392);
  // params["normalize"].set(false);

  std::shared_ptr<prx::plant_t> plant;
  std::string plant_name;

  // LQRptr lqr;
  if (not params.exists("plant"))
  {
    prx_throw("Need plant param: {acrobot, pendubot} ");
  }
  if (params["plant"].as<>() == "acrobot")
  {
    plant_name = "acrobot_u2";
  }
  else if (params["plant"].as<>() == "pendubot")
  {
    plant_name = "pendubot_dp";
  }
  else
  {
    prx_throw("Plant " << params["plant"].as<>() << " not supported.");
  }
  auto system_aux = prx::system_factory_t::create_system(plant_name, plant_name);
  plant = std::dynamic_pointer_cast<prx::plant_t>(system_aux);
  prx_assert(plant != nullptr, "Plant is nullptr!");
  // plant->init(params["/plant"]);

  prx::world_model_t world_model({ plant }, {});

  world_model.create_context("context", { plant_name }, {});
  prx::simulation_context context{ world_model.get_context("context") };

  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };
  std::shared_ptr<prx::collision_group_t> cg{ prx::collision_group(context) };

  prx::condition_check_t cond_check("sim_time", params["sim_time"].as<double>());  // 5 secs

  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };
  prx::plan_t plan{ cs };
  prx::trajectory_t traj{ ss };

  double_pendulum::lqr_query_t<4, 2> lqr_query;
  // lqr_query.x_goal = Eigen::Vector4d::Zero();
  std::vector<double> Qin{ params["Q"].as<std::vector<double>>() };
  std::vector<double> Rin{ params["R"].as<std::vector<double>>() };
  ss->copy(lqr_query.x_goal, params["goal"].as<std::vector<double>>());

  for (int i = 0; i < 4; ++i)
  {
    lqr_query.Q.diagonal()[i] = Qin[i];
  }
  for (int i = 0; i < 2; ++i)
  {
    lqr_query.R.diagonal()[i] = Rin[i];
  }
  lqr_query.normalize = params["normalize"].as<bool>();
  // PRX_DBG_VARS(lqr_query.R);
  // PRX_DBG_VARS(lqr_query.normalize, params["normalize"].as<bool>());

  // lqr_query.Q = Eigen::DiagonalMatrix<double, 4>(1.0, 1.0, 10., 10.);
  LQRptr lqr{ double_pendulum::create_lqr(plant, lqr_query) };

  const std::vector<double> start{ params["start"].as<std::vector<double>>() };
  // const std::vector<double> finish{ params["finish"].as<std::vector<double>>() };
  prx::space_point_t pt{ ss->make_point(start) };
  prx::space_point_t ctrl{ cs->make_point() };
  // ss->set_bounds(start, finish);

  // const double step{ params["step"].as<double>() };
  PRX_DBG_VARS(lqr_query.R);
  PRX_DBG_VARS(lqr_query.Q);
  // lqr->lqr().K() = Eigen::RowVector4d(10, 0, 0, 0);
  PRX_DBG_VARS(lqr->lqr().A());
  PRX_DBG_VARS(lqr->lqr().B());
  PRX_DBG_VARS(lqr->lqr().R());
  PRX_DBG_VARS(lqr->lqr().Q());
  PRX_DBG_VARS(lqr->lqr().K());
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
