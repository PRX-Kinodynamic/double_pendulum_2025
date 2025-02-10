#include <fstream>
#include <prx/utilities/defs.hpp>
#include <prx/utilities/general/param_loader.hpp>

#include <prx/simulation/plants/plants.hpp>
#include <prx/simulation/loaders/obstacle_loader.hpp>

#include <prx/planning/world_model.hpp>
#include <prx/planning/planners/aorrt.hpp>
#include <prx/planning/planners/planner.hpp>

#include <prx/visualization/three_js_group.hpp>

#include "ML4KP_interface/simulation/acrobot.hpp"

int main(int argc, char* argv[])
{
  prx::param_loader params{ argc, argv };

  params["/planner/simulation_step"].set(0.002);
  params["/planner/random_seed"].set(112392);

  prx::system_ptr_t plant;
  std::string plant_name;
  if (not params.exists("plant"))
  {
    prx_throw("Need plant param: {acrobot, pendubot} ");
  }
  if (params["plant"].as<>() == "acrobot")
  {
    plant_name = "acrobot_dp";
    plant = prx::system_factory_t::create_system(plant_name, plant_name);
    prx_assert(plant != nullptr, "Plant is nullptr!");
    prx::param_loader params_plant{ plant->init() };

    const std::string filename{ params.exists("plant_file") ? params["plant_file"].as<>() : "plant.yaml" };
    PRX_DBG_VARS(filename);
    params_plant.save(filename);
  }
  prx_assert(plant != nullptr, "Plant is nullptr!");
  prx::world_model_t world_model({ plant }, {});

  world_model.create_context("context", { plant_name }, {});
  prx::simulation_context context{ world_model.get_context("context") };

  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };
  std::shared_ptr<prx::collision_group_t> cg{ prx::collision_group(context) };

  prx::aorrt_t aorrt("AORRT");
  prx::aorrt_specification_t aorrt_spec(sg, cg);
  prx::param_loader params_plan{ aorrt_spec.init() };

  prx::condition_check_t checker("time", 60.0);

  params_plan["checker"] = checker.init();

  const std::string filename{ params.exists("planner_file") ? params["planner_file"].as<>() : "planner.yaml" };
  PRX_DBG_VARS(filename);
  params_plan.save(filename);
}
