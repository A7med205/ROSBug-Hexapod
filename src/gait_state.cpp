#include "hexapod_sim/gait_state.hpp"

#include <limits>

#include "hexapod_sim/gait_config.hpp"

namespace hexapod_sim
{

std::vector<Leg> create_default_legs()
{
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  return {
    {1, {"jl11", "jl12", "jl13"}, {-0.0535, 0.0900, 135.0}, {kXHome, kYHome, kZHome}, {nan, nan, nan}},
    {2, {"jl21", "jl22", "jl23"}, {-0.0700, 0.0000, 180.0}, {kXHome, kYHome, kZHome}, {nan, nan, nan}},
    {3, {"jl31", "jl32", "jl33"}, {-0.0535, -0.0900, -135.0}, {kXHome, kYHome, kZHome}, {nan, nan, nan}},
    {4, {"jl41", "jl42", "jl43"}, {0.0535, 0.0900, 45.0}, {kXHome, kYHome, kZHome}, {nan, nan, nan}},
    {5, {"jl51", "jl52", "jl53"}, {0.0700, 0.0000, 0.0}, {kXHome, kYHome, kZHome}, {nan, nan, nan}},
    {6, {"jl61", "jl62", "jl63"}, {0.0535, -0.0900, -45.0}, {kXHome, kYHome, kZHome}, {nan, nan, nan}}
  };
}

void initialize_path_tip_state(GaitState & state)
{
  for (auto & tip : state.path_tip_state) {
    tip = {kXHome, kYHome, kZHome};
  }
}

}  // namespace hexapod_sim
