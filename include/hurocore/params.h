// Copyright 2025 Ioannis Tsikelis

#ifndef HUROCORE_PARAMS_H_
#define HUROCORE_PARAMS_H_

#include <cstddef>
#include <string>
#include <vector>

namespace hurocore {
struct Params {
  bool high_freq;
  bool info_imu;
  bool info_motors;
  size_t n_motors;
  std::vector<std::string> joint_names;
};

} // namespace hurocore
#endif // HUROCORE_PARAMS_H_
