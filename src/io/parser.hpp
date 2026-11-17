#pragma once
#include <string>
#include "model/galaxy.hpp"

namespace chronos {

// Format: see docs/dataset_format.md. Both return false and fill *err on failure.
bool load_galaxy(const std::string& path, Galaxy& g, std::string* err = nullptr);
bool load_events(const std::string& path, std::vector<Event>& events, std::string* err = nullptr);

}  // namespace chronos
