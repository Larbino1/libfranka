#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;


json open_points_json();