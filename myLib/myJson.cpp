#include "myJson.h"

json open_points_json() {
  const char *homedir;
  homedir = getenv("HOME");
  fs::path filepath(homedir);
  filepath = filepath / "data" / "points5.json";
  std::ifstream f;
  f.open(filepath);
  bool failed = false;
  if (f.is_open()) {
    if (f.peek() == std::ifstream::traits_type::eof()) {
      std::cerr << "Specified file " << filepath << " appears to be empty.\n";
      failed = true;
    }
    else {
      json data = json::parse(f);
      return data;
    }
  } else {
      std::cout << "Could not open file " << filepath << ". " << strerror(errno) << std::endl;
      failed = true;
  }
  if (failed) {
    exit(0);
  }
}