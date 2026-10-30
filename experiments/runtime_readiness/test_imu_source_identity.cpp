#include "imu_source_identity.hpp"
#include <filesystem>
#include <fstream>
#include <cassert>
#include <unistd.h>
int main() {
  namespace fs=std::filesystem;
  auto root=fs::temp_directory_path()/("imu-reader-identity-"+std::to_string(getpid()));
  fs::create_directory(root);auto p=root/"source";
  bool rejected=false;try {ImuSourceIdentity g(p);}catch(const std::runtime_error&){rejected=true;}
  assert(rejected);
  std::ofstream(p)<<"old";ImuSourceIdentity old(p);assert(old.unchanged());
  std::ofstream(p,std::ios::app)<<"updated";assert(old.unchanged());
  std::ofstream(root/"next")<<"new";fs::rename(root/"next",p);assert(!old.unchanged());
  ImuSourceIdentity current(p);assert(current.unchanged());assert(current.inode()!=old.inode());
  fs::remove(p);assert(!current.unchanged());
  std::ofstream(root/"target")<<"real";fs::create_symlink(root/"target",p);
  rejected=false;try {ImuSourceIdentity g(p);}catch(const std::runtime_error&){rejected=true;}
  assert(rejected);fs::remove_all(root);
}
