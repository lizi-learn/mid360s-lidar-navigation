#pragma once
#include <sys/stat.h>
#include <stdexcept>
#include <string>

// Capture BEFORE constructing a long-lived reader. An unlinked POSIX mapping
// remains readable, so successful reads alone do not identify the live source.
class ImuSourceIdentity {
  std::string path_;
  struct stat original_{};
 public:
  explicit ImuSourceIdentity(std::string path):path_(std::move(path)) {
    if (::lstat(path_.c_str(), &original_) || !S_ISREG(original_.st_mode))
      throw std::runtime_error("IMU source is absent or is not a regular mapping");
  }
  bool unchanged() const noexcept {
    struct stat current{};
    return !::lstat(path_.c_str(), &current) && S_ISREG(current.st_mode) &&
      current.st_dev==original_.st_dev && current.st_ino==original_.st_ino;
  }
  unsigned long long inode() const noexcept {return original_.st_ino;}
};
