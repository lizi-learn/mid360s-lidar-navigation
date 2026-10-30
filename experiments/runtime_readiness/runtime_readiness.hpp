#pragma once
// Static commissioning facts, bound to this host and the current driver process.
// Never substitutes for live hardware, operator, ground or localization checks.
#include <atomic>
#include <chrono>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <cstdint>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

namespace p2_nav {
struct ReadinessRecord {
  char magic[16]{};
  std::uint64_t driver_pid{},driver_start_ticks{},driver_exe_device{},driver_exe_inode{},issued_monotonic_ns{};
  std::uint32_t protection_verified{},commissioning_verified{};
  char machine_id[32]{};
  std::uint64_t calibration_device{},calibration_inode{},calibration_mtime_ns{},calibration_size{};
};
static_assert(sizeof(ReadinessRecord)==128);
inline std::uint64_t process_start_ticks(std::uint64_t pid) {
  std::ifstream file("/proc/"+std::to_string(pid)+"/stat");std::string line;
  if(!std::getline(file,line))return 0;
  auto end=line.rfind(')');if(end==std::string::npos)return 0;
  std::istringstream fields(line.substr(end+1));std::string value;
  for(int i=0;i<20;++i)if(!(fields>>value))return 0;
  try{return std::stoull(value);}catch(...){return 0;}
}
inline std::string local_machine_id() {
  std::ifstream file("/etc/machine-id");std::string id;file>>id;return id;
}
inline std::uint64_t file_mtime_ns(const struct stat& s) {
  return std::uint64_t(s.st_mtim.tv_sec)*1000000000ULL+s.st_mtim.tv_nsec;
}
class RuntimeReadiness {
  std::string path_,machine_,calibration_;uid_t owner_;
  std::atomic<unsigned> flags_{0};std::atomic<std::uint64_t> checked_{0};
  std::atomic<bool> stop_{false};std::thread thread_;
  static std::uint64_t monotonic_ns(){
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
 public:
  RuntimeReadiness(std::string path,std::string machine,std::string calibration,uid_t owner=0)
    :path_(std::move(path)),machine_(std::move(machine)),calibration_(std::move(calibration)),owner_(owner){}
  RuntimeReadiness(const RuntimeReadiness&)=delete;
  ~RuntimeReadiness(){stop_=true;if(thread_.joinable())thread_.join();}
  unsigned refresh() noexcept {
    unsigned accepted=0;
    try {
      const int fd=::open(path_.c_str(),O_RDONLY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK);
      if(fd<0){flags_=0;return 0;}
      struct stat file{};ReadinessRecord record{};
      bool valid=!::fstat(fd,&file)&&S_ISREG(file.st_mode)&&file.st_uid==owner_&&!(file.st_mode&0022)&&file.st_size==sizeof(record);
      valid=valid&&::read(fd,&record,sizeof(record))==sizeof(record);::close(fd);
      const auto now=monotonic_ns();
      valid=valid&&!std::memcmp(record.magic,"P2NAVREADYv1",12)&&machine_.size()==32&&
        !std::memcmp(record.machine_id,machine_.data(),32)&&record.protection_verified<=1&&record.commissioning_verified<=1&&
        record.issued_monotonic_ns>0&&record.issued_monotonic_ns<=now&&record.driver_pid>0&&
        record.driver_start_ticks>0&&process_start_ticks(record.driver_pid)==record.driver_start_ticks;
      struct stat executable{};
      valid=valid&&!::stat(("/proc/"+std::to_string(record.driver_pid)+"/exe").c_str(),&executable)&&
        std::uint64_t(executable.st_dev)==record.driver_exe_device&&std::uint64_t(executable.st_ino)==record.driver_exe_inode;
      if(valid&&record.protection_verified)accepted|=1;
      if(valid&&record.commissioning_verified&&!calibration_.empty()) {
        struct stat calibration{};
        if(!::lstat(calibration_.c_str(),&calibration)&&S_ISREG(calibration.st_mode)&&calibration.st_uid==owner_&&!(calibration.st_mode&0022)&&
           std::uint64_t(calibration.st_dev)==record.calibration_device&&std::uint64_t(calibration.st_ino)==record.calibration_inode&&
           file_mtime_ns(calibration)==record.calibration_mtime_ns&&std::uint64_t(calibration.st_size)==record.calibration_size)accepted|=2;
      }
      flags_=accepted;checked_=now;return accepted;
    }catch(...){flags_=0;return 0;}
  }
  void start(){thread_=std::thread([this]{while(!stop_){refresh();std::this_thread::sleep_for(std::chrono::milliseconds(50));}});}
  unsigned flags() const noexcept {
    const auto now=monotonic_ns(),stamp=checked_.load();
    return stamp&&now>=stamp&&now-stamp<=200000000ULL?flags_.load():0;
  }
};
}
