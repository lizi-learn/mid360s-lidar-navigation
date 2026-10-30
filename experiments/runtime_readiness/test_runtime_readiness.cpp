#include "runtime_readiness.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>
int main(){
 namespace fs=std::filesystem;using namespace p2_nav;
 auto dir=fs::temp_directory_path()/("navigation-evidence-"+std::to_string(getpid()));fs::create_directory(dir);
 auto file=dir/"proof",cal=dir/"calibration";std::ofstream(cal)<<"validated test fixture";chmod(cal.c_str(),0600);
 const auto machine=local_machine_id();assert(machine.size()==32);
 RuntimeReadiness guard(file,machine,cal,geteuid());assert(guard.refresh()==0);
 ReadinessRecord r{};std::memcpy(r.magic,"P2NAVREADYv1",12);std::memcpy(r.machine_id,machine.data(),32);
 r.driver_pid=getpid();r.driver_start_ticks=process_start_ticks(getpid());struct stat st{};assert(!stat("/proc/self/exe",&st));
 r.driver_exe_device=st.st_dev;r.driver_exe_inode=st.st_ino;r.issued_monotonic_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();r.protection_verified=1;
 auto write=[&]{std::ofstream out(file,std::ios::binary|std::ios::trunc);out.write(reinterpret_cast<char*>(&r),sizeof(r));out.close();chmod(file.c_str(),0600);};
 write();assert(guard.refresh()==1);assert(guard.flags()==1);
 r.driver_start_ticks++;write();assert(guard.refresh()==0);r.driver_start_ticks--;
 r.driver_exe_inode++;write();assert(guard.refresh()==0);r.driver_exe_inode--;
 r.machine_id[0]^=1;write();assert(guard.refresh()==0);r.machine_id[0]^=1;
 write();chmod(file.c_str(),0666);assert(guard.refresh()==0);chmod(file.c_str(),0600);
 r.commissioning_verified=1;write();assert(guard.refresh()==1);
 assert(!lstat(cal.c_str(),&st));r.calibration_device=st.st_dev;r.calibration_inode=st.st_ino;r.calibration_mtime_ns=file_mtime_ns(st);r.calibration_size=st.st_size;
 write();assert(guard.refresh()==3);
 std::ofstream(cal,std::ios::app)<<"changed";assert(guard.refresh()==1);
 r.issued_monotonic_ns=UINT64_MAX;write();assert(guard.refresh()==0);
 fs::remove(file);fs::create_symlink(cal,file);assert(guard.refresh()==0);fs::remove(file);
 r.issued_monotonic_ns=1;r.commissioning_verified=0;write();assert(guard.refresh()==1);
 std::this_thread::sleep_for(std::chrono::milliseconds(220));assert(guard.flags()==0);
 guard.start();std::this_thread::sleep_for(std::chrono::milliseconds(80));assert(guard.flags()==1);
 fs::remove(file);std::this_thread::sleep_for(std::chrono::milliseconds(80));assert(guard.flags()==0);
 fs::remove_all(dir);
}
