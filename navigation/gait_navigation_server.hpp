#pragma once
#include "gait_navigation.hpp"
#include <atomic>
#include <cerrno>
#include <filesystem>
#include <poll.h>
#include <stdexcept>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>

namespace p2_nav {
// Ordinary worker thread only. No socket/file I/O is performed by step().
class GaitNavigationServer {
  LocalMailbox box_;
  GaitNavigationInput loop_;
  std::thread worker_;
  std::atomic<bool> stopping_{false};
  int listener_=-1,native_listener_=-1;
  std::string path_;
  bool native_path_owned_=false;
  uid_t peer_uid_{::geteuid()};
  static bool valid(const LocalFrame& p){
    return p.magic==frame_magic&&p.version==1&&p.client&&p.sequence&&p.monotonic_ns&&
      !p.reserved&&!(p.flags&~frame_flags)&&
      static_cast<unsigned>(p.operation)>=1&&static_cast<unsigned>(p.operation)<=5;
  }
  bool trusted(int fd) const {
    ucred cred{};socklen_t length=sizeof(cred);
    return ::getsockopt(fd,SOL_SOCKET,SO_PEERCRED,&cred,&length)==0&&(cred.uid==peer_uid_||cred.uid==0);
  }
  void worker(){
    int peer=-1,native_peer=-1;std::uint64_t client=0,seq=0,last_stamp=0;
    NativeOperatorGuard guard;
    auto close_peer=[&](){if(peer>=0)::close(peer);peer=-1;client=seq=last_stamp=0;box_.disconnect();};
    auto close_native=[&](){if(native_peer>=0)::close(native_peer);native_peer=-1;box_.native_disconnect();};
    while(!stopping_.load(std::memory_order_acquire)){
      pollfd fds[4]={{listener_,POLLIN,0},{peer,POLLIN,0},{native_listener_,POLLIN,0},{native_peer,POLLIN,0}};
      if(::poll(fds,4,10)<0){if(errno==EINTR)continue;close_peer();close_native();break;}
      if(fds[0].revents&POLLIN){
        const int fd=::accept4(listener_,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
        if(fd>=0){if(peer>=0||!trusted(fd))::close(fd);else{peer=fd;client=seq=last_stamp=0;box_.connect();}}
      }
      if(fds[2].revents&POLLIN){
        const int fd=::accept4(native_listener_,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
        if(fd>=0){if(native_peer>=0||!trusted(fd))::close(fd);else native_peer=fd;}
      }
      if(native_peer>=0&&(fds[3].revents&(POLLERR|POLLHUP|POLLNVAL)))close_native();
      if(native_peer>=0&&(fds[3].revents&POLLIN)){
        NativeOperatorPacket sample{};const auto n=::recv(native_peer,&sample,sizeof(sample),MSG_DONTWAIT|MSG_TRUNC);
        if(n<0&&(errno==EAGAIN||errno==EINTR))continue;
        const auto now=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        if(n!=sizeof(sample)||!guard.accept(sample,now)){close_native();continue;}
        box_.post_native(sample);
      }
      if(peer>=0&&(fds[1].revents&(POLLERR|POLLHUP|POLLNVAL))){close_peer();continue;}
      if(peer>=0&&(fds[1].revents&POLLIN)){
        LocalFrame f{};const ssize_t n=::recv(peer,&f,sizeof(f),MSG_DONTWAIT|MSG_TRUNC);
        if(n<0&&(errno==EAGAIN||errno==EINTR))continue;
        if(n!=sizeof(f)||!valid(f)||(client&&f.client!=client)||f.sequence<=seq||f.monotonic_ns<last_stamp){close_peer();continue;}
        client=f.client;seq=f.sequence;last_stamp=f.monotonic_ns;box_.post(f);
        auto reply=box_.reply();const auto epoch=box_.epoch();
        // A new connection must never receive an old connection's loop reply.
        // Wait only in this ordinary worker; the100Hz loop never waits for I/O.
        for(int i=0;reply.transport_epoch!=epoch&&i<25;++i){
          std::this_thread::sleep_for(std::chrono::milliseconds(1));reply=box_.reply();
        }
        if(reply.transport_epoch!=epoch||::send(peer,&reply,sizeof(reply),MSG_DONTWAIT|MSG_NOSIGNAL)!=sizeof(reply))close_peer();
      }
    }
    close_peer();close_native();
  }

 public:
  GaitNavigationServer()=default;
  GaitNavigationServer(const GaitNavigationServer&)=delete;
  ~GaitNavigationServer(){stop();}
  void start(const std::string& path, uid_t client_uid=::geteuid()){
    if(client_uid!=::geteuid()&&::geteuid()!=0)throw std::runtime_error("Only root may select another navigation client UID");
    peer_uid_=client_uid;
    if(listener_>=0)throw std::runtime_error("Navigation socket already started");
    sockaddr_un address{};address.sun_family=AF_UNIX;
    if(path.empty()||path.size()>=sizeof(address.sun_path))throw std::runtime_error("Invalid navigation socket path");
    struct stat st{};
    const auto parent=std::filesystem::path(path).parent_path();
    if(::lstat(parent.c_str(),&st)||!S_ISDIR(st.st_mode)||st.st_uid!=peer_uid_||(st.st_mode&0077))
      throw std::runtime_error("Navigation socket requires an owned0700 directory");
    if(::lstat(path.c_str(),&st)==0)throw std::runtime_error("Navigation socket path already exists; inspect owner before removing");
    listener_=::socket(AF_UNIX,SOCK_SEQPACKET|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    if(listener_<0)throw std::runtime_error("Cannot create navigation socket");
    std::strncpy(address.sun_path,path.c_str(),sizeof(address.sun_path)-1);
    if(::bind(listener_,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0){::close(listener_);listener_=-1;throw std::runtime_error("Cannot bind navigation socket");}
    path_=path;
    if((::geteuid()==0&&::chown(path.c_str(),peer_uid_,static_cast<gid_t>(-1)))||::chmod(path.c_str(),0600)||::listen(listener_,1)){::close(listener_);listener_=-1;::unlink(path.c_str());path_.clear();throw std::runtime_error("Cannot protect/listen navigation socket");}
    const std::string source_path=path+".operator";
    if(source_path.size()>=sizeof(address.sun_path)){stop();throw std::runtime_error("Native source path too long");}
    native_listener_=::socket(AF_UNIX,SOCK_SEQPACKET|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    std::memset(address.sun_path,0,sizeof(address.sun_path));std::strncpy(address.sun_path,source_path.c_str(),sizeof(address.sun_path)-1);
    if(native_listener_<0||::bind(native_listener_,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0){stop();throw std::runtime_error("Cannot bind native source socket");}
    native_path_owned_=true;
    if((::geteuid()==0&&::chown(source_path.c_str(),peer_uid_,static_cast<gid_t>(-1)))||::chmod(source_path.c_str(),0600)||::listen(native_listener_,1)){stop();throw std::runtime_error("Cannot protect native source socket");}
    stopping_.store(false,std::memory_order_release);worker_=std::thread([this](){worker();});
  }
  void stop(){
    stopping_.store(true,std::memory_order_release);
    if(worker_.joinable())worker_.join();
    if(listener_>=0)::close(listener_);
    if(native_listener_>=0)::close(native_listener_);
    native_listener_=-1;
    listener_=-1;
    if(!path_.empty()){::unlink(path_.c_str());if(native_path_owned_)::unlink((path_+".operator").c_str());}
    native_path_owned_=false;
    path_.clear();
  }
  LoopReply step(const NativeGaitState& n){return loop_.step(box_,n);}
};
} // namespace p2_nav
