#include "lgh_st3215_maintenance/common.hpp"
#include "lgh_st3215_driver/joint_map.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace lgh_st3215_driver;
using namespace lgh_st3215_maintenance;
namespace fs = std::filesystem;

std::string utcStamp(){ const auto now=std::chrono::system_clock::now(); const std::time_t tt=std::chrono::system_clock::to_time_t(now); std::tm tm{}; gmtime_r(&tt,&tm); std::ostringstream s; s<<std::put_time(&tm,"%Y%m%dT%H%M%SZ"); return s.str(); }

int main(int argc,char** argv){
  std::string port_name="/dev/ttyS3", map_path=ament_index_cpp::get_package_share_directory("lgh_st3215_driver")+"/config/servo_map.yaml";
  fs::path output_root=fs::path(std::getenv("HOME")?std::getenv("HOME"):".")/".ros"/"lgh_st3215_backups";
  int baud=1000000,address=0,length=0x47,timeout_ms=15;
  try{
    for(int i=1;i<argc;++i){std::string arg=argv[i];auto next=[&](){if(++i>=argc)throw std::invalid_argument("missing value for "+arg);return std::string(argv[i]);};
      if(arg=="--port")port_name=next();else if(arg=="--baud")baud=parseInt(next());else if(arg=="--servo-map")map_path=next();else if(arg=="--output-root")output_root=next();else if(arg=="--address")address=parseInt(next());else if(arg=="--length")length=parseInt(next());else if(arg=="--timeout-ms")timeout_ms=parseInt(next());else if(arg=="--help"){std::cout<<"backup_control_tables [--servo-map path] [--output-root dir]\n";return kPass;}else throw std::invalid_argument("unknown argument: "+arg);}
    if(address<0||length<=0||address+length>256)throw std::invalid_argument("invalid address/length");
    const JointMap map=JointMap::loadFromYaml(map_path,0,0); SerialPort serial; serial.openPort(port_name,baud);
    YAML::Node root; root["schema_version"]=1; root["port"]=port_name; root["baud"]=baud; root["address"]=address; root["length"]=length; bool all_ok=true;
    for(const auto& joint:map.joints()){
      FrameReadResult frame;std::string error;YAML::Node entry;entry["joint"]=joint.name;entry["id"]=static_cast<int>(joint.servo_id);
      if(transact(serial,buildReadRequest(joint.servo_id,address,length),joint.servo_id,5,timeout_ms,frame,error)){
        std::uint8_t status=0;std::vector<std::uint8_t> data;if(parseReadReply(frame.frame,joint.servo_id,length,status,data,error)){entry["ok"]=(status==0);entry["status_error"]=static_cast<int>(status);entry["data_hex"]=hexBytes(data);if(status!=0)all_ok=false;}else{entry["ok"]=false;entry["error"]=error;all_ok=false;}
      }else{entry["ok"]=false;entry["error"]=error;all_ok=false;}
      root["servos"].push_back(entry); std::cout<<(entry["ok"].as<bool>()?"PASS ":"FAIL ")<<joint.name<<" id="<<static_cast<int>(joint.servo_id)<<"\n";
    }
    fs::create_directories(output_root);const fs::path target=output_root/(utcStamp()+"_st3215_control_tables.yaml");std::ofstream file(target);file<<root;
    std::cout<<"ST3215 CONTROL-TABLE BACKUP: "<<(all_ok?"PASS":"PARTIAL")<<"\n"<<target<<"\n";return all_ok?kPass:kTestFail;
  }catch(const std::invalid_argument&e){std::cerr<<"CONFIG ERROR: "<<e.what()<<"\n";return kConfig;}catch(const std::exception&e){std::cerr<<"CONTROL-TABLE BACKUP REFUSED/FAILED: "<<e.what()<<"\n";return exceptionExit(e);}
}
