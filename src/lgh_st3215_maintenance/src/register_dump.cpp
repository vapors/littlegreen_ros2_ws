#include "lgh_st3215_maintenance/common.hpp"
#include <fstream>
#include <iostream>
#include <string>

using namespace lgh_st3215_driver;
using namespace lgh_st3215_maintenance;

int main(int argc, char** argv) {
  std::string port_name = "/dev/ttyS3", output;
  int baud=1000000, id=-1, address=-1, length=-1, timeout_ms=10;
  try {
    for (int i=1;i<argc;++i) {
      std::string arg=argv[i]; auto next=[&](){ if(++i>=argc) throw std::invalid_argument("missing value for "+arg); return std::string(argv[i]);};
      if(arg=="--port") port_name=next(); else if(arg=="--baud") baud=parseInt(next());
      else if(arg=="--id") id=parseInt(next()); else if(arg=="--address") address=parseInt(next());
      else if(arg=="--length") length=parseInt(next()); else if(arg=="--timeout-ms") timeout_ms=parseInt(next());
      else if(arg=="--output") output=next(); else if(arg=="--help") { std::cout<<"register_dump --id N --address 0x00 --length N [--output dump.yaml]\n"; return kPass; }
      else throw std::invalid_argument("unknown argument: "+arg);
    }
    if(id<0||id>253||address<0||address>255||length<=0||length>253||address+length>256) throw std::invalid_argument("invalid id/address/length");
    SerialPort serial; serial.openPort(port_name, baud);
    FrameReadResult frame; std::string error;
    if(!transact(serial, buildReadRequest(id,address,length), id, 5, timeout_ms, frame, error)) { std::cerr<<"READ FAILED: "<<error<<"\n"; return error=="timeout"?kTimeout:kHardware; }
    std::uint8_t status=0; std::vector<std::uint8_t> data;
    if(!parseReadReply(frame.frame,id,length,status,data,error)) { std::cerr<<"PARSE FAILED: "<<error<<"\n"; return kHardware; }
    std::cout<<"ST3215 REGISTER DUMP: PASS id="<<id<<" address=0x"<<std::hex<<address<<std::dec<<" length="<<length<<" status_error="<<static_cast<unsigned int>(status)<<"\n";
    std::cout<<hexBytes(data)<<"\n";
    if(!output.empty()) { std::ofstream f(output); f<<"schema_version: 1\nid: "<<id<<"\naddress: "<<address<<"\nlength: "<<length<<"\nstatus_error: "<<static_cast<unsigned int>(status)<<"\ndata_hex: \""<<hexBytes(data)<<"\"\n"; }
    return status == 0 ? kPass : kTestFail;
  } catch(const std::invalid_argument& e){ std::cerr<<"CONFIG ERROR: "<<e.what()<<"\n"; return kConfig; }
    catch(const std::exception& e){ std::cerr<<"REGISTER DUMP REFUSED/FAILED: "<<e.what()<<"\n"; return exceptionExit(e); }
}
