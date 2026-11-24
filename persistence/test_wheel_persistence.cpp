#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include "wheel_persistence_policy.h"
#include "wheel_current_limits.h"

namespace ODriveAxisState { constexpr int AXIS_STATE_IDLE=1; }
struct Get_Error_msg_t { uint32_t Active_Errors=0,Disarm_Reason=0; };
struct Get_Version_msg_t {
 uint8_t Protocol_Version=2,Hw_Version_Major=5,Hw_Version_Minor=2,Hw_Version_Variant=0;
 uint8_t Fw_Version_Major=0,Fw_Version_Minor=6,Fw_Version_Revision=11,Fw_Version_Unreleased=0;
};
struct FakeSerial {void println(const char*) {} void printf(const char*,...) {}} Serial;
struct Beat {int Axis_State=1;uint32_t Axis_Error=0;};
struct Data {uint32_t heartbeat_sequence=1;Beat last_heartbeat; } odrive_data[1];
constexpr int NUM_LOCAL_ODRIVES=1,LOCAL_ODRIVE_NODE_ID=0;
uint32_t tick=0;
uint32_t millis(){return tick;}
void delay(unsigned n){tick+=n;}
struct Env {
 bool cancelled=false,drainOk=true,sendOk=true,respondBool=true,heartbeat=true;
 bool versionOk=true,currentOk=true,errorsOk=true,saveOk=true;
 bool cancelInDrain=false,cancelInPump=false;
 uint32_t active=0,disarm=0,commit=0xb3c4cf45;
 float soft=35,hard=50;
 unsigned saves=0,queries=0;
 uint8_t flags[5]={0,0,0,0,0};
 uint16_t pendingBool=0;
 Get_Version_msg_t version;
} e;
wheel_persistence::BoolSdo wheel_persistence_bool_sdo;
void pumpCanOnce(){
 tick+=1;
 if(e.cancelInPump)e.cancelled=true;
 if(e.heartbeat)++odrive_data[0].heartbeat_sequence;
 if(e.pendingBool && e.respondBool){
  auto ep=e.pendingBool;e.pendingBool=0;
  auto first=wheel_persistence::startupFirst(e.version.Fw_Version_Revision);
  uint8_t data[5]={0,uint8_t(ep),uint8_t(ep>>8),0,e.flags[ep-first]};
  wheel_persistence_bool_sdo.observe(5,5,data);
 }
}
struct WheelCurrentSdoIO {
 int index;
 bool cancelled()const{return e.cancelled;}
 bool drain()const{if(e.cancelInDrain)e.cancelled=true;return e.drainOk;}
 bool send(uint32_t id,uint8_t len,const uint8_t* data)const{
  assert(id==4 && len==4 && data[0]==0);++e.queries;
  e.pendingBool=uint16_t(data[1])|uint16_t(data[2])<<8;return e.sendOk;
 }
 uint32_t now()const{return millis();}
 void pump()const{pumpCanOnce();}
 void pause()const{delay(1);}
};
struct FakeCurrent {
 bool readU32(WheelCurrentSdoIO& io,uint8_t,uint16_t ep,uint32_t& v){assert(ep==13);v=e.commit;return e.currentOk&&!io.cancelled();}
 bool readFloat(WheelCurrentSdoIO& io,uint8_t,uint16_t ep,float& v){v=(ep==305||ep==325)?e.soft:e.hard;return e.currentOk&&!io.cancelled();}
} wheel_current_sdo;
struct ODriveCAN {
 enum ResetAction {SaveConfiguration=1};
 bool getError(Get_Error_msg_t& msg,int){msg.Active_Errors=e.active;msg.Disarm_Reason=e.disarm;return e.errorsOk;}
 bool getVersion(Get_Version_msg_t& msg,int){msg=e.version;return e.versionOk;}
 bool reset(ResetAction action){assert(action==SaveConfiguration);++e.saves;return e.saveOk;}
} drive;
ODriveCAN* odrives[1]={&drive};
#include "wheel_persistence_runtime.h"
unsigned checks=0;
#define CHECK(x) do{++checks;assert(x);}while(0)
void reset(){e=Env{};tick=0;odrive_data[0]=Data{};wheel_persistence_bool_sdo.cancel();}
void blocked(){CHECK(!saveWheelCurrentLimits(0));CHECK(e.saves==0);}
int main(){
 reset();CHECK(saveWheelCurrentLimits(0));CHECK(e.saves==1);CHECK(e.queries==5);
 reset();e.version.Fw_Version_Revision=12;e.commit=0xd92a33fe;CHECK(saveWheelCurrentLimits(0));CHECK(e.saves==1);
 reset();e.commit=0;blocked();
 reset();e.version.Fw_Version_Revision=10;blocked();
 reset();e.version.Hw_Version_Major=4;blocked();
 reset();e.versionOk=false;blocked();
 reset();e.currentOk=false;blocked();
 reset();e.soft=65;blocked();
 reset();e.hard=38;blocked();
 reset();e.hard=85;blocked();
 reset();e.soft=NAN;blocked();
 reset();e.hard=INFINITY;blocked();
 for(unsigned i=0;i<5;++i){reset();e.flags[i]=1;blocked();}
 reset();e.respondBool=false;blocked();CHECK(tick<1200);
 reset();e.flags[0]=2;blocked();
 reset();e.active=1;blocked();
 reset();e.disarm=0x4000000;blocked();
 reset();e.errorsOk=false;blocked();
 reset();e.cancelled=true;blocked();
 reset();e.cancelInDrain=true;blocked();CHECK(e.queries==0);
 reset();e.cancelInPump=true;blocked();
 reset();e.drainOk=false;blocked();
 reset();e.sendOk=false;blocked();
 reset();e.heartbeat=false;blocked();CHECK(tick<=1002);
 reset();odrive_data[0].last_heartbeat.Axis_State=8;blocked();
 reset();e.saveOk=false;CHECK(!saveWheelCurrentLimits(0));CHECK(e.saves==1);
 reset();CHECK(!saveWheelCurrentLimits(-1));CHECK(e.saves==0);
 // Strict matching: wrong node/endpoint/length and malformed bool are ignored.
 wheel_persistence::BoolSdo s;s.waiting=true;s.endpoint=219;
 uint8_t b[8]={0,219,0,0,0,0,0,0};
 s.observe(37,5,b);CHECK(!s.received);
 s.observe(5,4,b);CHECK(!s.received);
 s.observe(5,8,b);CHECK(!s.received);
 b[1]=218;s.observe(5,5,b);CHECK(!s.received);
 b[1]=219;b[4]=2;s.observe(5,5,b);CHECK(!s.received);
 b[4]=0;s.observe(5,5,b);CHECK(s.received&&s.value==0);
 s.cancel();s.observe(5,5,b);CHECK(!s.received);
 CHECK(wheel_persistence::startupFirst(11)==219);CHECK(wheel_persistence::startupFirst(12)==238);CHECK(wheel_persistence::startupFirst(13)==0);
 printf("wheel persistence: %u checks passed\n",checks);
}
