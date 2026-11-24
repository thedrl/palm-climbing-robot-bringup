"""Compile the actual stop functions with a fake RoboClaw; no hardware access."""
from pathlib import Path
import subprocess
import tempfile

source = (Path(__file__).parent / "src/bringup.cpp").read_text()
drive = source.split("bool actuatorDriveChannelOpenLoop(uint8_t channel, int power) {", 1)[1]
drive = "bool actuatorDriveChannelOpenLoop(uint8_t channel, int power) {" + drive.split("\nbool actuatorDriveOpenLoop(int power)", 1)[0]
stop = source.split("void actuatorStop() {", 1)[1]
stop = "void actuatorStop() {" + stop.split("\nvoid resetRoboClawSerialLink", 1)[0]
prefix = r'''
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>
struct Call {int channel, mode, power;};
struct Fake {
    std::vector<Call> calls;
    bool fwd=true, rev=true;
    bool ForwardM1(int, int p) {calls.push_back({1,1,p}); return fwd;}
    bool BackwardM1(int, int p) {calls.push_back({1,-1,p}); return rev;}
    bool ForwardM2(int, int p) {calls.push_back({2,1,p}); return fwd;}
    bool BackwardM2(int, int p) {calls.push_back({2,-1,p}); return rev;}
    bool SpeedM1(int, int p) {calls.push_back({1,2,p}); return true;}
    bool SpeedM2(int, int p) {calls.push_back({2,2,p}); return true;}
} roboclaw;
int roboclaw_addr=128;
uint8_t clampActuatorPower(int p) {return std::min(std::abs(p),127);}
'''
suffix = r'''
int main() {
  int scenarios=0;
  for(int channel: {1,2}) for(int direction: {-40,40}) {
    roboclaw.calls.clear(); roboclaw.fwd=roboclaw.rev=true;
    assert(actuatorDriveChannelOpenLoop(channel,direction));
    assert(roboclaw.calls.size()==1);
    assert(roboclaw.calls[0].power==40);
    assert(roboclaw.calls[0].mode==(direction>0?1:-1));
    roboclaw.calls.clear();
    assert(actuatorDriveChannelOpenLoop(channel,0));
    assert(roboclaw.calls.size()==(UNIT_ID==1?2:3));
    for(auto c:roboclaw.calls) {
      assert(c.channel==channel && c.power==0);
      if(UNIT_ID==1) assert(c.mode!=2);
    }
    ++scenarios;
  }
  for(int channel: {1,2}) for(bool f: {false,true}) for(bool r: {false,true}) {
    roboclaw.fwd=f; roboclaw.rev=r;
    assert(actuatorDriveChannelOpenLoop(channel,0)==(f||r));
    ++scenarios;
  }
  roboclaw.calls.clear(); roboclaw.fwd=roboclaw.rev=true;
  actuatorStop();
  assert(roboclaw.calls.size()==(UNIT_ID==1?4:6));
  bool seen1=false, seen2=false;
  for(auto c:roboclaw.calls) {
    assert(c.power==0);
    if(UNIT_ID==1) assert(c.mode!=2);
    seen1|=c.channel==1; seen2|=c.channel==2;
  }
  assert(seen1 && seen2);
  std::cout << "unit " << UNIT_ID << ": " << scenarios+1 << " scenarios passed\n";
}
'''
adapter = "\nbool actuatorDriveChannelOpenLoopWithRetry(uint8_t c,int p) {return actuatorDriveChannelOpenLoop(c,p);}\n"
with tempfile.TemporaryDirectory() as directory:
    cpp = Path(directory) / "stop.cpp"
    cpp.write_text(prefix + drive + adapter + stop + suffix)
    for unit in (0,1,2,3):
        binary = Path(directory) / f"stop{unit}"
        subprocess.run(["c++", "-std=c++17", f"-DUNIT_ID={unit}", str(cpp), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
