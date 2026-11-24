"""Prepared single-board upload. Default prints the plan; --run requires actual user authorization."""
import argparse,hashlib,importlib.util,json,subprocess,time
from pathlib import Path
ROOT=Path('/home/frc-orin/V17-FarmBot21SEP2026-CurrentPersistence')
NEW='48CA432DF684'; OLD='744DBDA10314'
IMAGE=ROOT/'units/arm1/firmware_images/bringup_unit1_v17_current_persistence.bin'
SHA='7085533e3073678a2dcc8c08d9efa2434a9b316c3a5c0979e6c2837cba085742'
MARK='2026-09-21-v17-current-persistence'
UP=Path('/home/frc-orin/.platformio/packages/tool-dfuutil-arduino/dfu-util')
PLAN={'unit':1,'new_usb_serial':NEW,'previous_usb_serial':OLD,'firmware':'V17 Explicit Current Persistence','image':str(IMAGE),'sha256':SHA,'upload_count':1,'motor_commands':False,'controller_configuration_saves':False,'verify':['unit','firmware','can status'],'other_units_changed':False}
def require(ok,msg):
 if not ok:raise RuntimeError(msg)
def run(authfile):
 auth=json.loads(Path(authfile).read_text())
 for k in ('approved_unit1_flash','replacement_is_unit1','supported','unit1_wheel_and_actuator_power_off','usb_connected'):
  require(auth.get(k) is True,'Missing actual confirmation: '+k)
 require(auth.get('user_message'),'Actual user confirmation required')
 require(auth.get('plan')==PLAN,'Authorization plan mismatch')
 data=IMAGE.read_bytes();require(hashlib.sha256(data).hexdigest()==SHA and MARK.encode() in data,'Image mismatch')
 spec=importlib.util.spec_from_file_location('h',ROOT/'evidence/v17-persistence/v17_persistence_host.py');h=importlib.util.module_from_spec(spec);spec.loader.exec_module(h)
 h.ALLOW={'unit','firmware','can status'};h.unowned()
 probe=subprocess.run([str(UP),'-d','2341:0070','-S',NEW,'-l'],capture_output=True,text=True,timeout=10)
 require(probe.returncode==0 and probe.stdout.count('serial="'+NEW+'"')==1,'Target is not uniquely selectable')
 out=Path('/home/frc-orin/V18-FarmBot21SEP2026-ActuatorRecovery/evidence')/('replacement-unit1-'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime()));out.mkdir(exist_ok=False)
 rec={'plan':PLAN,'authorization':auth,'steps':[],'upload_attempted':False,'runtime_verified':False,'nonzero_commands_sent':False,'nvm_saves_sent':False}
 def save():(out/'result.json').write_text(json.dumps(rec,indent=2)+'\n')
 (out/'runner.py').write_text(Path(__file__).read_text());save()
 link=None
 try:
  rec['upload_attempted']=True;rec['started_epoch']=time.time();save()
  print('Uploading verified V17 Unit 1 image to '+NEW,flush=True)
  with (out/'upload.log').open('w') as log:
   result=subprocess.run([str(UP),'-d','2341:0070','-S',NEW,'-Q','-D',str(IMAGE)],stdout=log,stderr=subprocess.STDOUT,timeout=60)
  rec['upload_exit_code']=result.returncode;save();require(result.returncode==0,'Upload did not report success; inspect log before retry')
  time.sleep(2.5)
  port=Path('/dev/serial/by-id')/('usb-Arduino_NanoESP32_'+NEW+'-if01')
  end=time.monotonic()+15
  while not port.exists() and time.monotonic()<end:time.sleep(.25)
  require(port.exists(),'Upload completed, but runtime USB did not return')
  link=h.Link({'usb_serial':NEW},rec,save)
  unit=link.cmd('unit',2)
  require('unit id: 1, local ODrive CAN node id: 0' in unit,'Expected Unit 1 identity absent')
  firmware=link.cmd('firmware',1.5)
  require('Firmware: V17 Explicit Current Persistence; unit=1; revision='+MARK in firmware,'Firmware revision not verified')
  rec['can_status']=link.cmd('can status',1.5)
  rec['runtime_verified']=True
  rec['status']='V17 Unit 1 uploaded and runtime identity verified; powered CAN readback pending'
  (out/'replacement-identity.json').write_text(json.dumps({'unit':1,'usb_serial':NEW,'replaced_usb_serial':OLD,'firmware':'V17','image_sha256':SHA,'verified':True},indent=2)+'\n')
 except Exception as exc:
  rec['error']=str(exc);rec['status']='Review evidence; no automatic flash retry';raise
 finally:
  if link:link.s.close();(out/'runtime.txt').write_text('\n'.join(link.raw)+'\n'+link.pending.decode(errors='replace'))
  rec['completed_epoch']=time.time();save();print(json.dumps({k:rec.get(k) for k in ('status','upload_exit_code','runtime_verified','error')},indent=2),flush=True);print('EVIDENCE',out,flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--authorization');a=p.parse_args()
 if not a.run:print(json.dumps(PLAN,indent=2))
 else:
  require(a.authorization,'Explicit authorization record required');run(a.authorization)
