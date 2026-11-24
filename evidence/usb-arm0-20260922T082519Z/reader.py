"""Capture ODrive scalar properties. Sole wire command: ASCII r property."""
import json,os,re,time,datetime,sys
from pathlib import Path
import serial
ROOT=Path('/home/frc-orin/V17-FarmBot21SEP2026-CurrentPersistence')
E=json.loads((ROOT/'persistence/endpoints-0.6.11.json').read_text())['endpoints']
BASE=['serial_number','commit_hash','fw_version_major','fw_version_minor','fw_version_revision','fw_version_unreleased','hw_version_major','hw_version_minor','hw_version_variant','axis0.current_state','axis0.active_errors','axis0.disarm_reason','axis0.procedure_result','vbus_voltage','ibus','axis0.pos_estimate','axis0.vel_estimate','axis0.config.load_encoder','axis0.config.commutation_encoder','axis0.config.commutation_encoder_bandwidth']
PREFIXES=('axis0.config.motor.','axis0.controller.config.','axis0.commutation_mapper.config.','axis0.pos_vel_mapper.config.','rs485_encoder_group0.')
KEYS=list(dict.fromkeys(BASE+[k for k,v in E.items() if k.startswith(PREFIXES) and v['type']!='function']))
KEYS += [k for k in ['axis0.motor.foc.p_gain','axis0.motor.foc.i_gain','axis0.motor.foc.Iq_measured','axis0.motor.foc.Iq_setpoint','axis0.motor.fet_thermistor.temperature','axis0.motor.motor_thermistor.temperature','axis0.controller.input_vel','axis0.controller.input_torque','axis0.pos_vel_mapper.status','axis0.commutation_mapper.status','axis0.pos_vel_mapper.working_offset','axis0.commutation_mapper.working_offset','config.inverter0.current_soft_max','config.inverter0.current_hard_max'] if k not in KEYS]
KEYS=[k for k in KEYS if k in E]
assert all(re.fullmatch(r'[A-Za-z0-9_]+(?:\.[A-Za-z0-9_]+)*',k) for k in KEYS)
def run(expected_serial,label):
 assert re.fullmatch('[0-9A-F]{12}',expected_serial)
 assert label in ['arm0','comparison']
 port=Path('/dev/serial/by-id')/f'usb-ODrive_Robotics_ODrive_S1_{expected_serial}-if00'
 assert port.exists(),'Expected ODrive absent'
 for proc in Path('/proc').glob('[0-9]*'):
  if int(proc.name)==os.getpid():continue
  try:
   cmd=(proc/'cmdline').read_bytes()
   assert b'climber_gui5.py' not in cmd,'Robot GUI open'
   for fd in (proc/'fd').iterdir():
    try:target=os.readlink(fd)
    except OSError:continue
    assert target!=str(port.resolve()),'ODrive serial port already owned'
  except (PermissionError,FileNotFoundError):pass
 out=ROOT/'evidence'/('usb-'+label+'-'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime()))
 out.mkdir(exist_ok=False)
 records=[];result={'port':str(port),'expected_serial':expected_serial,'read_only':True,'records':records}
 def checkpoint():(out/'snapshot.json').write_text(json.dumps(result,indent=2)+'\n')
 link=serial.Serial(port=None,baudrate=115200,timeout=1,write_timeout=1,exclusive=True);link.dtr=False;link.rts=False;link.port=str(port)
 def read(k):
  assert k in KEYS
  rec={'property':k,'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'wire_command':'r '+k};records.append(rec);checkpoint()
  link.write(('r '+k+'\n').encode('ascii'));link.flush();reply=link.readline()
  rec['reply']=reply.decode('ascii',errors='replace').strip();checkpoint()
  assert reply.endswith(b'\n'),'Incomplete reply; stop before issuing another request'
  assert not link.in_waiting,'Unexpected extra response'
  time.sleep(.025);return rec['reply']
 try:
  link.open();time.sleep(.1);assert not link.read_all(),'Unexpected existing serial output'
  assert int(read('serial_number'))==int(expected_serial,16),'Identity mismatch'
  state=read('axis0.current_state');result['initial_state']=state
  assert state=='1','ODrive not IDLE; report state before more inspection'
  for k in KEYS:
   if k not in ['serial_number','axis0.current_state']:read(k)
  result['status']='COMPLETE READ-ONLY SNAPSHOT'
 except Exception as ex:result.update(status='INCOMPLETE',error=str(ex));raise
 finally:
  link.close();checkpoint();(out/'reader.py').write_text(Path(__file__).read_text());print('EVIDENCE',out,flush=True)
 values={r['property']:r['reply'] for r in records}
 print(json.dumps(values,indent=2))
if __name__=='__main__':run(sys.argv[1],sys.argv[2])
