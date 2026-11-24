from pathlib import Path
import importlib.util,json,time,re
BASE=Path('/home/frc-orin/V17-FarmBot21SEP2026-CurrentPersistence');ROOT=Path('/home/frc-orin/V18-FarmBot21SEP2026-ActuatorRecovery')
spec=importlib.util.spec_from_file_location('transport',BASE/'evidence/v17-persistence/v17_persistence_host.py');h=importlib.util.module_from_spec(spec);spec.loader.exec_module(h)
h.ALLOW={'unit','firmware','act safety','act stop','act diag','act limits','status','act pulse m1 rev','stop'}
out=ROOT/'evidence'/('act3-retract-pulse-'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime()));out.mkdir(parents=True)
row=next(r for r in json.loads((BASE/'FIRMWARE_MAP.json').read_text()) if r['arm']==3)
rec={'arm':3,'steps':[],'motion_attempted':False,'native_pulse_ms':250,'power_ceiling_127':20,'user_authorization':'Do what testing you need to get it working; confirmed supported, actuator travel clear, hands clear, at cutoff, GUIs closed.','limit_writes':False};link=None
h.unowned()
def save():(out/'result.json').write_text(json.dumps(rec,indent=2)+'\n')
try:
 link=h.Link(row,rec,save)
 h.require('unit id: 3, local ODrive CAN node id: 0' in link.cmd('unit',1),'Wrong unit')
 h.require('unit=3; revision=2026-09-21-v17-current-persistence' in link.cmd('firmware',1),'Unexpected firmware')
 result=link.cmd('act safety',1);h.require('enabled (200 ms)' in result,'200ms serial watchdog not acknowledged')
 result=link.cmd('act stop',1.5);h.require('Actuator zero commands acknowledged.' in result,'Stop unconfirmed')
 diag=link.cmd('act diag',2);h.actuator_idle(diag);rec['before_diag']=diag
 limits=link.cmd('act limits',1.5);f=h.actuator_fields(limits);h.require(670<=f['1']['maximum_raw']<=675,'Current cap not verified')
 before=link.cmd('status',1.5);h.require('limit REV/IN pin 5:  open' in before,'Retract limit not open')
 rec['before_encoder']=int(re.search(r'actuator encoder: (-?\d+)',before).group(1));save()
 print('ARM3 actuator preflight: zero PWM/current, reverse limit open, 6.74 A cap, 200ms deadman.',flush=True)
 rec['motion_attempted']=True;save()
 result=link.cmd('act pulse m1 rev',2.0);print(result,flush=True);rec['pulse_response']=result
 h.require('Actuator pulse complete.' in result,'Pulse completion not confirmed; do not retry')
 result=link.cmd('act stop',1.5);h.require('Actuator zero commands acknowledged.' in result,'Post-test Stop not acknowledged')
 diag=link.cmd('act diag',2);h.actuator_idle(diag);rec['after_diag']=diag;print(diag,flush=True)
 after=link.cmd('status',1.5);print(after,flush=True)
 rec['after_encoder']=int(re.search(r'actuator encoder: (-?\d+)',after).group(1));rec['encoder_delta']=rec['after_encoder']-rec['before_encoder'];rec['result']='Completed; fresh zero PWM/M1 current confirmed'
except Exception as ex:
 rec['error']=str(ex);print('TEST STOPPED:',ex,flush=True)
 if link and rec['motion_attempted']:
  try:print(link.cmd('stop',3),flush=True)
  except Exception as stop_ex:rec['emergency_stop_error']=str(stop_ex);print('Stop confirmation failed; use physical cutoff.',flush=True)
finally:
 if link:link.s.close();(out/'serial.txt').write_text('\n'.join(link.raw))
 save();(out/'runner.py').write_text(Path(__file__).read_text());print('RESULT',json.dumps({k:rec.get(k) for k in ['result','motion_attempted','encoder_delta','error']}),flush=True);print('EVIDENCE',out,flush=True)
