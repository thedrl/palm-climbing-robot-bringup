"""Prepared diagnostic: one arm0 motion trial only after fresh explicit consent.
Default invocation prints the plan and never imports serial or opens a device.
"""
import argparse,datetime,json,math,os,re,time
from pathlib import Path
SERIAL='003CF6749599'
PORT=Path('/dev/serial/by-id')/f'usb-ODrive_Robotics_ODrive_S1_{SERIAL}-if00'
ROOT=Path('/home/frc-orin/V18-FarmBot21SEP2026-ActuatorRecovery/evidence')
DURATION=1.0
TARGET=5.0/60.0
SOFT_LIMIT=3.0
ABORT_CURRENT=4.0
ABORT_RPM=15.0
PLAN={'arm':0,'serial':SERIAL,'rpm':5,'nominal_closed_loop_seconds':DURATION,'temporary_soft_limit_a':SOFT_LIMIT,'hard_trip_a_unchanged':38,'acceleration_rpm_per_sec':10,'one_explicit_fault_clear_before_test':False,'retries':0,'watchdog_seconds':0.5,'abort_iq_a':ABORT_CURRENT,'abort_abs_rpm':ABORT_RPM,'abort_reverse_rpm':-2,'save_configuration':False,'cleanup':'zero and IDLE, verify, then restore temporary config; preserve any new fault'}
TEMP={'axis0.config.motor.current_soft_max':3.0,'axis0.controller.config.control_mode':2,'axis0.controller.config.input_mode':2,'axis0.controller.config.vel_ramp_rate':1/6,'axis0.controller.config.vel_limit':0.25,'axis0.config.watchdog_timeout':0.5,'axis0.config.enable_watchdog':1}
SNAP=['serial_number','commit_hash','axis0.current_state','axis0.active_errors','axis0.disarm_reason','axis0.pos_estimate','axis0.vel_estimate','axis0.controller.input_vel','axis0.controller.input_torque','axis0.config.motor.current_hard_max','axis0.controller.config.enable_overspeed_error','axis0.controller.config.spinout_mechanical_power_threshold','axis0.controller.config.spinout_electrical_power_threshold']+list(TEMP)
TRACE=['axis0.current_state','axis0.active_errors','axis0.disarm_reason','axis0.controller.input_vel','axis0.pos_estimate','axis0.vel_estimate','axis0.motor.foc.Iq_setpoint','axis0.motor.foc.Iq_measured','axis0.motor.electrical_power','axis0.motor.mechanical_power','vbus_voltage']
READ_ALLOWED=set(SNAP+TRACE)
WRITE_ALLOWED=set(TEMP)|{'axis0.controller.input_vel','axis0.controller.input_torque','axis0.requested_state'}
def require(x,msg):
 if not x:raise RuntimeError(msg)
def movement_abort(r,baseline):
 if int(r['axis0.current_state'])!=8:return 'left CLOSED_LOOP'
 if int(r['axis0.active_errors']) or int(r['axis0.disarm_reason']):return 'ODrive fault'
 for k in ['axis0.pos_estimate','axis0.vel_estimate','axis0.motor.foc.Iq_setpoint','axis0.motor.foc.Iq_measured','vbus_voltage']:
  if not math.isfinite(r[k]):return 'invalid feedback '+k
 rpm=r['axis0.vel_estimate']*60
 if abs(rpm)>ABORT_RPM:return 'overspeed'
 if rpm < -2:return 'unexpected reverse encoder motion'
 if abs(r['axis0.motor.foc.Iq_measured'])>ABORT_CURRENT:return 'test current threshold'
 if abs(r['axis0.pos_estimate']-baseline)>.1:return 'unexpected displacement'
 return None

def run(authfile):
 import serial
 auth=json.loads(Path(authfile).read_text())
 require(0<=time.time()-auth['confirmed_epoch']<300,'Need fresh actual user approval')
 for k in ['approved_one_test','supported','wheel_free','hands_clear','at_cutoff','gui_closed']:
  require(auth.get(k) is True,'Missing confirmation: '+k)
 require(auth.get('user_message') and auth.get('plan')==PLAN,'Authorization must name this exact test plan')
 require(PORT.exists(),'Expected arm0 ODrive is absent')
 usb_nodes=set()
 for device in Path('/sys/bus/usb/devices').iterdir():
  try:
   if (device/'serial').read_text().strip()==SERIAL:
    usb_nodes.add('/dev/bus/usb/%03d/%03d'%(int((device/'busnum').read_text()),int((device/'devnum').read_text())))
  except (OSError,ValueError):pass
 require(len(usb_nodes)==1,'Could not uniquely identify arm0 USB bus device')
 for proc in Path('/proc').glob('[0-9]*'):
  if int(proc.name)==os.getpid():continue
  try:
   require(b'climber_gui5.py' not in (proc/'cmdline').read_bytes(),'Close robot GUI')
   for fd in (proc/'fd').iterdir():
    try:target=os.readlink(fd)
    except OSError:continue
    require(not target.startswith(('/dev/ttyACM','/dev/ttyUSB')) and target not in usb_nodes,'Another process owns robot serial/USB: '+proc.name)
  except (FileNotFoundError,PermissionError):pass
 out=ROOT/('arm0-low-current-test-'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime()));out.mkdir(exist_ok=False)
 rec={'plan':PLAN,'authorization':auth,'before':{},'writes':[],'samples':[],'read_log':[],'settings_mutated':False,'motion_attempted':False,'nvm_save':False,'clear_count':0}
 def checkpoint():(out/'result.json').write_text(json.dumps(rec,indent=2)+'\n')
 s=serial.Serial(port=None,baudrate=115200,timeout=.045,write_timeout=.045,exclusive=True);s.dtr=False;s.rts=False;s.port=str(PORT)
 moving_deadline=None
 stream_uncertain=False
 def deadline_check():
  if moving_deadline is not None:require(time.monotonic()<moving_deadline,'Commanded test duration reached')
 def send(data):
  nonlocal stream_uncertain
  deadline_check()
  encoded=data.encode('ascii')
  try:
   require(s.write(encoded)==len(encoded),'Partial serial write')
  except Exception:
   stream_uncertain=True;raise
 def read(k):
  nonlocal stream_uncertain
  require(k in READ_ALLOWED,'Read not allowed')
  start=time.monotonic();send('r '+k+'\n')
  try:b=s.readline()
  except Exception:
   stream_uncertain=True;raise
  item={'property':k,'start_monotonic':start,'end_monotonic':time.monotonic(),'reply':b.decode(errors='replace').strip()};rec['read_log'].append(item)
  try: pending=s.in_waiting
  except Exception:
   stream_uncertain=True;raise
  if not b.endswith(b'\n') or pending:
   stream_uncertain=True;raise RuntimeError('Incomplete or ambiguous response: '+k)
  try:value=float(b.decode().strip())
  except Exception:
   stream_uncertain=True;raise
  deadline_check()
  return value
 def write(k,v):
  require(k in WRITE_ALLOWED and math.isfinite(float(v)),'Write not allowed')
  if k=='axis0.requested_state':require(float(v) in (1,8),'Only IDLE or CLOSED_LOOP permitted')
  if k=='axis0.controller.input_vel':require(float(v) in (0,TARGET),'Only zero or5RPM permitted')
  if k=='axis0.controller.input_torque':require(float(v)==0,'Only zero torque feedforward permitted')
  rec['writes'].append({'property':k,'value':v,'utc':datetime.datetime.now(datetime.timezone.utc).isoformat()})
  send('w '+k+' '+format(v,'.9g')+'\n')
 def feed():send('u 0\n')
 def verify(k,v):
  got=read(k);require(math.isclose(got,float(v),rel_tol=1e-5,abs_tol=1e-5),'Readback mismatch '+k)
 try:
  s.open();time.sleep(.1);require(not s.read_all(),'Unexpected pending output')
  require(int(read('serial_number'))==int(SERIAL,16),'Identity mismatch')
  before={k:read(k) for k in SNAP};rec['before']=before;checkpoint()
  require(int(before['commit_hash'])==0xb3c4cf45,'Unexpected firmware build')
  require(before['axis0.current_state']==1,'Expected IDLE')
  require(abs(before['axis0.vel_estimate']*60)<2,'Not stationary')
  require(before['axis0.controller.input_vel']==0 and before['axis0.controller.input_torque']==0,'Nonzero retained input')
  require(before['axis0.active_errors']==0 and before['axis0.disarm_reason']==0,'Unexpected fault; preserve and review')
  require(before['axis0.config.motor.current_soft_max']==35 and before['axis0.config.motor.current_hard_max']==38,'Unexpected current limits')
  require(before['axis0.controller.config.enable_overspeed_error']==1,'Overspeed protection must remain enabled')
  require(before['axis0.controller.config.spinout_mechanical_power_threshold']==-10 and before['axis0.controller.config.spinout_electrical_power_threshold']==10,'Unexpected spinout protection')
  require(before['axis0.config.enable_watchdog']==0,'Unexpected existing watchdog configuration')
  for k,v in TEMP.items():
   # Seed the native watchdog after configuring its timeout, before enabling.
   if k=='axis0.config.enable_watchdog': feed()
   rec['settings_mutated']=True;write(k,v);verify(k,v)
   if k=='axis0.config.enable_watchdog': feed()
   checkpoint()
  # The previously recorded latch was already cleared in stationary recovery.
  # Preserve any new fault instead of clearing it for this test.
  require(read('axis0.active_errors')==0 and read('axis0.disarm_reason')==0,'Fault before test; no arming')
  verify('axis0.config.motor.current_hard_max',38)
  baseline=read('axis0.pos_estimate')
  feed();rec['motion_attempted']=True;checkpoint()
  rec['arm_request_monotonic']=time.monotonic()
  moving_deadline=rec['arm_request_monotonic']+DURATION
  write('axis0.requested_state',8)
  while True:
   feed();state=read('axis0.current_state')
   require(read('axis0.active_errors')==0 and read('axis0.disarm_reason')==0,'Fault while arming')
   if state==8:break
   require(state==1,'Unexpected state while arming');time.sleep(.005)
  write('axis0.controller.input_vel',TARGET);feed()
  rec['target_set_monotonic']=time.monotonic()
  while time.monotonic()<moving_deadline:
   feed();row={'monotonic':time.monotonic(),'complete':False};rec['samples'].append(row)
   for k in TRACE:
    row[k]=read(k)
    if k=='axis0.current_state':require(row[k]==8,'Left CLOSED_LOOP')
    elif k in ['axis0.active_errors','axis0.disarm_reason']:require(row[k]==0,'ODrive fault')
    elif k=='axis0.vel_estimate':
     require(math.isfinite(row[k]),'Invalid velocity')
     require(-2<=row[k]*60<=ABORT_RPM,'Unexpected encoder direction/speed')
    elif k=='axis0.pos_estimate':require(math.isfinite(row[k]) and abs(row[k]-baseline)<=.1,'Unexpected displacement')
    elif k=='axis0.motor.foc.Iq_measured':require(math.isfinite(row[k]) and abs(row[k])<=ABORT_CURRENT,'Test current threshold or invalid current')
   row['state_after']=read('axis0.current_state');require(row['state_after']==8,'Left CLOSED_LOOP during current sample')
   row['completed_monotonic']=time.monotonic();row['complete']=True
   require(row['completed_monotonic']-row['monotonic']<.15,'Feedback sample took too long')
   reason=movement_abort(row,baseline);require(reason is None,reason)
   time.sleep(.005)
  rec['result']='Completed bounded test'
 except Exception as ex:rec['result']=str(ex)
 finally:
  moving_deadline=None
  if rec['settings_mutated']:
   rec['cleanup_confirmed']=False
   try:
    # Stop first; no exception in later cleanup may prevent these attempts.
    zero_error=None
    try:write('axis0.controller.input_vel',0)
    except Exception as ex:zero_error=str(ex)
    write('axis0.requested_state',1)
    rec['stop_commands_sent_monotonic']=time.monotonic()
    require(not stream_uncertain,'Serial stream uncertain; lower limit/watchdog not restored. Switch wheel power off.')
    require(zero_error is None,'Zero write failed: '+str(zero_error))
    deadline=time.monotonic()+.5
    while time.monotonic()<deadline:
     if read('axis0.current_state')==1:break
    require(read('axis0.current_state')==1,'IDLE not confirmed: switch wheel power off')
    rec['idle_confirmed_monotonic']=time.monotonic()
    quiet_count=0;settle_end=time.monotonic()+1
    while time.monotonic()<settle_end and quiet_count<3:
     velocity=read('axis0.vel_estimate')
     quiet_count=quiet_count+1 if math.isfinite(velocity) and abs(velocity)*60<2 else 0
     time.sleep(.03)
    require(quiet_count>=3,'Stationary feedback not confirmed; keep low limit and report')
    rec['stationary_feedback_confirmed']=True
    write('axis0.controller.input_torque',0)
    # Keep native watchdog enabled until all other temporary settings are restored.
    for k in TEMP:
     if k in ['axis0.config.watchdog_timeout','axis0.config.enable_watchdog']:continue
     write(k,rec['before'][k]);verify(k,rec['before'][k])
    write('axis0.config.enable_watchdog',rec['before']['axis0.config.enable_watchdog']);verify('axis0.config.enable_watchdog',rec['before']['axis0.config.enable_watchdog'])
    write('axis0.config.watchdog_timeout',rec['before']['axis0.config.watchdog_timeout']);verify('axis0.config.watchdog_timeout',rec['before']['axis0.config.watchdog_timeout'])
    rec['after']={k:read(k) for k in ['axis0.current_state','axis0.active_errors','axis0.disarm_reason','axis0.config.motor.current_soft_max','axis0.config.motor.current_hard_max','axis0.controller.input_vel','axis0.controller.input_torque']}
    require(rec['after']['axis0.current_state']==1 and rec['after']['axis0.controller.input_vel']==0 and rec['after']['axis0.controller.input_torque']==0,'Final IDLE/zero not confirmed')
    rec['cleanup_confirmed']=True
   except Exception as ex:rec['cleanup_error']=str(ex);print('CLEANUP NEEDS ATTENTION: switch wheel power off. '+str(ex),flush=True)
  s.close();checkpoint();(out/'runner.py').write_text(Path(__file__).read_text());print(json.dumps({k:rec.get(k) for k in ['result','motion_attempted','clear_count','cleanup_confirmed','cleanup_error','after']},indent=2));print('EVIDENCE',out)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--authorization');a=p.parse_args()
 if not a.run:print(json.dumps(PLAN,indent=2))
 else:
  require(a.authorization,'Authorization file required');run(a.authorization)
