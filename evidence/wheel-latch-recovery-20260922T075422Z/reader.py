from pathlib import Path
import importlib.util,json,time,re,concurrent.futures
BASE=Path('/home/frc-orin/V17-FarmBot21SEP2026-CurrentPersistence');ROOT=Path('/home/frc-orin/V18-FarmBot21SEP2026-ActuatorRecovery')
spec=importlib.util.spec_from_file_location('transport',BASE/'evidence/v17-persistence/v17_persistence_host.py');h=importlib.util.module_from_spec(spec);spec.loader.exec_module(h)
h.ALLOW={'unit','w0 diag','w0 clear','w0 current','can status'}
out=ROOT/'evidence'/('wheel-latch-recovery-'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime()));out.mkdir(parents=True)
h.unowned()
def inspect(row):
 arm=row['arm'];rec={'arm':arm,'steps':[],'nonzero_commands_sent':False,'clear_attempts':0,'limits_changed':False};link=None
 def save():(out/f'arm{arm}.json').write_text(json.dumps(rec,indent=2)+'\n')
 try:
  link=h.Link(row,rec,save)
  h.require(f'unit id: {arm}, local ODrive CAN node id: 0' in link.cmd('unit',1),'Wrong unit')
  before=link.cmd('w0 diag',3)
  h.require('heartbeat: online' in before and 'state: 1 IDLE,' in before,'Not live IDLE')
  h.require('active_errors: 0x00000000 none' in before,'Active fault present; no clearing')
  rpm=re.search(r'encoder: pos=[^\n]+ turns/s (-?[\d.]+) RPM',before)
  h.require(rpm and abs(float(rpm.group(1)))<2,'Stationary feedback unverified')
  bus=re.search(r'bus: ([\d.]+) V',before);h.require(bus and 40<float(bus.group(1))<55,'Normal wheel voltage unverified')
  rec['clear_attempts']=1;save()
  result=link.cmd('w0 clear',2.5);print(f'ARM {arm} clear\n{result}',flush=True)
  h.require('Wheel recovered in IDLE.' in result,'Clear not verified; do not retry automatically')
  after=link.cmd('w0 diag',3);print(f'ARM {arm} after\n{after}',flush=True)
  h.healthy_idle(after)
  result=link.cmd('w0 current',2);print(f'ARM {arm} current\n{result}',flush=True)
  h.require(h.wheel_values(result)==(35.0,38.0),'Current limits changed unexpectedly')
  rec['result']='Fault latch clear; fresh healthy IDLE; 35/38 unchanged. Motion remains untested.'
 except Exception as ex:rec['error']=str(ex);print(f'ARM {arm} ERROR {ex}',flush=True)
 finally:
  if link:link.s.close();(out/f'arm{arm}.txt').write_text('\n'.join(link.raw))
  save()
 return rec
rows=[r for r in json.loads((BASE/'FIRMWARE_MAP.json').read_text()) if r['arm'] in (0,3)]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:result=list(pool.map(inspect,rows))
(out/'summary.json').write_text(json.dumps(result,indent=2)+'\n');(out/'reader.py').write_text(Path(__file__).read_text())
print('EVIDENCE',out,flush=True)
