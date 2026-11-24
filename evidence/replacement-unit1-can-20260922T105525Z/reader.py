from pathlib import Path
import importlib.util,json,time,sys,dataclasses
root=Path('/home/frc-orin/V17-FarmBot21SEP2026-CurrentPersistence')
s=importlib.util.spec_from_file_location('h',root/'evidence/v17-persistence/v17_persistence_host.py');h=importlib.util.module_from_spec(s);s.loader.exec_module(h)
h.ALLOW={'unit','firmware','can status','w0 diag','w0 current'}
h.unowned();sys.path.insert(0,'/home/frc-orin/V18-FarmBot21SEP2026-ActuatorRecovery/Gui');import odrive_live
out=Path('/home/frc-orin/V18-FarmBot21SEP2026-ActuatorRecovery/evidence')/('replacement-unit1-can-'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime()));out.mkdir(exist_ok=False)
rec={'unit':1,'usb_serial':'48CA432DF684','steps':[],'nonzero_commands_sent':False,'configuration_commands_sent':False}
def save():(out/'result.json').write_text(json.dumps(rec,indent=2)+'\n')
link=None
try:
 link=h.Link(rec,rec,save)
 h.require('unit id: 1, local ODrive CAN node id: 0' in link.cmd('unit',1.3),'Wrong identity')
 print(link.cmd('firmware',1.2),flush=True)
 status=link.cmd('can status',1.3);rec['can_before']=status;print(status,flush=True)
 link.collect(3)
 store=odrive_live.TelemetryStore()
 for line in link.raw:store.feed(1,line)
 sample=store._samples.get(1);rec['passive_sample']=dataclasses.asdict(sample) if sample else None
 # This establishes a fresh live connection before read-only CAN requests.
 if sample and sample.link==1 and 0<=sample.ages[0]<1000 and 'state=RUNNING' in status and 'recovery=READY' in status:
  diag=link.cmd('w0 diag',3);rec['diagnostics']=diag;print(diag,flush=True)
  rec['current_limits']=link.cmd('w0 current',2);print(rec['current_limits'],flush=True)
 else:rec['result']='USB identity verified; CAN not yet proven ready. No active diagnostic request sent.'
 rec['can_after']=link.cmd('can status',1.2);print(rec['can_after'],flush=True)
 link.collect(1)
except Exception as e:rec['error']=str(e);print('ERROR',str(e),flush=True)
finally:
 if link:link.s.close();(out/'serial.txt').write_text('\n'.join(link.raw)+'\n'+link.pending.decode(errors='replace'))
 save();(out/'reader.py').write_text(Path(__file__).read_text());print('EVIDENCE',out,flush=True)
