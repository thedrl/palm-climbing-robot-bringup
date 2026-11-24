import importlib.util,json,time
from pathlib import Path
root=Path('/home/frc-orin/V17-FarmBot21SEP2026-CurrentPersistence')
p=root/'evidence/v17-persistence/v17_persistence_host.py'
s=importlib.util.spec_from_file_location('h',p);h=importlib.util.module_from_spec(s);s.loader.exec_module(h)
h.ALLOW={'unit','firmware','status','can status'}
h.unowned()
out=Path('/home/frc-orin/V18-FarmBot21SEP2026-ActuatorRecovery/evidence')/('arm1-connections-'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime()));out.mkdir()
rec={'usb_serial':'48CA432DF684','steps':[],'commands':'identity and cached status only; no CAN init/reset or motion'}
def save():(out/'result.json').write_text(json.dumps(rec,indent=2)+'\n')
link=None
try:
 link=h.Link(rec,rec,save)
 r=link.cmd('unit',1.5);print(r,flush=True)
 if 'unit id: 1, local ODrive CAN node id: 0' in r:
  for cmd in ('firmware','status','can status'):
   print(cmd+'\n'+link.cmd(cmd,1.5),flush=True)
 else:rec['result']='Different or unrecognized firmware identity; no further commands sent'
except Exception as e:rec['error']=str(e);print('ERROR',str(e),flush=True)
finally:
 if link:link.s.close();(out/'serial.txt').write_text('\n'.join(link.raw)+'\n'+link.pending.decode(errors='replace'))
 save();(out/'reader.py').write_text(Path(__file__).read_text());print('EVIDENCE',out,flush=True)
