"""Strict, read-only RoboClaw snapshot display. Never opens ports or sends commands."""
from dataclasses import dataclass, replace
import math,threading,time
STALE_SECONDS=.6
STATES={0:'Waiting',1:'Idle',2:'Moving',3:'Paused · press Stop for idle readings',4:'Read fault · press Stop',5:'Stop NOT confirmed'}
@dataclass(frozen=True)
class Sample:
    unit:int
    sequence:int
    channel:int
    state:int
    ages:tuple[int,...]
    values:tuple[int,...]
    received:float
    started:float=0.0

def parse_sample(text,unit,now):
    marker=text.find('ACT_LIVE,')
    if marker<0:return None
    fields=text[marker:].strip().split(',')
    if len(fields)!=20 or fields[:2]!=['ACT_LIVE','1']:return None
    try:
        data=list(map(int,fields[2:]));board,seq,channel,state=data[:4]
        ages=tuple(data[4:9]);values=tuple(data[9:])
        if board!=unit or not 0<=board<4 or not 0<=seq<=0xffffffff or channel not in (1,2) or state not in STATES:return None
        if any(not -1<=a<=0xffffffff for a in ages):return None
        v,c1,c2,p1,p2,limit,error,command,setting=values
        if not 0<=v<=65535 or any(not -32768<=x<=32767 for x in (c1,c2,p1,p2)):return None
        if not 0<=limit<=0xffffffff or not 0<=error<=0xffffffff or not -127<=command<=127 or not 1<=setting<=60:return None
        return Sample(board,seq,channel,state,ages,values,now)
    except (ValueError,OverflowError):return None

class TelemetryStore:
    def __init__(self):self._lock=threading.Lock();self._samples={};self._parts={}
    def clear(self):
        with self._lock:self._samples.clear();self._parts.clear()
    def feed(self,unit,text):
        if 'ACT_PART,' not in text:return False
        now=time.monotonic()
        try:
            parts=text[text.find('ACT_PART,'):].strip().split(',',7)
            if len(parts)!=8 or parts[:2]!=['ACT_PART','1']:return True
            board,seq,index,total,age=map(int,parts[2:7]);fragment=parts[7]
            if board!=unit or not 0<=board<4 or not 0<=seq<=0xffffffff or not 1<=total<=11 or not 0<=index<total or not 0<=age<200 or not 1<=len(fragment)<=24 or (index<total-1 and len(fragment)!=24):return True
        except (ValueError,OverflowError):return True
        with self._lock:
            if index==0:self._parts[unit]={'seq':seq,'total':total,'age':0,'at':now,'data':[]}
            p=self._parts.get(unit)
            if p is None:return True
            if p['seq']!=seq or p['total']!=total or len(p['data'])!=index or age<p['age'] or now-p['at']>STALE_SECONDS:
                self._parts.pop(unit,None);return True
            p['data'].append(fragment);p['age']=age
            if index==total-1:
                self._parts.pop(unit,None)
                sample=parse_sample(''.join(p['data']),unit,now-age/1000)
                if sample and sample.sequence==seq:self._samples[unit]=replace(sample,started=p['at'])
        return True
    def sample(self,unit):
        with self._lock:return self._samples.get(unit)
    def rows(self,connected,now=None):
        now=time.monotonic() if now is None else now
        with self._lock:samples=dict(self._samples)
        return {u:display_row(samples.get(u),u in connected,now) for u in range(4)}

def display_row(sample,connected,now):
    # Supply V, selected motor A, output PWM %, configured controller limit A,
    # applied command %, manual ceiling %, channel/status.
    empty=('—',)*6
    if not connected:return (*empty,'Disconnected')
    if sample is None:return (*empty,'Waiting for actuator telemetry')
    age=max(0,now-sample.received)
    if age>STALE_SECONDS:return (*empty,'Stale')
    status=f'M{sample.channel} · {STATES[sample.state]}'
    v,c1,c2,p1,p2,limit,error,command,setting=sample.values
    if sample.state not in (1,2):return (*empty,status)
    def value(number,group,scale,max_age=600):
        a=sample.ages[group]
        return f'{number/scale:.2f}' if a>=0 and a+age*1000<=max_age else '—'
    current=c1 if sample.channel==1 else c2;pwm=p1 if sample.channel==1 else p2
    if sample.ages[4]>=0 and sample.ages[4]+age*1000<=600 and error&~0x03000000:status+=f' · status 0x{error:08X}'
    return (value(v,0,10),value(current,1,100),value(pwm,2,327.67),value(limit,3,100,30000),f'{command*100/127:.1f}',f'{setting*100/127:.1f}',status)
