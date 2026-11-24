"""Bounded, asynchronous robot diagnostics. This module never accesses hardware."""
from __future__ import annotations

import datetime
import itertools
import json
import os
from pathlib import Path
import queue
import re
import threading
import time
import uuid

REVISION = "V18 Actuator Recovery"
MAX_RECORD_BYTES = 16384
_OWN_LOG = re.compile(r"^climber-v18-\d{8}T\d{6}Z-\d+-[0-9a-f]{8}-\d{4,}\.jsonl$")


class SessionLog:
    """Producers only enqueue; disk operations belong to one daemon writer.

    Retains up to16 x8 MiB by default across sessions in this revision's logs
    directory. Rotation removes only regular files matching our exact naming
    scheme. Overflow preserves newer records and emits one visible warning.
    Flush every250 ms, fsync every1 s, on fault text and on orderly shutdown.
    A power loss can still lose the most recent buffered records.
    """
    def __init__(self, directory, *, revision=REVISION, queue_size=4096,
                 max_file_bytes=8*1024*1024, max_files=16,
                 flush_seconds=.25, sync_seconds=1.0):
        if queue_size<1 or max_files<2 or max_file_bytes<MAX_RECORD_BYTES or flush_seconds<=0 or sync_seconds<=0:
            raise ValueError("Invalid bounded logger settings")
        self.directory=Path(directory)
        self.revision=revision
        stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        self.session=f'{stamp}-{os.getpid()}-{uuid.uuid4().hex[:8]}'
        self._queue=queue.Queue(maxsize=queue_size)
        self._warnings=queue.SimpleQueue()
        self._warn_lock=threading.Lock()
        self._warned=False
        self._closed=threading.Event()
        self._failed=threading.Event()
        self._sequence=itertools.count()
        self._max_file_bytes=max_file_bytes
        self._max_files=max_files
        self._flush_seconds=flush_seconds
        self._sync_seconds=sync_seconds
        self._dropped=0
        self._file=None
        self._size=0
        self._segment=0
        self._unit_mapping=[]
        self._thread=threading.Thread(target=self._writer,name='robot-session-log',daemon=True)
        self._thread.start()
        self.record('session_start',revision_directory=str(self.directory.parent))

    def _warn(self, detail):
        # This lock protects only an in-memory flag, never disk I/O.
        with self._warn_lock:
            if not self._warned:
                self._warned=True
                self._warnings.put(f'[logging] {detail}. Robot controls remain available.')

    def pop_warning(self):
        try:return self._warnings.get_nowait()
        except queue.Empty:return None

    def record(self, event, **fields):
        """Best-effort nonblocking enqueue; never propagates an exception."""
        if self._closed.is_set() or self._failed.is_set():return False
        try:
            now=datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='milliseconds')
            item=dict(fields)
            # Bound common incoming text before placing it into the queue.
            for key,value in list(item.items()):
                if isinstance(value,str) and len(value)>8192:
                    item[key]=value[:8192]
                    item['text_truncated']=True
            item.update(event=str(event)[:64],utc=now,monotonic_ns=time.monotonic_ns(),
                        sequence=next(self._sequence),session=self.session,revision=self.revision)
            try:self._queue.put_nowait(item)
            except queue.Full:
                lost=0
                try:
                    self._queue.get_nowait();lost+=1
                except queue.Empty:pass
                queued=True
                try:self._queue.put_nowait(item)
                except queue.Full:lost+=1;queued=False
                if lost:
                    with self._warn_lock:self._dropped+=lost
                    self._warn('Diagnostic queue overflow; some records were lost')
                if not queued:return False
            return True
        except Exception as exc:
            self._warn(f'Diagnostic record was lost ({type(exc).__name__})')
            return False

    def close(self, timeout=.3):
        """Bounded wait, called only after the existing robot stop/disconnect."""
        self.record('session_end',queue_dropped=self._dropped)
        self._closed.set()
        self._thread.join(timeout=max(0,timeout))
        return not self._thread.is_alive() and not self._failed.is_set()

    def _prune(self):
        files=[]
        for path in self.directory.iterdir():
            if _OWN_LOG.fullmatch(path.name) and not path.is_symlink() and path.is_file():
                files.append(path)
        # Filenames contain UTC start times; mtime also orders session segments.
        files.sort(key=lambda p:(p.stat().st_mtime_ns,p.name))
        for path in files[:-self._max_files]:
            if path!=Path(self._file.name):path.unlink()

    def _open_segment(self):
        if self._file is not None:
            self._file.flush();os.fsync(self._file.fileno());self._file.close()
        self.directory.mkdir(parents=True,exist_ok=True)
        path=self.directory/f'climber-v18-{self.session}-{self._segment:04d}.jsonl'
        self._segment+=1
        self._file=path.open('xb')
        header={'event':'segment_start','session':self.session,'revision':self.revision,
                'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
                'segment':self._segment-1,'queue_dropped':self._dropped,
                'unit_mapping':self._unit_mapping}
        blob=(json.dumps(header,separators=(',',':'))+'\n').encode()
        self._file.write(blob);self._size=len(blob)
        self._prune()

    def _write_record(self, item):
        if item.get('event')=='unit_mapping':
            self._unit_mapping=item.get('units',[])
        blob=(json.dumps(item,ensure_ascii=True,allow_nan=False,separators=(',',':'))+'\n').encode()
        if len(blob)>MAX_RECORD_BYTES:
            self._warn('Oversized diagnostic record was truncated')
            item={key:item[key] for key in ('utc','monotonic_ns','sequence','session','revision','event')}
            item['record_truncated']=True
            blob=(json.dumps(item,separators=(',',':'))+'\n').encode()
        if self._size+len(blob)>self._max_file_bytes:self._open_segment()
        self._file.write(blob);self._size+=len(blob)
        text=str(item.get('text','')).lower()
        return any(word in text for word in ('fault','error','trip','spinout','bus_off','blocked'))

    def _writer(self):
        try:
            self._open_segment()
            last_flush=last_sync=time.monotonic()
            while not self._closed.is_set() or not self._queue.empty():
                fault=False
                try:
                    item=self._queue.get(timeout=min(.1,self._flush_seconds))
                    fault=self._write_record(item)
                except queue.Empty:pass
                now=time.monotonic()
                if fault or now-last_flush>=self._flush_seconds:
                    self._file.flush();last_flush=now
                if fault or now-last_sync>=self._sync_seconds:
                    self._file.flush();os.fsync(self._file.fileno());last_sync=now
            self._file.flush();os.fsync(self._file.fileno())
        except Exception as exc:
            self._failed.set()
            self._warn(f'Persistent diagnostic logging stopped ({type(exc).__name__}: {exc})')
        finally:
            if self._file is not None:
                try:self._file.close()
                except Exception:pass
