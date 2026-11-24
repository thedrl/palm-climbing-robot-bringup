"""Persistent logging regression tests: temporary files and fake GUI units only."""
import json
from pathlib import Path
import queue
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import Mock, patch

from session_log import SessionLog, MAX_RECORD_BYTES
import test_climber_gui5

gui=test_climber_gui5.gui

def records(directory):
    rows=[]
    for path in sorted(Path(directory).glob('climber-v18-*.jsonl')):
        if path.is_symlink():continue
        for line in path.read_text().splitlines():
            rows.append(json.loads(line))
    return rows

class SessionLogTests(unittest.TestCase):
    def test_records_survive_close_and_new_session_with_timestamps_and_fault_history(self):
        with tempfile.TemporaryDirectory() as tmp:
            log=SessionLog(tmp)
            first=log.session
            log.record('command_request',command='u0 w0 10',units=[0])
            log.record('serial_receive',unit=0,text='ODRIVE_PART,1,0,5,0,3,0,...')
            log.record('serial_receive',unit=0,text='SPINOUT_DETECTED')
            self.assertTrue(log.close(timeout=2))
            second=SessionLog(tmp);self.assertNotEqual(first,second.session)
            self.assertTrue(second.close(timeout=2))
            events=[r for r in records(tmp) if r.get('session')==first and r['event']!='segment_start']
            self.assertEqual([r['event'] for r in events],['session_start','command_request','serial_receive','serial_receive','session_end'])
            self.assertTrue(all(r['utc'].endswith('+00:00') and r['monotonic_ns']>0 for r in events))
            self.assertEqual([r['sequence'] for r in events],sorted(r['sequence'] for r in events))
            self.assertIn('ODRIVE_PART',events[2]['text'])
            self.assertIn('SPINOUT',events[3]['text'])

    def test_periodic_flush_is_readable_before_gui_close(self):
        with tempfile.TemporaryDirectory() as tmp:
            log=SessionLog(tmp,flush_seconds=.02,sync_seconds=.03)
            self.addCleanup(log.close,1)
            log.record('serial_receive',unit=2,text='recent live sample')
            deadline=time.monotonic()+1
            while time.monotonic()<deadline:
                if any(r.get('text')=='recent live sample' for r in records(tmp)):break
                time.sleep(.01)
            else:self.fail('Writer did not persist live history before close')
            self.assertFalse(log._closed.is_set())

    def test_queue_is_bounded_and_slow_disk_does_not_block_producers_or_close(self):
        gate=threading.Event();original=SessionLog._open_segment
        def delayed(log):gate.wait(2);original(log)
        with tempfile.TemporaryDirectory() as tmp,patch.object(SessionLog,'_open_segment',delayed):
            log=SessionLog(tmp,queue_size=4)
            started=time.monotonic()
            for i in range(500):log.record('serial_receive',unit=0,text=f'sample{i}')
            self.assertLess(time.monotonic()-started,.5)
            self.assertLessEqual(log._queue.qsize(),4)
            self.assertIn('overflow',log.pop_warning())
            self.assertIsNone(log.pop_warning())
            started=time.monotonic();self.assertFalse(log.close(timeout=.01))
            self.assertLess(time.monotonic()-started,.2)
            gate.set();log._thread.join(2)
            self.assertTrue(any(r.get('text')=='sample499' for r in records(tmp)))
            self.assertTrue(any(r.get('queue_dropped',0)>0 for r in records(tmp)))

    def test_disk_creation_failure_disables_logging_and_warns_once(self):
        with tempfile.TemporaryDirectory() as tmp:
            parent=Path(tmp)/'file';parent.write_text('not a directory')
            log=SessionLog(parent/'logs')
            log._thread.join(1)
            self.assertTrue(log._failed.is_set())
            self.assertFalse(log.record('command_request',command='stop'))
            self.assertIn('logging stopped',log.pop_warning())
            self.assertIsNone(log.pop_warning())
            self.assertFalse(log.close(.1))

    def test_disk_write_failure_is_reported_once_without_raising(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(SessionLog,'_write_record',side_effect=OSError('disk full')):
            log=SessionLog(tmp);log._thread.join(1)
            self.assertIn('disk full',log.pop_warning())
            self.assertIsNone(log.pop_warning())
            self.assertFalse(log.record('serial_receive',text='fault'))
            self.assertFalse(log.close(.1))

    def test_rotation_is_size_bounded_and_preserves_unrelated_files_and_symlinks(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);unrelated=root/'notes.jsonl';unrelated.write_text('user notes')
            target=root/'important.txt';target.write_text('preserve')
            link=root/'climber-v18-20000101T000000Z-123-abcdef12-0000.jsonl';link.symlink_to(target)
            log=SessionLog(tmp,max_file_bytes=MAX_RECORD_BYTES,max_files=3)
            log.record('unit_mapping',units=[{'unit':0,'port':'/dev/mock','usb_serial':'SERIAL0'}])
            for i in range(20):log.record('serial_receive',unit=0,text=f'{i}:'+('x'*8000))
            self.assertTrue(log.close(2))
            files=[p for p in root.glob('climber-v18-*.jsonl') if not p.is_symlink()]
            self.assertLessEqual(len(files),3)
            self.assertTrue(all(p.stat().st_size<=MAX_RECORD_BYTES for p in files))
            self.assertEqual(unrelated.read_text(),'user notes');self.assertEqual(target.read_text(),'preserve');self.assertTrue(link.is_symlink())
            self.assertTrue(any(r.get('text','').startswith('19:') for r in records(tmp)))
            self.assertTrue(all(r['unit_mapping'][0]['usb_serial']=='SERIAL0' for r in records(tmp) if r['event']=='segment_start'))

    def test_oversized_text_and_record_are_bounded(self):
        with tempfile.TemporaryDirectory() as tmp:
            log=SessionLog(tmp,max_file_bytes=MAX_RECORD_BYTES)
            log.record('serial_receive',text='x'*100000)
            log.record('large',text='y'*100000,other='z'*100000)
            self.assertTrue(log.close(2))
            rows=records(tmp)
            self.assertTrue(any(r.get('text_truncated') for r in rows))
            self.assertTrue(any(r.get('record_truncated') for r in rows))
            self.assertTrue(all(p.stat().st_size<=MAX_RECORD_BYTES for p in Path(tmp).glob('*.jsonl')))

class GuiLoggingTests(unittest.TestCase):
    def app(self):
        a=gui.ClimberGUI.__new__(gui.ClimberGUI)
        a.output_queue=queue.Queue();a.units=[types.SimpleNamespace(index=0),types.SimpleNamespace(index=1)]
        a.session_log=Mock();a.act_store=Mock();a.live_store=Mock()
        return a

    def test_logger_exception_does_not_prevent_stop_reaching_other_units(self):
        a=self.app();a.session_log.record.side_effect=OSError('logger failure')
        handler=Mock(side_effect=[OSError('unit0 disconnected'),None])
        with patch.object(gui.sup,'handle_supervisor_command',handler):
            self.assertFalse(a._run_silent('stop'))
        self.assertEqual(handler.call_count,2)
        self.assertEqual(handler.call_args_list[1].args,([a.units[1]],'stop'))
        self.assertEqual(sum('logging unavailable' in line for line in list(a.output_queue.queue)),1)

    def test_raw_telemetry_is_logged_even_when_hidden_from_console(self):
        a=self.app();a.act_store.feed.return_value=True
        self.assertTrue(a._feed_live(3,'ACT_PART,1,3,0,0,3,0,...'))
        a.session_log.record.assert_called_once_with('serial_receive',unit=3,text='ACT_PART,1,3,0,0,3,0,...')
        a.live_store.feed.assert_not_called()

    def test_received_fault_keeps_unit_identity(self):
        a=self.app();a.act_store.feed.return_value=False;a.live_store.feed.return_value=False
        self.assertFalse(a._feed_live(0,'w0 SPINOUT_DETECTED'))
        a.session_log.record.assert_called_once_with('serial_receive',unit=0,text='w0 SPINOUT_DETECTED')

    def test_command_logging_labels_request_without_claiming_physical_success(self):
        a=self.app()
        with patch.object(gui.sup,'handle_supervisor_command'):
            self.assertTrue(a._run_silent('u0 w0 10'))
        self.assertEqual(a.session_log.record.call_args_list[0].args,('command_request',))
        self.assertEqual(a.session_log.record.call_args_list[-1].args,('command_dispatch_result',))
        self.assertEqual(a.session_log.record.call_args_list[-1].kwargs,{'command':'u0 w0 10','succeeded':True})

    def test_mapping_contains_usb_serial_and_is_shown_in_console(self):
        a=self.app();a.root=Mock();a.status_var=Mock();a.disconnect_btn=Mock()
        unit=types.SimpleNamespace(index=3,port='/dev/mock3',identified=True,odrive_node_id=0)
        with patch.object(gui.sup.list_ports,'comports',return_value=[types.SimpleNamespace(device='/dev/mock3',serial_number='SERIAL3')]):
            a._on_connected([unit])
        mapped=a.session_log.record.call_args.kwargs['units'][0]
        self.assertEqual(mapped['unit'],3);self.assertEqual(mapped['usb_serial'],'SERIAL3')
        self.assertIn('SERIAL3',a.output_queue.get_nowait())

if __name__=='__main__':unittest.main()
