"""Live telemetry tests; all serial traffic is simulated."""
import importlib.util
import queue
import threading
import unittest
from unittest.mock import Mock, patch
import odrive_live as live
import test_all_working as integration


def frame(unit=0, seq=1, link=1, state=8, error=0, ages='20,30,40,20', values='44.02,1.23,5.0,4.8,120.0'):
    return f'ODRIVE_LIVE,1,{unit},{seq},{link},{state},{error},{ages},{values}'

class SampleTests(unittest.TestCase):
    def test_each_arm_is_attributed_by_identity(self):
        store=live.TelemetryStore()
        for arm in (3,0,2,1):
            with patch.object(live.time,'monotonic',return_value=100):
                self.assertTrue(store.feed(arm,frame(unit=arm,values=f'{40+arm},1,2,3,4')))
        rows=store.rows(set(range(4)),100)
        for arm in range(4): self.assertEqual(rows[arm][0], f'{40+arm:.2f}')

    def test_wrong_unit_and_malformed_values_never_replace_good_sample(self):
        store=live.TelemetryStore()
        with patch.object(live.time,'monotonic',return_value=100):
            store.feed(0,frame())
            for line in (frame(unit=1),frame(seq=-1),frame(link=9),frame(error=-1),frame(ages='20,-2,0,0'),frame(values='inf,1,1,1,1'),frame()+',extra','ODRIVE_LIVE,2,0'):
                store.feed(0,line)
        self.assertEqual(store.rows({0},100)[0][0],'44.02')

    def test_stale_data_blanks_values_and_state(self):
        sample=live.parse_sample(frame(),0,100)
        row=live.display_row(sample,True,100.7)
        self.assertEqual(row[:6],('—',)*6)
        self.assertIn('Stale',row[-1])

    def test_can_age_and_host_age_are_combined(self):
        sample=live.parse_sample(frame(ages='20,500,0,0'),0,100)
        row=live.display_row(sample,True,100.2)
        self.assertEqual(row[:2],('—','—'))
        self.assertEqual(row[2],'4.80')

    def test_idle_hides_retained_motor_current(self):
        row=live.display_row(live.parse_sample(frame(state=1),0,100),True,100)
        self.assertEqual(row[2:4],('—','—'))
        self.assertEqual(row[5],'IDLE')

    def test_offline_unknown_legacy_and_disconnect(self):
        self.assertIn('Waiting',live.display_row(None,True,100)[-1])
        for link in (0,2,3,4,5):
            row=live.display_row(live.parse_sample(frame(link=link),0,100),True,100)
            self.assertEqual(row[:6],('—',)*6)
        row=live.display_row(live.parse_sample(frame(),0,100),False,100)
        self.assertEqual(row[-1],'Disconnected')

    def test_stale_heartbeat_hides_all_measurements(self):
        row=live.display_row(live.parse_sample(frame(ages='1100,0,0,0'),0,100),True,100)
        self.assertEqual(row[:6],('—',)*6)
        self.assertIn('heartbeat',row[-1])

    def test_errors_signed_currents_and_missing_fields(self):
        row=live.display_row(live.parse_sample(frame(error=256,values='44,-1.2,-5,-4.5,-120'),0,100),True,100)
        self.assertEqual(row[1:5],('-1.20','-4.50','-5.00','-120.00'))
        self.assertIn('00000100',row[-1])
        row=live.display_row(live.parse_sample(frame(values='nan,nan,nan,nan,nan'),0,100),True,100)
        self.assertEqual(row[:5],('—',)*5)

    def test_spinout_is_named_for_each_arm_and_idle_current_stays_hidden(self):
        for arm in range(4):
            sample=live.parse_sample(frame(unit=arm,state=1,error=0x04000000),arm,100)
            row=live.display_row(sample,True,100)
            self.assertEqual(row[-1],'SPINOUT_DETECTED · 0x04000000')
            self.assertEqual(row[2:4],('—','—'))

    def test_common_odrive_faults_and_unknown_bits_format_cleanly(self):
        cases = (
            (0x00001000, 'CURRENT_LIMIT_VIOLATION · 0x00001000'),
            (0x00008000, 'VELOCITY_LIMIT_VIOLATION · 0x00008000'),
            (0x00000200, 'DC_BUS_UNDER_VOLTAGE · 0x00000200'),
            (0x00000100, 'DC_BUS_OVER_VOLTAGE · 0x00000100'),
            (0x04001000, 'SPINOUT_DETECTED, CURRENT_LIMIT_VIOLATION · 0x04001000'),
            (0x00000080, 'Error 0x00000080'),
        )
        for code, expected in cases:
            self.assertEqual(live.format_error(code), expected)
            row = live.display_row(live.parse_sample(frame(state=1, error=code), 0, 100), True, 100)
            self.assertEqual(row[-1], expected)

    def test_reconnect_clears_previous_data(self):
        store=live.TelemetryStore(); store.feed(0,frame()); store.clear()
        self.assertIn('Waiting',store.rows({0})[0][-1])

    def test_bounded_latest_sample_storage_and_prompt_prefix(self):
        store=live.TelemetryStore()
        for seq in range(1000): store.feed(0,'bringup> '+frame(seq=seq))
        self.assertEqual(len(store._samples),1)
        self.assertEqual(store._samples[0].sequence,999)

class ReaderTests(unittest.TestCase):
    def test_split_records_reassemble_without_console_flood_or_writes(self):
        sup=integration.sup
        stop=threading.Event(); output=queue.Queue(); store=live.TelemetryStore()
        data=(frame()+'\nWheel stopped.\n').encode()
        chunks=iter([data[:8],b'',data[8:44],data[44:]])
        ser=Mock()
        def read():
            try: return next(chunks)
            except StopIteration: stop.set(); return b''
        ser.readline.side_effect=read
        sup.reader_loop(0,ser,stop,output,store.feed)
        self.assertEqual(output.get_nowait(),'[u0] Wheel stopped.')
        self.assertTrue(output.empty())
        self.assertEqual(store._samples[0].voltage,44.02)
        ser.write.assert_not_called(); ser.flush.assert_not_called()

    def test_refresh_schedules_200ms_without_sending_commands(self):
        gui=integration.gui
        app=gui.ClimberGUI.__new__(gui.ClimberGUI)
        app._closing=False;app.root=Mock();app.units=[];app.live_store=live.TelemetryStore()
        app.live_vars={i:[Mock() for _ in range(7)] for i in range(4)}
        app.act_store=gui.act_live.TelemetryStore();app.act_global_vars={i:Mock() for i in range(4)};app.act_vars={i:[Mock() for _ in range(7)] for i in range(4)}
        app._run_silent=Mock();app._refresh_live()
        app.root.after.assert_called_once_with(200,app._refresh_live)
        app._run_silent.assert_not_called()

if __name__=='__main__': unittest.main()
