import unittest
from unittest.mock import Mock,patch
from types import SimpleNamespace
import actuator_live as live
import climber_gui5 as gui

def record(unit=0,seq=77,channel=1,state=2,ages=(0,0,0,1000,0),values=(216,510,25,10321,0,698,0x03000000,40,40)):
    return ','.join(map(str,('ACT_LIVE',1,unit,seq,channel,state,*ages,*values)))
def pieces(text,unit=0,seq=77,age=0):
    fragments=[text[i:i+24] for i in range(0,len(text),24)]
    return [f'ACT_PART,1,{unit},{seq},{i},{len(fragments)},{age},{part}' for i,part in enumerate(fragments)]
class ActuatorTests(unittest.TestCase):
    def setUp(self):self.store=live.TelemetryStore()
    def feed(self,lines,now=100,unit=0):
        with patch.object(live.time,'monotonic',return_value=now):
            for line in lines:self.assertTrue(self.store.feed(unit,line))
    def test_real_units_selected_channel_and_scaling(self):
        self.feed(pieces(record()));row=self.store.rows({0},100)[0]
        self.assertEqual(row[:4],('21.60','5.10','31.50','6.98'));self.assertEqual(row[4:6],('31.5','31.5'))
        self.feed(pieces(record(channel=2)));row=self.store.rows({0},100)[0]
        self.assertEqual(row[1:3],('0.25','0.00'));self.assertIn('M2',row[-1])
    def test_stale_and_disconnected_data_is_hidden(self):
        self.feed(pieces(record()));self.assertEqual(self.store.rows({0},101)[0][:6],('—',)*6)
        self.assertEqual(self.store.rows(set(),100)[0][-1],'Disconnected')
    def test_partial_expired_out_of_order_never_publishes(self):
        p=pieces(record());self.feed(p[:-1]);self.assertNotIn(0,self.store._samples)
        self.feed(p[-1:],101);self.assertNotIn(0,self.store._samples)
        self.feed(p[:1]+p[2:]);self.assertNotIn(0,self.store._samples)
    def test_source_age_and_transmit_age_are_accounted(self):
        self.feed(pieces(record(ages=(500,500,500,29900,500)),age=150));row=self.store.rows({0},100)[0]
        self.assertEqual(row[:4],('—',)*4)
    def test_fault_pause_and_unknown_stop_hide_current(self):
        for state in (0,3,4,5):
            self.feed(pieces(record(state=state)));self.assertEqual(self.store.rows({0},100)[0][:6],('—',)*6)
    def test_crc_verified_source_fault_bits_remain_visible(self):
        self.feed(pieces(record(values=(216,510,0,10321,0,698,0x10,40,40))))
        self.assertIn('0x00000010',self.store.rows({0},100)[0][-1])
    def test_invalid_and_wrong_unit_frames_are_rejected(self):
        for text in (record(unit=1),record(channel=3),record(state=6),record(seq=-1),record(ages=(-2,0,0,0,0)),record(values=(216,99999,0,0,0,698,0,40,40)),record(values=(216,0,0,0,0,698,0,40,61))):
            self.store.clear();self.feed(pieces(text));self.assertFalse(self.store._samples)
        self.assertFalse(self.store.feed(0,'normal diagnostic text'))
        self.assertFalse(self.store.feed(0,'ODRIVE_PART,1,0,4,0,1,0,x'))
    def test_four_arms_and_bounded_partial_storage(self):
        for u in range(4):self.feed(pieces(record(unit=u),unit=u),unit=u)
        self.assertEqual(len(self.store._samples),4)
        for seq in range(100):self.feed(pieces(record(seq=seq),seq=seq)[:1])
        self.assertEqual(len(self.store._parts),1)
        self.store.clear();self.assertFalse(self.store._samples);self.assertFalse(self.store._parts)
    def test_refresh_is_read_only_and_updates_unit_and_global(self):
        app=gui.ClimberGUI.__new__(gui.ClimberGUI);app._closing=False;app.root=Mock();app.units=[SimpleNamespace(index=0,identified=True,serial_port=SimpleNamespace(is_open=True))]
        app.live_store=Mock();app.live_store.rows.return_value={}
        app.live_vars={};app.act_store=self.store
        app.act_vars={u:[Mock() for _ in range(7)] for u in range(4)};app.act_global_vars={u:Mock() for u in range(4)};app._run_silent=Mock()
        self.feed(pieces(record()))
        with patch.object(live.time,'monotonic',return_value=100):app._refresh_live()
        app.act_global_vars[0].set.assert_called_once_with('5.10 A');app.act_vars[0][2].set.assert_called_once_with('31.50')
        app._run_silent.assert_not_called();app.root.after.assert_called_once_with(200,app._refresh_live)
    def test_apply_power_only_sends_setting_to_selected_unit(self):
        app=gui.ClimberGUI.__new__(gui.ClimberGUI);app._run=Mock();app._set_actuator_power(3,'48');app._run.assert_called_once_with('u3 act power 48')
        for value in ('0','61','1.5','nan','40; act fwd'):
            app._run.reset_mock()
            with patch.object(gui.messagebox,'showerror'):app._set_actuator_power(0,value)
            app._run.assert_not_called()
if __name__=='__main__':unittest.main()
