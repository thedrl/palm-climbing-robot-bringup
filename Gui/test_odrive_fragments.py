"""Regression tests for real 64-byte USB packet boundaries; no hardware access."""
import unittest
from unittest.mock import patch
import odrive_live as live
from test_odrive_live import frame

def pieces(text, unit=0, seq=1234567890, age=0):
    chunks=[text[i:i+24] for i in range(0,len(text),24)]
    return [f'ODRIVE_PART,1,{unit},{seq},{i},{len(chunks)},{age},{part}' for i,part in enumerate(chunks)]

class FragmentTests(unittest.TestCase):
    def setUp(self):self.store=live.TelemetryStore()
    def feed(self, lines, now=100):
        with patch.object(live.time,'monotonic',return_value=now):
            for line in lines:self.store.feed(0,line)
    def test_long_powered_record_at_real_64_byte_capacity(self):
        source=frame(seq=1234567890,ages='99,199,199,99',values='43.930,-12.300,20.000,19.999,1234.567')
        self.assertGreater(len(source)+2,64)
        chunks=pieces(source)
        self.assertTrue(all(len(c)+2<=64 for c in chunks))
        self.feed(chunks[:-1]);self.assertNotIn(0,self.store._samples)
        self.feed(chunks[-1:])
        row=self.store.rows({0},100)[0]
        self.assertEqual(row[:4],('43.93','-12.30','20.00','20.00'))
        self.assertEqual(row[4],'1234.57')
    def test_missing_reordered_or_mixed_snapshots_never_publish(self):
        a=pieces(frame(seq=1234567890));b=pieces(frame(seq=1234567891),seq=1234567891)
        for broken in (a[1:],a[:1]+a[2:],a[:1]+list(reversed(a[1:])),a[:1]+b[1:]):
            self.store.clear();self.feed(broken);self.assertNotIn(0,self.store._samples)
        self.feed(b);self.assertEqual(self.store._samples[0].sequence,1234567891)
    def test_new_snapshot_replaces_incomplete_one(self):
        self.feed(pieces(frame(seq=1234567890))[:1])
        self.feed(pieces(frame(seq=1234567891),seq=1234567891))
        self.assertEqual(self.store._samples[0].sequence,1234567891)
        self.assertFalse(self.store._parts)
    def test_transmit_age_counts_toward_staleness(self):
        self.feed(pieces(frame(seq=1234567890,ages='20,500,0,0'),age=150))
        self.assertAlmostEqual(self.store._samples[0].received,99.85)
        self.assertEqual(self.store.rows({0},100)[0][:2],('—','—'))
    def test_console_output_between_pieces_is_preserved(self):
        for part in pieces(frame(seq=1234567890)):
            self.feed(['bringup> '+part])
            self.assertFalse(self.store.feed(0,'Wheel stopped.'))
        self.assertIn(0,self.store._samples)
    def test_bad_envelope_and_conflicting_payload_identity_are_rejected(self):
        for args in ({'unit':1},{'seq':-1},{'seq':2**32},{'age':200},{'age':-1}):
            self.store.clear();self.feed(pieces(frame(seq=1234567890),**args));self.assertNotIn(0,self.store._samples)
        self.feed(pieces(frame(unit=1,seq=1234567890)));self.assertNotIn(0,self.store._samples)
        self.feed(pieces(frame(seq=12)));self.assertNotIn(0,self.store._samples)
    def test_delayed_assembly_and_reconnect_drop_partial_data(self):
        p=pieces(frame(seq=1234567890));self.feed(p[:1],100);self.feed(p[1:],101)
        self.assertNotIn(0,self.store._samples)
        self.feed(p[:1]);self.store.clear();self.feed(p[1:]);self.assertNotIn(0,self.store._samples)
    def test_four_units_are_isolated_and_storage_is_bounded(self):
        for unit in range(4):
            for line in pieces(frame(unit=unit,seq=1234567890),unit=unit):
                self.store.feed(unit,line)
        self.assertEqual(len(self.store._samples),4);self.assertFalse(self.store._parts)
        for seq in range(1000):
            for line in pieces(frame(seq=seq),seq=seq)[:1]:self.store.feed(0,line)
        self.assertEqual(len(self.store._parts),1)

if __name__=='__main__':unittest.main()
