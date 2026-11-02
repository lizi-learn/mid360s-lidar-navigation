import base64,gzip,json,sys,tempfile,time,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from flight_recorder import FlightRecorder

class FlightRecorderTest(unittest.TestCase):
    def test_exact_source_and_costmap_bytes_roundtrip(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'flight.private.jsonl.gz';r=FlightRecorder(p)
            source=1730000000000000123;cells=bytes(range(256))
            r.submit('costmap',dict(source_ns=source,cells=cells))
            r.submit('path',dict(source_ns=source,poses=[[1.,2.,0.,0.,0.,1.]],revision=9))
            r.close();rows=[json.loads(x)for x in gzip.decompress(p.read_bytes()).splitlines()]
            self.assertEqual(rows[0]['data']['source_ns'],source)
            self.assertEqual(base64.b64decode(rows[0]['data']['cells']['data']),cells)
            self.assertEqual(rows[1]['data']['revision'],9)
            self.assertEqual(r.status()['errors'],0)
            self.assertEqual(p.stat().st_mode & 0o777,0o600)
    def test_backpressure_never_blocks_producer(self):
        with tempfile.TemporaryDirectory() as d:
            r=FlightRecorder(Path(d)/'f.gz',queue_size=1)
            start=time.monotonic()
            for n in range(10000):r.submit('path',{'revision':n})
            self.assertLess(time.monotonic()-start,1.)
            self.assertGreater(r.dropped,0);r.close()
    def test_io_failure_is_observable_and_does_not_kill_worker(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'parent';p.write_text('not directory');r=FlightRecorder(p/'f.gz')
            r.submit('path',dict(source_ns=1));r.close()
            self.assertEqual(r.errors,1);self.assertEqual(r.written,0)
    def test_rotation_retains_original_records_and_reports_loss_boundary(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'f.gz';r=FlightRecorder(p,limit_bytes=1,queue_size=8)
            for n in range(3):r.submit('path',{'source_ns':n})
            r.close();self.assertEqual(r.rotations,2)
            self.assertEqual(json.loads(gzip.decompress(p.read_bytes()))['data']['source_ns'],2)
            self.assertEqual(json.loads(gzip.decompress(Path(str(p)+'.1').read_bytes()))['data']['source_ns'],1)
    def test_nonfinite_is_reported_not_silently_relabelled(self):
        with tempfile.TemporaryDirectory() as d:
            r=FlightRecorder(Path(d)/'f.gz');r.submit('path',{'x':float('nan')});r.close()
            self.assertEqual(r.errors,1)
