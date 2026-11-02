import os,sys,tempfile,time,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from native_operator_source import NATIVE
from native_operator_snapshot import NativeSnapshotWriter,NativeSnapshotReader

class NativeSnapshotTests(unittest.TestCase):
    def test_original_packet_time_and_identity_are_never_renewed(self):
        with tempfile.TemporaryDirectory()as d:
            path=d+'/source';w=NativeSnapshotWriter(path);r=NativeSnapshotReader(path)
            try:
                stamp=time.monotonic_ns();packet=NATIVE.pack(0x4e4f5031,1,40,7,stamp,123456,1,3,0.,0.,0.,0)
                self.assertTrue(w.publish(packet));self.assertEqual(Path(path).read_bytes(),packet)
                self.assertEqual(r.sample(40,stamp+99_000_000),dict(source=40,sequence=7,stamp_ns=stamp))
                self.assertIsNone(r.sample(41,stamp));self.assertIsNone(r.sample(40,stamp+101_000_000));self.assertIsNone(r.sample(40,stamp-6_000_000))
            finally:r.close();w.close()
    def test_unverified_invalid_and_changed_sources_are_not_authority(self):
        with tempfile.TemporaryDirectory()as d:
            path=d+'/source';w=NativeSnapshotWriter(path);r=NativeSnapshotReader(path);stamp=time.monotonic_ns()
            try:
                for source,mode,flags in [(40,0,3),(40,1,0),(41,1,3)]:
                    w.publish(NATIVE.pack(0x4e4f5031,1,source,1,stamp,123456,mode,flags,0.,0.,0.,0));self.assertIsNone(r.sample(40,stamp))
            finally:r.close();w.close()
    def test_snapshot_cannot_follow_symlinks_or_read_public_files(self):
        with tempfile.TemporaryDirectory()as d:
            target=Path(d)/'target';target.write_bytes(bytes(NATIVE.size));link=Path(d)/'link';link.symlink_to(target)
            with self.assertRaises(OSError):NativeSnapshotWriter(link)
            r=NativeSnapshotReader(link)
            try:self.assertIsNone(r.sample(40))
            finally:r.close()
            target.chmod(0o666);r=NativeSnapshotReader(target)
            try:self.assertIsNone(r.sample(40))
            finally:r.close()
    def test_missing_snapshot_does_not_invent_a_source(self):
        with tempfile.TemporaryDirectory()as d:
            r=NativeSnapshotReader(d+'/absent')
            try:self.assertIsNone(r.sample(40))
            finally:r.close()

if __name__=='__main__':unittest.main()
