import os,sys,tempfile,time,unittest,subprocess
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from native_operator_source import NATIVE
from native_operator_snapshot import NativeSnapshotWriter,NativeSnapshotReader

class NativeSnapshotTests(unittest.TestCase):
    def test_receiver_packet_published_during_slow_read_is_not_future(self):
        from unittest.mock import patch
        with tempfile.TemporaryDirectory()as d:
            path=d+'/source';w=NativeSnapshotWriter(path);r=NativeSnapshotReader(path)
            w.publish(NATIVE.pack(0x4e4f5031,1,40,7,time.monotonic_ns(),123456,1,3,0.,0.,0.,0))
            actual_pread=os.pread;stamps=[]
            def delayed_read(fd,size,offset):
                time.sleep(.02);stamp=time.monotonic_ns();stamps.append(stamp)
                w.publish(NATIVE.pack(0x4e4f5031,1,40,8,stamp,123457,1,3,0.,0.,0.,0))
                return actual_pread(fd,size,offset)
            try:
                with patch('native_operator_snapshot.os.pread',side_effect=delayed_read):sample=r.sample(40)
                self.assertIsNotNone(sample);self.assertEqual(sample['stamp_ns'],stamps[0]);self.assertEqual(sample['sequence'],8)
            finally:r.close();w.close()
    def test_original_packet_time_and_identity_are_never_renewed(self):
        with tempfile.TemporaryDirectory()as d:
            path=d+'/source';w=NativeSnapshotWriter(path);r=NativeSnapshotReader(path)
            try:
                stamp=time.monotonic_ns();packet=NATIVE.pack(0x4e4f5031,1,40,7,stamp,123456,1,3,0.,0.,0.,0)
                self.assertTrue(w.publish(packet));self.assertEqual(Path(path).read_bytes(),packet)
                self.assertEqual(r.sample(40,stamp+99_000_000),dict(source=40,sequence=7,stamp_ns=stamp,mode=1,flags=3))
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
    def test_parent_reads_current_original_switch_without_waiting_for_rpc(self):
        from gait_transport_process import GaitTransportProcess
        with tempfile.TemporaryDirectory()as d:
            path=d+'/source';w=NativeSnapshotWriter(path);parent=GaitTransportProcess.__new__(GaitTransportProcess)
            parent.native_instance=40;parent.native_snapshot=NativeSnapshotReader(path)
            def blocked(*args):raise AssertionError('Source observation waited for a controller RPC')
            parent._call=blocked
            try:
                stamp=time.monotonic_ns();w.publish(NATIVE.pack(0x4e4f5031,1,40,8,stamp,123456,2,3,0.,0.,0.,0))
                value=parent.operator_authority();self.assertEqual(value['mode'],'paused');self.assertEqual(value['sequence'],8)
                self.assertEqual(value['stamp'],stamp/1e9);self.assertIsNone(value['reply_stamp']);self.assertEqual(value['source_transport'],'native_packet_snapshot')
            finally:parent.native_snapshot.close();w.close()
    def test_contended_reader_retains_original_time_and_still_expires(self):
        with tempfile.TemporaryDirectory()as d:
            path=d+'/source';w=NativeSnapshotWriter(path);r=NativeSnapshotReader(path);stamp=time.monotonic_ns()
            w.publish(NATIVE.pack(0x4e4f5031,1,40,7,stamp,123456,1,3,0.,0.,0.,0));self.assertIsNotNone(r.sample(40,stamp))
            code='import fcntl,os,sys,time; f=os.open(sys.argv[1],os.O_WRONLY);fcntl.lockf(f,fcntl.LOCK_EX);print("locked",flush=True);time.sleep(2)'
            child=subprocess.Popen([sys.executable,'-c',code,path],stdout=subprocess.PIPE,text=True)
            try:
                self.assertEqual(child.stdout.readline().strip(),'locked');self.assertEqual(r.sample(40,stamp+99_000_000)['stamp_ns'],stamp)
                self.assertIsNone(r.sample(40,stamp+101_000_000));self.assertIsNone(r.sample(41,stamp))
            finally:child.terminate();child.communicate(timeout=2);r.close();w.close()

if __name__=='__main__':unittest.main()
