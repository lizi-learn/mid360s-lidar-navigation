import base64,json,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from reception_transfer import ContentTransfer
from reception_catalog import Catalog,blank
from reception_cloud import CloudStore
class TransferTest(unittest.TestCase):
    def test_atomic_unicode_reordered_retry_and_conflict(self):
        with tempfile.TemporaryDirectory()as root:
            store=Catalog(Path(root)/'catalog');cloud=CloudStore(Path(root)/'cloud');transfer=ContentTransfer(store,cloud)
            d=blank('map');d['products']=[{'id':'x','name':'介绍','script':'双足服务机器人。'*200,'tags':''}]
            raw=json.dumps(d,ensure_ascii=False).encode();parts=[raw[i:i+1000]for i in range(0,len(raw),1000)]
            def call(i):return transfer.receive({'operation':'stage','upload_id':'batch','map_id':'map','kind':'catalog','revision':0,'index':i,'total':len(parts),'chunk':base64.b64encode(parts[i]).decode()})
            for i in reversed(range(1,len(parts))):self.assertFalse(call(i)['committed']);self.assertEqual(store.read('map')['revision'],0)
            final=call(0);self.assertTrue(final['committed']);self.assertEqual(store.read('map')['revision'],1);self.assertEqual(call(0),final)
            parts[0]=b'other'
            with self.assertRaises(ValueError):call(0)
    def test_invalid_complete_transfer_never_replaces_content(self):
        with tempfile.TemporaryDirectory()as root:
            c=Catalog(root);t=ContentTransfer(c,CloudStore(Path(root)/'cloud'))
            for raw in [b'{broken',json.dumps(blank('different')).encode()]:
                with self.assertRaises(ValueError):t.receive({'operation':'stage','upload_id':'a','map_id':'map','kind':'catalog','revision':0,'index':0,'total':1,'chunk':base64.b64encode(raw).decode()})
                self.assertEqual(c.read('map')['revision'],0)
if __name__=='__main__':unittest.main()
