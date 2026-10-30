import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from map_persistence import configure_save_request,validate_map_metadata


class MapPersistenceTests(unittest.TestCase):
    def metadata(self, directory, **values):
        data={'image':'mapped.pgm','mode':'trinary','negate':0,
              'resolution':.05,'origin':[0,0,0],'free_thresh':.196,'occupied_thresh':.65}
        data.update(values);path=Path(directory)/'mapped.yaml'
        path.write_text(yaml.safe_dump(data));return path

    def test_saved_unknown_pixel_stays_unknown_on_humble_reload(self):
        req=configure_save_request(SimpleNamespace(),'fixture/mapped')
        # Actual Humble output palette and loader inequalities; this catches
        # the former .25 save/load configuration that made 205 free.
        cells=[]
        for pixel in (0,205,254):
            occupancy=1-pixel/255
            cells.append(100 if occupancy>req.occupied_thresh else
                         0 if occupancy<req.free_thresh else -1)
        self.assertEqual(cells,[100,-1,0])
        self.assertEqual((req.map_mode,req.image_format,req.map_topic),('trinary','pgm','/map'))

    def test_load_guard_rejects_legacy_metadata_before_use(self):
        with tempfile.TemporaryDirectory() as d:
            path=self.metadata(d,free_thresh=.25)
            original=path.read_bytes()
            with self.assertRaisesRegex(ValueError,'未知区域'):validate_map_metadata(path)
            self.assertEqual(path.read_bytes(),original)

    def test_standard_saved_metadata_is_accepted(self):
        with tempfile.TemporaryDirectory() as d:
            path=self.metadata(d)
            self.assertEqual(validate_map_metadata(path)['free_thresh'],.196)

    def test_invalid_thresholds_and_other_encodings_are_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            for values in ({'free_thresh':float('nan')},{'occupied_thresh':float('inf')},
                           {'occupied_thresh':.1},{'free_thresh':0},
                           {'negate':1},{'mode':'scale'}):
                with self.subTest(values=values),self.assertRaises(ValueError):
                    validate_map_metadata(self.metadata(d,**values))


if __name__=='__main__':unittest.main()
