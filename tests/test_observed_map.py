import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile
import numpy as np
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'navigation'))
from observed_map import ObservedMap, FILENAME, signature, saved_cells, read_layer
from observed_map import ObservationWorker
from reception_maps import MapLibrary
from reception_catalog import Catalog, blank
from reception_annotations import AnnotationLayer


def fixture():
    cells = [0]*6400
    cells[40*80+60] = 100
    cells[40*80+61] = -1
    return dict(width=80, height=80, resolution=.05, origin=[-2., -2.], cells=cells, revision=1)


def packet(t, distance=4.):
    return dict(source_ns=int(t*1e9), source_monotonic=t, x=.025, y=.025, yaw=0.,
                angle_min=-math.pi, angle_increment=2*math.pi/3600,
                range_min=.12, range_max=12., ranges=[distance]*3600)


def clear(model, start=100.):
    result=[model.observe(packet(start+i*.3), start+i*.3+.01) for i in range(9)]
    model.merge(manual=True)
    return result


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.g = fixture()
        self.model = ObservedMap()
        self.model.bind(self.g, self.root)

    def test_repeated_finite_crossings_clear_old_occupancy_not_unknown_or_base(self):
        self.assertFalse(any(clear(self.model)[:7]))
        self.assertEqual(self.model.apply(self.g)[3260], 0)
        self.assertEqual(self.model.apply(self.g)[3261], -1)
        self.assertEqual(self.g['cells'][3260], 100)
        self.assertEqual(self.model.status()['cleared_cells'], 1)

    def test_single_frame_duplicate_old_future_and_empty_bins_cannot_clear(self):
        for i in range(20):
            self.model.observe(packet(100), 100.01)
        self.assertEqual(self.model.status()['frames'], 1)
        for i in range(15):
            t = 101+i*.3
            for value in (math.inf, math.nan, .01, 99.):
                self.model.observe(packet(t, value), t+.01)
        self.assertEqual(self.model.apply(self.g)[3260], 100)
        self.assertFalse(self.model.observe(packet(200), 201.))
        self.assertFalse(self.model.observe(packet(210), 209.))
        self.assertEqual(self.model.status()['source_ns'], int(105.2*1e9))

    def test_temporary_return_does_not_shrink_previously_merged_map(self):
        clear(self.model)
        hit = packet(104.)
        hit['ranges'][1800] = 1.0
        self.assertFalse(self.model.observe(hit, 104.01))
        self.assertEqual(self.model.apply(self.g)[3260], 0)
        self.assertEqual(self.model.status()['cleared_cells'], 1)
        clear(self.model, 105.)
        self.assertEqual(self.model.apply(self.g)[3260], 0)

    def test_new_person_on_base_free_does_not_become_permanent(self):
        for i in range(15):
            p = packet(100+i*.3)
            p['ranges'][1800] = .5
            self.model.observe(p, 100+i*.3+.01)
        self.assertEqual(self.model.apply(self.g)[40*80+50], 0)
        self.assertEqual(self.model.apply(self.g)[3260], 100)

    def test_occlusion_long_gap_and_lost_localization_reset_accumulation(self):
        for i in range(7):
            self.model.observe(packet(100+i*.3), 100+i*.3+.01)
        self.model.observe(packet(102.1), 102.11, localized=False)
        self.model.observe(packet(102.4), 102.41)
        self.assertEqual(self.model.status()['cleared_cells'], 0)
        self.model.observe(packet(105.), 105.01)
        self.assertEqual(self.model.status()['cleared_cells'], 0)
        clear(self.model, 106.)
        before = self.model.apply(self.g)
        self.model.observe(packet(110., math.inf), 110.01)
        self.assertEqual(self.model.apply(self.g), before)

    def test_partial_scan_outside_field_of_view_is_not_free(self):
        for i in range(10):
            p = packet(100+i*.3)
            p['angle_min'] = math.pi/2
            p['ranges'] = [4.]*300
            self.model.observe(p, 100+i*.3+.01)
        self.assertEqual(self.model.apply(self.g)[3260], 100)

    def test_stopped_learning_keeps_prior_map_and_freezes_new_changes(self):
        clear(self.model)
        self.model.set_enabled(False)
        self.model.observe(packet(104., 1.), 104.01)
        self.assertEqual(self.model.apply(self.g)[3260], 0)
        self.model.set_enabled(True)
        self.model.observe(packet(105., 1.), 105.01)
        self.assertEqual(self.model.apply(self.g)[3260], 0)
        with self.assertRaises(ValueError):
            self.model.set_enabled('yes')

    def test_restart_persistence_source_provenance_and_base_fingerprint(self):
        clear(self.model)
        self.model.flush(force=True)
        d = json.loads((self.root/FILENAME).read_text())
        self.assertEqual(d['entries'][0]['first_source_ns'], 100000000000)
        self.assertGreaterEqual(d['entries'][0]['observations'], 8)
        next_model = ObservedMap(); next_model.bind(self.g, self.root)
        self.assertEqual(next_model.apply(self.g)[3260], 0)
        other = dict(self.g, cells=list(self.g['cells']), revision=2)
        other['cells'][0] = 100
        next_model.bind(other, self.root)
        self.assertEqual(next_model.apply(other)[3260], 100)
        self.assertEqual(self.model.apply(other)[3260], 100)
        ros = dict(self.g, resolution=.05000000074505806)
        self.assertEqual(signature(ros), signature(self.g))

    def test_invalid_persisted_unknown_cell_is_rejected_without_clearing(self):
        clear(self.model); self.model.flush(force=True)
        data = json.loads((self.root/FILENAME).read_text())
        data['entries'][0]['index'] = 3261
        (self.root/FILENAME).write_text(json.dumps(data))
        with self.assertRaises(ValueError):
            read_layer(self.root/FILENAME, self.g)
        m = ObservedMap(); m.bind(self.g, self.root)
        self.assertFalse(m.status()['enabled'])
        self.assertEqual(m.apply(self.g), self.g['cells'])

    def test_authored_keepout_applies_after_clear(self):
        clear(self.model)
        measured = dict(self.g, cells=self.model.apply(self.g))
        cells, _ = AnnotationLayer().apply(measured, [dict(id='wall', kind='keepout', name='wall',
            vertices=[[1.025, -.2], [1.025, .2]], width=.1)])
        self.assertEqual(cells[3260], 100)

    def test_old_map_packet_and_unbinding_cannot_modify_restored_layer(self):
        wrong = packet(100.); wrong['map_revision'] = 0
        self.assertFalse(self.model.observe(wrong, 100.01))
        clear(self.model); self.model.unbind()
        before = (self.root/FILENAME).read_bytes()
        self.assertFalse(self.model.observe(packet(105.), 105.01))
        self.assertEqual(self.model.apply(self.g), self.g['cells'])
        self.model.flush(force=True)
        self.assertEqual((self.root/FILENAME).read_bytes(), before)

    def test_corrupt_json_and_flush_error_do_not_abort_shutdown(self):
        for value in ([], dict(schema=1, enabled=True, base_sha256=signature(self.g), entries=[None])):
            (self.root/FILENAME).write_text(json.dumps(value))
            with self.assertRaises(ValueError):
                read_layer(self.root/FILENAME, self.g)
        worker = ObservationWorker(self.model, lambda: True, lambda: None)
        def fail(*args, **kwargs):
            raise OSError('synthetic storage failure')
        self.model.flush = fail
        worker.close()
        self.assertFalse(worker.thread.is_alive())
        self.assertEqual(self.model.error, 'OSError')

    def test_map_versions_export_import_and_reset_include_measured_layer(self):
        directory = self.root/'maps/office'; directory.mkdir(parents=True)
        g = self.g
        pixels = np.asarray(g['cells']).reshape(80, 80)[::-1]
        pixels = np.where(pixels < 0, 205, np.where(pixels > 60, 0, 254)).astype(np.uint8)
        (directory/'mapped.pgm').write_bytes(b'P5\n80 80\n255\n'+pixels.tobytes())
        (directory/'mapped.yaml').write_text(yaml.safe_dump(dict(image='mapped.pgm', resolution=.05,
            origin=[-2., -2., 0.], negate=0, occupied_thresh=.65, free_thresh=.196)))
        catalog = Catalog(self.root/'catalog'); catalog.save(blank('office'), 0)
        library = MapLibrary(self.root/'maps', catalog)
        model = ObservedMap(); model.bind(g, directory); clear(model); model.flush(force=True)
        before = (directory/'mapped.pgm').read_bytes()
        version = library.snapshot('office', 'observed')
        raw, manifest = library.export('office')
        self.assertIn('map/'+FILENAME, manifest['files'])
        library.import_bundle(raw, 'another_robot')
        self.assertEqual(library.raster('another_robot')['cells'][3260], 0)
        model.reset()
        self.assertEqual(library.raster('office')['cells'][3260], 100)
        library.restore('office', version['id'])
        self.assertEqual(library.raster('office')['cells'][3260], 0)
        self.assertEqual((directory/'mapped.pgm').read_bytes(), before)
        model.reset()
        for i in range(9):model.observe(packet(200+i),200+i+.01)
        model.flush(force=True)
        raw,_=library.export('office');library.import_bundle(raw,'pending_robot')
        imported=library.raster('pending_robot')
        self.assertEqual(imported['cells'][3260],100)
        self.assertEqual(imported['map_observations']['pending_cells'],1)

    def test_collection_keeps_fixed_map_until_daily_merge_and_survives_restart(self):
        revision=self.model.revision
        for i in range(9):self.model.observe(packet(100+i),100+i+.01)
        self.assertEqual(self.model.status()['pending_cells'],1)
        self.assertEqual(self.model.apply(self.g)[3260],100)
        self.assertEqual(self.model.revision,revision)
        wall=self.model.last_merge_wall
        self.assertFalse(self.model.merge(now_wall=wall+86399))
        self.model.flush(force=True)
        reloaded=ObservedMap();reloaded.bind(self.g,self.root)
        self.assertEqual(reloaded.status()['pending_cells'],1)
        self.assertEqual(reloaded.apply(self.g)[3260],100)
        self.assertTrue(reloaded.merge(now_wall=wall+86400))
        self.assertEqual(reloaded.apply(self.g)[3260],0)
        self.assertEqual(reloaded.status()['pending_cells'],0)
        self.assertEqual(reloaded.status()['cleared_cells'],1)
        self.assertFalse(reloaded.merge_due(wall+86401))

    def test_manual_merge_waits_for_idle_without_interrupting_navigation(self):
        import time
        for i in range(9):self.model.observe(packet(100+i),100+i+.01)
        idle=[False];published=[]
        worker=ObservationWorker(self.model,lambda:True,lambda:published.append(True),lambda:idle[0])
        try:
            self.model.request_merge();worker.event.set();time.sleep(.1)
            self.assertEqual(published,[])
            self.assertEqual(self.model.apply(self.g)[3260],100)
            idle[0]=True;worker.event.set()
            end=time.monotonic()+2
            while not published and time.monotonic()<end:time.sleep(.02)
            self.assertEqual(published,[True])
            self.assertEqual(self.model.apply(self.g)[3260],0)
            self.assertFalse(self.model.merge_requested)
        finally:worker.close()

    def test_daily_automatic_merge_waits_for_idle_and_does_not_repeat(self):
        import time
        for i in range(9):self.model.observe(packet(100+i),100+i+.01)
        self.model.last_merge_wall=time.time()-86401
        idle=[False];published=[]
        worker=ObservationWorker(self.model,lambda:True,lambda:published.append(True),lambda:idle[0])
        try:
            worker.event.set();time.sleep(.1)
            self.assertEqual(published,[])
            idle[0]=True;worker.event.set();end=time.monotonic()+2
            while not published and time.monotonic()<end:time.sleep(.02)
            self.assertEqual(published,[True])
            self.assertFalse(self.model.merge_due(time.time()+10))
        finally:worker.close()

    def test_previous_schema_retains_already_confirmed_map_and_pause(self):
        clear(self.model);self.model.flush(force=True)
        data=json.loads((self.root/FILENAME).read_text())
        data['schema']=1;data['enabled']=False;data.pop('pending');data.pop('last_merge_wall')
        (self.root/FILENAME).write_text(json.dumps(data))
        new=ObservedMap();new.bind(self.g,self.root)
        self.assertEqual(new.apply(self.g)[3260],0)
        self.assertEqual(new.status()['pending_cells'],0)
        self.assertFalse(new.enabled)


if __name__ == '__main__':
    unittest.main()
