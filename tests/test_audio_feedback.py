import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from audio_feedback import AudioSettings, CuePolicy, PHRASES, save_resource
from audio_worker import LocalPlayer, status_url


def status(phase, generation=1, token='same'):
    return {'session':{'phase':phase,'generation':generation},'request_token':token}


class AudioPolicyTests(unittest.TestCase):
    def test_attach_to_active_navigation_is_silent(self):
        p=CuePolicy();self.assertIsNone(p.observe(status('executing'),1.))
        self.assertIsNone(p.observe(status('executing'),2.))

    def test_only_actual_execution_ack_starts_voice(self):
        p=CuePolicy();p.observe(status('idle',0),1.)
        for i,phase in enumerate(['planning','ready','starting']):self.assertIsNone(p.observe(status(phase),2.+i))
        cue=p.observe(status('executing'),5.);self.assertEqual(cue.kind,'started')
        for i in range(1000):self.assertIsNone(p.observe(status('executing'),5.1+i*.01))

    def test_transient_obstacle_and_periodic_replan_silent(self):
        p=CuePolicy();p.observe(status('executing'),1.)
        self.assertIsNone(p.observe(status('replanning'),2.))
        self.assertIsNone(p.observe(status('executing'),2.9))

    def test_obstacle_is_merged_with_replanning_and_cooldown(self):
        p=CuePolicy();p.observe(status('executing'),1.);p.observe(status('replanning'),2.)
        cue=p.observe(status('replanning'),3.6);self.assertEqual(cue.kind,'obstacle')
        self.assertIn('调整路线',PHRASES[cue.kind]);self.assertIsNone(p.observe(status('replanning'),4.))
        self.assertIsNone(p.observe(status('executing'),4.2))
        p.observe(status('replanning'),5.);self.assertIsNone(p.observe(status('replanning'),7.))
        self.assertIsNone(p.observe(status('executing'),8.))

    def test_long_obstacle_resume_is_announced(self):
        p=CuePolicy();p.observe(status('executing'),1.);p.observe(status('replanning'),2.)
        p.observe(status('replanning'),4.);self.assertEqual(p.observe(status('executing'),9.).kind,'continued')

    def test_stop_invalidates_queued_start(self):
        p=CuePolicy();p.observe(status('starting'),1.);cue=p.observe(status('executing'),2.)
        stopped=p.observe(status('cancelled'),2.1);self.assertEqual(stopped.kind,'stopped')
        self.assertFalse(p.valid(cue,status('cancelled'),2.2));self.assertTrue(p.valid(stopped,status('cancelled'),2.2))

    def test_takeover_pause_arrival_failure_are_distinct(self):
        for end,kind in [('manual','manual'),('paused','paused'),('arrived','arrived'),('blocked','failed'),('stopped','stopped')]:
            p=CuePolicy();p.observe(status('executing'),1.);self.assertEqual(p.observe(status(end),2.).kind,kind)
            self.assertIsNone(p.observe(status(end),3.))
        p=CuePolicy();p.observe(status('planning'),1.);self.assertEqual(p.observe(status('blocked'),2.).kind,'no_path')

    def test_reconnect_restart_and_new_goal_drop_old_speech(self):
        p=CuePolicy();p.observe(status('starting'),1.);cue=p.observe(status('executing'),2.)
        p.disconnected();self.assertIsNone(p.observe(status('executing'),3.));self.assertFalse(p.valid(cue,status('executing'),3.1))
        self.assertIsNone(p.observe(status('executing',token='newserver'),4.))
        p.observe(status('planning',2,token='newserver'),5.);self.assertFalse(p.valid(cue,status('executing',2),5.))

    def test_queued_voice_expires_but_playing_sentence_is_not_cut_at_3s(self):
        p=CuePolicy();p.observe(status('starting'),1.);cue=p.observe(status('executing'),2.)
        self.assertFalse(p.valid(cue,status('executing'),6.))
        self.assertTrue(p.valid(cue,status('executing'),6.,max_age=None))
        self.assertIsNone(p.observe(status('executing'),1.));self.assertFalse(p.valid(cue,status('executing'),1.))


class AudioSettingsTests(unittest.TestCase):
    def test_persistent_volume_mute_and_test_event(self):
        with tempfile.TemporaryDirectory()as d:
            s=AudioSettings(Path(d)/'audio-settings.json');self.assertEqual(s.read()['volume'],55)
            s.update({'enabled':False,'volume':30});self.assertFalse(s.read()['enabled'])
            r=s.update({'test':True});self.assertEqual(r['test_generation'],1)
            self.assertEqual(AudioSettings(s.path).read(),r)

    def test_bad_configuration_does_not_break_navigation_status(self):
        with tempfile.TemporaryDirectory()as d:
            s=AudioSettings(Path(d)/'audio-settings.json');s.path.write_text('[bad')
            self.assertFalse(s.status()['enabled']);self.assertEqual(s.status()['error'],'audio_settings_invalid')
            s.path.write_text('[]');self.assertFalse(s.status()['enabled'])
            s.update({'enabled':True});self.assertTrue(s.status()['enabled'])
            s.path.with_name('audio-feedback-state.json').write_text('[]');self.assertIsNone(s.status()['player'])

    def test_reject_invalid_settings_without_partial_write(self):
        with tempfile.TemporaryDirectory()as d:
            s=AudioSettings(Path(d)/'audio-settings.json')
            for data in [{'volume':True},{'volume':-1},{'volume':101},{'volume':float('nan')},{'enabled':'yes'},{'test':1},{'start':True}]:
                with self.assertRaises(ValueError):s.update(data)
            self.assertFalse(s.path.exists())

    def test_legacy_g12_save_route_keeps_audio_and_map_operations_separate(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        with tempfile.TemporaryDirectory() as d:
            node=SimpleNamespace(audio=AudioSettings(Path(d)/'audio-settings.json'),save_map=Mock(return_value={'map_saved':True}))
            result=save_resource(node,{'audio_feedback':{'volume':33,'test':True}})
            self.assertEqual(result['volume'],33);self.assertEqual(result['test_generation'],1);node.save_map.assert_not_called()
            with self.assertRaises(ValueError):save_resource(node,{'audio_feedback':{'enabled':False},'name':'office'})
            self.assertTrue(node.audio.read()['enabled']);node.save_map.assert_not_called()
            self.assertEqual(save_resource(node,{'name':'office'}),{'map_saved':True});node.save_map.assert_called_once_with('office')

    def test_only_loopback_get_observer(self):
        self.assertEqual(status_url('http://127.0.0.1:8930'),'http://127.0.0.1:8930/api/status?summary=1')
        for url in ['http://remote.example','file:///tmp/a','http://localhost:8930/api/confirm','http://user:pw@localhost:8930']:
            with self.assertRaises(ValueError):status_url(url)
        source=(Path(__file__).resolve().parents[1]/'navigation/audio_worker.py').read_text()
        self.assertNotIn('method="POST"',source);self.assertNotIn('gait_client',source);self.assertNotIn('/api/confirm',source)

    def test_player_uses_fixed_local_cue_and_own_stream_volume(self):
        with tempfile.TemporaryDirectory()as d:
            p=LocalPlayer(d,d,'body-speaker');policy=CuePolicy();policy.observe(status('starting'),1.);cue=policy.observe(status('executing'),2.)
            with patch('audio_worker.subprocess.Popen')as spawn:
                p.play(cue,55,2.);args=spawn.call_args.args[0]
                self.assertEqual(args[0],'paplay');self.assertIn('--device=body-speaker',args);self.assertIn('--volume=36045',args)
                self.assertNotIn('shell',spawn.call_args.kwargs)
            import wave
            with wave.open(str(Path(d)/'started.wav'))as wav:self.assertLess(wav.getnframes()/wav.getframerate(),1.)
            with self.assertRaises(ValueError):p.path('../../unsafe')

if __name__=='__main__':unittest.main()
