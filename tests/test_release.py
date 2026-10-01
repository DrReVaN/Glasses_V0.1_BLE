import importlib.util
import json
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import hashlib
import zlib
import shutil

TOOLS=Path(__file__).resolve().parents[1]/'tools'
spec=importlib.util.spec_from_file_location('release',TOOLS/'release.py')
release=importlib.util.module_from_spec(spec);spec.loader.exec_module(release)

class ReleaseTests(unittest.TestCase):
    def test_new_package_does_not_regenerate_historical_release_assets(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary); folder=self.fixture(root)
            app=root/'build/application';app.mkdir()
            for ext in ('bin','json'):
                shutil.copy2(folder/('Smartglasses-0.3.0-OTA.'+ext),app/('smartglasses.'+ext))
            legacy=root/'releases/legacy';legacy.mkdir(parents=True)
            for ext in ('bin','json'):
                shutil.copy2(release.ROOT/'releases/legacy'/('Smartglasses-0.2.0-OTA.'+ext),legacy)
            old=root/'build/releases/v0.2.0';old.mkdir()
            (old/'Anleitung.md').write_text('Original published instructions')
            with patch.object(release,'ROOT',root),patch.object(release,'source_commit',lambda:'1'*40),patch.object(release,'current',lambda:'0.3.0'):
                release.prepare()
                self.assertEqual(list(old.iterdir()),[old/'Anleitung.md'])
                self.assertEqual((old/'Anleitung.md').read_text(),'Original published instructions')
                self.assertTrue((folder/'Smartglasses-0.3.0-OTA.zip').exists())
                binary=legacy/'Smartglasses-0.2.0-OTA.bin'
                binary.write_bytes(binary.read_bytes()[:-1])
                with self.assertRaises(ValueError): release.prepare()

    def fixture(self,root):
        folder=root/'build/releases/v0.3.0';folder.mkdir(parents=True)
        data=bytearray(512);struct.pack_into('<II',data,0,0x20007800,0x0801014d)
        data[0x140:0x14c]=b'SGV1'+struct.pack('<HHHBB',0,3,0,1,0)
        name='Smartglasses-0.3.0-OTA'
        (folder/(name+'.bin')).write_bytes(data)
        info={'format':1,'target':'STM32WB35CE','profile':'application','address':'0x08010000','size':512,
              'version':'0.3.0','protocol':1,'min_bootloader':'0.2.0','crc32':f'{zlib.crc32(data):08x}',
              'sha256':hashlib.sha256(data).hexdigest(),'source_commit':'1'*40}
        (folder/(name+'.json')).write_text(json.dumps(info));(folder/'Release-Notes.md').write_text('Test')
        return folder

    def test_complete_draft_publish_is_retryable_and_never_clobbers(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);folder=self.fixture(root); state={'tag':'','release':None};commands=[]
            def run(*args):
                commands.append(args)
                if args[0:2]==('git','ls-remote'): return state['tag']+'\trefs/tags/v0.3.0' if state['tag'] else ''
                return ''
            def api(path):
                if state['release'] is None: raise subprocess.CalledProcessError(1,['gh','api'])
                return state['release']
            def call(args,**kwargs):
                commands.append(tuple(args)); self.assertNotIn('--clobber',args)
                if args[:2]==['git','push']: state['tag']='1'*40
                elif args[:3]==['gh','release','create']: state['release']={'draft':True,'assets':[]}
                elif args[:3]==['gh','release','upload']:
                    self.assertTrue(state['release']['draft'])
                    p=Path(args[-1]);state['release']['assets'].append({'name':p.name,'digest':'sha256:'+hashlib.sha256(p.read_bytes()).hexdigest()})
                elif args[:3]==['gh','release','edit']:
                    self.assertEqual(len(state['release']['assets']),3);state['release']['draft']=False
            with patch.object(release,'ROOT',root),patch.object(release,'run',run),patch.object(release,'api',api),patch.object(release.subprocess,'run',call),patch.dict(release.os.environ,{'GITHUB_REPOSITORY':'DrReVaN/Glasses_V0.1_BLE'}):
                release.publish(); self.assertFalse(state['release']['draft']);before=len(commands)
                release.publish(); self.assertFalse(any(c[:3]==('gh','release','upload') for c in commands[before:]))
                (folder/'Release-Notes.md').write_text('Changed')
                with self.assertRaises(ValueError): release.publish()
                state['tag']='2'*40
                with self.assertRaises(ValueError): release.publish()

    def test_invalid_package_cannot_create_a_tag_or_release(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);folder=self.fixture(root)
            binary=folder/'Smartglasses-0.3.0-OTA.bin';binary.write_bytes(binary.read_bytes()[:-1])
            with patch.object(release,'ROOT',root),patch.object(release,'run') as run,patch.object(release,'api') as api,patch.dict(release.os.environ,{'GITHUB_REPOSITORY':'DrReVaN/Glasses_V0.1_BLE'}):
                with self.assertRaises(ValueError): release.publish()
                run.assert_not_called();api.assert_not_called()

    def test_empty_owner_release_is_hidden_before_uploading(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);folder=self.fixture(root)
            state={'draft':False,'assets':[]};commands=[]
            def run(*args): return '1'*40+'\trefs/tags/v0.3.0'
            def call(args,**kwargs):
                commands.append(args)
                if args[:3]==['gh','release','edit']:
                    if args[-1]=='--draft=true': state['draft']=True
                    else:
                        self.assertEqual(len(state['assets']),3);state['draft']=False
                elif args[:3]==['gh','release','upload']:
                    self.assertTrue(state['draft'])
                    p=Path(args[-1]);state['assets'].append({'name':p.name,'digest':'sha256:'+hashlib.sha256(p.read_bytes()).hexdigest()})
            with patch.object(release,'ROOT',root),patch.object(release,'run',run),patch.object(release,'api',lambda path:state),patch.object(release.subprocess,'run',call),patch.dict(release.os.environ,{'GITHUB_REPOSITORY':'DrReVaN/Glasses_V0.1_BLE'}):
                release.publish()
                self.assertFalse(state['draft'])
                self.assertEqual(commands[0],['gh','release','edit','v0.3.0','--draft=true'])
                state['assets'].pop()
                with self.assertRaises(ValueError): release.publish()
                self.assertFalse(state['draft'])

    def test_draft_lookup_uses_authenticated_release_collection(self):
        draft={'tag_name':'v0.3.0','draft':True,'assets':[],'id':7}
        def run(*args):
            if '/releases/tags/' in args[-1]: raise subprocess.CalledProcessError(1,['gh','api'])
            self.assertEqual(args[-1],'repos/DrReVaN/Glasses_V0.1_BLE/releases?per_page=100&page=1')
            return json.dumps([draft])
        with patch.object(release,'run',run):
            self.assertEqual(release.api('repos/DrReVaN/Glasses_V0.1_BLE/releases/tags/v0.3.0'),draft)

    def test_publisher_repair_keeps_existing_firmware_tag_identity(self):
        state={'changed':'tools/release.py\ntests/test_release.py'}
        def run(*args):
            if args[1]=='rev-parse': return '2'*40
            if args[1]=='ls-remote': return '1'*40+'\trefs/tags/v0.3.0'
            if args[1]=='diff': return state['changed']
            self.assertEqual(args[1:3],('fetch','--no-tags'))
            return ''
        with patch.object(release,'run',run),patch.object(release,'current',lambda:'0.3.0'):
            self.assertEqual(release.source_commit(),'1'*40)
            state['changed']='Core/Src/glasses_core.c'
            with self.assertRaises(ValueError): release.source_commit()

if __name__=='__main__': unittest.main()
