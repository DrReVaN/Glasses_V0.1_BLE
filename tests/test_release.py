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

TOOLS=Path(__file__).resolve().parents[1]/'tools'
spec=importlib.util.spec_from_file_location('release',TOOLS/'release.py')
release=importlib.util.module_from_spec(spec);spec.loader.exec_module(release)

class ReleaseTests(unittest.TestCase):
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

if __name__=='__main__': unittest.main()
