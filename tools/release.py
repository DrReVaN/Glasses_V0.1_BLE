"""Prepare verified OTA release pairs and publish them without replacing versions."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import zipfile
from package import validate
from version import current, parse

ROOT=Path(__file__).resolve().parents[1]
LEGACY_COMMIT='f55c08171d64295b727fa613c2b06f6ab733096e'
LEGACY_SHA='0a83823a101f77a41048a1f020ce46e6cd29c149104a56febe2e66f6ce9bfb79'

def run(*args):
    if args[0]=='git': args=('git','-c','safe.directory='+ROOT.as_posix(),*args[1:])
    return subprocess.check_output(args,text=True,cwd=ROOT).strip()

def source_commit():
    head=run('git','rev-parse','HEAD')
    tag='v'+current()
    refs=run('git','ls-remote','--tags','origin','refs/tags/'+tag,'refs/tags/'+tag+'^{}').splitlines()
    if not refs: return head
    source=next((line.split()[0] for line in refs if line.endswith('^{}')),refs[0].split()[0])
    if source!=head:
        run('git','fetch','--no-tags','origin',source)
        changed=run('git','diff','--name-only',source,head).splitlines()
        # A publisher repair must not reassign an already allocated firmware
        # version. All firmware/build inputs still have to match its tag.
        if any(p not in {'tools/release.py','tests/test_release.py'} for p in changed):
            raise ValueError('Tagged firmware inputs changed; increment version')
    return source

def prepare():
    commit=source_commit()
    packages=[(ROOT/'build/application/smartglasses.bin',ROOT/'build/application/smartglasses.json',commit),
              (ROOT/'releases/legacy/Smartglasses-0.2.0-OTA.bin',ROOT/'releases/legacy/Smartglasses-0.2.0-OTA.json',LEGACY_COMMIT)]
    for binary,manifest,source in packages:
        image,info=validate(binary,manifest)
        version=info['version']; parse(version)
        if source==LEGACY_COMMIT:
            if version!='0.2.0' or hashlib.sha256(image).hexdigest()!=LEGACY_SHA: raise ValueError('Historical archive changed')
        elif version!=current(): raise ValueError('Build version does not match source')
        info['source_commit']=source; info['protocol']=1; info['min_bootloader']='0.2.0'
        out=ROOT/'build/releases'/('v'+version); out.mkdir(parents=True,exist_ok=True)
        name='Smartglasses-'+version+'-OTA'
        (out/(name+'.bin')).write_bytes(image)
        (out/(name+'.json')).write_text(json.dumps(info,indent=2)+'\n')
        text=('Smartglasses '+version+' für STM32WB35CE.\n\n'
              'Anwendungsupdate über BLE-OTA, Protokoll 1. Benötigt den installierten OTA-Löschkorrektur-Bootloader (f55c081 oder neuer). '
              'CPU2, Option Bytes und Geräteschlüssel werden nicht verändert.\n\n'
              'App 1.2.0 unterstützt Versionsauswahl, Updateprüfung und Rückwechsel. Vor Firmware 0.3.0 zuerst die neue App installieren.\n\n'
              '0.3.0 enthält die Sonderzeichen-/X=6-Korrektur und eine eindeutige Versionskennung. '
              '0.2.0 ist der archivierte OTA-Löschkorrektur-Stand; seine Gerätekennung unterscheidet ältere Builds noch nicht.\n\n'
              'Entwicklungsversion: Desktoptests/Build geprüft; die Abnahme aller neuen Funktionen auf der Brille bleibt separat.\n\n'
              f'Quellstand: {source}\nSHA-256: {info["sha256"]}\n')
        (out/'Release-Notes.md').write_text(text,encoding='utf-8')
        if (ROOT/'docs/RELEASES.md').exists(): shutil.copy2(ROOT/'docs/RELEASES.md',out/'Anleitung.md')
        with zipfile.ZipFile(out/(name+'.zip'),'w',zipfile.ZIP_DEFLATED) as z:
            for p in sorted(out.iterdir()):
                if p.name!='SHA256SUMS.txt' and p.suffix!='.zip':
                    entry=zipfile.ZipInfo(p.name,date_time=(1980,1,1,0,0,0))
                    entry.compress_type=zipfile.ZIP_DEFLATED
                    z.writestr(entry,p.read_bytes())
        assets=sorted(p for p in out.iterdir() if p.is_file())
        (out/'SHA256SUMS.txt').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.name+'\n' for p in assets if p.name!='SHA256SUMS.txt'))
        print(f'Prepared v{version}: {len(image)} bytes, {source}')

def api(path):
    try: return json.loads(run('gh','api',path))
    except subprocess.CalledProcessError:
        # GitHub's by-tag endpoint returns only published releases; drafts
        # remain visible to this authenticated publisher in the collection.
        if '/releases/tags/' not in path: raise
        base,tag=path.split('/tags/',1)
        for page in range(1,4):
            releases=json.loads(run('gh','api',base+f'?per_page=100&page={page}'))
            for release in releases:
                if release['tag_name']==tag: return release
            if len(releases)<100: break
        raise

def publish():
    repository=os.environ['GITHUB_REPOSITORY']
    if repository!='DrReVaN/Glasses_V0.1_BLE': raise ValueError('Wrong repository')
    for folder in sorted((ROOT/'build/releases').iterdir(),key=lambda p:parse(p.name[1:]),reverse=True):
        tag=folder.name; name='Smartglasses-'+tag[1:]+'-OTA'
        info=json.loads((folder/(name+'.json')).read_text()); source=info['source_commit']
        if info['version']!=tag[1:] or len(source)!=40 or any(c not in '0123456789abcdef' for c in source):
            raise ValueError('Release tag, version and source do not agree')
        validate(folder/(name+'.bin'),folder/(name+'.json'))
        # A pre-existing tag must never be moved, including during a retry.
        refs=run('git','ls-remote','--tags','origin','refs/tags/'+tag,'refs/tags/'+tag+'^{}').splitlines()
        if not refs:
            run('git','tag',tag,source)
            subprocess.run(['git','push','origin','refs/tags/'+tag],check=True)
        else:
            resolved=next((line.split()[0] for line in refs if line.endswith('^{}')),refs[0].split()[0])
            if resolved!=source: raise ValueError('Version tag already belongs to another commit; increment version')
        path='repos/'+repository+'/releases/tags/'+tag
        try: release=api(path)
        except subprocess.CalledProcessError:
            subprocess.run(['gh','release','create',tag,'--verify-tag','--draft','--prerelease',
                            '--title','Smartglasses '+tag[1:], '--notes-file',str(folder/'Release-Notes.md')],check=True)
            release=api(path)
        # The owner can create an empty historical release through GitHub when
        # GITHUB_TOKEN cannot tag a different workflow revision. Hide it again
        # before uploading; any release with assets remains immutable.
        if not release['draft'] and not release['assets']:
            subprocess.run(['gh','release','edit',tag,'--draft=true'],check=True)
            release=api(path)
            if not release['draft'] or release['assets']:
                raise ValueError('Empty release could not be safely prepared')
        existing={a['name']:a for a in release['assets']}
        for asset in sorted(folder.iterdir()):
            if not asset.is_file(): continue
            wanted='sha256:'+hashlib.sha256(asset.read_bytes()).hexdigest()
            if asset.name in existing:
                if existing[asset.name].get('digest')!=wanted: raise ValueError('Release asset differs; never overwrite versions')
            elif release['draft']:
                subprocess.run(['gh','release','upload',tag,str(asset)],check=True)
            else: raise ValueError('Published release is incomplete; manual review required')
        verified=api(path)
        actual={a['name']:a for a in verified['assets']}
        for asset in folder.iterdir():
            if actual[asset.name].get('digest')!='sha256:'+hashlib.sha256(asset.read_bytes()).hexdigest():
                raise ValueError('Uploaded asset digest does not match')
        if verified['draft']: subprocess.run(['gh','release','edit',tag,'--draft=false'],check=True)
        print('Published and verified '+tag)

if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--publish',action='store_true'); args=p.parse_args()
    if args.publish: publish()
    else: prepare()
