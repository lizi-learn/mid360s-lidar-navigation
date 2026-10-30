#!/usr/bin/env python3
"""Check DDS UDP socket capacity, or explicitly install larger Linux maximums.

Does not change socket defaults, interfaces, routes, clocks, or running processes.
Restart only the relevant ROS services after applying limits/profile changes.
"""
import argparse,json,os,pathlib,socket,time

CAPACITY=4194304
KEYS=('rmem_max','wmem_max','rmem_default','wmem_default')

def check():
    values={k:int((pathlib.Path('/proc/sys/net/core')/k).read_text()) for k in KEYS}
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,CAPACITY)
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,CAPACITY)
        actual={k:sock.getsockopt(socket.SOL_SOCKET,v) for k,v in [('SO_RCVBUF',socket.SO_RCVBUF),('SO_SNDBUF',socket.SO_SNDBUF)]}
    # Linux reports twice the requested socket size to include kernel overhead.
    return {'requested_bytes':CAPACITY,'linux_limits':values,'actual_socket_bytes':actual,'ready':values['rmem_max']>=CAPACITY and values['wmem_max']>=CAPACITY and min(actual.values())>=2*CAPACITY,'packets_sent':0}

def apply():
    if os.geteuid()!=0:raise PermissionError('--apply requires root; default check is read-only')
    before=check();backup=pathlib.Path('/var/lib/mid360s-navigation/dds-buffers');backup.mkdir(parents=True,exist_ok=True)
    config=pathlib.Path('/etc/sysctl.d/70-p2-navigation-dds-buffers.conf')
    stamp=str(time.time_ns());record={'before':before,'original_sysctl_file':config.read_text() if config.exists() else None}
    (backup/(stamp+'.json')).write_text(json.dumps(record,indent=2))
    # Raising maximums enables explicit per-DDS-socket allocation. Do not reduce
    # an administrator's larger existing maximum, or modify default sizes.
    limits={k:max(CAPACITY,before['linux_limits'][k]) for k in ('rmem_max','wmem_max')}
    config.write_text('# DDS point clouds: maximum caps only; default socket sizes unchanged.\n'+''.join('net.core.'+k+' = '+str(v)+'\n' for k,v in limits.items()));config.chmod(0o644)
    for k,v in limits.items():(pathlib.Path('/proc/sys/net/core')/k).write_text(str(v)+'\n')
    after=check();assert after['ready']
    assert all(before['linux_limits'][k]==after['linux_limits'][k] for k in ('rmem_default','wmem_default'))
    return {'before':before,'after':after,'backup':str(backup/(stamp+'.json')),'restart_required':True}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--apply',action='store_true',help='Explicitly raise and persist Linux maximum socket-buffer limits; requires root');args=p.parse_args()
    report=apply() if args.apply else check();print(json.dumps(report,indent=2))
    if not args.apply and not report['ready']:raise SystemExit(2)
