"""Local installation identity and data separation; contains no motor settings."""
import argparse,hashlib,ipaddress,json,os,re,uuid
from pathlib import Path
NAME=re.compile(r'[A-Za-z0-9_-]{1,48}\Z')

def validate(data):
    if not isinstance(data,dict) or type(data.get('schema_version')) is not int or data.get('schema_version')!=1 or not isinstance(data.get('name'),str) or not NAME.fullmatch(data['name']):raise ValueError('Invalid robot profile name/schema')
    serial=data.get('serial')
    if not isinstance(serial,str) or not re.fullmatch(r'[A-Za-z0-9._-]{6,128}',serial):raise ValueError('Robot serial required')
    try:uuid.UUID(data['installation_id'])
    except (ValueError,KeyError,TypeError,AttributeError):raise ValueError('Installation UUID required')
    for key in ('host_ip','lidar_ip'):
        try:ip=ipaddress.ip_address(data[key])
        except (ValueError,KeyError,TypeError):raise ValueError('Invalid LiDAR endpoint')
        if ip.version!=4 or ip.is_unspecified or ip.is_multicast or ip.is_loopback or ip.is_reserved:raise ValueError('Unicast sensor IPv4 required')
    if data['host_ip']==data['lidar_ip']:raise ValueError('Host and sensor must differ')
    return data

def create(name,serial,host_ip,lidar_ip):
    return validate(dict(schema_version=1,name=name,serial=serial,installation_id=str(uuid.uuid4()),host_ip=host_ip,lidar_ip=lidar_ip))

def load(path):return validate(json.loads(Path(path).read_text()))

def identity(profile):
    return {'name':profile['name'],'installation_id':profile['installation_id'],'serial_sha256':hashlib.sha256(profile['serial'].encode()).hexdigest()}

def check_machine(profile,actual_serial=None):
    if actual_serial is None:
        p=Path('/proc/device-tree/serial-number')
        if not p.is_file():raise ValueError('Cannot verify this machine serial; physical profile refused')
        actual_serial=p.read_bytes().strip(b'\0\n').decode()
    if actual_serial!=profile['serial']:raise ValueError('Robot identity mismatch; use a profile for this machine')

def runtime_path(profile,base):return Path(base).resolve()/profile['name']/profile['installation_id']

def workflow_paths(profile,base,sensor_override='',workflow_override=''):
    sensor=Path(sensor_override).resolve() if sensor_override else runtime_path(profile,base)
    workflow=Path(workflow_override).resolve() if workflow_override else sensor/'workflow'
    return sensor,workflow

def check_simulation_runtime(runtime):
    runtime=Path(runtime).resolve()
    if any((p/'robot-identity.json').exists() for p in (runtime,*runtime.parents)):
        raise ValueError('Simulation runtime is inside a physical installation; choose a separate directory')

def bind_runtime(profile,runtime):
    runtime=Path(runtime).resolve();runtime.mkdir(parents=True,exist_ok=True);marker=runtime/'robot-identity.json';expected=identity(profile)
    if marker.exists():
        if json.loads(marker.read_text())!=expected:raise ValueError('Runtime belongs to another robot/installation; choose a fresh directory')
        return runtime
    if any(runtime.iterdir()):raise ValueError('Unbound runtime contains old data; choose a fresh directory')
    try:
        fd=os.open(marker,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
        with os.fdopen(fd,'w') as f:json.dump(expected,f);f.flush();os.fsync(f.fileno())
    except FileExistsError:
        if json.loads(marker.read_text())!=expected:raise ValueError('Concurrent runtime identity mismatch')
    return runtime

def check_sensor_config(profile,path):
    data=json.loads(Path(path).read_text())
    if data['Mid360s']['host_net_info'][0]['host_ip']!=profile['host_ip'] or data['lidar_configs'][0]['ip']!=profile['lidar_ip']:raise ValueError('Sensor config differs from robot profile')

def check_calibration(profile,data):
    if data.get('robot_serial')!=profile['serial'] or data.get('installation_id')!=profile['installation_id']:raise ValueError('Body calibration belongs to another installation or lacks its identity')

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--profile',required=True);p.add_argument('--runtime',default='');p.add_argument('--runtime-base',default='runtime/robots');p.add_argument('--sensor-config',default='');args=p.parse_args()
    profile=load(args.profile);check_machine(profile)
    if args.sensor_config:check_sensor_config(profile,args.sensor_config)
    runtime=bind_runtime(profile,args.runtime or runtime_path(profile,args.runtime_base));print(runtime)
if __name__=='__main__':main()
