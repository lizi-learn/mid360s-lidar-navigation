#!/usr/bin/env python3
"""Write LiDAR endpoint config; never changes network interfaces or motors."""
import argparse,ipaddress,json,pathlib
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--host-ip',required=True,help='IPv4 already assigned to the LiDAR-facing interface')
parser.add_argument('--lidar-ip',required=True,help='Actual IPv4 of your MID-360S')
parser.add_argument('--robot-name',help='Optional installation name, e.g. P2-1003; requires --robot-serial')
parser.add_argument('--robot-serial',help='This board serial from /proc/device-tree/serial-number')
parser.add_argument('--force',action='store_true',help='Replace an existing local generated config')
args=parser.parse_args()
for value in [args.host_ip,args.lidar_ip]:
 if ipaddress.ip_address(value).version!=4:parser.error('IPv4 required')
if args.host_ip==args.lidar_ip:parser.error('Host and LiDAR addresses must differ')
root=pathlib.Path(__file__).resolve().parents[1];target=root/'config/MID360S.json'
if target.exists() and not args.force:parser.error('Config exists; review it or explicitly use --force')
profile=None
if bool(args.robot_name)!=bool(args.robot_serial):parser.error('Provide both --robot-name and --robot-serial')
if args.robot_name:
 import sys
 sys.path.insert(0,str(root))
 from robot_profile import create
 try:profile=create(args.robot_name,args.robot_serial,args.host_ip,args.lidar_ip)
 except ValueError as e:parser.error(str(e))
 if (root/'config/robot.local.json').exists() and not args.force:parser.error('Robot profile exists; explicitly use --force to create a new installation')
config=json.loads((root/'config/MID360S.example.json').read_text())
config['Mid360s']['host_net_info'][0]['host_ip']=args.host_ip
config['lidar_configs'][0]['ip']=args.lidar_ip
target.write_text(json.dumps(config,indent=2)+'\n')
if profile:(root/'config/robot.local.json').write_text(json.dumps(profile,indent=2)+'\n')
print('Wrote '+str(target)+'. No interface addresses were changed.')
