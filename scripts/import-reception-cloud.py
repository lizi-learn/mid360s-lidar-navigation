#!/usr/bin/env python3
"""Convert an actual privately saved PCD to the workbench's XYZ preview JSON."""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from reception_pcd import convert
from audio_feedback import atomic_json

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pcd');parser.add_argument('--map-id',required=True);parser.add_argument('--frame-id',required=True)
    parser.add_argument('--transform',help='Private JSON4x4 matrix transforming the source frame into this map')
    parser.add_argument('--output',required=True);args=parser.parse_args()
    matrix=json.loads(Path(args.transform).read_text())if args.transform else None
    asset,report=convert(args.pcd,args.map_id,args.frame_id,matrix);atomic_json(args.output,asset)
    print(json.dumps(report,ensure_ascii=False))
