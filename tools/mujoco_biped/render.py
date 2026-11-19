"""Render the actual imported MuJoCo state; no drawing or reconstructed skeleton."""
import argparse
from pathlib import Path
import struct
import zlib
import mujoco
from .model import build_model


def png(path, pixels):
    """Small RGB PNG writer avoids an extra image dependency."""
    h,w,_ = pixels.shape
    def chunk(tag,data):
        return struct.pack('!I',len(data))+tag+data+struct.pack('!I',zlib.crc32(tag+data)&0xffffffff)
    payload=b''.join(b'\0'+row.tobytes() for row in pixels)
    Path(path).write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',w,h,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(payload))+chunk(b'IEND',b''))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    model,data,_,_,_=build_model(args.output.parent/'render-model')
    camera=mujoco.MjvCamera()
    camera.lookat[:]=[0,0,.32]
    camera.distance=1.5
    camera.azimuth=135
    camera.elevation=-12
    with mujoco.Renderer(model,height=480,width=640) as renderer:
        renderer.update_scene(data,camera=camera)
        png(args.output,renderer.render())
    print(args.output)

if __name__=='__main__':
    main()
