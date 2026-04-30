import urx
import math3d as m3d
from urx_compat import patch_urx_math3d
patch_urx_math3d()
from config import load_config


def main(host=None):
    cfg = load_config()
    rob = urx.Robot(host or cfg.surgeon_ip)

    pose = rob.getl()
    rob.movel((pose[0],pose[1],pose[2],3.14/2,0,0), acc = 0.2, vel=0.2)


if __name__ == "__main__":
    main()
