from mujoco_base import MuJoCoBase
from examples.biped import Biped


def main():
    xml_path = "./xml/biped.xml"
    sim = Biped(xml_path)
    sim.reset()
    sim.simulate()


if __name__ == "__main__":
    main()
