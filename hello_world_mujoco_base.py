from mujoco_base import MuJoCoBase


def main():
    xml_path = "./xml/ball.xml"
    mjb = MuJoCoBase(xml_path)
    mjb.simulate()


if __name__ == "__main__":
    main()
