from pathlib import Path


RVIZ_CONFIG = Path(
    "/opt/create3_ws/install/irobot_create_common_bringup/"
    "share/irobot_create_common_bringup/rviz/irobot_create_view.rviz"
)

DISPLAY = """    - Class: rviz_default_plugins/Marker
      Enabled: true
      Marker Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /litter/target_marker
      Name: LitterTarget
      Namespaces:
        litter_target: true
      Queue Size: 10
      Value: true
"""


def main():
    config = RVIZ_CONFIG.read_text()
    if "Name: LitterTarget" in config:
        return
    anchor = "    - Class: rviz_default_plugins/TF\n"
    if anchor not in config:
        raise RuntimeError("RViz display insertion point was not found")
    RVIZ_CONFIG.write_text(config.replace(anchor, DISPLAY + anchor, 1))


if __name__ == "__main__":
    main()
