from pathlib import Path


RVIZ_CONFIG = Path(
    "/opt/create3_ws/install/irobot_create_common_bringup/"
    "share/irobot_create_common_bringup/rviz/irobot_create_view.rviz"
)
GUI_CONFIG = Path(
    "/opt/create3_ws/install/irobot_create_gz_bringup/"
    "share/irobot_create_gz_bringup/gui/create3/gui.config"
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
    - Class: rviz_default_plugins/Marker
      Enabled: true
      Marker Topic:
        Depth: 10
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /pai/presentation_markers
      Name: PAI Presentation
      Namespaces:
        pai_demo: true
      Queue Size: 100
      Value: true
"""


def main():
    config = RVIZ_CONFIG.read_text()
    if "Name: PAI Presentation" not in config:
        anchor = "    - Class: rviz_default_plugins/TF\n"
        if anchor not in config:
            raise RuntimeError("RViz display insertion point was not found")
        config = config.replace(anchor, DISPLAY + anchor, 1)
    config = config.replace("Background Color: 48; 48; 48", "Background Color: 18; 24; 34")
    config = config.replace("Distance: 1.952379584312439", "Distance: 3.2")
    config = config.replace("Pitch: 0.6197972297668457", "Pitch: 0.78")
    RVIZ_CONFIG.write_text(config)

    gui = GUI_CONFIG.read_text()
    gui = gui.replace(
        "<camera_pose>-6 0 6 0 0.5 0</camera_pose>",
        "<camera_pose>-4.8 -4.2 4.4 0 0.62 0.72</camera_pose>",
    )
    GUI_CONFIG.write_text(gui)


if __name__ == "__main__":
    main()
