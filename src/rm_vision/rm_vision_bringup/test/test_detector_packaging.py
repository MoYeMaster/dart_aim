from pathlib import Path


BRINGUP_PACKAGE = Path(__file__).resolve().parents[1]
SOURCE_ROOT = BRINGUP_PACKAGE.parents[1]
DETECTOR_PACKAGE = SOURCE_ROOT / "rm_auto_aim" / "detector"


def test_detector_configuration_is_owned_by_bringup():
    assert (BRINGUP_PACKAGE / "config" / "detector_params.yaml").is_file()
    assert not (DETECTOR_PACKAGE / "config" / "detector_params.yaml").exists()


def test_detector_uses_component_generated_executable():
    cmake = (DETECTOR_PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "rclcpp_components_register_node" in cmake
    assert "EXECUTABLE detector_node" in cmake
    assert "add_executable(detector_node" not in cmake
    assert not (DETECTOR_PACKAGE / "src" / "main.cpp").exists()
