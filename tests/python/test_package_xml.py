"""Static checks on every package.xml in the repo.

We treat package.xml as the canonical manifest: if a package disagrees with
itself here, the rest of the build is a coin-flip. These tests verify that
every manifest:

1. Is well-formed XML with ``<package format="3">`` at the root. Format 2 was
   ROS1-era and we don't want it sneaking back.
2. Declares a ROS2 build system via ``<buildtool_depend>`` -- one of
   ``ament_cmake``, ``ament_cmake_python``, ``ament_python`` or
   ``rosidl_default_generators`` (the last one is a legitimate secondary
   buildtool dep for message packages).
3. Does *not* carry ``catkin`` as a buildtool -- that's the ROS1 marker.
4. Contains ``<export><build_type>ament_cmake</build_type></export>`` or the
   ament_python equivalent; colcon uses this to pick the right builder.
5. Has no ``<depend>message_generation</depend>`` or ``<depend>message_runtime</depend>``
   ROS1 holdovers -- ROS2 uses rosidl_default_generators/runtime instead.

Test IDs are parametrised by relative package-xml path so failures point at a
specific manifest without having to read the traceback.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List

import pytest

from _packages import REPO_ROOT


_VALID_BUILDTOOLS = {
    "ament_cmake",
    "ament_cmake_python",
    "ament_python",
    "rosidl_default_generators",
}

_VALID_BUILD_TYPES = {"ament_cmake", "ament_python"}

_BANNED_ROS1_DEPS = {"message_generation", "message_runtime"}


def _collect_package_xmls() -> List[Path]:
    return sorted(REPO_ROOT.rglob("package.xml"))


# Precompute at module load so pytest can emit one test ID per manifest.
_PACKAGE_XMLS = _collect_package_xmls()
_IDS = [str(p.relative_to(REPO_ROOT)).replace("\\", "/") for p in _PACKAGE_XMLS]


@pytest.mark.parametrize("xml_path", _PACKAGE_XMLS, ids=_IDS)
def test_package_xml_is_format_3(xml_path: Path) -> None:
    """``<package format="3">`` must appear at the XML root."""

    tree = ET.parse(xml_path)
    root = tree.getroot()
    assert root.tag == "package", (
        "expected <package> root element in {}, got <{}>".format(xml_path, root.tag)
    )
    fmt = root.attrib.get("format", "")
    assert fmt == "3", (
        "expected format=\"3\" in {}, got format={!r}".format(xml_path, fmt)
    )


@pytest.mark.parametrize("xml_path", _PACKAGE_XMLS, ids=_IDS)
def test_package_xml_has_valid_buildtool(xml_path: Path) -> None:
    """At least one <buildtool_depend> must name a ROS2 builder, and none may be ``catkin``."""

    tree = ET.parse(xml_path)
    buildtools = [
        (el.text or "").strip()
        for el in tree.getroot().findall("buildtool_depend")
    ]
    assert buildtools, "no <buildtool_depend> tags in {}".format(xml_path)

    assert "catkin" not in buildtools, (
        "{} still lists <buildtool_depend>catkin</buildtool_depend> -- ROS1 leftover"
        .format(xml_path)
    )

    found = [b for b in buildtools if b in _VALID_BUILDTOOLS]
    assert found, (
        "{} has no ROS2 <buildtool_depend>; saw {}, expected one of {}"
        .format(xml_path, buildtools, sorted(_VALID_BUILDTOOLS))
    )


@pytest.mark.parametrize("xml_path", _PACKAGE_XMLS, ids=_IDS)
def test_package_xml_has_export_build_type(xml_path: Path) -> None:
    """``<export><build_type>ament_cmake|ament_python</build_type></export>`` must exist."""

    tree = ET.parse(xml_path)
    export = tree.getroot().find("export")
    assert export is not None, "no <export> section in {}".format(xml_path)

    build_type_el = export.find("build_type")
    assert build_type_el is not None, (
        "no <export><build_type> in {}".format(xml_path)
    )
    build_type = (build_type_el.text or "").strip()
    assert build_type in _VALID_BUILD_TYPES, (
        "{} has <build_type>{}</build_type>, expected one of {}"
        .format(xml_path, build_type, sorted(_VALID_BUILD_TYPES))
    )


@pytest.mark.parametrize("xml_path", _PACKAGE_XMLS, ids=_IDS)
def test_package_xml_has_no_ros1_message_deps(xml_path: Path) -> None:
    """``message_generation``/``message_runtime`` are ROS1; they must be gone."""

    tree = ET.parse(xml_path)
    root = tree.getroot()
    offenders: List[str] = []
    # Check every dep-like tag type; ROS1 message hooks have been seen in all
    # of <depend>, <build_depend>, <exec_depend>, etc.
    for tag in ("depend", "build_depend", "exec_depend", "run_depend"):
        for el in root.findall(tag):
            text = (el.text or "").strip()
            if text in _BANNED_ROS1_DEPS:
                offenders.append("<{}>{}</{}>".format(tag, text, tag))
    assert not offenders, (
        "{} still carries ROS1 message-gen deps: {}".format(xml_path, offenders)
    )
