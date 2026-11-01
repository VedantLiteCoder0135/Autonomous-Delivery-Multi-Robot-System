"""Static guards for the Stage 2 architectural constraints (parsed from source, not imported)."""

import ast
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[1] / "src" / "aifr"
FORBIDDEN_ROOTS = {"bpy", "bmesh", "mathutils", "gpu", "numpy", "scipy"}
# Layering: which in-project packages each package may import (besides itself and stdlib).
ALLOWED = {
    "common": set(),
    "config": {"common"},
    "orders": {"common"},
    "fleet": {"common", "config"},
    "scheduling": {"common", "orders", "fleet"},
    "navigation": {"environment"},
    "perception": {"navigation"},
    "simulation": {"navigation", "perception"},
}
# The scheduler must not see any geometry/navigation vocabulary.
BANNED_IN_SCHEDULING = ("distance", "path_cost", "route", "position", "navigation", "hypot", "sqrt")


def _modules():
    return sorted(p for p in SRC.rglob("*.py"))


def _imports(path: Path) -> set[str]:
    tree = ast.parse(path.read_text())
    found: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            found |= {a.name for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            found.add(node.module)
    return found


@pytest.mark.parametrize("path", _modules(), ids=lambda p: str(p.relative_to(SRC)))
def test_no_blender_or_third_party_imports(path):
    roots = {name.split(".")[0] for name in _imports(path)}
    assert not (roots & FORBIDDEN_ROOTS), f"{path.name} imports {roots & FORBIDDEN_ROOTS}"


@pytest.mark.parametrize("path", _modules(), ids=lambda p: str(p.relative_to(SRC)))
def test_package_layering_is_respected(path):
    rel = path.relative_to(SRC)
    if len(rel.parts) < 2:
        return
    package = rel.parts[0]
    for name in _imports(path):
        parts = name.split(".")
        if parts[0] == "aifr" and len(parts) > 1 and parts[1] != package:
            assert parts[1] in ALLOWED[package], f"{rel}: '{package}' must not import aifr.{parts[1]}"


def test_scheduling_package_has_no_geometry_or_cost_vocabulary():
    for path in (SRC / "scheduling").glob("*.py"):
        code = "\n".join(
            line.split("#")[0] for line in path.read_text().splitlines()
        )
        # Ignore docstrings (they legitimately state the *absence* of these concepts).
        tree = ast.parse(code)
        for node in ast.walk(tree):
            if isinstance(node, (ast.Name, ast.Attribute, ast.arg, ast.alias)):
                ident = getattr(node, "id", None) or getattr(node, "attr", None) \
                    or getattr(node, "arg", None) or getattr(node, "name", "")
                assert not any(b in ident.lower() for b in BANNED_IN_SCHEDULING), \
                    f"{path.name}: identifier '{ident}' suggests cost/geometry coupling"
