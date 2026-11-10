#!/usr/bin/env python3
"""Reproduce the fixed RC1 physical checkpoint; reject changed RTL or constraints."""
import argparse
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile

ROOT = Path(__file__).resolve().parents[1]
TOP = "tt_um_openblastcore32_tt08"
IMAGE = "ghcr.io/librelane/librelane@sha256:f91b21d75f79871f9ccf37451020b5d2f7b3236881a997ff709c561e8280a30f"
PDK_REVISION = "8afc8346a57fe1ab7934ba5a6056ea8b43078e71"
CORNERS = {f"{rc}_{pvt}" for rc in ("nom", "min", "max")
           for pvt in ("tt_025C_1v80", "ss_100C_1v60", "ff_n40C_1v95")}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def instances(text):
    return {
        name: (cell, {key: value.strip().removeprefix("\\")
                      for key, value in re.findall(r"\.(\w+)\s*\(([^)]*)\)", body)})
        for cell, name, body in re.findall(
            r"(sky130\S+)\s+(\S+)\s*\((.*?)\);", text, re.S)
    }


def logical_graph(text):
    """Contract transparent buffers, retaining every functional cell terminal."""
    cells = instances(text)
    require(bool(cells), "Empty checkpoint netlist")
    parents = {}

    def find(net):
        parents.setdefault(net, net)
        if parents[net] != net:
            parents[net] = find(parents[net])
        return parents[net]

    def join(a, b):
        parents[find(a)] = find(b)

    for cell, ports in cells.values():
        if re.fullmatch(r"sky130_fd_sc_hd__(?:clkbuf|buf)_\d+", cell):
            require(set(ports) in ({"A", "X"}, {"A"}), "Unexpected buffer terminals")
            if "X" in ports:
                join(ports["A"], ports["X"])
    for left, right in re.findall(r"\bassign\s+(\S+)\s*=\s*(\S+)\s*;", text):
        join(left.removeprefix("\\"), right.removeprefix("\\"))
    groups, functional = {}, {}
    passive = {"diode", "decap", "fill", "tapvpwrvgnd"}
    for name, (cell, ports) in cells.items():
        kind = re.sub(r"_\d+$", "", cell.removeprefix("sky130_fd_sc_hd__"))
        if kind in passive or kind in {"buf", "clkbuf"}:
            continue
        functional[name] = kind
        for port, net in ports.items():
            groups.setdefault(find(net), []).append(f"{name}/{port}")
    declarations = re.findall(
        r"\b(?:input|output|inout)\s+(?:wire\s+)?(?:\[(\d+):(\d+)\]\s*)?([^;]+);",
        text)
    for high, low, declaration in declarations:
        for port in declaration.split(","):
            port = port.strip().removeprefix("\\")
            require(not re.search(r"\s", port), "Invalid mapped port")
            names = [f"{port}[{bit}]" for bit in range(int(low), int(high) + 1)] if high else [port]
            for name in names:
                groups.setdefault(find(name), []).append(f"port/{name}")
    for net in list(parents):
        if re.fullmatch(r"(?:\d+)'[bh][01xXzZ]+", net):
            groups.setdefault(find(net), []).append(f"constant/{net}")
    return functional, sorted(tuple(sorted(group)) for group in groups.values())


def geometry(text):
    section = text.split("\nPINS ", 1)[1].split("END PINS", 1)[0]
    pins = {}
    for name, body in re.findall(r"(?:^|\n)\s*-\s+(\S+)(.*?);", section, re.S):
        pins[name] = " ".join(re.sub(r"\+\s+NET\s+\S+", "", body).split())
    require(len(pins) == 45, "Expected 45 physical pins")
    die = re.search(r"DIEAREA\s+\(.*?;", text, re.S)
    require(die is not None, "Missing die boundary")
    rows = [line for line in text.splitlines() if line.startswith("ROW ")]
    require(bool(rows), "Missing placement rows")
    return {"die": " ".join(die[0].split()), "rows": rows, "pins": pins}


def inputs():
    import yaml

    folder = ROOT / "checkpoint"
    manifest = json.loads((folder / "manifest.json").read_text())
    require(manifest["image"] == IMAGE and manifest["pdk_revision"] == PDK_REVISION,
            "Checkpoint toolchain mismatch")
    require(manifest["source_hash_format"] == "sha256-crlf-to-lf",
            "Unsupported source hash format")
    for name, expected in manifest["sources"].items():
        require(digest((ROOT / name).read_bytes().replace(b"\r\n", b"\n")) == expected,
                f"Changed RC1 input: {name}; this checkpoint cannot build changed hardware inputs")
    info = yaml.safe_load((ROOT / "info.yaml").read_text())
    project = info["project"]
    expected_project = manifest["project"]
    require(isinstance(project, dict) and project.keys() == expected_project.keys(),
            "Changed RC1 project fields")
    for key in ("language", "clock_hz", "tiles", "top_module", "source_files"):
        require(project[key] == expected_project[key],
                f"Changed RC1 hardware project field: {key}")
    require(info["pinout"] == manifest["pinout"], "Changed RC1 pinout")
    require(info["yaml_version"] == 6, "Unexpected manifest version")
    declared = {f"src/{name}" for name in info["project"]["source_files"]}
    require(declared == set(manifest["sources"]) - {"src/config.json"},
            "Checkpoint source inventory mismatch")
    data = (folder / "rc1.tar.gz").read_bytes()
    require(digest(data) == manifest["archive_sha256"], "Changed checkpoint archive")
    assets = {}
    with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as archive:
        for member in archive:
            require(member.isfile() and "/" not in member.name and "\\" not in member.name,
                    "Invalid checkpoint member")
            require(member.name in manifest["files"] and member.name not in assets,
                    "Unexpected or duplicate checkpoint member")
            require(member.size <= 16_000_000, "Oversized checkpoint member")
            content = archive.extractfile(member).read()
            require(digest(content) == manifest["files"][member.name],
                    f"Changed checkpoint asset: {member.name}")
            assets[member.name] = content
    require(assets.keys() == manifest["files"].keys(), "Incomplete checkpoint")
    require(logical_graph(assets["reference.nl.v"].decode()) ==
            logical_graph(assets[f"{TOP}.nl.v"].decode()),
            "Checkpoint ECOs changed the reference functional connectivity")
    require(geometry(assets[f"{TOP}.def"].decode()) == manifest["geometry"],
            "Checkpoint physical geometry mismatch")
    print("Verified RC1 sources (CRLF/LF equivalent), constraints, checkpoint and buffer-contracted equivalence.",
          flush=True)
    return manifest, assets


def audit(run, assets, manifest):
    fresh = {}
    for log in sorted(run.glob("*-openroad-stapostpnr/*/sta.log")):
        for kind, key, value in re.findall(
                r"^%OL_METRIC_([IF]) (\S+) (\S+)$", log.read_text(), re.M):
            fresh[key] = int(value) if kind == "I" else float(value)
    checks = {}
    for kind in ("setup", "hold"):
        for metric in ("ws", "tns"):
            prefix = f"timing__{kind}__{metric}__corner:"
            values = {key[len(prefix):]: value for key, value in fresh.items()
                      if key.startswith(prefix)}
            checks[f"{kind}_{metric}"] = values.keys() == CORNERS and all(
                math.isfinite(value) and (value >= 0 if metric == "ws" else value == 0)
                for value in values.values())
    for kind in ("slew", "cap", "fanout"):
        prefix = f"design__max_{kind}_violation__count__corner:"
        values = {key[len(prefix):]: value for key, value in fresh.items()
                  if key.startswith(prefix)}
        checks[kind] = values.keys() == CORNERS and all(v == 0 for v in values.values())
    final = run / "final"
    metrics = json.loads((final / "metrics.json").read_text())
    for key in ("route__drc_errors", "route__antenna_violation__count",
                "antenna__violating__nets", "antenna__violating__pins",
                "magic__drc_error__count", "klayout__drc_error__count",
                "design__xor_difference__count", "magic__illegal_overlap__count",
                "design__critical_disconnected_pin__count", "design__lvs_error__count"):
        checks[key] = metrics.get(key) == 0
    checks["geometry"] = geometry((final / "def" / f"{TOP}.def").read_text()) == manifest["geometry"]
    before = instances(assets[f"{TOP}.nl.v"].decode())
    after = instances((final / "nl" / f"{TOP}.nl.v").read_text())
    checks["unchanged_checkpoint_connections"] = all(after.get(k) == v for k, v in before.items())
    nets = {net for _, ports in before.values() for net in ports.values()}
    checks["only_passive_cells_added"] = all(
        (cell in {"sky130_fd_sc_hd__decap_3", "sky130_fd_sc_hd__fill_1",
                  "sky130_fd_sc_hd__fill_2"} and not ports) or
        (cell == "sky130_fd_sc_hd__diode_2" and set(ports) == {"DIODE"}
         and ports["DIODE"] in nets)
        for name, (cell, ports) in after.items() if name not in before)
    gds = final / "gds" / f"{TOP}.gds"
    checks["new_gds"] = gds.is_file() and gds.stat().st_size > 0
    result = {"build_kind": "checkpoint_replay", "checks": checks,
              "fresh_sta_metrics": fresh, "sources": manifest["sources"],
              "source_hash_format": manifest["source_hash_format"],
              "checkpoint_sha256": manifest["archive_sha256"],
              "gds_sha256": digest(gds.read_bytes())}
    (run / "closure.json").write_text(json.dumps(result, indent=2) + "\n")
    failed = [key for key, value in checks.items() if not value]
    require(not failed, f"Physical closure failed: {', '.join(failed)}")
    print(f"Passed {len(checks)} fresh closure gates across all nine corners.", flush=True)
    return result


def submission(run, result, manifest, config):
    import klayout.db as pya

    out = ROOT / "tt_submission"
    out.mkdir(exist_ok=False)
    final = run / "final"
    for kind, suffix in (("gds", "gds"), ("lef", "lef")):
        name = f"{TOP}.{suffix}"
        shutil.copyfile(final / kind / name, out / name)
    shutil.copyfile(final / "pnl" / f"{TOP}.pnl.v", out / f"{TOP}.v")
    layout = pya.Layout()
    layout.read(str(out / f"{TOP}.gds"))
    layout.set_property("TT_PDK", "sky130A")
    layout.write(str(out / f"{TOP}.oas"))
    for spef in final.glob("spef/*/*.spef"):
        shutil.copyfile(spef, out / spef.name)
    shutil.copyfile(run / "resolved.json", out / "resolved.json")
    (out / "pdk.json").write_text(json.dumps(
        {"FLOW_NAME": "LibreLane", "FLOW_VERSION": "3.0.14", "PDK": "sky130A",
         "PDK_SOURCE": "open_pdks", "PDK_VERSION": PDK_REVISION},
        indent=2) + "\n")
    metadata = {
        "app": "LibreLane RC1 checkpoint replay",
        "commit": os.environ.get("GITHUB_SHA"),
        "repo": (
            f"{os.environ['GITHUB_SERVER_URL']}/{os.environ['GITHUB_REPOSITORY']}"
            if all(key in os.environ for key in ("GITHUB_SERVER_URL", "GITHUB_REPOSITORY"))
            else None),
        "workflow_url": (
            f"{os.environ['GITHUB_SERVER_URL']}/{os.environ['GITHUB_REPOSITORY']}"
            f"/actions/runs/{os.environ['GITHUB_RUN_ID']}"
            if all(key in os.environ for key in
                   ("GITHUB_SERVER_URL", "GITHUB_REPOSITORY", "GITHUB_RUN_ID")) else None),
        "build_kind": "checkpoint_replay", "source_commit": manifest["source_commit"],
        "checkpoint_sha256": manifest["archive_sha256"],
        "image": IMAGE, "sources": manifest["sources"],
        "source_hash_format": manifest["source_hash_format"],
    }
    (out / "commit_id.json").write_text(json.dumps(metadata, indent=2) + "\n")
    (out / "closure.json").write_text(json.dumps(result, indent=2) + "\n")
    stats = out / "stats"
    stats.mkdir()
    shutil.copyfile(final / "metrics.csv", stats / "metrics.csv")
    shutil.copyfile(run / "checkpoint" / "stat.rpt", stats / "synthesis-stats.txt")
    require(json.loads((out / "resolved.json").read_text())["CLOCK_PERIOD"] == config["CLOCK_PERIOD"],
            "Submission timing configuration mismatch")
    print("Created tt_submission from this replay's freshly verified outputs.", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-only", action="store_true")
    parser.add_argument("--pdk-root", type=Path, default=Path("/pdk"))
    parser.add_argument("--output", type=Path, default=ROOT / "runs" / "rc1")
    args = parser.parse_args()
    manifest, assets = inputs()
    if args.verify_only:
        return
    import librelane

    require(librelane.__version__ == "3.0.14", "Use the pinned LibreLane image")
    pdk = args.pdk_root.resolve()
    require((pdk / "sky130A/SOURCES").read_text().strip() == f"open_pdks {PDK_REVISION}",
            "Sky130 PDK revision mismatch")
    require((pdk / "sky130A" / "libs.ref/sky130_fd_sc_hd/lib/sky130_fd_sc_hd__tt_025C_1v80.lib").is_file(),
            "Pinned Sky130 PDK is missing")
    require(not (ROOT / "tt_submission").exists(), "Submission directory already exists")
    run = args.output.resolve()
    run.mkdir(parents=True, exist_ok=False)
    checkpoint = run / "checkpoint"
    checkpoint.mkdir()
    for name, data in assets.items():
        (checkpoint / name).write_bytes(data)
    state = json.loads(assets["state.json"])
    for key, value in list(state.items()):
        if key != "metrics":
            state[key] = str(checkpoint / value)
    state_path = run / "initial_state.json"
    state_path.write_text(json.dumps(state, indent=2) + "\n")
    config = json.loads((ROOT / "src/config.json").read_text())
    config.pop("//", None)
    config.update({
        "DESIGN_NAME": TOP,
        "VERILOG_FILES": [str(ROOT / name) for name in manifest["sources"] if name.endswith(".v")],
        "DIE_AREA": [0, 0, 682.64, 225.76],
        "FP_DEF_TEMPLATE": str(checkpoint / "tt_block_4x2_pg.def"),
        "VDD_PIN": "VPWR", "GND_PIN": "VGND", "RT_MAX_LAYER": "met4",
    })
    config_path = run / "config.json"
    config_path.write_text(json.dumps(config, indent=2) + "\n")
    flow = run / "flow"
    flow.mkdir()
    subprocess.run([
        sys.executable, "-m", "librelane", "--manual-pdk", "--pdk-root", str(pdk),
        "--pdk", "sky130A", "--scl", "sky130_fd_sc_hd", "--jobs", "2",
        "--design-dir", str(ROOT), "--force-run-dir", str(flow),
        "--with-initial-state", str(state_path), "--from", "OpenROAD.DetailedRouting",
        str(config_path),
    ], check=True, timeout=3600)
    result = audit(flow, assets, manifest)
    # Keep synthesis statistics explicitly associated with the frozen checkpoint.
    (flow / "checkpoint").mkdir()
    shutil.copyfile(checkpoint / "stat.rpt", flow / "checkpoint" / "stat.rpt")
    submission(flow, result, manifest, config)


if __name__ == "__main__":
    main()
