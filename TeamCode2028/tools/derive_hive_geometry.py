"""Reproduce neutral tag positions from the official BIOBUZZ STEP assembly.

Python standard library only. The drawing's 17x5 inch sticker has its printed
bottom notches at CAD local Y=5in; printed right is -local X. Production PDF
tag centers are X=2,5.75,11.25,15in from printed left and Y=2.25in from top.
"""
import hashlib
import io
import json
import math
from pathlib import Path
import re
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[2]
ARCHIVE = ROOT / "build/hive-reference/field-cad-step.zip"
URL = "https://ftc-resources.firstinspires.org/ftc/field/field-cad-step"
SHA256 = "5e768b731f1ec8dcd14debba53225c43718877923c351ce08504305f68f7fe00"
data = ARCHIVE.read_bytes() if ARCHIVE.exists() else urllib.request.urlopen(URL, timeout=60).read()
if hashlib.sha256(data).hexdigest() != SHA256:
    raise SystemExit("CAD revision changed: inspect the new assembly before updating the model")
text = zipfile.ZipFile(io.BytesIO(data)).read("field-cad-step.step").decode()
records = {int(m[1]): m[2].replace("\n", "") for m in re.finditer(r"#(\d+)\s*=\s*(.*?);", text, re.S)}


def refs(value):
    return [int(n) for n in re.findall(r"#(\d+)", re.sub("'[^']*'", "", value))]


def xyz(key):
    return [float(n) for n in re.search(r",\(([^()]*)\)\)", records[key])[1].split(",")]


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


products = {k: re.findall("'([^']*)'", v)[0] for k, v in records.items() if v.startswith("PRODUCT(")}
formations = {k: refs(v)[0] for k, v in records.items() if v.startswith("PRODUCT_DEFINITION_FORMATION")}
definitions = {k: formations[refs(v)[0]] for k, v in records.items() if v.startswith("PRODUCT_DEFINITION(")}
shapes = {k: refs(v)[0] for k, v in records.items() if v.startswith("PRODUCT_DEFINITION_SHAPE(")}
relations = {shapes[refs(v)[1]]: refs(v)[0] for v in records.values() if v.startswith("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(")}
placements = {}
for key, value in records.items():
    if value.startswith("NEXT_ASSEMBLY_USAGE_OCCURRENCE("):
        parent, child = refs(value)[:2]
        name = products[definitions[child]]
        parent_name = products[definitions[parent]]
        transform = refs(records[relations[key]])[-1]
        placement = refs(records[transform])[-1]
        placements[(parent_name, name)] = list(map(xyz, refs(records[placement])))

result = {}
for color, angle in (("Blue", 30), ("Red", -30)):
    pivot_parent = "Goal Pivot Assembly <1>"
    # The two similarly named pivot assemblies have distinct placements. Resolve
    # the axle by its hive X coordinate (positive blue, negative red).
    candidates = []
    for key, value in records.items():
        if not value.startswith("NEXT_ASSEMBLY_USAGE_OCCURRENCE("):
            continue
        parent, child = refs(value)[:2]
        if products[definitions[parent]] == pivot_parent and "Stepped Spacer" in products[definitions[child]]:
            transform = refs(records[relations[key]])[-1]
            placement = refs(records[transform])[-1]
            candidates.append(xyz(refs(records[placement])[0]))
    pivot = next(p for p in candidates if (p[0] > 0) == (color == "Blue"))
    for side in ("Audience", "Scoring"):
        location, z, x = next(v for (parent, child), v in placements.items()
                              if color + " Cell (" + side + ")" in parent and "April Tag" in child)
        y = cross(z, x)
        centers = []
        for local_x in (15, 11.25, 5.75, 2):
            p = [location[i] + x[i]*local_x*.0254 + y[i]*2.25*.0254 + z[i]*.000254 - pivot[i] for i in range(3)]
            a = math.radians(-angle)
            neutral = [p[0], math.cos(a)*p[1]-math.sin(a)*p[2], math.sin(a)*p[1]+math.cos(a)*p[2]]
            centers.append([neutral[0]/.0254, -neutral[2]/.0254, neutral[1]/.0254])
        result[color + " " + side] = {"tag_centers_inches_relative_to_pivot": centers}
print(json.dumps(result, indent=2))
