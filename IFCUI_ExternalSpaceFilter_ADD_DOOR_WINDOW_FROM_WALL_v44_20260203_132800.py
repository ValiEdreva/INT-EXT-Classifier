import bpy, bmesh
from mathutils import Vector
from mathutils.bvhtree import BVHTree
import os, re
import random
IE_BUILD_TAG = "v43_add_door_window_from_wall_2026-02-03"

# =========================================================
# Einstellungen
# =========================================================

TARGET_STOREYS = ["IfcBuildingStorey/-1"]  # [] = alle

# --- Space-BVH Tuning (Wände/Coverings) ---
MIN_SPACES_PER_SIDE = 1

Z_THR_WALL = 0.30  # abs(n.z) < ... => vertikale Faces

EPS         = 0.03
SEARCH_MAX  = 0.12
DEPTH_STEPS = 5
EDGE_JITTER = 0.015
JITTER_SAMPLES = 3
GRID_U      = 14
GRID_V      = 3
DOT_STRICT  = 0.5
MAJOR_FRACTION   = 0.8
HEIGHT_FRAC_LOW  = 0.20
HEIGHT_FRAC_HIGH = 0.80


# =========================================================
# IfcSpace Filter (NEU): Externe Spaces ignorieren
# =========================================================
IGNORE_EXTERNAL_SPACES_BY_NAME = True

# Keywords für "Außenräume" (DE + EN) -> werden in IfcSpace Name/LongName/ObjectType/Objektname gesucht
EXTERNAL_SPACE_KEYWORDS = [
    # Deutsch
    "terrasse", "dachterrasse", "balkon", "loggia", "rampe", "außen", "aussen",
    "vorplatz", "hof", "innenhof", "garten", "patio", "eingang", "zugang",
    "überdacht", "ueberdacht", "freifläche", "freiflaeche", "außenfläche", "aussenflaeche",

    # Englisch
    "terrace", "roof terrace", "balcony", "ramp", "outside", "exterior", "outdoor",
    "courtyard", "garden", "entrance", "porch", "patio",
]

# optional: nur wenn das Keyword als "Wortteil" vorkommt (default True)
EXTERNAL_SPACE_SUBSTRING_MATCH = True



# =========================================================
# IfcCovering Orientierung per PredefinedType (NEU)
# =========================================================
# Ziel: IfcCovering bleibt EIN Button, aber die Space-Logik unterscheidet:
# - vertikale Coverings: CLADDING / INSULATION  -> bestehende Covering-Logik (vertikale Seiten)
# - horizontale Coverings: FLOORING / CEILING / ROOFING / TOPPING -> Top/Bottom (oder geneigt) statt Seiten
COVERING_PTYPE_VERTICAL = {"CLADDING", "INSULATION"}
COVERING_PTYPE_HORIZONTAL = {"FLOORING", "CEILING", "ROOFING", "TOPPING"}

# Für horizontale Coverings (FLOORING/CEILING/ROOFING/TOPPING): nur Top/Bottom Faces (sehr strikt)
COVERING_HOR_FACE_Z_ABS_THR = 0.80  # abs(n.z) >= thr => horizontal (Top/Bottom)

# =========================================================
# IfcSlab (NEU): Wände-Logik 1:1, aber für horizontale Elemente
# - Face-Auswahl: Top/Bottom statt Seiten
# =========================================================
SLAB_HOR_FACE_Z_ABS_THR = 0.80  # abs(n.z) >= thr => Slab Top/Bottom Faces
Z_THR_SLAB_HOR = 0.30  # Fallback: "horizontal-ish" Faces bei geneigten/unsauberen Slabs

# IfcSlab (NEU): "geklebt" darunterliegende Slabs überspringen (z.B. Fundament)
SLAB_GLUE_IGNORE_BELOW = True
SLAB_GLUE_Z_EPS = 0.01          # m (Kontakt-Toleranz zwischen src_bottom und hit_top)
SLAB_GLUE_OVERLAP_FRAC = 0.30

# IfcSlab (FIX): echte Top/Bottom-Oberflächen finden (auch bei geneigten Dächern)
# Ansatz: dominante Flächennormalen bestimmen (flächen-gewichtet), dann nur Faces wählen,
# deren Normalen stark mit dieser Achse übereinstimmen. Top/Bottom über Extremwerte entlang dieser Achse.
SLAB_SURFACE_AXIS_DOT_STRICT = 0.85
SLAB_SURFACE_AXIS_DOT_LOOSE  = 0.65
SLAB_SURFACE_EPS_MIN         = 0.005   # m
SLAB_SURFACE_EPS_FRAC        = 0.35    # Anteil der gemessenen "Dicke" entlang Achse
SLAB_AXIS_CLUSTER_DOT        = 0.90    # Normalen-Cluster (Hemisphere) Dot-Threshold

   # Anteil der XY-Überdeckung (bezogen auf src)




# =========================================================
# Debug: Touch-Spaces speichern & markieren (NEU)
# =========================================================
DEBUG_TOUCH_SPACES_PROP_A = "IFCIE_touch_spaces_A"
DEBUG_TOUCH_SPACES_PROP_B = "IFCIE_touch_spaces_B"
DEBUG_SPACE_MARK_PROP = "IFCIE_debug_space_mark"
DEBUG_SPACE_OLD_COLOR_PROP = "IFCIE_debug_old_color"
DEBUG_SPACE_OLD_WIRE_PROP = "IFCIE_debug_old_show_wire"
DEBUG_SPACE_OLD_FRONT_PROP = "IFCIE_debug_old_show_in_front"
DEBUG_SPACE_MARK_COLOR = (0.90, 0.10, 0.90, 1.00)

# Debug: AABB Rays visualisieren (NEU)
DEBUG_AABB_RAYS_COLLECTION = "IFC_IE_DEBUG"
DEBUG_AABB_RAYS_PREFIX = "IFCIE_AABB_RAYS"
DEBUG_AABB_RAYS_OBJ_PROP = "IFCIE_debug_aabb_rays_obj"
DEBUG_AABB_RAYS_COLOR_ESCAPE = (0.10, 0.90, 0.10, 1.00)
DEBUG_AABB_RAYS_COLOR_BLOCK  = (0.90, 0.10, 0.10, 1.00)


# "horizontal-ish" Faces (inkl. geneigte Dächer): abs(n.z) >= ...
Z_THR_COVERING_HOR = 0.30
# --- AABB-Escape Tuning (dein Prinzip, plus Fixes) ---
VERTICAL_TOL_WALL = 0.20
FACE_COUNT        = 16
REL_AREA_CUT      = 0.10
START_EPS_AABB    = 0.03
STEP_EPS_AABB     = 0.03
MAX_STEPS_AABB    = 16

# --- Column AABB Ray Robustness (NEU) ---
# Problem: Bei Fassaden-/Eckstützen kann ein einzelner Ray durch winzige Mesh-Gaps (Storey-Joints, Segmentfugen) "durchrutschen"
# -> scheinbar identische Rays, aber inkonsistente Internal/External Ergebnisse.
# Lösung: pro Richtung mehrere Z-Samples; eine Richtung gilt nur dann als "Escape", wenn ALLE Samples escapen.
COLUMN_AABB_Z_SAMPLES  = 3        # 1..3 (bewusst klein halten)
COLUMN_AABB_Z_FRAC     = 0.25     # Offset-Fraction der Säulenhöhe (für +/- Samples)
COLUMN_AABB_Z_MARGIN   = 0.05     # Clamp-Margin innerhalb der AABB (Meter)

# FIX 4: Nur Side-Normalen zulassen (gegen Querfaces/Reveals bei Öffnungen)
AABB_SIDE_NORMAL_ONLY = True
AABB_SIDE_NORMAL_DOT_THR = 0.90  # 0.85..0.95 (höher = strenger, weniger Reveal-Faces)


# FIX 1: "Opening-Startpunkte" vermeiden (mehrere Startpunkte pro Face)
OPENING_NEAR_PROBE_DIST = 0.12   # wenn hier gleich Door/Window/Opening der SAME wall kommt -> Startpunkt überspringen
OPENING_AVOID_OFFSET    = 0.14   # seitliche Offsets auf dem Face (m)

# FIX 3: Door/Window als "massive" Blocker behandeln auch wenn Mesh-Löcher (nur Rahmen etc.)
USE_DOOR_WINDOW_AABB_BLOCKERS = True

# Beam-Fix: CurtainWall als "massive" Fassaden-Blocker behandeln (AABB-Proxy),
# weil IfcCurtainWall-Geometrie oft aus Rahmen/Mullions besteht und Rays sonst durch die Lücken "entkommen".
# CurtainWall AABB Proxy Blockers (Fassade als "geschlossene Hülle")
# - IfcCurtainWall ist oft perforiert (Rahmen/Mullions/Paneele als getrennte Geometrie).
# - Rays können dann "durch Lücken" entkommen, obwohl die Fassade als Hülle wirkt.
# -> Wir behandeln CurtainWalls zusätzlich als AABB-Blocker (leicht inflated + optional gemerged).
USE_CURTAINWALL_AABB_BLOCKERS_FOR_AABB = True   # apply to ALL AABB-Escape checks (Wall/Covering/Slab/Column/Beam)
USE_CURTAINWALL_AABB_BLOCKERS_FOR_BEAMS = USE_CURTAINWALL_AABB_BLOCKERS_FOR_AABB
CURTAINWALL_AABB_BLOCKER_INFLATE = 0.02         # m (kleines Inflate, um kleine Lücken zu überbrücken)
CURTAINWALL_AABB_BLOCKER_MERGE_GAP = 0.10       # m (merge benachbarte CurtainWall-Teile zu größeren Blockern)
_IE_CW_AABB_BLOCKERS = []                       # per run befüllt

# Columns: Door/Window-AABB-Blocker nur anwenden, wenn der Ray-Startpunkt in einem INTERNAL IfcSpace liegt.
# (Verhindert false-blocks bei Fassaden-/Eckstützen, deren Rays außen an Fenster-AABBs vorbeilaufen.)
COLUMNS_DW_BLOCKERS_ONLY_IF_START_IN_INTERNAL_SPACE = True


TYPE_PRIORITY = {
    "IfcWall": 1,
    "IfcWallStandardCase": 1,
    "IfcDoor": 1,
    "IfcWindow": 1,
    "IfcCovering": 2,
    "IfcColumn": 2,
    "IfcBeam": 2,
    "IfcSlab": 3,
}

# =========================================================
# Layer-Stack Handling (NEU)  ✅✅✅
# =========================================================
# Ziel: Wenn eine "Core-Wall" zwischen separaten Bekleidungen liegt,
# sollen Rays/Space-Probes die Bekleidung überspringen dürfen.
ENABLE_LAYER_STACK = True

# Nur sehr nahe Treffer als "Layer direkt davor" behandeln
LAYER_SKIP_NEAR_DIST = 0.25   # m

# Maximal wie weit Layer insgesamt übersprungen werden dürfen
LAYER_SKIP_MAX_DIST  = 0.80   # m

# Max. Anzahl übersprungener Layer in Folge
LAYER_SKIP_MAX_STEPS = 6

# kleiner Offset nach AABB-Exit (damit wir wirklich raus sind)
LAYER_SKIP_EXIT_EPS  = 0.01   # m

# IFC-Typen, die immer als Layer zählen
LAYER_SKIP_TYPES = {"IfcCovering"}

# Optional: sehr dünne Walls auch als Layer zählen (Putzschalen als IfcWall)
ALLOW_THIN_WALL_AS_LAYER = False
THIN_WALL_THICKNESS_MAX  = 0.12  # m (12cm)


# =========================================================
# IFC Zugriff (Bonsai / BlenderBIM)
# =========================================================
IfcStore = None
try:
    from bonsai.bim.ifc import IfcStore
except Exception:
    try:
        from blenderbim.bim.ifc import IfcStore
    except Exception:
        IfcStore = None

if not IfcStore:
    raise SystemExit("Bonsai/BlenderBIM (IfcStore) nicht gefunden – bitte Add-on aktivieren und IFC importieren.")

ifc = IfcStore.get_file()
if not ifc:
    raise SystemExit("Kein IFC in IfcStore geöffnet – bitte IFC mit Bonsai/BlenderBIM laden.")

def _ifc_ent(obj):
    ent = getattr(obj, "BIMObjectProperties", None)
    cid = getattr(ent, "ifc_definition_id", None) if ent else None
    if not cid:
        cid = obj.get("ifc_definition_id")
    try:
        return ifc.by_id(int(cid)) if cid else None
    except Exception:
        return None

def _is_ifcclass(obj, *names):
    e = _ifc_ent(obj)
    return bool(e and any(e.is_a(n) for n in names))

def get_ifc_guid_and_type(o):
    ent = _ifc_ent(o)
    if not ent:
        return None, "", ""
    guid = getattr(ent, "GlobalId", "") or ""
    ifc_type = ent.is_a()
    return ent.id(), guid, ifc_type



def _build_ifc_id_obj_map(meshes):
    """Map IFC STEP id -> Blender object (für schnelle Host-Lookups)."""
    m = {}
    for o in meshes:
        try:
            ent = _ifc_ent(o)
            if ent:
                m[int(ent.id())] = o
        except Exception:
            pass
    return m

def _host_wall_obj_for_door_window(obj, ifc_id_to_obj):
    """Gibt das Host-WALL Objekt (IfcWall/IfcWallStandardCase) für IfcDoor/IfcWindow zurück, sonst None."""
    ent = _ifc_ent(obj)
    if not ent:
        return None
    try:
        if not (ent.is_a("IfcDoor") or ent.is_a("IfcWindow")):
            return None
    except Exception:
        return None

    try:
        fills = getattr(ent, "FillsVoids", None) or []
    except Exception:
        fills = []

    for rel_fill in fills:
        opening = getattr(rel_fill, "RelatingOpeningElement", None)
        if not opening:
            continue
        try:
            voids = getattr(opening, "VoidsElements", None) or []
        except Exception:
            voids = []
        for rel_void in voids:
            wall = getattr(rel_void, "RelatingBuildingElement", None)
            if not wall:
                continue
            try:
                if wall.is_a("IfcWall") or wall.is_a("IfcWallStandardCase"):
                    wobj = ifc_id_to_obj.get(int(wall.id()))
                    if wobj:
                        return wobj
            except Exception:
                continue
    return None


def get_covering_predefined_type(o) -> str:
    """Gibt IfcCovering.PredefinedType als UPPERCASE String zurück (oder '')."""
    ent = _ifc_ent(o)
    if not ent:
        return ""
    try:
        pt = getattr(ent, "PredefinedType", None)
        if pt is None:
            return ""
        # IfcOpenShell kann Enum/Str liefern
        if hasattr(pt, "value"):
            pt = pt.value
        return str(pt).strip().upper()
    except Exception:
        return ""
# =========================================================
# Mesh / BVH Hilfen
# =========================================================
def mesh_copy_world(obj):
    dg = bpy.context.evaluated_depsgraph_get()
    eo = obj.evaluated_get(dg)
    me = bpy.data.meshes.new_from_object(eo, preserve_all_data_layers=True, depsgraph=dg)
    me.transform(obj.matrix_world)

    bm = bmesh.new()
    bm.from_mesh(me)
    bm.normal_update()
    bm.to_mesh(me)
    bm.free()

    me.calc_loop_triangles()
    me.update()
    return me

def bvh_from_mesh(me):
    if hasattr(BVHTree, "FromMesh"):
        return BVHTree.FromMesh(me)
    bm = bmesh.new()
    bm.from_mesh(me)
    bvh = BVHTree.FromBMesh(bm)
    bm.free()
    return bvh

# =========================================================
# Scene helpers
# =========================================================
def scene_meshes():
    return [o for o in bpy.context.scene.objects if o.type == 'MESH' and o.visible_get()]

# =========================================================
# Space BVH (nur bauen, wenn nötig)
# =========================================================
SPACES = []



EXTERNAL_SPACES = []  # externe Räume (für 'touch external' Regel)

def _space_text_blob(space_obj) -> str:
    """
    Sammelt alle relevanten Namensfelder für die Filterung:
    - Blender Objektname
    - IFC Space Name / LongName / ObjectType (wenn vorhanden)
    """
    parts = []
    try:
        parts.append(space_obj.name or "")
    except Exception:
        pass

    ent = _ifc_ent(space_obj)
    if ent:
        try:
            parts.append(getattr(ent, "Name", "") or "")
        except Exception:
            pass
        try:
            parts.append(getattr(ent, "LongName", "") or "")
        except Exception:
            pass
        try:
            parts.append(getattr(ent, "ObjectType", "") or "")
        except Exception:
            pass

    return " | ".join([p for p in parts if p]).lower()

def is_external_space_by_name(space_obj) -> bool:
    if not IGNORE_EXTERNAL_SPACES_BY_NAME:
        return False

    blob = _space_text_blob(space_obj)
    if not blob:
        return False

    for kw in EXTERNAL_SPACE_KEYWORDS:
        k = (kw or "").strip().lower()
        if not k:
            continue
        if EXTERNAL_SPACE_SUBSTRING_MATCH:
            if k in blob:
                return True
        else:
            # streng: nur wenn als eigenes Wort vorkommt
            if re.search(rf"\b{re.escape(k)}\b", blob):
                return True

    return False

def build_spaces_bvh_or_die():
    global SPACES, EXTERNAL_SPACES
    SPACES = []
    EXTERNAL_SPACES = []

    kept = 0
    kept_external = 0

    for o in bpy.data.objects:
        if o.type == "MESH" and _is_ifcclass(o, "IfcSpace"):

            # ✅ externe Spaces: nicht als "intern" zählen, aber separat behalten,
            # damit wir "touch external" für die Klassifikation erkennen können.
            if is_external_space_by_name(o):
                me = mesh_copy_world(o)
                EXTERNAL_SPACES.append((o.name, bvh_from_mesh(me)))
                bpy.data.meshes.remove(me, do_unlink=True)
                kept_external += 1
                continue

            me = mesh_copy_world(o)
            SPACES.append((o.name, bvh_from_mesh(me)))
            bpy.data.meshes.remove(me, do_unlink=True)
            kept += 1

    if not SPACES:
        raise SystemExit("[Abbruch] Keine (internen) IfcSpace-Objekte gefunden – Space-Methode braucht Räume.")

    print(f"[IfcSpace BVH] kept_internal={kept}, kept_external={kept_external}")

# =========================================================
# Space Inside-Test
# =========================================================
def inside_bvh(point, bvh):
    for d in (Vector((1,0,0)), Vector((0,1,0)), Vector((0,0,1))):
        o = point + d*1e-4
        hits, rem = 0, 1e5
        while True:
            loc, nor, face, dist = bvh.ray_cast(o, d, rem)
            if face is None:
                break
            hits += 1
            step = dist + 1e-5
            o += d*step
            rem -= step
        if hits % 2 == 1:
            return True
    return False

def _spaces_at_point_from_list(pt, spaces_list):
    hit = set()
    for nm, bvh in spaces_list:
        if inside_bvh(pt, bvh):
            hit.add(nm)
    return hit

def spaces_at_point(pt):
    return _spaces_at_point_from_list(pt, SPACES)

def external_spaces_at_point(pt):
    return _spaces_at_point_from_list(pt, EXTERNAL_SPACES)


def face_center(me, poly):
    c = Vector((0,0,0))
    for vi in poly.vertices:
        c += me.vertices[vi].co
    return c / len(poly.vertices)

def face_basis(me, poly):
    ids = list(poly.vertices)
    v0, v1 = me.vertices[ids[0]].co, me.vertices[ids[1]].co
    n = poly.normal.normalized()
    t1 = (v1 - v0) - ((v1 - v0).dot(n))*n
    if t1.length < 1e-6 and len(ids) > 2:
        v2 = me.vertices[ids[2]].co
        t1 = (v2 - v0) - ((v2 - v0).dot(n))*n
    t1 = t1.normalized()
    t2 = n.cross(t1).normalized()
    return t1, t2, n

def face_extents(me, poly, t1, t2, c):
    us, vs = [], []
    for vi in poly.vertices:
        p = me.vertices[vi].co - c
        us.append(p.dot(t1))
        vs.append(p.dot(t2))
    return (min(us), max(us)), (min(vs), max(vs))

def sample_grid(me, poly, n_u, n_v):
    c = face_center(me, poly)
    t1, t2, n = face_basis(me, poly)
    (umin, umax), (vmin, vmax) = face_extents(me, poly, t1, t2, c)
    bases = []
    for i in range(n_u):
        u = umin + (umax-umin)*(i+0.5)/n_u
        for j in range(n_v):
            v = vmin + (vmax-vmin)*(j+0.5)/n_v
            bases.append((c + t1*u + t2*v, t1, t2, n))
    return bases


def sample_poly_tri_points(me, poly, n_u, n_v, seed_tag=""):
    """Samplepunkte NUR innerhalb der Polygon-Triangles (kein AABB-Extents-Leak).
    Liefert wie sample_grid: (base, t1, t2, n).
    Deterministisch via seed_tag + poly.index.
    """
    c = face_center(me, poly)
    t1, t2, n = face_basis(me, poly)

    # Loop-Triangles pro Polygon sammeln
    tris = [lt for lt in me.loop_triangles if lt.polygon_index == poly.index]
    if not tris:
        # Fallback auf Grid (sollte selten passieren)
        return sample_grid(me, poly, n_u, n_v)

    # Flächen für area-weighted Auswahl
    tri_areas = []
    tri_verts = []
    total = 0.0
    for lt in tris:
        ids = lt.vertices
        a = me.vertices[ids[0]].co
        b = me.vertices[ids[1]].co
        c3 = me.vertices[ids[2]].co
        area = ((b - a).cross(c3 - a)).length * 0.5
        if area <= 1e-12:
            continue
        total += area
        tri_areas.append(total)
        tri_verts.append((a, b, c3))
    if not tri_verts:
        return sample_grid(me, poly, n_u, n_v)

    # deterministische RNG
    r = random.Random()
    try:
        r.seed(f"{seed_tag}|{poly.index}")
    except Exception:
        r.seed(poly.index)

    count = max(int(n_u) * int(n_v), 1)
    out = []
    for _i in range(count):
        # area-weighted triangle pick
        x = r.random() * tri_areas[-1]
        lo, hi = 0, len(tri_areas) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if x <= tri_areas[mid]:
                hi = mid
            else:
                lo = mid + 1
        a, b, c3 = tri_verts[lo]

        # uniform point in triangle
        u = r.random()
        v = r.random()
        if u + v > 1.0:
            u = 1.0 - u
            v = 1.0 - v
        base = a + (b - a) * u + (c3 - a) * v
        out.append((base, t1, t2, n))
    return out

def filter_major_faces(faces, fraction=MAJOR_FRACTION):
    if not faces:
        return faces
    faces_sorted = sorted(faces, key=lambda p: p.area, reverse=True)
    total = sum(p.area for p in faces_sorted)
    limit = total * fraction
    acc = 0.0
    result = []
    for p in faces_sorted:
        result.append(p)
        acc += p.area
        if acc >= limit:
            break
    return result

def depths():
    if DEPTH_STEPS <= 1:
        return [EPS]
    inc = (SEARCH_MAX - EPS) / max(DEPTH_STEPS-1, 1)
    return [EPS + i*inc for i in range(DEPTH_STEPS)]

def jitter_offsets(t1, t2):
    offs = [Vector((0,0,0))]
    if JITTER_SAMPLES >= 3:
        offs += [ t1*EDGE_JITTER, -t1*EDGE_JITTER ]
    if JITTER_SAMPLES >= 5:
        offs += [ t2*EDGE_JITTER, -t2*EDGE_JITTER ]
    return offs

# =========================================================
# Storey / Typ / Auswahl
# =========================================================
def storey_name(o):
    for c in o.users_collection:
        if "IfcBuildingStorey" in c.name:
            return c.name
    return ""

def is_in_target_storeys(o, target_names):
    if not target_names:
        return True
    for c in o.users_collection:
        if c.name in target_names:
            return True
    return False

def ifc_type_and_role(o):
    base = o.name.split(".")[0]
    parts = base.split("/")
    ifc_type = parts[0]
    role = parts[1] if len(parts) > 1 else ""
    return ifc_type, role

def obj_ifc_type_name(o):
    ent = _ifc_ent(o)
    if ent:
        return ent.is_a()
    ifc_type, _ = ifc_type_and_role(o)
    return ifc_type

def is_selected_ifc_type(o, allowed_types: set):
    if o.type != "MESH" or not o.visible_get():
        return False
    return obj_ifc_type_name(o) in allowed_types

# =========================================================
# Storey Enum Items (sortiert + stabile Labels)
# =========================================================
def _storey_items(self, context):
    tmp = []
    for c in bpy.data.collections:
        if c.name.startswith("IfcBuildingStorey/"):
            label = c.name.split("IfcBuildingStorey/", 1)[1].strip()
            if not label:
                label = c.name
            tmp.append((c.name, label))

    def sort_key(item):
        label = item[1]
        l = label.lower()

        # Basement zuerst
        if l.startswith(("ba", "ug", "-1")):
            return (-1000, label)

        # E0, E1, E2 ...
        m = re.match(r"^e\s*([+-]?\d+)", l)
        if m:
            return (int(m.group(1)), label)

        # fallback: erste Zahl irgendwo
        m2 = re.search(r"([+-]?\d+)", label)
        if m2:
            return (int(m2.group(1)), label)

        return (9999, label)

    tmp.sort(key=sort_key)

    items = [(ident, name, "") for ident, name in tmp]
    if not items:
        items = [("", "<keine Storeys>", "")]
    return items

# =========================================================
# AABB Helpers (mit per-wall ignore_map + opening-avoid samples)
# =========================================================
def aabb_of_objects(objs):
    inf = float('inf')
    ninf = float('-inf')
    bb_min = Vector((inf, inf, inf))
    bb_max = Vector((ninf, ninf, ninf))
    for o in objs:
        for c in o.bound_box:
            wp = o.matrix_world @ Vector(c)
            bb_min.x = min(bb_min.x, wp.x)
            bb_min.y = min(bb_min.y, wp.y)
            bb_min.z = min(bb_min.z, wp.z)
            bb_max.x = max(bb_max.x, wp.x)
            bb_max.y = max(bb_max.y, wp.y)
            bb_max.z = max(bb_max.z, wp.z)
    return bb_min, bb_max


def point_inside_aabb(p: Vector, bb_min: Vector, bb_max: Vector, eps: float = 1e-6) -> bool:
    return (bb_min.x - eps <= p.x <= bb_max.x + eps and
            bb_min.y - eps <= p.y <= bb_max.y + eps and
            bb_min.z - eps <= p.z <= bb_max.z + eps)

def ray_aabb_exit_distance(origin: Vector, direction: Vector, bb_min: Vector, bb_max: Vector):
    tmin, tmax = -1e30, 1e30
    for i in range(3):
        o = origin[i]
        d = direction[i]
        lo = bb_min[i]
        hi = bb_max[i]
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
        else:
            t1 = (lo - o) / d
            t2 = (hi - o) / d
            tmin = max(tmin, min(t1, t2))
            tmax = min(tmax, max(t1, t2))
    if tmax < max(0.0, tmin):
        return None
    return tmax if tmax > 0 else None

# ---- Door/Window AABB Blocker Helpers ----
def ray_aabb_intervals(origin: Vector, direction: Vector, bb_min: Vector, bb_max: Vector):
    tmin, tmax = -1e30, 1e30
    for i in range(3):
        o = origin[i]
        d = direction[i]
        lo = bb_min[i]
        hi = bb_max[i]
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None, None
        else:
            t1 = (lo - o) / d
            t2 = (hi - o) / d
            ta = min(t1, t2)
            tb = max(t1, t2)
            tmin = max(tmin, ta)
            tmax = min(tmax, tb)
            if tmax < tmin:
                return None, None
    return tmin, tmax

def segment_intersects_aabb(p0: Vector, p1: Vector, bb_min: Vector, bb_max: Vector) -> bool:
    v = (p1 - p0)
    L = v.length
    if L < 1e-9:
        return False
    d = v / L
    tmin, tmax = ray_aabb_intervals(p0, d, bb_min, bb_max)
    if tmin is None:
        return False
    if tmax < 0.0:
        return False
    if tmin > L:
        return False
    return True


def segment_aabb_first_hit_dist(p0: Vector, p1: Vector, bb_min: Vector, bb_max: Vector):
    """Gibt die Eintrittsdistanz (in Metern) entlang des Segments p0->p1 zurück, oder None."""
    v = (p1 - p0)
    L = v.length
    if L < 1e-9:
        return None
    d = v / L
    tmin, tmax = ray_aabb_intervals(p0, d, bb_min, bb_max)
    if tmin is None:
        return None
    if tmax < 0.0:
        return None
    t_entry = max(tmin, 0.0)
    if t_entry > L:
        return None
    return t_entry

# =========================================================
# Layer Skip Helpers (NEU) ✅✅✅
# =========================================================
_AABB_CACHE = {}

def aabb_cached(o):
    key = o.name
    if key in _AABB_CACHE:
        return _AABB_CACHE[key]
    bmin, bmax = aabb_of_objects([o])
    _AABB_CACHE[key] = (bmin, bmax)
    return bmin, bmax

def is_glued_slab_below_obj(hit_obj, src_obj, dirn: Vector) -> bool:
    """True wenn hit_obj ein IfcSlab ist, der direkt UNTER src_obj klebt (Fundament etc.)."""
    if not SLAB_GLUE_IGNORE_BELOW:
        return False
    if (hit_obj is None) or (src_obj is None) or (hit_obj == src_obj):
        return False

    # Nur für downward-Rays relevant
    try:
        if dirn.z >= -0.20:
            return False
    except Exception:
        return False

    ent_src = _ifc_ent(src_obj)
    ent_hit = _ifc_ent(hit_obj)
    if not ent_src or not ent_hit:
        return False
    if not ent_src.is_a("IfcSlab") or not ent_hit.is_a("IfcSlab"):
        return False

    try:
        smin, smax = aabb_cached(src_obj)
        hmin, hmax = aabb_cached(hit_obj)
    except Exception:
        return False

    # Hit muss unterhalb liegen und quasi anliegen: hit_top ~= src_bottom
    if hmax.z > (smin.z + SLAB_GLUE_Z_EPS):
        return False
    if abs(hmax.z - smin.z) > SLAB_GLUE_Z_EPS:
        return False

    # XY-Überdeckung prüfen (nur "geklebt" wenn ausreichend Deckung)
    sx = max(0.0, smax.x - smin.x)
    sy = max(0.0, smax.y - smin.y)
    area_src = sx * sy
    if area_src < 1e-9:
        return False

    ox = min(smax.x, hmax.x) - max(smin.x, hmin.x)
    oy = min(smax.y, hmax.y) - max(smin.y, hmin.y)
    if ox <= 0.0 or oy <= 0.0:
        return False

    overlap = ox * oy
    return (overlap / area_src) >= SLAB_GLUE_OVERLAP_FRAC

def aabb_extent_along_dir(bb_min: Vector, bb_max: Vector, d: Vector) -> float:
    d = d.normalized()
    corners = [
        Vector((bb_min.x, bb_min.y, bb_min.z)),
        Vector((bb_min.x, bb_min.y, bb_max.z)),
        Vector((bb_min.x, bb_max.y, bb_min.z)),
        Vector((bb_min.x, bb_max.y, bb_max.z)),
        Vector((bb_max.x, bb_min.y, bb_min.z)),
        Vector((bb_max.x, bb_min.y, bb_max.z)),
        Vector((bb_max.x, bb_max.y, bb_min.z)),
        Vector((bb_max.x, bb_max.y, bb_max.z)),
    ]
    projs = [c.dot(d) for c in corners]
    return max(projs) - min(projs)

def is_skippable_layer_obj(hit_obj, dirn: Vector, dist_to_hit: float) -> bool:
    if not ENABLE_LAYER_STACK:
        return False
    if dist_to_hit > LAYER_SKIP_NEAR_DIST:
        return False

    ent = _ifc_ent(hit_obj)
    if not ent:
        return False

    if ent.is_a() in LAYER_SKIP_TYPES:
        return True

    if ALLOW_THIN_WALL_AS_LAYER and (ent.is_a("IfcWall") or ent.is_a("IfcWallStandardCase")):
        bmin, bmax = aabb_cached(hit_obj)
        thick = aabb_extent_along_dir(bmin, bmax, dirn)
        return thick <= THIN_WALL_THICKNESS_MAX

    return False

def advance_past_obj_aabb(p: Vector, d: Vector, hit_obj):
    try:
        bmin, bmax = aabb_cached(hit_obj)
        tmin, tmax = ray_aabb_intervals(p, d, bmin, bmax)
        if tmax is None:
            return None
        if tmax < 0.0:
            return None
        return max(tmax + LAYER_SKIP_EXIT_EPS, 0.0)
    except Exception:
        return None

def build_blocker_aabbs(all_visible_meshes):
    blockers = []
    for o in all_visible_meshes:
        ent = _ifc_ent(o)
        if not ent:
            continue
        if ent.is_a("IfcDoor") or ent.is_a("IfcWindow"):
            bmin, bmax = aabb_of_objects([o])
            blockers.append((o, bmin, bmax))
    return blockers


def build_curtainwall_aabbs(all_visible_meshes):
    """AABB-Proxy-Blocker für IfcCurtainWall (gemerged).
    Motivation:
      - CurtainWalls sind oft als viele kleine Elemente modelliert (Rahmen/Mullions/Paneele),
        sodass ein Ray zwischen Teil-Objekten "entkommen" kann.
      - Für die Klassifikation wollen wir die Fassade aber wie eine geschlossene Hülle behandeln.
    Umsetzung:
      - pro IfcCurtainWall ein AABB (leicht inflated)
      - danach optionales Merge von benachbarten AABBs (CURTAINWALL_AABB_BLOCKER_MERGE_GAP),
        damit Teil-Objekte gemeinsam als Blocker wirken.
    """
    raw = []
    inflate = float(CURTAINWALL_AABB_BLOCKER_INFLATE) if 'CURTAINWALL_AABB_BLOCKER_INFLATE' in globals() else 0.0
    merge_gap = float(CURTAINWALL_AABB_BLOCKER_MERGE_GAP) if 'CURTAINWALL_AABB_BLOCKER_MERGE_GAP' in globals() else 0.0

    for o in all_visible_meshes:
        ent = _ifc_ent(o)
        if not ent:
            continue
        if ent.is_a("IfcCurtainWall"):
            bmin, bmax = aabb_of_objects([o])
            if inflate > 0.0:
                bmin = Vector((bmin.x - inflate, bmin.y - inflate, bmin.z - inflate))
                bmax = Vector((bmax.x + inflate, bmax.y + inflate, bmax.z + inflate))
            raw.append((o, bmin, bmax))

    if not raw or merge_gap <= 0.0:
        return raw

    def _aabb_close(a_min, a_max, b_min, b_max, gap: float) -> bool:
        # Overlap/near in all three axes (expanded by gap)
        if a_min.x > b_max.x + gap or a_max.x + gap < b_min.x:
            return False
        if a_min.y > b_max.y + gap or a_max.y + gap < b_min.y:
            return False
        if a_min.z > b_max.z + gap or a_max.z + gap < b_min.z:
            return False
        return True

    merged = list(raw)
    changed = True
    while changed:
        changed = False
        out = []
        while merged:
            o0, mn0, mx0 = merged.pop()
            i = 0
            while i < len(merged):
                o1, mn1, mx1 = merged[i]
                if _aabb_close(mn0, mx0, mn1, mx1, merge_gap):
                    mn0 = Vector((min(mn0.x, mn1.x), min(mn0.y, mn1.y), min(mn0.z, mn1.z)))
                    mx0 = Vector((max(mx0.x, mx1.x), max(mx0.y, mx1.y), max(mx0.z, mx1.z)))
                    merged.pop(i)
                    changed = True
                    continue
                i += 1
            out.append((o0, mn0, mx0))
        merged = out

    return merged
def build_wall_local_ignore_map(elems):
    """
    src_wall.name -> set(step_ids) von:
      - IfcOpeningElement(s), die diese Wand voiden
      - deren Fillings (IfcDoor/IfcWindow/...)
    """
    m = {}
    for el in elems:
        ids = set()
        ee = _ifc_ent(el)
        if ee:
            try:
                for rel_voids in (getattr(ee, "HasOpenings", None) or []):
                    opening = getattr(rel_voids, "RelatedOpeningElement", None)
                    if opening:
                        try:
                            ids.add(opening.id())
                        except Exception:
                            pass

                        for rel_fill in (getattr(opening, "HasFillings", None) or []):
                            fill = getattr(rel_fill, "RelatedBuildingElement", None)
                            if fill:
                                try:
                                    ids.add(fill.id())
                                except Exception:
                                    pass
            except Exception:
                pass
        m[el.name] = ids
    return m

def should_ignore_hit_for_wall(hit_obj, src_obj, ignore_map):
    if hit_obj == src_obj:
        return True

    ent = _ifc_ent(hit_obj)
    if not ent:
        return False  # konservativ: blockt

    # Spaces blocken nie
    if ent.is_a("IfcSpace"):
        return True

    try:
        hid = ent.id()
    except Exception:
        return False

    return hid in ignore_map.get(src_obj.name, set())

def is_opening_like_object_for_wall(hit_obj, src_obj, ignore_map):
    """
    True wenn Hit ein Opening/Filling der SAME wall ist (also: Startpunkt liegt in Öffnung)
    """
    ent = _ifc_ent(hit_obj)
    if not ent:
        return False
    try:
        hid = ent.id()
    except Exception:
        return False

    if hid not in ignore_map.get(src_obj.name, set()):
        return False

    return ent.is_a("IfcOpeningElement") or ent.is_a("IfcDoor") or ent.is_a("IfcWindow")

def ray_face_sample_offsets(n_world: Vector):
    t1 = n_world.orthogonal()
    if t1.length < 1e-9:
        t1 = Vector((1, 0, 0))
    t1.normalize()
    t2 = n_world.cross(t1)
    if t2.length < 1e-9:
        t2 = Vector((0, 1, 0))
    t2.normalize()

    L = OPENING_AVOID_OFFSET
    return [
        Vector((0,0,0)),
        +t1 * L,
        -t1 * L,
        +t2 * L,
        -t2 * L,
    ]

def cast_outwards_until_exit(scene, deps, start: Vector, dirn: Vector,
                             bb_min, bb_max, src_obj, ignore_map, blockers):
    """
    True  = freie Sicht bis zur AABB-Grenze -> External
    False = blockiert -> Internal
    """
    p = start.copy()
    d = dirn.normalized()
    limit = ray_aabb_exit_distance(p, d, bb_min, bb_max)
    if limit is None:
        # If we start outside the model AABB (tight bounds + START_EPS), treat as immediate escape.
        if not point_inside_aabb(p, bb_min, bb_max):
            return True
        return False

    remaining = max(0.0, limit - 1e-4)
    steps = 0

    while remaining > 0 and steps < MAX_STEPS_AABB:
        hit, loc, nor, idx, obj, *_ = scene.ray_cast(deps, p, d, distance=remaining)
        if not hit:
            # Escape -> aber vorher Door/Window AABB Blocker prüfen
            if USE_DOOR_WINDOW_AABB_BLOCKERS and blockers:
                endpos = p + d * remaining
                for bo, bmin, bmax in blockers:
                    # eigene Öffnung/Filling derselben Wand NICHT blocken
                    if should_ignore_hit_for_wall(bo, src_obj, ignore_map):
                        continue
                    if segment_intersects_aabb(p, endpos, bmin, bmax):
                        return False
            # CurtainWall-AABB-Blocker: Fassade als geschlossene Hülle behandeln
            if USE_CURTAINWALL_AABB_BLOCKERS_FOR_AABB and _IE_CW_AABB_BLOCKERS:
                endpos_cw = p + d * remaining
                for bo, bmin, bmax in _IE_CW_AABB_BLOCKERS:
                    if segment_intersects_aabb(p, endpos_cw, bmin, bmax):
                        return False
            return True

        dist_hit = (loc - p).length

        # ---- NEU: Slab-Stack Skip (geklebt unten) ----
        if is_glued_slab_below_obj(obj, src_obj, d):
            adv_aabb = advance_past_obj_aabb(p, d, obj)
            advance = adv_aabb if adv_aabb is not None else (dist_hit + STEP_EPS_AABB)
            advance = max(advance, dist_hit + STEP_EPS_AABB)
            p = p + d * advance
            remaining -= advance
            steps += 1
            continue

        if should_ignore_hit_for_wall(obj, src_obj, ignore_map):
            advance = dist_hit + STEP_EPS_AABB
            p = p + d * advance
            remaining -= advance
            steps += 1
            continue

        # ---- NEU: Layer-Stack Skip (Covering / dünne Schale) ----
        if is_skippable_layer_obj(obj, d, dist_hit):
            adv_aabb = advance_past_obj_aabb(p, d, obj)
            advance = adv_aabb if adv_aabb is not None else (dist_hit + STEP_EPS_AABB)
            advance = max(advance, dist_hit + STEP_EPS_AABB)
            p = p + d * advance
            remaining -= advance
            steps += 1
            continue

        return False

    return remaining <= 0

def aabb_classify_one_object_improved(obj, bb_min, bb_max, deps, scene, ignore_map, blockers):
    eo = obj.evaluated_get(deps)
    me = eo.to_mesh()

    R = obj.matrix_world.to_3x3()
    N_xf = R.inverted().transposed()


    ent = _ifc_ent(obj)
    ifc_type = ent.is_a() if ent else ""
    ptype = (str(getattr(ent, "PredefinedType", "") or "").upper() if ent else "")
    is_covering_hor = (ifc_type == "IfcCovering" and ptype in COVERING_PTYPE_HORIZONTAL)
    is_slab = (ifc_type == "IfcSlab")
    faces = []
    max_area = 0.0

    for poly in me.polygons:
        n_world = (N_xf @ poly.normal).normalized()
        if (is_covering_hor and abs(n_world.z) >= COVERING_HOR_FACE_Z_ABS_THR) or (is_slab and abs(n_world.z) >= SLAB_HOR_FACE_Z_ABS_THR) or ((not is_covering_hor) and (not is_slab) and abs(n_world.z) < VERTICAL_TOL_WALL):
            area = poly.area
            max_area = max(max_area, area)
            faces.append((area, poly.index, n_world))

    if not faces:
        # horizontale Coverings können bei leicht geneigten Flächen keine strikt-horizontalen Faces liefern
        if is_covering_hor:
            for poly in me.polygons:
                n_world = (N_xf @ poly.normal).normalized()
                if abs(n_world.z) >= 0.30:
                    area = poly.area
                    max_area = max(max_area, area)
                    faces.append((area, poly.index, n_world))

        # IfcSlab: analog zu Wänden, aber horizontale Faces (Top/Bottom) – Fallback für geneigte/unsaubere Geometrie
        if (not faces) and is_slab:
            for poly in me.polygons:
                n_world = (N_xf @ poly.normal).normalized()
                if abs(n_world.z) >= Z_THR_SLAB_HOR:
                    area = poly.area
                    max_area = max(max_area, area)
                    faces.append((area, poly.index, n_world))
        if not faces:
            eo.to_mesh_clear()
            return "Internal"

    # FIX 4: Nur Side-Normalen zulassen (gegen Querfaces/Reveals)
    # Idee: dominante Seiten-Normale über größte Kandidatenfläche bestimmen,
    # dann nur Faces zulassen, deren Normale stark parallel/antiparallel dazu ist.
    if AABB_SIDE_NORMAL_ONLY and (not is_covering_hor) and faces:
        try:
            nA = max(faces, key=lambda t: t[0])[2]
            if nA.length > 0:
                nA = nA.normalized()
                filtered = [(a, i, n) for (a, i, n) in faces if abs(n.dot(nA)) >= AABB_SIDE_NORMAL_DOT_THR]
                if filtered:
                    faces = filtered
        except Exception:
            pass

    area_cut = max_area * REL_AREA_CUT
    faces = [(a, i, n) for (a, i, n) in faces if a >= area_cut]
    faces.sort(reverse=True)
    faces = faces[:FACE_COUNT]

    is_external = False
    # IfcCovering (horizontal): require escapes on BOTH sides (one-side blocked => Internal)
    esc_up = False
    esc_down = False

    for _, idx, n_world in faces:
        p_local = me.polygons[idx].center
        p_world_center = obj.matrix_world @ p_local

        offsets = ray_face_sample_offsets(n_world)

        for sgn in (+1.0, -1.0):
            d = (n_world * sgn).normalized()

            for off in offsets:
                start = p_world_center + off + d * START_EPS_AABB

                # Probe: wenn sofort eine Tür/Fenster/Opening DERSELBEN Wand kommt -> skip
                hit, loc, nor, face_i, hit_obj, *_ = scene.ray_cast(deps, start, d, distance=OPENING_NEAR_PROBE_DIST)
                if hit and is_opening_like_object_for_wall(hit_obj, obj, ignore_map):
                    continue

                if cast_outwards_until_exit(scene, deps, start, d, bb_min, bb_max, obj, ignore_map, blockers):
                    if is_covering_hor:
                        # For horizontal coverings, a single escape on one side is NOT enough.
                        # External only if we can escape both upward and downward.
                        if d.z >= 0.0:
                            esc_up = True
                        else:
                            esc_down = True
                        if esc_up and esc_down:
                            is_external = True
                            break
                        # escape found for this side; no need more offsets in same direction
                        break
                    else:
                        is_external = True
                        break

            if is_external:
                break

        if is_external:
            break

    eo.to_mesh_clear()
    return "External" if is_external else "Internal"


# =========================================================
# IfcColumn (NEU): Embedded-Wall Ignore + 4-Side Probes
# =========================================================
def _aabb_overlap_frac_xy(src_min: Vector, src_max: Vector, other_min: Vector, other_max: Vector) -> float:
    ox = min(src_max.x, other_max.x) - max(src_min.x, other_min.x)
    oy = min(src_max.y, other_max.y) - max(src_min.y, other_min.y)
    if ox <= 0.0 or oy <= 0.0:
        return 0.0
    area_src = max((src_max.x - src_min.x) * (src_max.y - src_min.y), 1e-9)
    return (ox * oy) / area_src

def _aabb_overlap_frac_z(src_min: Vector, src_max: Vector, other_min: Vector, other_max: Vector) -> float:
    oz = min(src_max.z, other_max.z) - max(src_min.z, other_min.z)
    if oz <= 0.0:
        return 0.0
    h_src = max(src_max.z - src_min.z, 1e-9)
    return oz / h_src

def build_column_embedded_wall_ignore_map(columns, walls):
    """src_column.name -> set(step_ids) von Walls, in denen die Stütze 'steckt' (AABB-Overlap-basiert).
    Ziel: Rays/Space-Probes sollen diese Walls ignorieren können (Column-in-Wall Modellierung).
    """
    m = {}
    for col in columns:
        ids = set()
        ent_col = _ifc_ent(col)
        if not ent_col:
            m[col.name] = ids
            continue

        try:
            cmin, cmax = aabb_cached(col)
        except Exception:
            m[col.name] = ids
            continue

        for w in walls:
            if w == col:
                continue
            ent_w = _ifc_ent(w)
            if not ent_w:
                continue
            if not (ent_w.is_a("IfcWall") or ent_w.is_a("IfcWallStandardCase")):
                continue

            try:
                wmin, wmax = aabb_cached(w)
            except Exception:
                continue

            # robust: require overlap in XY + enough Z overlap (stütze steckt in wand, nicht nur berühren)
            frac_xy = _aabb_overlap_frac_xy(cmin, cmax, wmin, wmax)
            if frac_xy < 0.25:
                continue
            frac_z = _aabb_overlap_frac_z(cmin, cmax, wmin, wmax)
            if frac_z < 0.50:
                continue

            try:
                ids.add(ent_w.id())
            except Exception:
                pass

        m[col.name] = ids
    return m

def should_ignore_hit_for_column(hit_obj, src_obj, col_wall_ignore_map):
    # NOTE: For AABB/Raycast classification we ignore:
    # - self hits (numerical/START_EPS)
    # - IfcSpace objects (they should never block geometry rays)
    # - IfcCovering objects (cladding/finishes should not block geometry rays)
    if hit_obj == src_obj:
        return True

    ent = _ifc_ent(hit_obj)
    if not ent:
        return False  # konservativ: blockt

    # Spaces blocken nie
    if ent.is_a("IfcSpace"):
        return True

    # Coverings blocken für Column-Rays nicht (Außenbekleidungen etc.)
    if ent.is_a("IfcCovering"):
        return True

    return False




def should_ignore_hit_for_beam(hit_obj, src_obj, col_wall_ignore_map=None):
    # NOTE: For AABB/Raycast classification on BEAMS we ignore:
    # - self hits (numerical/START_EPS)
    # - IfcSpace objects (they should never block geometry rays)
    # We do NOT ignore IfcCovering here, because facade/skins are often modeled as coverings and must block.
    if hit_obj == src_obj:
        return True

    ent = _ifc_ent(hit_obj)
    if not ent:
        return False  # konservativ: blockt

    if ent.is_a("IfcSpace"):
        return True

    return False

def should_ignore_hit_for_column_space(hit_obj, src_obj, col_wall_ignore_map):
    # NOTE: For IfcSpace probing on columns we additionally ignore:
    # - embedded walls (Column-in-Wall Modellierung) based on col_wall_ignore_map
    if hit_obj == src_obj:
        return True

    ent = _ifc_ent(hit_obj)
    if not ent:
        return False  # konservativ: blockt

    if ent.is_a("IfcSpace"):
        return True

    # Coverings sind keine "Raumgrenzen" -> beim Space-Probing überspringen
    if ent.is_a("IfcCovering"):
        return True

    # Embedded-Walls für diese Column überspringen
    try:
        ids = col_wall_ignore_map.get(src_obj.name, set()) if (col_wall_ignore_map and src_obj) else set()
        if ids and (ent.is_a("IfcWall") or ent.is_a("IfcWallStandardCase")):
            try:
                if ent.id() in ids:
                    return True
            except Exception:
                pass
    except Exception:
        pass

    return False


def cast_outwards_until_exit_column(scene, deps, start: Vector, dirn: Vector,
                                    bb_min, bb_max, src_obj, col_wall_ignore_map, blockers):
    """True = Escape => External; False = Blocker => Internal.
    NOTE: AABB/Raycast für Columns ignoriert KEINE embedded walls (nur self + IfcSpace).
    """
    p = start.copy()
    d = dirn.normalized()
    # Robust: nutze Modell-AABB Diagonale als Max-Distanz (statt Exit-Distance),
    # damit Rays auch dann sauber laufen, wenn der Startpunkt minimal außerhalb einer "tight" AABB liegt.
    remaining = (bb_max - bb_min).length + 1.0
    steps = 0

    # Column-Fix: Door/Window-AABB-Blocker nur nutzen, wenn der Ray-Startpunkt in einem INTERNAL Space liegt.
    start_in_internal_space = False
    if COLUMNS_DW_BLOCKERS_ONLY_IF_START_IN_INTERNAL_SPACE:
        try:
            start_in_internal_space = bool(spaces_at_point(start))
        except Exception:
            start_in_internal_space = False

    while remaining > 0 and steps < MAX_STEPS_AABB:
        hit, loc, nor, idx, obj, *_ = scene.ray_cast(deps, p, d, distance=remaining)
        if not hit:
            # optional: Door/Window AABB Blocker prüfen (konservativ)
            # Für Columns nur dann aktivieren, wenn der Ray-Startpunkt in einem INTERNAL Space liegt.
            if USE_DOOR_WINDOW_AABB_BLOCKERS and blockers and (not COLUMNS_DW_BLOCKERS_ONLY_IF_START_IN_INTERNAL_SPACE or start_in_internal_space):
                endpos = p + d * remaining
                for bo, bmin, bmax in blockers:
                    # keine speziellen "same-wall" Ausnahmen für Columns
                    if segment_intersects_aabb(p, endpos, bmin, bmax):
                        return False
            # CurtainWall-AABB-Blocker: Fassade als geschlossene Hülle behandeln
            if USE_CURTAINWALL_AABB_BLOCKERS_FOR_AABB and _IE_CW_AABB_BLOCKERS:
                endpos_cw = p + d * remaining
                for bo, bmin, bmax in _IE_CW_AABB_BLOCKERS:
                    if segment_intersects_aabb(p, endpos_cw, bmin, bmax):
                        return False
            return True

        dist_hit = (loc - p).length

        if should_ignore_hit_for_column(obj, src_obj, col_wall_ignore_map):
            adv_aabb = advance_past_obj_aabb(p, d, obj)
            advance = adv_aabb if adv_aabb is not None else (dist_hit + STEP_EPS_AABB)
            advance = max(advance, dist_hit + STEP_EPS_AABB)
            p = p + d * advance
            remaining -= advance
            steps += 1
            continue

        # sonst: echter Blocker
        return False

    return False





def cast_outwards_until_exit_beam(scene, deps, start: Vector, dirn: Vector,
                                  bb_min, bb_max, src_obj, col_wall_ignore_map, blockers, cw_blockers):
    """True = Escape => External; False = Blocker => Internal.
    BEAM variant: ignores only self + IfcSpace (does NOT ignore IfcCovering).
    """
    p = start.copy()
    d = dirn.normalized()
    remaining = (bb_max - bb_min).length + 1.0
    steps = 0

    start_in_internal_space = False
    if COLUMNS_DW_BLOCKERS_ONLY_IF_START_IN_INTERNAL_SPACE:
        try:
            start_in_internal_space = bool(spaces_at_point(start))
        except Exception:
            start_in_internal_space = False

    while remaining > 0 and steps < MAX_STEPS_AABB:
        hit, loc, nor, idx, obj, *_ = scene.ray_cast(deps, p, d, distance=remaining)
        if not hit:
            if USE_DOOR_WINDOW_AABB_BLOCKERS and blockers and (not COLUMNS_DW_BLOCKERS_ONLY_IF_START_IN_INTERNAL_SPACE or start_in_internal_space):
                endpos = p + d * remaining
                for bo, bmin, bmax in blockers:
                    if segment_intersects_aabb(p, endpos, bmin, bmax):
                        return False

            # Beam-Fix: CurtainWall-AABB-Blocker (Fassade als geschlossene Hülle behandeln)
            if USE_CURTAINWALL_AABB_BLOCKERS_FOR_BEAMS and cw_blockers:
                endpos2 = p + d * remaining
                for bo, bmin, bmax in cw_blockers:
                    if segment_intersects_aabb(p, endpos2, bmin, bmax):
                        return False
            return True
        dist_hit = (loc - p).length

        if should_ignore_hit_for_beam(obj, src_obj, col_wall_ignore_map):
            adv_aabb = advance_past_obj_aabb(p, d, obj)
            advance = adv_aabb if adv_aabb is not None else (dist_hit + STEP_EPS_AABB)
            advance = max(advance, dist_hit + STEP_EPS_AABB)
            p = p + d * advance
            remaining -= advance
            steps += 1
            continue

        return False

    return False

def _column_local_bb_params(obj):
    """Column local bounding box params based on obj.bound_box (object local coords).
    Returns: (bbminL, bbmaxL, centerL, hx, hy, hz)
    """
    bb = [Vector(c) for c in getattr(obj, "bound_box", [])] if hasattr(obj, "bound_box") else []
    if not bb or len(bb) < 8:
        raise RuntimeError("no_bound_box")

    minx = min(v.x for v in bb); maxx = max(v.x for v in bb)
    miny = min(v.y for v in bb); maxy = max(v.y for v in bb)
    minz = min(v.z for v in bb); maxz = max(v.z for v in bb)

    bbminL = Vector((minx, miny, minz))
    bbmaxL = Vector((maxx, maxy, maxz))
    centerL = (bbminL + bbmaxL) * 0.5
    hx = (maxx - minx) * 0.5
    hy = (maxy - miny) * 0.5
    hz = (maxz - minz) * 0.5
    return bbminL, bbmaxL, centerL, hx, hy, hz


def _column_axes_world(obj):
    """Returns normalized world axes for the object local X/Y/Z."""
    R = obj.matrix_world.to_3x3()
    ax = (R @ Vector((1.0, 0.0, 0.0)))
    ay = (R @ Vector((0.0, 1.0, 0.0)))
    az = (R @ Vector((0.0, 0.0, 1.0)))

    if ax.length < 1e-12:
        ax = Vector((1.0, 0.0, 0.0))
    if ay.length < 1e-12:
        ay = Vector((0.0, 1.0, 0.0))
    if az.length < 1e-12:
        az = Vector((0.0, 0.0, 1.0))

    ax.normalize(); ay.normalize(); az.normalize()
    return ax, ay, az


def _column_obb_sample_starts(obj, bvh, bbminL, bbmaxL, centerL, axis_key: str, sign: float, d_world: Vector, z_samples_override=None):
    """Startpunkte für Column-Rays basierend auf OBB (objektlokale bound_box + Objektachsen).
    axis_key: 'x' oder 'y', sign: +1/-1 für jeweilige Seite, d_world: Welt-Richtung (normalized).
    """
    # local extents
    ext_x = max(0.0, (bbmaxL.x - bbminL.x))
    ext_y = max(0.0, (bbmaxL.y - bbminL.y))
    height = max(0.0, (bbmaxL.z - bbminL.z))

    # Z-sampling in local Z (entlang Objekt-Z-Achse)
    used_samples = int(z_samples_override) if (z_samples_override is not None) else int(COLUMN_AABB_Z_SAMPLES)
    used_samples = max(1, used_samples)

    offsets = [0.0]
    if used_samples >= 2 and height > 1e-9:
        off = COLUMN_AABB_Z_FRAC * height
        offsets += [off, -off]

    starts = []
    for oz in offsets[:used_samples]:
        z = centerL.z + oz
        if height > 1e-9:
            z = max(bbminL.z + COLUMN_AABB_Z_MARGIN, min(bbmaxL.z - COLUMN_AABB_Z_MARGIN, z))

        if axis_key == "x":
            p_query_local = Vector((centerL.x + sign * (0.6 * ext_x + 0.05), centerL.y, z))
            p_fallback_local = Vector((centerL.x + sign * (0.5 * ext_x), centerL.y, z))
        else:
            p_query_local = Vector((centerL.x, centerL.y + sign * (0.6 * ext_y + 0.05), z))
            p_fallback_local = Vector((centerL.x, centerL.y + sign * (0.5 * ext_y), z))

        try:
            near = bvh.find_nearest(p_query_local)
        except Exception:
            near = None

        if near and near[0] is not None:
            loc_local = near[0]
            loc_world = obj.matrix_world @ loc_local
            start = loc_world + d_world * START_EPS_AABB
        else:
            loc_world = obj.matrix_world @ p_fallback_local
            start = loc_world + d_world * START_EPS_AABB

        starts.append(start)

    return starts


def _column_aabb_sample_starts(obj, bvh, invM, cmin, cmax, center, d: Vector):
    # Returns multiple start points per direction (Z-sampling) for robust AABB rays on columns.
    ext = aabb_extent_along_dir(cmin, cmax, d)
    height = max(0.0, (cmax.z - cmin.z))
    zc = center.z

    offsets = [0.0]
    if COLUMN_AABB_Z_SAMPLES >= 2 and height > 1e-9:
        off = COLUMN_AABB_Z_FRAC * height
        offsets += [off, -off]

    starts = []
    for oz in offsets[:max(1, int(COLUMN_AABB_Z_SAMPLES))]:
        z = zc + oz
        if height > 1e-9:
            z = max(cmin.z + COLUMN_AABB_Z_MARGIN, min(cmax.z - COLUMN_AABB_Z_MARGIN, z))

        p_query_world = center + d * (0.6 * ext + 0.05)
        p_query_world = Vector((p_query_world.x, p_query_world.y, z))
        p_query_local = invM @ p_query_world

        try:
            near = bvh.find_nearest(p_query_local)
        except Exception:
            near = None

        if near and near[0] is not None:
            loc_locall = near[0]
            loc_world = obj.matrix_world @ loc_locall
            start = loc_world + d * START_EPS_AABB
        else:
            start = Vector((center.x, center.y, z)) + d * (0.5 * ext + START_EPS_AABB)

        starts.append(start)

    return starts

def aabb_classify_one_column(obj, bb_min, bb_max, deps, scene, col_wall_ignore_map, blockers):
    """IfcColumn AABB-Methode:
    - 4 Richtungen (±X/±Y in Welt-XY)
    - Startpunkte über nächstem Punkt auf Mesh (BVH.find_nearest), damit Kreis/IPE/HBA ok ist
    - Robustness: pro Richtung mehrere Z-Samples; Richtung zählt nur dann als Escape, wenn alle Samples escapen
    - External wenn irgendeine Richtung bis zur Modell-AABB entkommt (nach obiger Robustness-Regel)
    """
    eo = obj.evaluated_get(deps)
    me = eo.to_mesh()
    bvh = bvh_from_mesh(me)
    invM = obj.matrix_world.inverted()

    # Column AABB + Center
    try:
        cmin, cmax = aabb_cached(obj)
    except Exception:
        eo.to_mesh_clear()
        return "Internal"
    # Column local OBB params (object-local bound_box) + World axes
    try:
        bbminL, bbmaxL, centerL, hx, hy, hz = _column_local_bb_params(obj)
    except Exception:
        eo.to_mesh_clear()
        return "Internal"
    center = obj.matrix_world @ centerL
    ax, ay, az = _column_axes_world(obj)

    sides = [
        ("x", +1.0, ax),
        ("x", -1.0, -ax),
        ("y", +1.0, ay),
        ("y", -1.0, -ay),
    ]

    for axis_key, sign, d in sides:
        starts = _column_obb_sample_starts(obj, bvh, bbminL, bbmaxL, centerL, axis_key, sign, d)

        # Richtung gilt nur dann als "Escape", wenn ALLE Samples escapen.
        all_escape = True
        for start in starts:
            if not cast_outwards_until_exit_column(scene, deps, start, d, bb_min, bb_max, obj, col_wall_ignore_map, blockers):
                all_escape = False
                break

        if all_escape:
            eo.to_mesh_clear()
            return "External"

    eo.to_mesh_clear()
    return "Internal"


    center = (cmin + cmax) * 0.5

    # 4 relevante Seiten-Richtungen (Weltachsen in XY) -> AABB-Seiten X+/X-/Y+/Y-
    dirs = [
        Vector((1.0, 0.0, 0.0)),
        Vector((-1.0, 0.0, 0.0)),
        Vector((0.0, 1.0, 0.0)),
        Vector((0.0, -1.0, 0.0)),
    ]

    for d in dirs:
        # Query point außerhalb auf dieser Seite (AABB-basiert) -> nearest point on mesh
        ext = aabb_extent_along_dir(cmin, cmax, d)
        p_query_world = center + d * (0.6 * ext + 0.05)
        p_query_local = invM @ p_query_world

        try:
            near = bvh.find_nearest(p_query_local)
        except Exception:
            near = None

        if near and near[0] is not None:
            loc_locall = near[0]
            loc_world = obj.matrix_world @ loc_locall
            start = loc_world + d * START_EPS_AABB
        else:
            start = center + d * (0.5 * ext + START_EPS_AABB)

        if cast_outwards_until_exit_column(scene, deps, start, d, bb_min, bb_max, obj, col_wall_ignore_map, blockers):
            eo.to_mesh_clear()
            return "External"

    eo.to_mesh_clear()
    return "Internal"



# =========================================================
# IfcBeam (NEU): 4-Side Probes wie Column (Space + AABB)
# =========================================================
def _beam_local_bb_params(obj):
    # identisch zu _column_local_bb_params (nur Wrapper für Lesbarkeit)
    return _column_local_bb_params(obj)

def _beam_axes_world(obj):
    # identisch zu _column_axes_world (nur Wrapper)
    return _column_axes_world(obj)

def _beam_length_axis_key(hx, hy, hz):
    lx, ly, lz = (2.0*hx), (2.0*hy), (2.0*hz)
    if lx >= ly and lx >= lz:
        return "x"
    if ly >= lx and ly >= lz:
        return "y"
    return "z"

def _beam_cross_axis_keys(length_key: str):
    keys = ["x", "y", "z"]
    try:
        keys.remove(length_key)
    except Exception:
        pass
    # should be two keys
    return keys[:2]

def _beam_obb_sample_starts(obj, bvh, bbminL, bbmaxL, centerL,
                           axis_key: str, sign: float, d_world: Vector,
                           length_key: str,
                           len_samples_override=None):
    """Startpunkte für Beam-Rays basierend auf OBB (objektlokale bound_box + Objektachsen).
    axis_key: eine der Querschnittsachsen ('x','y','z'), sign: +1/-1 für jeweilige Seite.
    Sampling entlang Beam-Längsachse (length_key) -> analog zu Column-Z-Sampling.
    """
    ext = {
        "x": max(0.0, (bbmaxL.x - bbminL.x)),
        "y": max(0.0, (bbmaxL.y - bbminL.y)),
        "z": max(0.0, (bbmaxL.z - bbminL.z)),
    }

    # Samples entlang LENGTH axis (analog zu Column entlang local Z)
    used_samples = int(len_samples_override) if (len_samples_override is not None) else int(COLUMN_AABB_Z_SAMPLES)
    used_samples = max(1, used_samples)

    length = ext.get(length_key, 0.0)
    offsets = [0.0]
    if used_samples >= 2 and length > 1e-9:
        off = COLUMN_AABB_Z_FRAC * length
        offsets += [off, -off]

    starts = []
    for oL in offsets[:used_samples]:
        # base at center
        x = centerL.x
        y = centerL.y
        z = centerL.z

        # set length axis coordinate
        if length_key == "x":
            x = centerL.x + oL
            if length > 1e-9:
                x = max(bbminL.x + COLUMN_AABB_Z_MARGIN, min(bbmaxL.x - COLUMN_AABB_Z_MARGIN, x))
        elif length_key == "y":
            y = centerL.y + oL
            if length > 1e-9:
                y = max(bbminL.y + COLUMN_AABB_Z_MARGIN, min(bbmaxL.y - COLUMN_AABB_Z_MARGIN, y))
        else:
            z = centerL.z + oL
            if length > 1e-9:
                z = max(bbminL.z + COLUMN_AABB_Z_MARGIN, min(bbmaxL.z - COLUMN_AABB_Z_MARGIN, z))

        # offset to the queried face on axis_key
        if axis_key == "x":
            xq = centerL.x + sign * (0.6 * ext["x"] + 0.05)
            p_query_local = Vector((xq, y, z))
            p_fallback_local = Vector((centerL.x + sign * (0.5 * ext["x"]), y, z))
        elif axis_key == "y":
            yq = centerL.y + sign * (0.6 * ext["y"] + 0.05)
            p_query_local = Vector((x, yq, z))
            p_fallback_local = Vector((x, centerL.y + sign * (0.5 * ext["y"]), z))
        else:
            zq = centerL.z + sign * (0.6 * ext["z"] + 0.05)
            p_query_local = Vector((x, y, zq))
            p_fallback_local = Vector((x, y, centerL.z + sign * (0.5 * ext["z"])))

        try:
            near = bvh.find_nearest(p_query_local)
        except Exception:
            near = None

        if near and near[0] is not None:
            loc_local = near[0]
            loc_world = obj.matrix_world @ loc_local
            start = loc_world + d_world * START_EPS_AABB
        else:
            loc_world = obj.matrix_world @ p_fallback_local
            start = loc_world + d_world * START_EPS_AABB

        starts.append(start)

    return starts


def aabb_classify_one_beam(obj, bb_min, bb_max, deps, scene, col_wall_ignore_map, blockers, cw_blockers):
    """IfcBeam AABB-Methode (1:1 wie Column):
    - 4 Richtungen (± in den zwei Querschnittsachsen)
    - Startpunkte über nächstem Punkt auf Mesh (BVH.find_nearest)
    - Robustness: pro Richtung mehrere Samples entlang Beam-Längsachse; Richtung zählt nur dann als Escape, wenn alle Samples escapen
    - External wenn irgendeine Richtung bis zur Modell-AABB entkommt (nach obiger Robustness-Regel)
    """
    ent = _ifc_ent(obj)
    ifc_type = ent.is_a() if ent else ""
    if ifc_type != "IfcBeam":
        return "Internal"

    eo = obj.evaluated_get(deps)
    me = eo.to_mesh()
    bvh = bvh_from_mesh(me)

    try:
        bbminL, bbmaxL, centerL, hx, hy, hz = _beam_local_bb_params(obj)
    except Exception:
        eo.to_mesh_clear()
        return "Internal"

    ax, ay, az = _beam_axes_world(obj)
    length_key = _beam_length_axis_key(hx, hy, hz)
    cross_keys = _beam_cross_axis_keys(length_key)

    axis_map = {"x": ax, "y": ay, "z": az}
    sides = []
    for k in cross_keys:
        v = axis_map.get(k, Vector((1.0, 0.0, 0.0)))
        vv = v.normalized()
        sides.append((k, +1.0, vv))
        sides.append((k, -1.0, -vv))
    for axis_key, sign, d in sides:
        starts = _beam_obb_sample_starts(obj, bvh, bbminL, bbmaxL, centerL, axis_key, sign, d, length_key)

        all_escape = True
        for start in starts:
            if not cast_outwards_until_exit_beam(scene, deps, start, d, bb_min, bb_max, obj, col_wall_ignore_map, blockers, cw_blockers):
                all_escape = False
                break

        if all_escape:
            # Beam-Fix: rein vertikale Escapes (top/bottom leaks) NICHT als "External" werten.
            try:
                if abs(d.dot(Vector((0.0, 0.0, 1.0)))) >= 0.85:
                    continue
            except Exception:
                pass
            eo.to_mesh_clear()
            return "External"

    eo.to_mesh_clear()
    return "Internal"


def classify_beam_spaces_four_sides(obj, scene=None, deps=None, col_wall_ignore_map=None):
    """IfcBeam Space-Methode (1:1 wie Column):
    - 4 Seiten (± in den zwei Querschnittsachsen)
    - 'INTERNAL' wenn mind. 2 Seiten einen (internen) IfcSpace treffen, sonst 'EXTERNAL'
    - ignoriert Coverings + embedded walls (wenn in ignore_map für diesen Beam)
    """
    ent = _ifc_ent(obj)
    ifc_type = ent.is_a() if ent else ""
    if ifc_type != "IfcBeam":
        return "EXTERNAL", "not_a_beam", 0, 0

    if scene is None:
        scene = bpy.context.scene
    if deps is None:
        deps = bpy.context.evaluated_depsgraph_get()
    if col_wall_ignore_map is None:
        col_wall_ignore_map = {}

    try:
        bbminL, bbmaxL, centerL, hx, hy, hz = _beam_local_bb_params(obj)
    except Exception:
        return "EXTERNAL", "no_bound_box", 0, 0

    ax, ay, az = _beam_axes_world(obj)
    length_key = _beam_length_axis_key(hx, hy, hz)
    cross_keys = _beam_cross_axis_keys(length_key)
    axis_map = {"x": ax, "y": ay, "z": az}

    sides = []
    for k in cross_keys:
        v = axis_map.get(k, Vector((1.0, 0.0, 0.0)))
        vv = v.normalized()
        sides.append((k, +1.0, vv))
        sides.append((k, -1.0, -vv))

    eo = obj.evaluated_get(deps)
    me = eo.to_mesh()
    bvh = bvh_from_mesh(me)

    side_spaces = []
    for axis_key, sign, d in sides:
        starts = _beam_obb_sample_starts(obj, bvh, bbminL, bbmaxL, centerL, axis_key, sign, d, length_key, len_samples_override=1)
        if starts:
            start = starts[0] + d * START_EPS_AABB
        else:
            start = (obj.matrix_world @ centerL) + d * (START_EPS_AABB * 2.0)

        sp = probe_spaces_with_column_embed_skip(scene, deps, start, d, obj, col_wall_ignore_map, want_external=False)
        side_spaces.append(sp)

    eo.to_mesh_clear()

    sides_with = sum(1 for sp in side_spaces if len(sp) >= MIN_SPACES_PER_SIDE)
    uniq = set()
    for sp in side_spaces:
        uniq |= set(sp)

    try:
        obj[DEBUG_TOUCH_SPACES_PROP_A] = [f"S{idx+1}:" + "|".join(sorted(list(sp))) for idx, sp in enumerate(side_spaces)]
        obj[DEBUG_TOUCH_SPACES_PROP_B] = sorted(list(uniq))
    except Exception:
        pass

    cls = "INTERNAL" if sides_with >= 2 else "EXTERNAL"
    rule = f"beam_spaces_4(sides={sides_with}/4,min_sides=2,uniq={len(uniq)})"
    return cls, rule, sides_with, len(uniq)



def probe_spaces_with_column_embed_skip(scene, deps, start_pt: Vector, dirn: Vector,
                                        src_obj, col_wall_ignore_map, want_external: bool = False):
    """Space-Probe für Columns: nur Walls überspringen, in denen die Column steckt.
    want_external=True -> sucht in EXTERNAL_SPACES (Keyword-Spaces)
    """
    getter = external_spaces_at_point if want_external else spaces_at_point

    p = start_pt.copy()
    d = dirn.normalized()
    remaining = LAYER_SKIP_MAX_DIST
    steps = 0

    while steps <= LAYER_SKIP_MAX_STEPS and remaining > 0:
        sp = getter(p)
        if sp:
            return sp

        hit, loc, nor, idx, hit_obj, *_ = scene.ray_cast(deps, p, d, distance=remaining)
        if not hit:
            sp2 = getter(p + d * min(0.10, SEARCH_MAX))
            return sp2

        dist_hit = (loc - p).length

        if should_ignore_hit_for_column_space(hit_obj, src_obj, col_wall_ignore_map):
            adv_aabb = advance_past_obj_aabb(p, d, hit_obj)
            advance = adv_aabb if adv_aabb is not None else (dist_hit + STEP_EPS_AABB)
            advance = max(advance, dist_hit + STEP_EPS_AABB)
            p = p + d * advance
            remaining -= advance
            steps += 1
            continue

        return set()

    return set()



# =========================================================
# Space Layer-Skip Probe (NEU) ✅✅✅
# =========================================================
def probe_spaces_with_layer_skip(scene, deps, start_pt: Vector, dirn: Vector,
                                 src_obj, ignore_map, want_external: bool = False):
    """
    Findet Spaces auch dann, wenn direkt vor dem Start eine Bekleidung / dünne Schale liegt.
    want_external=True -> sucht in EXTERNAL_SPACES (Keyword-Spaces)
    """
    getter = external_spaces_at_point if want_external else spaces_at_point

    if not ENABLE_LAYER_STACK:
        return getter(start_pt)

    p = start_pt.copy()
    d = dirn.normalized()
    remaining = LAYER_SKIP_MAX_DIST
    steps = 0

    while steps <= LAYER_SKIP_MAX_STEPS and remaining > 0:
        sp = getter(p)
        if sp:
            return sp

        hit, loc, nor, idx, hit_obj, *_ = scene.ray_cast(deps, p, d, distance=remaining)
        if not hit:
            sp2 = getter(p + d * min(0.10, SEARCH_MAX))
            return sp2

        dist_hit = (loc - p).length

        # ---- NEU: Slab-Stack Skip (geklebt unten) ----
        if is_glued_slab_below_obj(hit_obj, src_obj, d):
            adv_aabb = advance_past_obj_aabb(p, d, hit_obj)
            advance = adv_aabb if adv_aabb is not None else (dist_hit + STEP_EPS_AABB)
            advance = max(advance, dist_hit + STEP_EPS_AABB)
            p = p + d * advance
            remaining -= advance
            steps += 1
            continue

        if should_ignore_hit_for_wall(hit_obj, src_obj, ignore_map):
            advance = dist_hit + STEP_EPS_AABB
            p = p + d * advance
            remaining -= advance
            steps += 1
            continue

        if is_skippable_layer_obj(hit_obj, d, dist_hit):
            adv_aabb = advance_past_obj_aabb(p, d, hit_obj)
            advance = adv_aabb if adv_aabb is not None else (dist_hit + STEP_EPS_AABB)
            advance = max(advance, dist_hit + STEP_EPS_AABB)
            p = p + d * advance
            remaining -= advance
            steps += 1
            continue

        return set()

    return set()



def collect_spaces_on_faces(me, faces, height_window=None,
                            scene=None, deps=None, src_obj=None, ignore_map=None,
                            want_external: bool = False,
                            align_z=None,
                            force_dir=None,
                            tri_sampling: bool = False):
    found = set()
    if not faces:
        return found

    getter = external_spaces_at_point if want_external else spaces_at_point

    z_lo, z_hi = None, None
    if height_window:
        z_lo, z_hi = height_window

    for p in faces:
        seed_tag = (src_obj.name if src_obj else "")
        if tri_sampling:
            samples = sample_poly_tri_points(me, p, GRID_U, GRID_V, seed_tag)
        else:
            samples = sample_grid(me, p, GRID_U, GRID_V)

        for base, t1, t2, n in samples:
            # Richtung festlegen:
            # - force_dir: für Slabs echte Top/Bottom-Achse erzwingen (unabhängig von Face-Normalen)
            # - align_z: legacy (global Z) für frühere Slab-Variante
            if force_dir is not None:
                n_use = Vector(force_dir)
                if n_use.length > 1e-12:
                    n_use.normalize()
                else:
                    n_use = Vector(n)
            else:
                n_use = Vector(n)
                # Für Slabs/TopBottom: Normalen-Richtung robust machen (z.B. bei flipped winding)
                if align_z == +1 and n_use.z < 0.0:
                    n_use = -n_use
                elif align_z == -1 and n_use.z > 0.0:
                    n_use = -n_use

            for off in jitter_offsets(t1, t2):
                for d in depths():
                    pt = base + off + n_use*d
                    if height_window and not (z_lo <= pt.z <= z_hi):
                        continue

                    hit = getter(pt)

                    # ---- Layer-Stack Fallback, wenn Space nicht erreichbar wegen Bekleidung ----
                    if (not hit) and ENABLE_LAYER_STACK and scene and deps and src_obj and (ignore_map is not None):
                        start_pt = base + off + n_use * EPS
                        hit = probe_spaces_with_layer_skip(scene, deps, start_pt, n_use, src_obj, ignore_map, want_external=want_external)

                    if hit:
                        found |= hit
                        break
    return found



def split_wall_faces_into_sides(me, polys):
    polys = list(polys)
    polys.sort(key=lambda p: p.area, reverse=True)
    nA = polys[0].normal.normalized()
    A = [p for p in polys if p.normal.normalized().dot(nA) >= +DOT_STRICT]
    B = [p for p in polys if p.normal.normalized().dot(nA) <= -DOT_STRICT]
    if not A or not B:
        A = [p for p in polys if p.normal.normalized().dot(nA) >= +0.3]
        B = [p for p in polys if p.normal.normalized().dot(nA) <= -0.3]
    if not A:
        A = [polys[0]]
    if not B:
        best = None
        bestdot = +1.0
        for p in polys[1:]:
            d = p.normal.normalized().dot(nA)
            if d < bestdot:
                best, bestdot = p, d
        B = [best] if best else [polys[-1]]
    return A, B



def slab_dominant_axis_from_polys(me, polys):
    """Dominante Slab-Oberflächenachse (sign-invariant) aus Face-Normalen bestimmen.
    - Hemisphären-Clustering über flächen-gewichtete Normalen (|n|=1)
    - Ergebnis wird so orientiert, dass dot(axis, world_up) >= 0 ist.
    Rückgabe: Vector oder None
    """
    polys = list(polys) if polys else []
    if not polys:
        return None

    clusters = []  # list of dict: {'dir':Vector, 'w':float}
    for p in polys:
        a = float(p.area)
        if a <= 1e-12:
            continue
        n = Vector(p.normal)
        ln = n.length
        if ln <= 1e-12:
            continue
        n = n / ln

        # hemisphere normalize: make sign-invariant (choose side by world_up)
        if n.z < 0.0:
            n = -n

        placed = False
        for c in clusters:
            if c['dir'].dot(n) >= SLAB_AXIS_CLUSTER_DOT:
                # weighted accumulate then renormalize
                c['dir'] = (c['dir'] * c['w'] + n * a)
                c['w'] += a
                if c['dir'].length > 1e-12:
                    c['dir'].normalize()
                placed = True
                break
        if not placed:
            clusters.append({'dir': n.copy(), 'w': a})

    if not clusters:
        return None

    clusters.sort(key=lambda c: c['w'], reverse=True)
    axis = clusters[0]['dir'].copy()
    if axis.length <= 1e-12:
        return None
    axis.normalize()

    # ensure axis points "up-ish" (helps interpret TOP as outside for roofs)
    if axis.dot(Vector((0.0, 0.0, 1.0))) < 0.0:
        axis.negate()
    return axis

def split_slab_faces_top_bottom(me, polys, axis=None):
    """Split für IfcSlab: echte Top/Bottom-Oberflächen ermitteln.
    Wichtig bei geneigten Dachflächen:
    - NICHT nach globalem Z clustern (führt bei starker Neigung zu Top/Bottom-Mix)
    - Stattdessen: dominante Oberflächenachse aus Normalen bestimmen und Extremflächen entlang dieser Achse wählen.
    Fallback: alte Z-KMeans-Logik, falls Achse nicht zuverlässig bestimmbar.
    """
    polys = list(polys)
    if not polys:
        return [], []

    # --- Achse bestimmen (sign-invariant) ---
    if axis is None:
        axis = slab_dominant_axis_from_polys(me, polys)

    # --- Achsen-basierte Oberflächen-Selektion ---
    if axis is not None and axis.length > 1e-12:
        axis = axis.normalized()

        # Nur Faces, deren Normalen stark mit Achse übereinstimmen (Top/Bottom-Flächen)
        aligned = [p for p in polys if abs(Vector(p.normal).dot(axis)) >= SLAB_SURFACE_AXIS_DOT_STRICT]
        if not aligned:
            aligned = [p for p in polys if abs(Vector(p.normal).dot(axis)) >= SLAB_SURFACE_AXIS_DOT_LOOSE]

        if aligned:
            ss = [(p, face_center(me, p).dot(axis)) for p in aligned]
            smin = min(s for _p, s in ss)
            smax = max(s for _p, s in ss)
            thick = (smax - smin)
            if thick > 1e-6:
                eps = max(SLAB_SURFACE_EPS_MIN, thick * SLAB_SURFACE_EPS_FRAC)
                top = [p for p, s in ss if s >= (smax - eps)]
                bottom = [p for p, s in ss if s <= (smin + eps)]
                if top and bottom:
                    return top, bottom

    # --- Fallback: alte Z-basierte 1D k-means (k=2) auf center.z ---
    zs = [face_center(me, p).z for p in polys]
    zmin = min(zs)
    zmax = max(zs)
    if (zmax - zmin) < 1e-6:
        # Degenerat: fallback auf Normal-Split
        return split_wall_faces_into_sides(me, polys)

    c1, c2 = zmin, zmax
    g1, g2 = [], []
    for _ in range(8):
        g1, g2 = [], []
        for p, z in zip(polys, zs):
            if abs(z - c1) <= abs(z - c2):
                g1.append((p, z))
            else:
                g2.append((p, z))
        if g1:
            c1 = sum(z for _, z in g1) / float(len(g1))
        if g2:
            c2 = sum(z for _, z in g2) / float(len(g2))

    if c1 >= c2:
        top = [p for p, _z in g1]
        bottom = [p for p, _z in g2]
    else:
        top = [p for p, _z in g2]
        bottom = [p for p, _z in g1]

    if not top or not bottom:
        return split_wall_faces_into_sides(me, polys)

    return top, bottom


def classify_wall_spaces_both_sides(obj, scene=None, deps=None, ignore_map=None):
    me = mesh_copy_world(obj)

    zs = [v.co.z for v in me.vertices]
    z_min = min(zs)
    z_max = max(zs)

    height = max(z_max - z_min, 0.0)
    if height > 0.0:
        h_lo = z_min + height * HEIGHT_FRAC_LOW
        h_hi = z_min + height * HEIGHT_FRAC_HIGH
        hwin_default = (h_lo, h_hi)
    else:
        hwin_default = None

    ent = _ifc_ent(obj)
    ifc_type = ent.is_a() if ent else ""

    # =====================================================
    # Face-Auswahl: Wände bleiben unverändert
    # Coverings: abhängig von PredefinedType vertikal/horizontal
    # =====================================================
    hwin = hwin_default

    # =====================================================
    # Face-Auswahl:
    # - Wände: wie bisher (vertikale Faces)
    # - IfcCovering: abhängig von PredefinedType vertikal/horizontal
    # - IfcSlab: 1:1 wie Wände, aber horizontal (Top/Bottom) statt Seiten
    # =====================================================

    if ifc_type == "IfcCovering":
        ptype = get_covering_predefined_type(obj)

        # horizontale / geneigte Coverings: Top/Bottom (oder geneigt) statt Seiten
        if ptype in COVERING_PTYPE_HORIZONTAL:
            polys_all = [p for p in me.polygons if abs(p.normal.z) >= COVERING_HOR_FACE_Z_ABS_THR]
            hwin = None  # bei Top/Bottom nicht mit Höhenfenster wegfiltern

            # Fallback: falls das Modell sehr schräg/unsauber ist
            if not polys_all:
                polys_all = [p for p in me.polygons if abs(p.normal.z) >= 0.15]

        else:
            # vertikale Coverings (CLADDING/INSULATION + Default): wie bisher über vertikale Faces
            polys_all = [p for p in me.polygons if abs(p.normal.z) < Z_THR_WALL]

    elif ifc_type == "IfcSlab":
        # IfcSlab: horizontale Faces (Top/Bottom) statt Seiten
        polys_all = [p for p in me.polygons if abs(p.normal.z) >= SLAB_HOR_FACE_Z_ABS_THR]
        hwin = None  # Slabs sind dünn -> Höhenfenster würde alle Samples wegfiltern

        # Fallback: bei geneigten/unsauberen Slabs
        if not polys_all:
            polys_all = [p for p in me.polygons if abs(p.normal.z) >= 0.15]

    else:
        # Wände / WallStandardCase: exakt wie bisher
        polys_all = [p for p in me.polygons if abs(p.normal.z) < Z_THR_WALL]

    if not polys_all:
        bpy.data.meshes.remove(me, do_unlink=True)
        if (ifc_type == "IfcCovering" and get_covering_predefined_type(obj) in COVERING_PTYPE_HORIZONTAL) or (ifc_type == "IfcSlab"):
            return "EXTERNAL", "no_horizontal_faces", 0, 0
        return "EXTERNAL", "no_vertical_faces", 0, 0

    if ifc_type == "IfcSlab":
        # IfcSlab: Top/Bottom robust nach Lage splitten (nicht nach Normalenrichtung)
        slab_axis_tmp = slab_dominant_axis_from_polys(me, polys_all)
        A_faces, B_faces = split_slab_faces_top_bottom(me, polys_all, axis=slab_axis_tmp)
    else:
        A_faces, B_faces = split_wall_faces_into_sides(me, polys_all)
    A_faces = filter_major_faces(A_faces)
    B_faces = filter_major_faces(B_faces)

    sideA = set()
    sideB = set()


    # Covering (horizontal): nur "Raum-Seite" prüfen (Fallback auf Gegenseite)
    if ifc_type == "IfcCovering":
        ptype_fast = get_covering_predefined_type(obj)
        if ptype_fast in COVERING_PTYPE_HORIZONTAL:
            def _avg_z(_faces):
                if not _faces:
                    return 0.0
                return sum(p.normal.z for p in _faces) / float(len(_faces))

            up_is_A = _avg_z(A_faces) >= _avg_z(B_faces)
            prefer_up = ptype_fast in {"FLOORING", "TOPPING"}
            prefer_down = ptype_fast in {"CEILING", "ROOFING"}
            if prefer_up:
                preferred_is_A = up_is_A
            elif prefer_down:
                preferred_is_A = (not up_is_A)
            else:
                preferred_is_A = up_is_A

            if preferred_is_A:
                sideA = collect_spaces_on_faces(me, A_faces, height_window=hwin,
                                                scene=scene, deps=deps, src_obj=obj, ignore_map=ignore_map)
                if len(sideA) < MIN_SPACES_PER_SIDE:
                    sideB = collect_spaces_on_faces(me, B_faces, height_window=hwin,
                                                    scene=scene, deps=deps, src_obj=obj, ignore_map=ignore_map)
            else:
                sideB = collect_spaces_on_faces(me, B_faces, height_window=hwin,
                                                scene=scene, deps=deps, src_obj=obj, ignore_map=ignore_map)
                if len(sideB) < MIN_SPACES_PER_SIDE:
                    sideA = collect_spaces_on_faces(me, A_faces, height_window=hwin,
                                                    scene=scene, deps=deps, src_obj=obj, ignore_map=ignore_map)
        else:
            sideA = collect_spaces_on_faces(me, A_faces, height_window=hwin,
                                            scene=scene, deps=deps, src_obj=obj, ignore_map=ignore_map)
            sideB = collect_spaces_on_faces(me, B_faces, height_window=hwin,
                                            scene=scene, deps=deps, src_obj=obj, ignore_map=ignore_map)
    else:
        if ifc_type == "IfcSlab":
            # IfcSlab: echte Top/Bottom-Achse erzwingen (auch bei geneigten Dachflächen)
            slab_axis = slab_dominant_axis_from_polys(me, polys_all)
            if slab_axis is None:
                slab_axis = Vector((0.0, 0.0, 1.0))
            sideA = collect_spaces_on_faces(me, A_faces, height_window=hwin,
                                            scene=scene, deps=deps, src_obj=obj, ignore_map=ignore_map,
                                            force_dir=slab_axis, tri_sampling=True)
            sideB = collect_spaces_on_faces(me, B_faces, height_window=hwin,
                                            scene=scene, deps=deps, src_obj=obj, ignore_map=ignore_map,
                                            force_dir=-slab_axis, tri_sampling=True)
        else:
            sideA = collect_spaces_on_faces(me, A_faces, height_window=hwin,
                                            scene=scene, deps=deps, src_obj=obj, ignore_map=ignore_map)
            sideB = collect_spaces_on_faces(me, B_faces, height_window=hwin,
                                            scene=scene, deps=deps, src_obj=obj, ignore_map=ignore_map)


    # ---- DEBUG: Touch-Spaces pro Element speichern ----
    try:
        obj[DEBUG_TOUCH_SPACES_PROP_A] = sorted(list(sideA))
        obj[DEBUG_TOUCH_SPACES_PROP_B] = sorted(list(sideB))
    except Exception:
        pass

    bpy.data.meshes.remove(me, do_unlink=True)

    countA, countB = len(sideA), len(sideB)

    if ifc_type == "IfcCovering":
        total = countA + countB
        cls = "INTERNAL" if total >= MIN_SPACES_PER_SIDE else "EXTERNAL"

        ptype = get_covering_predefined_type(obj)
        if ptype in COVERING_PTYPE_HORIZONTAL:
            rule = f"covering_h(A={countA},B={countB},min_any={MIN_SPACES_PER_SIDE},ptype={ptype or 'NA'})"
        else:
            # vertikal: Rule-String wie vorher lassen (damit nichts im Workflow bricht)
            rule = f"covering(A={countA},B={countB},min_any={MIN_SPACES_PER_SIDE})"
    else:
        cls = "INTERNAL" if (countA >= MIN_SPACES_PER_SIDE and countB >= MIN_SPACES_PER_SIDE) else "EXTERNAL"
        rule = f"spaces_lr(A={countA},B={countB},min={MIN_SPACES_PER_SIDE})"

    return cls, rule, countA, countB


def classify_column_spaces_four_sides(obj, scene=None, deps=None, col_wall_ignore_map=None):
    """IfcColumn Space-Methode:
    - 4 Seiten (±X/±Y in Objekt-XY)
    - 'INTERNAL' wenn mind. 2 Seiten einen (internen) IfcSpace treffen, sonst 'EXTERNAL'
    - ignoriert Walls, in denen die Column steckt (Column-in-Wall Modellierung)
    """
    ent = _ifc_ent(obj)
    ifc_type = ent.is_a() if ent else ""
    if ifc_type != "IfcColumn":
        return "EXTERNAL", "not_a_column", 0, 0

    if scene is None:
        scene = bpy.context.scene
    if deps is None:
        deps = bpy.context.evaluated_depsgraph_get()
    if col_wall_ignore_map is None:
        col_wall_ignore_map = {}

    # Column AABB + Center
    try:
        cmin, cmax = aabb_cached(obj)
    except Exception:
        return "EXTERNAL", "no_aabb", 0, 0
    center = (cmin + cmax) * 0.5

    # Column local OBB params (object-local bound_box) + World axes
    try:
        bbminL, bbmaxL, centerL, hx, hy, hz = _column_local_bb_params(obj)
    except Exception:
        return "EXTERNAL", "no_bound_box", 0, 0
    center = obj.matrix_world @ centerL
    ax, ay, az = _column_axes_world(obj)

    sides = [
        ("x", +1.0, ax),
        ("x", -1.0, -ax),
        ("y", +1.0, ay),
        ("y", -1.0, -ay),
    ]

    eo = obj.evaluated_get(deps)
    me = eo.to_mesh()
    bvh = bvh_from_mesh(me)
    invM = obj.matrix_world.inverted()

    side_spaces = []
    for si, (axis_key, sign, d) in enumerate(sides):
        starts = _column_obb_sample_starts(obj, bvh, bbminL, bbmaxL, centerL, axis_key, sign, d, z_samples_override=1)
        if starts:
            start = starts[0] + d * START_EPS_AABB
        else:
            start = center + d * (START_EPS_AABB * 2.0)

        sp = probe_spaces_with_column_embed_skip(scene, deps, start, d, obj, col_wall_ignore_map, want_external=False)
        side_spaces.append(sp)

    eo.to_mesh_clear()

    # Count sides that see at least one INTERNAL space
    sides_with = sum(1 for sp in side_spaces if len(sp) >= MIN_SPACES_PER_SIDE)
    uniq = set()
    for sp in side_spaces:
        uniq |= set(sp)

    # DEBUG: Touch-Spaces pro Side speichern (als Strings)
    try:
        obj[DEBUG_TOUCH_SPACES_PROP_A] = [f"S{idx+1}:" + "|".join(sorted(list(sp))) for idx, sp in enumerate(side_spaces)]
        obj[DEBUG_TOUCH_SPACES_PROP_B] = sorted(list(uniq))
    except Exception:
        pass

    cls = "INTERNAL" if sides_with >= 2 else "EXTERNAL"
    rule = f"col_spaces_4(sides={sides_with}/4,min_sides=2,uniq={len(uniq)})"
    return cls, rule, sides_with, len(uniq)


# =========================================================
# Runner - getrennte Runs für BOTH
# =========================================================
def _space_pass(elems, scene, deps, ignore_map):
    build_spaces_bvh_or_die()

    # Embedded-Wall Ignore (nur für Columns)
    try:
        all_walls = [w for w in scene_meshes() if _is_ifcclass(w, "IfcWall", "IfcWallStandardCase")]
    except Exception:
        all_walls = []
    cols = [o for o in elems if _is_ifcclass(o, "IfcColumn")]
    col_wall_ignore_map = build_column_embedded_wall_ignore_map(cols, all_walls) if cols else {}
    beams = [o for o in elems if _is_ifcclass(o, "IfcBeam")]
    beam_wall_ignore_map = build_column_embedded_wall_ignore_map(beams, all_walls) if beams else {}

    # DEBUG: alte Touch-Spaces löschen
    for _o in elems:
        try:
            _o.pop(DEBUG_TOUCH_SPACES_PROP_A, None)
            _o.pop(DEBUG_TOUCH_SPACES_PROP_B, None)
        except Exception:
            pass

    out = {}
    for o in elems:
        if _is_ifcclass(o, "IfcColumn"):
            cls_space_raw, rule_space, nA, nB = classify_column_spaces_four_sides(
                o, scene=scene, deps=deps, col_wall_ignore_map=col_wall_ignore_map
            )
        elif _is_ifcclass(o, "IfcBeam"):
            cls_space_raw, rule_space, nA, nB = classify_beam_spaces_four_sides(
                o, scene=scene, deps=deps, col_wall_ignore_map=beam_wall_ignore_map
            )
        else:
            cls_space_raw, rule_space, nA, nB = classify_wall_spaces_both_sides(
                o, scene=scene, deps=deps, ignore_map=ignore_map
            )

        class_space = "Internal" if cls_space_raw == "INTERNAL" else "External"
        out[o.name] = (class_space, nA, nB, rule_space)
    return out

def _aabb_pass_improved(elems_all, elems, deps, scene):
    # AABB aus ALLEN IFC-Meshes in der Szene (nicht nur Auswahl),
    # damit Column-Rays nicht an einer zu "tighten" AABB (nur Stütze) kaputtgehen.
    try:
        _all = [o for o in scene_meshes() if not o.name.startswith(DEBUG_AABB_RAYS_PREFIX)]
        bb_min, bb_max = aabb_of_objects(_all) if _all else aabb_of_objects(elems_all)
    except Exception:
        bb_min, bb_max = aabb_of_objects(elems_all)

    
    # Column-Fix: Für start_in_space detection (DW-Blocker gating) brauchen wir IfcSpace-BVHs auch im AABB-Pass.
    # Wenn keine Spaces vorhanden sind, läuft AABB trotzdem weiter (dann ohne start_in_space gating).
    if COLUMNS_DW_BLOCKERS_ONLY_IF_START_IN_INTERNAL_SPACE:
        try:
            build_spaces_bvh_or_die()
        except BaseException:
            pass

    # Türen/Fenster/Openings nur ignorieren, wenn sie zur SAME wall gehören
    ignore_map = build_wall_local_ignore_map(elems_all)

    # Door/Window AABB Blocker einmal bauen (aus sichtbaren Meshes)
    blockers = build_blocker_aabbs(scene_meshes()) if USE_DOOR_WINDOW_AABB_BLOCKERS else []
    cw_blockers = build_curtainwall_aabbs(scene_meshes()) if USE_CURTAINWALL_AABB_BLOCKERS_FOR_AABB else []
    global _IE_CW_AABB_BLOCKERS
    _IE_CW_AABB_BLOCKERS = cw_blockers

    # Embedded-Wall Ignore (nur für Columns)
    try:
        all_walls = [w for w in scene_meshes() if _is_ifcclass(w, "IfcWall", "IfcWallStandardCase")]
    except Exception:
        all_walls = []
    cols_all = []
    try:
        cols_all = [o for o in (list(elems_all) + list(elems)) if _is_ifcclass(o, "IfcColumn")]
    except Exception:
        cols_all = [o for o in elems if _is_ifcclass(o, "IfcColumn")]
    # unique by name
    _seen = set()
    cols_all_u = []
    for c in cols_all:
        if c.name in _seen:
            continue
        _seen.add(c.name)
        cols_all_u.append(c)
    col_wall_ignore_map = build_column_embedded_wall_ignore_map(cols_all_u, all_walls) if cols_all_u else {}

    out = {}
    ext_cnt = 0
    for o in elems:
        if _is_ifcclass(o, "IfcColumn"):
            class_aabb = aabb_classify_one_column(o, bb_min, bb_max, deps, scene, col_wall_ignore_map, blockers)
        elif _is_ifcclass(o, "IfcBeam"):
            class_aabb = aabb_classify_one_beam(o, bb_min, bb_max, deps, scene, col_wall_ignore_map, blockers, cw_blockers)
        else:
            class_aabb = aabb_classify_one_object_improved(o, bb_min, bb_max, deps, scene, ignore_map, blockers)

        out[o.name] = class_aabb
        if class_aabb == "External":
            ext_cnt += 1
    return out, ext_cnt

def run_from_ui(method: str, target_storeys: list[str], allowed_types: set):
    scene = bpy.context.scene
    deps  = bpy.context.evaluated_depsgraph_get()

    all_mesh = scene_meshes()
    ifc_id_to_obj = _build_ifc_id_obj_map(all_mesh)

    # UI-Auswahl -> alle Kandidaten (ohne Storey-Filter)
    elems_all_sel = [o for o in all_mesh if is_selected_ifc_type(o, allowed_types)]
    if not elems_all_sel:
        return []

    # Storey-Filter auf die UI-Auswahl
    elems_sel = [o for o in elems_all_sel if is_in_target_storeys(o, target_storeys)]
    if not elems_sel:
        return []

    # Doors/Windows: Klassifikation wird vom Host-Wall übernommen (Host kann auch dann nötig sein, wenn IfcWall nicht angehakt ist)
    doorwin_sel = [o for o in elems_sel if _is_ifcclass(o, "IfcDoor", "IfcWindow")]
    host_walls = []
    host_wall_by_name = {}
    seen_hw = set()
    for o in doorwin_sel:
        hw = _host_wall_obj_for_door_window(o, ifc_id_to_obj)
        host_wall_by_name[o.name] = hw
        if hw and hw.name not in seen_hw:
            host_walls.append(hw)
            seen_hw.add(hw.name)

    # Support-Set für Ignore-Maps etc. (Host-Walls ergänzen, aber NICHT exportieren wenn nicht ausgewählt)
    elems_all_support = list(elems_all_sel)
    sel_names = {o.name for o in elems_all_support}
    for hw in host_walls:
        if hw.name not in sel_names:
            elems_all_support.append(hw)
            sel_names.add(hw.name)

    # Ziele, die wirklich klassifiziert werden (Doors/Windows selbst NICHT; stattdessen deren Host-Walls)
    classify_targets = []
    seen = set()
    for o in elems_sel:
        if _is_ifcclass(o, "IfcDoor", "IfcWindow"):
            continue
        if o.name not in seen:
            classify_targets.append(o)
            seen.add(o.name)
    for hw in host_walls:
        if hw.name not in seen:
            classify_targets.append(hw)
            seen.add(hw.name)

    ignore_map = build_wall_local_ignore_map(elems_all_support)

    results = []

    if method == "SPACE":
        space_map = _space_pass(classify_targets, scene, deps, ignore_map)

        for o in elems_sel:
            if _is_ifcclass(o, "IfcDoor", "IfcWindow"):
                hw = host_wall_by_name.get(o.name)
                if hw:
                    class_space, spacesA, spacesB, rule_space = space_map.get(hw.name, ("N/A", 0, 0, "no_host_wall_result"))
                    rule_space = f"HOST:{hw.name} | {rule_space}"
                else:
                    class_space, spacesA, spacesB, rule_space = "N/A", 0, 0, "no_host_wall"
            else:
                class_space, spacesA, spacesB, rule_space = space_map.get(o.name, ("N/A", 0, 0, "missing"))

            class_aabb = "N/A"
            review = ""

            step_id, guid, ifc_type_ifc = get_ifc_guid_and_type(o)
            ifc_type_name, role = ifc_type_and_role(o)
            st_name = storey_name(o)

            results.append((
                o.name, step_id, guid,
                ifc_type_ifc or ifc_type_name,
                role, st_name,
                class_space, class_aabb, review,
                spacesA, spacesB, rule_space
            ))
        return results

    if method == "AABB":
        aabb_map, _ext_cnt = _aabb_pass_improved(elems_all_support, classify_targets, deps, scene)

        ext_cnt_sel = 0
        for o in elems_sel:
            class_space, rule_space, spacesA, spacesB = "N/A", "skipped", 0, 0

            if _is_ifcclass(o, "IfcDoor", "IfcWindow"):
                hw = host_wall_by_name.get(o.name)
                class_aabb = aabb_map.get(hw.name, "N/A") if hw else "N/A"
            else:
                class_aabb = aabb_map.get(o.name, "N/A")

            if class_aabb == "External":
                ext_cnt_sel += 1

            review = ""

            step_id, guid, ifc_type_ifc = get_ifc_guid_and_type(o)
            ifc_type_name, role = ifc_type_and_role(o)
            st_name = storey_name(o)

            results.append((
                o.name, step_id, guid,
                ifc_type_ifc or ifc_type_name,
                role, st_name,
                class_space, class_aabb, review,
                spacesA, spacesB, rule_space
            ))

        print(f"AABB-External: {ext_cnt_sel} / {len(elems_sel)} (Storey-Filter)")
        return results

    # BOTH
    space_map = _space_pass(classify_targets, scene, deps, ignore_map)
    aabb_map, _ext_cnt = _aabb_pass_improved(elems_all_support, classify_targets, deps, scene)

    ext_cnt_sel = 0
    for o in elems_sel:
        if _is_ifcclass(o, "IfcDoor", "IfcWindow"):
            hw = host_wall_by_name.get(o.name)
            if hw:
                class_space, spacesA, spacesB, rule_space = space_map.get(hw.name, ("N/A", 0, 0, "no_host_wall_result"))
                rule_space = f"HOST:{hw.name} | {rule_space}"
                class_aabb = aabb_map.get(hw.name, "N/A")
            else:
                class_space, spacesA, spacesB, rule_space = "N/A", 0, 0, "no_host_wall"
                class_aabb = "N/A"
        else:
            class_space, spacesA, spacesB, rule_space = space_map.get(o.name, ("N/A", 0, 0, "missing"))
            class_aabb = aabb_map.get(o.name, "N/A")

        if class_aabb == "External":
            ext_cnt_sel += 1

        if class_space == "N/A" or class_aabb == "N/A":
            review = "CHECK"
        else:
            review = "OK" if class_space == class_aabb else "CHECK"

        step_id, guid, ifc_type_ifc = get_ifc_guid_and_type(o)
        ifc_type_name, role = ifc_type_and_role(o)
        st_name = storey_name(o)

        results.append((
            o.name, step_id, guid,
            ifc_type_ifc or ifc_type_name,
            role, st_name,
            class_space, class_aabb, review,
            spacesA, spacesB, rule_space
        ))

    print(f"AABB-External: {ext_cnt_sel} / {len(elems_sel)} (Storey-Filter)")
    return results

# =========================================================
from bpy.types import Panel, Operator, PropertyGroup
from bpy.props import EnumProperty, BoolProperty, StringProperty, PointerProperty

class IFCIE_Settings(PropertyGroup):
    method: EnumProperty(
        name="Method",
        items=[
            ("AABB", "AABB-Escape", ""),
            ("SPACE", "IfcSpace - BVH", ""),
            ("BOTH", "Both", ""),
        ],
        default="BOTH",
    )

    use_ifcwall: BoolProperty(name="IfcWall", default=True)
    use_ifcwallstandardcase: BoolProperty(name="IfcWallStandardCase", default=False)
    use_ifccovering: BoolProperty(name="IfcCovering", default=False)
    use_ifcslab: BoolProperty(name="IfcSlab", default=False)
    use_ifccolumn: BoolProperty(name="IfcColumn", default=False)
    use_ifcbeam: BoolProperty(name="IfcBeam", default=False)
    use_ifcdoor: BoolProperty(name="IfcDoor", default=False)
    use_ifcwindow: BoolProperty(name="IfcWindow", default=False)

    enable_layer_stack: BoolProperty(name="Layer-Skip (Coverings)", default=True)

    use_storey_filter: BoolProperty(name="Storey-Filter", default=True)
    storeys: EnumProperty(name="Storeys", items=_storey_items, options={'ENUM_FLAG'})

    out_xml: StringProperty(
        name="XML",
        subtype="FILE_PATH",
        default="//elements_space_vs_aabb_UI.xml"
    )


# =========================================================
# Excel XML (SpreadsheetML) Writer mit Farben OK/Check
# =========================================================

def _xml_escape(s: str) -> str:
    if s is None:
        return ""
    s = str(s)
    return (s.replace("&", "&amp;")
             .replace("<", "&lt;")
             .replace(">", "&gt;")
             .replace('"', "&quot;")
             .replace("'", "&apos;"))


def write_excel_xml(res, out_path):
    """Write results to Excel 2003 XML (SpreadsheetML)."""

    res_sorted = sorted(
        res,
        key=lambda r: (
            TYPE_PRIORITY.get(r[3], 99),
            (r[4] or "").lower(),
            0 if r[7] == "External" else 1,
            (r[0] or "").lower(),
        )
    )

    out_xml = bpy.path.abspath(out_path) if out_path else bpy.path.abspath("//elements_space_vs_aabb_UI.xml")
    out_dir = os.path.dirname(out_xml)
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir, exist_ok=True)

    headers = [
        "Object","IfcDefinitionId","GlobalId","IfcType","Role","Storey",
        "Class_Space","Class_AABB","Review","SpacesA","SpacesB","Rule_Space"
    ]

    xml = []
    xml.append('<?xml version="1.0"?>')
    xml.append('<Workbook xmlns="urn:schemas-microsoft-com:office:spreadsheet" '
               'xmlns:o="urn:schemas-microsoft-com:office:office" '
               'xmlns:x="urn:schemas-microsoft-com:office:excel" '
               'xmlns:ss="urn:schemas-microsoft-com:office:spreadsheet" '
               'xmlns:html="http://www.w3.org/TR/REC-html40">')

    xml.append('<Styles>')
    xml.append('<Style ss:ID="sHeader">'
               '<Font ss:Bold="1"/>'
               '<Interior ss:Color="#E6E6E6" ss:Pattern="Solid"/>'
               '<Borders><Border ss:Position="Bottom" ss:LineStyle="Continuous" ss:Weight="1"/></Borders>'
               '</Style>')

    xml.append('<Style ss:ID="sOK"><Interior ss:Color="#C6EFCE" ss:Pattern="Solid"/></Style>')
    xml.append('<Style ss:ID="sCHECK"><Interior ss:Color="#FFC7CE" ss:Pattern="Solid"/></Style>')
    xml.append('</Styles>')

    xml.append('<Worksheet ss:Name="Results"><Table>')

    xml.append('<Row ss:StyleID="sHeader">')
    for h in headers:
        xml.append(f'<Cell><Data ss:Type="String">{_xml_escape(h)}</Data></Cell>')
    xml.append('</Row>')

    for row in res_sorted:
        review = row[8] if len(row) > 8 else ""
        style = "sOK" if review == "OK" else ("sCHECK" if review == "CHECK" else None)

        xml.append('<Row>')
        for ci, val in enumerate(row):
            if style and ci in (6, 7, 8):
                xml.append(f'<Cell ss:StyleID="{style}"><Data ss:Type="String">{_xml_escape(val)}</Data></Cell>')
            else:
                xml.append(f'<Cell><Data ss:Type="String">{_xml_escape(val)}</Data></Cell>')
        xml.append('</Row>')

    xml.append('</Table></Worksheet></Workbook>')

    with open(out_xml, "w", encoding="utf-8") as f:
        f.write("\n".join(xml))

    print("Excel XML geschrieben:", out_xml)
    return out_xml

class IFC_OT_RunInternalExternal(Operator):
    bl_idname = "ifc.run_internal_external_test"
    bl_label = "START"

    def execute(self, context):
        global ENABLE_LAYER_STACK
        s = context.scene.ifc_ie_settings
        ENABLE_LAYER_STACK = True  # always on (UI removed)
        target_storeys = list(s.storeys) if (s.use_storey_filter and s.storeys) else []

        allowed_types = set()
        # IfcWall checkbox steuert IfcWall + IfcWallStandardCase gemeinsam
        # (deprecated flag bleibt nur synchron, darf nichts automatisch aktivieren)
        s.use_ifcwallstandardcase = s.use_ifcwall
        if s.use_ifcwall:
            allowed_types.add("IfcWall")
            allowed_types.add("IfcWallStandardCase")
        if s.use_ifccovering:
            allowed_types.add("IfcCovering")
        if s.use_ifcslab:
            allowed_types.add("IfcSlab")
        if s.use_ifccolumn:
            allowed_types.add("IfcColumn")

        if s.use_ifcbeam:
            allowed_types.add("IfcBeam")
        if s.use_ifcdoor:
            allowed_types.add("IfcDoor")
        if s.use_ifcwindow:
            allowed_types.add("IfcWindow")
        if not allowed_types:
            self.report({"ERROR"}, "Bitte mindestens ein IFC-Element aktivieren.")
            return {"CANCELLED"}

        try:
            res = run_from_ui(method=s.method, target_storeys=target_storeys, allowed_types=allowed_types)
        except SystemExit as e:
            self.report({"ERROR"}, str(e))
            return {"CANCELLED"}
        except Exception as e:
            self.report({"ERROR"}, f"Fehler: {e}")
            return {"CANCELLED"}

        if not res:
            self.report({"WARNING"}, "Keine Ergebnisse (Filter leer?).")
            return {"CANCELLED"}
        try:
            out_xml = write_excel_xml(res, s.out_xml)
        except Exception as e:
            self.report({"ERROR"}, f"Fehler beim Schreiben der XML: {e}")
            return {"CANCELLED"}

        self.report({"INFO"}, f"Done. XML: {out_xml}")
        return {"FINISHED"}

# =========================================================
# Debug: AABB Rays (NEU)
# =========================================================
def _ie_get_or_create_debug_collection():
    try:
        scn = bpy.context.scene
    except Exception:
        scn = None
    col = bpy.data.collections.get(DEBUG_AABB_RAYS_COLLECTION)
    if col is None:
        col = bpy.data.collections.new(DEBUG_AABB_RAYS_COLLECTION)
        try:
            if scn and scn.collection:
                scn.collection.children.link(col)
        except Exception:
            try:
                bpy.context.scene.collection.children.link(col)
            except Exception:
                pass
    return col

def _ie_remove_object(obj):
    if not obj:
        return
    try:
        data = obj.data
    except Exception:
        data = None
    try:
        for c in list(obj.users_collection):
            try:
                c.objects.unlink(obj)
            except Exception:
                pass
        bpy.data.objects.remove(obj, do_unlink=True)
    except Exception:
        pass
    try:
        if data and getattr(data, "users", 0) == 0:
            if isinstance(data, bpy.types.Mesh):
                bpy.data.meshes.remove(data)
            elif isinstance(data, bpy.types.Curve):
                bpy.data.curves.remove(data)
    except Exception:
        pass

def _ie_clear_aabb_rays():
    for o in list(bpy.data.objects):
        try:
            if o.get(DEBUG_AABB_RAYS_OBJ_PROP, False) or o.name.startswith(DEBUG_AABB_RAYS_PREFIX):
                _ie_remove_object(o)
        except Exception:
            continue

def _ie_make_polyline_mesh(name: str, polylines: list[list[Vector]]):
    me = bpy.data.meshes.new(name + "_ME")
    verts = []
    edges = []
    v_idx = 0
    for pts in polylines:
        if not pts or len(pts) < 2:
            continue
        for p in pts:
            verts.append((float(p.x), float(p.y), float(p.z)))
        for i in range(len(pts) - 1):
            edges.append((v_idx + i, v_idx + i + 1))
        v_idx += len(pts)
    me.from_pydata(verts, edges, [])
    me.update()
    ob = bpy.data.objects.new(name, me)
    try:
        ob[DEBUG_AABB_RAYS_OBJ_PROP] = True
    except Exception:
        pass
    try:
        ob.display_type = 'WIRE'
        ob.show_wire = True
        ob.show_in_front = True
    except Exception:
        pass
    return ob

def _ie_debug_trace_aabb_ray_column(scene, deps, start: Vector, dirn: Vector,
                                    bb_min, bb_max, src_obj, col_wall_ignore_map, blockers):
    pts = [start.copy()]
    p = start.copy()
    d = dirn.normalized()
    # Robust: feste Max-Distanz (Modell-AABB Diagonale) statt Exit-Distance.
    remaining = (bb_max - bb_min).length + 1.0
    steps = 0

    # Column-Fix: Door/Window-AABB-Blocker nur nutzen, wenn der Ray-Startpunkt in einem INTERNAL Space liegt.
    start_in_internal_space = False
    if COLUMNS_DW_BLOCKERS_ONLY_IF_START_IN_INTERNAL_SPACE:
        try:
            start_in_internal_space = bool(spaces_at_point(start))
        except Exception:
            start_in_internal_space = False

    while remaining > 0 and steps < MAX_STEPS_AABB:
        hit, loc, nor, idx, obj, *_ = scene.ray_cast(deps, p, d, distance=remaining)
        if not hit:
            endpos = p + d * remaining
            # Für Columns: Door/Window-AABB-Blocker nur anwenden, wenn Startpunkt in INTERNAL Space liegt.
            if USE_DOOR_WINDOW_AABB_BLOCKERS and blockers and (not COLUMNS_DW_BLOCKERS_ONLY_IF_START_IN_INTERNAL_SPACE or start_in_internal_space):
                for bo, bmin, bmax in blockers:
                    if segment_intersects_aabb(p, endpos, bmin, bmax):
                        t = segment_aabb_first_hit_dist(p, endpos, bmin, bmax)
                        if t is not None:
                            pts.append(p + d * t)
                        else:
                            pts.append(endpos)
                        return False, pts
            pts.append(endpos)
            return True, pts

        pts.append(loc.copy())
        dist_hit = (loc - p).length

        if should_ignore_hit_for_column(obj, src_obj, col_wall_ignore_map):
            adv_aabb = advance_past_obj_aabb(p, d, obj)
            advance = adv_aabb if adv_aabb is not None else (dist_hit + STEP_EPS_AABB)
            advance = max(advance, dist_hit + STEP_EPS_AABB)
            p = p + d * advance
            remaining -= advance
            pts.append(p.copy())
            steps += 1
            continue

        return False, pts

    return False, pts




def _ie_debug_trace_aabb_ray_beam(scene, deps, start: Vector, dirn: Vector,
                                  bb_min, bb_max, src_obj, col_wall_ignore_map, blockers, cw_blockers):
    pts = [start.copy()]
    p = start.copy()
    d = dirn.normalized()
    remaining = (bb_max - bb_min).length + 1.0
    steps = 0

    start_in_internal_space = False
    if COLUMNS_DW_BLOCKERS_ONLY_IF_START_IN_INTERNAL_SPACE:
        try:
            start_in_internal_space = bool(spaces_at_point(start))
        except Exception:
            start_in_internal_space = False

    while remaining > 0 and steps < MAX_STEPS_AABB:
        hit, loc, nor, idx, obj, *_ = scene.ray_cast(deps, p, d, distance=remaining)
        if not hit:
            endpos = p + d * remaining
            if USE_DOOR_WINDOW_AABB_BLOCKERS and blockers and (not COLUMNS_DW_BLOCKERS_ONLY_IF_START_IN_INTERNAL_SPACE or start_in_internal_space):
                for bo, bmin, bmax in blockers:
                    if segment_intersects_aabb(p, endpos, bmin, bmax):
                        t = segment_aabb_first_hit_dist(p, endpos, bmin, bmax)
                        if t is not None:
                            pts.append(p + d * t)
                        else:
                            pts.append(endpos)
                        return False, pts

            # Beam-Fix: CurtainWall-AABB-Blocker (Fassade als geschlossene Hülle behandeln)
            if USE_CURTAINWALL_AABB_BLOCKERS_FOR_BEAMS and cw_blockers:
                endpos2 = endpos  # bereits berechnet
                for bo, bmin, bmax in cw_blockers:
                    if segment_intersects_aabb(p, endpos2, bmin, bmax):
                        t = segment_aabb_first_hit_dist(p, endpos2, bmin, bmax)
                        if t is not None:
                            pts.append(p + d * t)
                        else:
                            pts.append(endpos2)
                        return False, pts

            pts.append(endpos)
            return True, pts

        pts.append(loc.copy())
        dist_hit = (loc - p).length

        if should_ignore_hit_for_beam(obj, src_obj, col_wall_ignore_map):
            adv_aabb = advance_past_obj_aabb(p, d, obj)
            advance = adv_aabb if adv_aabb is not None else (dist_hit + STEP_EPS_AABB)
            advance = max(advance, dist_hit + STEP_EPS_AABB)
            p = p + d * advance
            remaining -= advance
            pts.append(p.copy())
            steps += 1
            continue

        return False, pts

    return False, pts

class IFC_OT_ShowAABBRays(Operator):
    bl_idname = "ifc_ie.show_aabb_rays"
    bl_label = "Show AABB Rays"

    scope: EnumProperty(
        name="Scope",
        items=[
            ("ACTIVE", "Active", "Use active object only"),
            ("SELECTED", "Selected", "Use all selected objects"),
        ],
        default="ACTIVE",
    )

    def execute(self, context):
        deps = context.evaluated_depsgraph_get()
        scene = context.scene

        objs = []
        if self.scope == "SELECTED":
            objs = [o for o in context.selected_objects if o and _is_ifcclass(o, "IfcColumn", "IfcBeam")]
        else:
            ao = context.active_object
            if ao and _is_ifcclass(ao, "IfcColumn", "IfcBeam"):
                objs = [ao]

        if not objs:
            self.report({"ERROR"}, "Bitte eine IfcColumn oder IfcBeam auswählen (active oder selected).")
            return {"CANCELLED"}

        s = context.scene.ifc_ie_settings if hasattr(context.scene, "ifc_ie_settings") else None
        allowed_types = set()
        if s:
            # IfcWall checkbox steuert IfcWall + IfcWallStandardCase gemeinsam
            s.use_ifcwallstandardcase = getattr(s, "use_ifcwall", False)
            if getattr(s, "use_ifcwall", False):
                allowed_types.add("IfcWall")
                allowed_types.add("IfcWallStandardCase")
            if getattr(s, "use_ifccovering", False):
                allowed_types.add("IfcCovering")
            if getattr(s, "use_ifcslab", False):
                allowed_types.add("IfcSlab")
            if getattr(s, "use_ifccolumn", False):
                allowed_types.add("IfcColumn")

            if getattr(s, "use_ifcbeam", False):
                allowed_types.add("IfcBeam")
        all_mesh = scene_meshes()
        elems_all = [o for o in all_mesh if (is_selected_ifc_type(o, allowed_types) if allowed_types else True)]
        if not elems_all:
            elems_all = list(all_mesh)
        try:
            # Globales Modell-AABB (nicht nach allowed_types filtern), damit Rays nie an "tighten" Bounds scheitern.
            _all = [o for o in all_mesh if not o.name.startswith(DEBUG_AABB_RAYS_PREFIX)]
            bb_min, bb_max = aabb_of_objects(_all) if _all else aabb_of_objects(all_mesh)
        except Exception:
            self.report({"ERROR"}, "Konnte Modell-AABB nicht bestimmen.")
            return {"CANCELLED"}


        # Column-Fix: Für start_in_space detection (DW-Blocker gating) brauchen wir IfcSpace-BVHs auch im Debug.
        if COLUMNS_DW_BLOCKERS_ONLY_IF_START_IN_INTERNAL_SPACE:
            try:
                build_spaces_bvh_or_die()
            except BaseException:
                pass
        blockers = build_blocker_aabbs(scene_meshes()) if USE_DOOR_WINDOW_AABB_BLOCKERS else []
        cw_blockers = build_curtainwall_aabbs(scene_meshes()) if USE_CURTAINWALL_AABB_BLOCKERS_FOR_AABB else []
        global _IE_CW_AABB_BLOCKERS
        _IE_CW_AABB_BLOCKERS = cw_blockers
        all_walls = [w for w in scene_meshes() if _is_ifcclass(w, "IfcWall", "IfcWallStandardCase")]
        col_wall_ignore_map = build_column_embedded_wall_ignore_map(objs, all_walls) if objs else {}

        _ie_clear_aabb_rays()
        dbg_col = _ie_get_or_create_debug_collection()

        created = 0
        for obj in objs:
            eo = obj.evaluated_get(deps)
            me = eo.to_mesh()
            bvh = bvh_from_mesh(me)

            try:
                cmin, cmax = aabb_cached(obj)
            except Exception:
                eo.to_mesh_clear()
                continue

            ent = _ifc_ent(obj)
            ifc_type = ent.is_a() if ent else ""

            polylines_escape = []
            polylines_block = []

            try:
                if ifc_type == "IfcBeam":
                    # Beam: 4 Seiten entlang der beiden Querschnittsachsen (wie aabb_classify_one_beam)
                    bbminL, bbmaxL, centerL, hx, hy, hz = _beam_local_bb_params(obj)
                    ax, ay, az = _beam_axes_world(obj)

                    length_key = _beam_length_axis_key(hx, hy, hz)
                    cross_keys = _beam_cross_axis_keys(length_key)
                    axis_map = {"x": ax, "y": ay, "z": az}

                    sides = []
                    for k in cross_keys:
                        v = axis_map.get(k, Vector((1.0, 0.0, 0.0)))
                        vv = v.normalized()
                        sides.append((k, +1.0, vv))
                        sides.append((k, -1.0, -vv))

                    for axis_key, sign, d in sides:
                        starts = _beam_obb_sample_starts(obj, bvh, bbminL, bbmaxL, centerL,
                                                        axis_key, sign, d, length_key)
                        for start in starts:
                            esc, pts = _ie_debug_trace_aabb_ray_beam(scene, deps, start, d, bb_min, bb_max,
                                                                       obj, col_wall_ignore_map, blockers, cw_blockers)
                            if esc:
                                polylines_escape.append(pts)
                            else:
                                polylines_block.append(pts)
                else:
                    # Column (bestehende Logik)
                    bbminL, bbmaxL, centerL, hx, hy, hz = _column_local_bb_params(obj)
                    ax, ay, az = _column_axes_world(obj)

                    sides = [
                        ("x", +1.0, ax),
                        ("x", -1.0, -ax),
                        ("y", +1.0, ay),
                        ("y", -1.0, -ay),
                    ]

                    for axis_key, sign, d in sides:
                        starts = _column_obb_sample_starts(obj, bvh, bbminL, bbmaxL, centerL, axis_key, sign, d)
                        for start in starts:
                            esc, pts = _ie_debug_trace_aabb_ray_column(scene, deps, start, d, bb_min, bb_max,
                                                                       obj, col_wall_ignore_map, blockers)
                            if esc:
                                polylines_escape.append(pts)
                            else:
                                polylines_block.append(pts)
            except Exception:
                eo.to_mesh_clear()
                continue

            if polylines_escape:
                obE = _ie_make_polyline_mesh(f"{DEBUG_AABB_RAYS_PREFIX}_ESC_{obj.name}", polylines_escape)
                try:
                    obE.color = DEBUG_AABB_RAYS_COLOR_ESCAPE
                except Exception:
                    pass
                try:
                    dbg_col.objects.link(obE)
                except Exception:
                    try:
                        scene.collection.objects.link(obE)
                    except Exception:
                        pass
                created += 1

            if polylines_block:
                obB = _ie_make_polyline_mesh(f"{DEBUG_AABB_RAYS_PREFIX}_BLK_{obj.name}", polylines_block)
                try:
                    obB.color = DEBUG_AABB_RAYS_COLOR_BLOCK
                except Exception:
                    pass
                try:
                    dbg_col.objects.link(obB)
                except Exception:
                    try:
                        scene.collection.objects.link(obB)
                    except Exception:
                        pass
                created += 1

            eo.to_mesh_clear()

        self.report({"INFO"}, f"AABB rays created: {created}")
        return {"FINISHED"}


class IFC_OT_ClearAABBRays(Operator):
    bl_idname = "ifc_ie.clear_aabb_rays"
    bl_label = "Clear AABB Rays"

    def execute(self, context):
        _ie_clear_aabb_rays()
        self.report({"INFO"}, "Cleared AABB rays.")
        return {"FINISHED"}


class IFC_OT_MarkTouchSpaces(Operator):
    bl_idname = "ifc_ie.mark_touch_spaces"
    bl_label = "Mark Touch Spaces"

    side: EnumProperty(
        name="Side",
        items=[
            ("A", "A", "Mark spaces found on side A"),
            ("B", "B", "Mark spaces found on side B"),
            ("BOTH", "Both", "Mark spaces found on both sides"),
        ],
        default="BOTH",
    )

    def execute(self, context):
        src = context.active_object
        if not src:
            self.report({"ERROR"}, "Kein aktives Objekt.")
            return {"CANCELLED"}

        names = set()
        try:
            if self.side in {"A", "BOTH"}:
                names |= set(src.get(DEBUG_TOUCH_SPACES_PROP_A, []) or [])
            if self.side in {"B", "BOTH"}:
                names |= set(src.get(DEBUG_TOUCH_SPACES_PROP_B, []) or [])
        except Exception:
            names = set()

        if not names:
            self.report({"WARNING"}, "Keine Touch-Spaces gespeichert (erst SPACE-Methode laufen lassen).")
            return {"CANCELLED"}

        found = 0
        missing = 0
        for nm in sorted(names):
            sp = bpy.data.objects.get(nm)
            if not sp:
                missing += 1
                continue
            try:
                if DEBUG_SPACE_MARK_PROP not in sp:
                    sp[DEBUG_SPACE_OLD_COLOR_PROP] = list(getattr(sp, "color", (1,1,1,1)))
                    sp[DEBUG_SPACE_OLD_WIRE_PROP] = bool(getattr(sp, "show_wire", False))
                    sp[DEBUG_SPACE_OLD_FRONT_PROP] = bool(getattr(sp, "show_in_front", False))
                sp[DEBUG_SPACE_MARK_PROP] = True
                sp.color = DEBUG_SPACE_MARK_COLOR
                sp.show_wire = True
                sp.show_in_front = True
                sp.select_set(True)
                found += 1
            except Exception:
                missing += 1

        self.report({"INFO"}, f"Marked spaces: {found} (missing: {missing})")
        return {"FINISHED"}


class IFC_OT_ClearSpaceMarks(Operator):
    bl_idname = "ifc_ie.clear_space_marks"
    bl_label = "Clear Space Marks"

    def execute(self, context):
        cleared = 0
        for o in bpy.data.objects:
            if DEBUG_SPACE_MARK_PROP in o:
                try:
                    old_col = o.get(DEBUG_SPACE_OLD_COLOR_PROP, [1,1,1,1])
                    o.color = tuple(old_col) if isinstance(old_col, (list, tuple)) and len(old_col) == 4 else (1,1,1,1)
                    o.show_wire = bool(o.get(DEBUG_SPACE_OLD_WIRE_PROP, False))
                    o.show_in_front = bool(o.get(DEBUG_SPACE_OLD_FRONT_PROP, False))
                    o.pop(DEBUG_SPACE_MARK_PROP, None)
                    o.pop(DEBUG_SPACE_OLD_COLOR_PROP, None)
                    o.pop(DEBUG_SPACE_OLD_WIRE_PROP, None)
                    o.pop(DEBUG_SPACE_OLD_FRONT_PROP, None)
                    cleared += 1
                except Exception:
                    pass
        self.report({"INFO"}, f"Cleared marks: {cleared}")
        return {"FINISHED"}


class VIEW3D_PT_IFC_InternalExternal(Panel):
    bl_label = "User Interface"
    bl_idname = "VIEW3D_PT_ifc_internal_external_ui"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "IFC Test"

    def draw(self, context):
        s = context.scene.ifc_ie_settings
        layout = self.layout

        boxM = layout.box()
        boxM.label(text="Method")
        boxM.prop(s, "method", expand=True)

        boxE = layout.box()
        boxE.label(text="Building Elements")
        col = boxE.column(align=True)
        col.prop(s, "use_ifcwall")
        col.prop(s, "use_ifcdoor")
        col.prop(s, "use_ifcwindow")
        col.prop(s, "use_ifccovering")
        col.prop(s, "use_ifcslab")
        col.prop(s, "use_ifccolumn")
        col.prop(s, "use_ifcbeam")

        layout.separator()
        layout.prop(s, "use_storey_filter")

        if s.use_storey_filter:
            boxS = layout.box()
            boxS.label(text="Storeys")

            items = _storey_items(s, context)

            if len(items) == 1 and items[0][0] == "":
                boxS.label(text="<keine Storeys gefunden>")
            else:
                cols = len(items)  # alles nebeneinander
                flow = boxS.grid_flow(columns=cols, even_columns=True, even_rows=True, align=True)
                for ident, name, _ in items:
                    flow.prop_enum(s, "storeys", ident, text=name)

        layout.separator()
        layout.prop(s, "out_xml")

        layout.separator()
        boxD = layout.box()
        boxD.label(text="Debug")
        row = boxD.row(align=True)
        opA = row.operator("ifc_ie.mark_touch_spaces", text="Mark Spaces A")
        opA.side = "A"
        opB = row.operator("ifc_ie.mark_touch_spaces", text="Mark Spaces B")
        opB.side = "B"
        opBoth = row.operator("ifc_ie.mark_touch_spaces", text="Mark Spaces Both")
        opBoth.side = "BOTH"
        boxD.operator("ifc_ie.clear_space_marks", text="Clear Marks", icon="X")
        row2 = boxD.row(align=True)
        opR1 = row2.operator("ifc_ie.show_aabb_rays", text="Show AABB Rays (Active)")
        opR1.scope = "ACTIVE"
        opR2 = row2.operator("ifc_ie.show_aabb_rays", text="Show AABB Rays (Selected)")
        opR2.scope = "SELECTED"
        boxD.operator("ifc_ie.clear_aabb_rays", text="Clear AABB Rays", icon="X")


        layout.operator("ifc.run_internal_external_test", text="START", icon="PLAY")

classes_ui = (
    IFCIE_Settings,
    IFC_OT_RunInternalExternal,
    IFC_OT_MarkTouchSpaces,
    IFC_OT_ClearSpaceMarks,
    IFC_OT_ShowAABBRays,
    IFC_OT_ClearAABBRays,
    VIEW3D_PT_IFC_InternalExternal,
)

def unregister_ui():
    try:
        if hasattr(bpy.types.Scene, "ifc_ie_settings"):
            del bpy.types.Scene.ifc_ie_settings
    except Exception:
        pass
    for c in reversed(classes_ui):
        try:
            bpy.utils.unregister_class(c)
        except Exception:
            pass

def register_ui():
    for c in classes_ui:
        try:
            bpy.utils.register_class(c)
        except Exception:
            pass
    if not hasattr(bpy.types.Scene, "ifc_ie_settings"):
        bpy.types.Scene.ifc_ie_settings = PointerProperty(type=IFCIE_Settings)

unregister_ui()
register_ui()
print("IFC UI registriert: 3D Viewport → N → IFC Test → User Interface")