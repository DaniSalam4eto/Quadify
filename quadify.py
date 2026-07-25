bl_info = {
    "name": "Quadify",
    "author": "Made for you",
    "version": (3, 2, 0),
    "blender": (3, 6, 0),
    "location": "Top bar > Quadify, and the right-click context menu",
    "description": "Rebuild a curved strip of faces into a clean quad grid and "
                   "map a straight, even, non-stretching UV along it. Ideal for "
                   "texturing curved taxiways and roads.",
    "category": "UV",
}

import math
from collections import defaultdict

import bmesh
import bpy
from bpy.props import BoolProperty, FloatProperty, IntProperty
from mathutils import Vector


# ===========================================================================
#  Geometry helpers  (all pure math, none of this changes your mesh)
# ===========================================================================

def _ordered_boundary_loop(sel_faces):
    """Return the boundary vertices of the selection as one ordered ring.

    The boundary is every edge that has exactly one selected face touching it.
    For a clean strip that outline is a single closed loop: two long rails and
    two short end caps. Returns None if the outline isn't a simple loop.
    """
    sel = set(sel_faces)

    boundary_edges = []
    seen = set()
    for f in sel_faces:
        for e in f.edges:
            if e in seen:
                continue
            seen.add(e)
            touching = sum(1 for lf in e.link_faces if lf in sel)
            if touching == 1:
                boundary_edges.append(e)

    if len(boundary_edges) < 4:
        return None

    # Build vertex adjacency along the boundary.
    adj = defaultdict(list)
    for e in boundary_edges:
        v0, v1 = e.verts
        adj[v0].append(v1)
        adj[v1].append(v0)

    # A simple loop needs every boundary vertex to have exactly two neighbours.
    if any(len(neigh) != 2 for neigh in adj.values()):
        return None

    start = boundary_edges[0].verts[0]
    loop = [start]
    prev, cur = None, start
    while True:
        a, b = adj[cur]
        nxt = a if a is not prev else b
        if nxt is start:
            break
        loop.append(nxt)
        prev, cur = cur, nxt
        if len(loop) > 500000:      # safety valve
            return None

    # If we didn't come back around to touch every boundary vertex, it wasn't
    # a single clean loop.
    if len(loop) != len(adj):
        return None
    return loop


def _four_corners(loop):
    """Indices of the four sharpest turns in the boundary loop (the corners)."""
    n = len(loop)
    turns = []
    for i in range(n):
        a = loop[(i - 1) % n].co
        b = loop[i].co
        c = loop[(i + 1) % n].co
        d1 = b - a
        d2 = c - b
        if d1.length == 0.0 or d2.length == 0.0:
            angle = 0.0
        else:
            angle = d1.angle(d2)
        turns.append((angle, i))
    turns.sort(reverse=True)
    return sorted(idx for _, idx in turns[:4])


def _segment(loop, i0, i1):
    """The run of vertices from index i0 to i1 walking forward along the loop."""
    n = len(loop)
    out = []
    i = i0
    while True:
        out.append(loop[i])
        if i == i1:
            break
        i = (i + 1) % n
    return out


def _length(points):
    return sum((points[j + 1] - points[j]).length for j in range(len(points) - 1))


def _resample(verts, n):
    """Evenly resample a polyline (list of verts) into n points by arc length."""
    pts = [v.co.copy() for v in verts]
    if len(pts) == 1:
        return [pts[0].copy() for _ in range(n)]

    cum = [0.0]
    for j in range(len(pts) - 1):
        cum.append(cum[-1] + (pts[j + 1] - pts[j]).length)
    total = cum[-1] or 1.0

    out = []
    for s in range(n):
        target = total * (s / (n - 1))
        k = 0
        while k < len(cum) - 2 and cum[k + 1] < target:
            k += 1
        seg = cum[k + 1] - cum[k]
        t = 0.0 if seg == 0.0 else (target - cum[k]) / seg
        out.append(pts[k].lerp(pts[k + 1], t))
    return out


def _polyline_cumlen(points):
    cum = [0.0]
    for j in range(len(points) - 1):
        cum.append(cum[-1] + (points[j + 1] - points[j]).length)
    return cum


def _project_onto_centerline(p, center, cum, normal):
    """Return (distance_along, signed_offset) of point p relative to the curve."""
    best_d2 = None
    best_s = 0.0
    best_off = 0.0
    for k in range(len(center) - 1):
        a = center[k]
        b = center[k + 1]
        ab = b - a
        L2 = ab.length_squared
        t = 0.0 if L2 == 0.0 else max(0.0, min(1.0, (p - a).dot(ab) / L2))
        proj = a + ab * t
        d2 = (p - proj).length_squared
        if best_d2 is None or d2 < best_d2:
            best_d2 = d2
            best_s = cum[k] + ab.length * t
            tangent = ab.normalized() if ab.length else Vector((1, 0, 0))
            side = normal.cross(tangent)
            best_off = (p - proj).dot(side)
    return best_s, best_off


# ===========================================================================
#  UV building
# ===========================================================================

def _vertex_adjacency(sel_faces):
    """vert -> set(neighbour verts), following the edges of the selected faces."""
    adj = defaultdict(set)
    seen = set()
    for f in sel_faces:
        for e in f.edges:
            if e in seen:
                continue
            seen.add(e)
            a, b = e.verts
            adj[a].add(b)
            adj[b].add(a)
    return adj


def _assign_run(run, p0, p1, uv):
    """Spread a boundary run's vertices evenly (by arc length) from p0 to p1."""
    pts = [v.co for v in run]
    cum = _polyline_cumlen(pts)
    total = cum[-1] or 1.0
    for k, v in enumerate(run):
        t = cum[k] / total
        uv[v] = [p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t]


def _rail_fractions(rail):
    """Normalised (0..1) arc-length position of each vertex along a rail."""
    cum = _polyline_cumlen([v.co for v in rail])
    total = cum[-1] or 1.0
    return [c / total for c in cum]


def _point_on_rail(rail, fracs, t):
    """3D point at arc-length fraction t (0..1) along a rail."""
    if t <= 0.0:
        return rail[0].co.copy()
    if t >= 1.0:
        return rail[-1].co.copy()
    for i in range(len(fracs) - 1):
        if fracs[i] <= t <= fracs[i + 1]:
            seg = fracs[i + 1] - fracs[i]
            a = 0.0 if seg == 0 else (t - fracs[i]) / seg
            return rail[i].co.lerp(rail[i + 1].co, a)
    return rail[-1].co.copy()


def _trace_rails(sel_faces):
    """Trace the strip outline and return (runs, length, width).

    runs[0] and runs[2] are the two long rails (opposite sides); runs[1] and
    runs[3] are the short end caps. length/width are the average rail/cap
    lengths. Returns None if the outline isn't a clean single loop.
    """
    loop = _ordered_boundary_loop(sel_faces)
    if loop is None or len(loop) < 4:
        return None
    corners = _four_corners(loop)
    if len(corners) != 4:
        return None

    def runs_for(cs):
        return [_segment(loop, cs[k], cs[(k + 1) % 4]) for k in range(4)]

    runs = runs_for(corners)
    lens = [_length([v.co for v in r]) for r in runs]
    # The two rails (long sides) must be the opposite pair runs[0] & runs[2].
    if (lens[1] + lens[3]) > (lens[0] + lens[2]):
        corners = corners[1:] + corners[:1]
        runs = runs_for(corners)
        lens = [_length([v.co for v in r]) for r in runs]

    length = 0.5 * (lens[0] + lens[2]) or 1.0
    width = 0.5 * (lens[1] + lens[3]) or 1.0
    return runs, length, width


def _rebuild_as_quads(bm, me, sel_faces, length_segments, width_segments):
    """Replace the selected strip with a clean quad grid flowing along it.

    Returns (new_faces, raw, width) where raw maps each new vertex to its
    (u, v) in world units: u = distance along the centreline, v = 0..width.
    Returns None if the strip can't be traced.
    """
    trace = _trace_rails(sel_faces)
    if trace is None:
        return None
    runs, _length, width = trace

    # Remember the strip's material, facing direction and shading BEFORE we
    # delete it, so the new quads keep the same texture (not slot 0 / a
    # transparent underlay) and don't end up back-facing (which reads as
    # transparent from above once the sim back-face culls them).
    mat_counts = {}
    for f in sel_faces:
        mat_counts[f.material_index] = mat_counts.get(f.material_index, 0) + 1
    mat_index = max(mat_counts, key=mat_counts.get) if mat_counts else 0
    use_smooth = any(f.smooth for f in sel_faces)
    up = Vector((0.0, 0.0, 0.0))
    for f in sel_faces:
        up = up + f.normal
    if up.length == 0.0:
        up = Vector((0.0, 0.0, 1.0))
    up.normalize()

    rail_lo = runs[0]                    # goes u: 0 -> length
    rail_hi = list(reversed(runs[2]))    # reversed so it also goes u: 0 -> length
    f_lo = _rail_fractions(rail_lo)
    f_hi = _rail_fractions(rail_hi)

    # How many segments along the length. Auto = keep the strip's own resolution
    # so the curve stays smooth.
    if length_segments and length_segments > 0:
        n_len = int(length_segments)
    else:
        n_len = max(len(rail_lo), len(rail_hi)) - 1
    n_len = max(n_len, 1)
    n_wid = max(int(width_segments), 1)

    # Build the grid of new vertices. Each row is a straight cross-section from
    # one rail to the other (correct for flat ground scenery).
    grid = [[None] * (n_wid + 1) for _ in range(n_len + 1)]
    centre = []
    for i in range(n_len + 1):
        t = i / n_len
        a = _point_on_rail(rail_lo, f_lo, t)
        b = _point_on_rail(rail_hi, f_hi, t)
        centre.append((a + b) * 0.5)
        for j in range(n_wid + 1):
            grid[i][j] = bm.verts.new(a.lerp(b, j / n_wid))
    centre_arc = _polyline_cumlen(centre)   # u coordinate = true centreline distance

    # Out with the old triangle soup.
    bmesh.ops.delete(bm, geom=sel_faces, context='FACES')

    # In with clean quads, and record each new vertex's (u, v).
    raw = {}
    new_faces = []
    for i in range(n_len + 1):
        for j in range(n_wid + 1):
            raw[grid[i][j]] = (centre_arc[i], (j / n_wid) * width)
    for i in range(n_len):
        for j in range(n_wid):
            try:
                f = bm.faces.new((grid[i][j], grid[i][j + 1],
                                  grid[i + 1][j + 1], grid[i + 1][j]))
                f.select = True
                f.material_index = mat_index    # keep the strip's texture
                f.smooth = use_smooth
                new_faces.append(f)
            except ValueError:
                pass  # duplicate face, skip

    # Make sure every new quad faces the same way the strip did (up), so it
    # isn't invisible/back-faced from above.
    bm.normal_update()
    for f in new_faces:
        if f.normal.dot(up) < 0.0:
            f.normal_flip()
    bm.normal_update()
    return new_faces, raw, width


def _ribbon_uvs(sel_faces):
    """Compute {vert: (u, v)} by pinning the strip's outline to a rectangle and
    relaxing the interior to fit (a Tutte/harmonic map).

    Because the outline is pinned to a convex rectangle, the result can never
    flare, fold, or overlap. Returns None if the outline can't be traced.
    """
    trace = _trace_rails(sel_faces)
    if trace is None:
        return None
    runs, length, width = trace

    # Pin the outline onto the rectangle's four edges.
    uv = {}
    _assign_run(runs[0], (0.0, 0.0), (length, 0.0), uv)      # bottom rail
    _assign_run(runs[1], (length, 0.0), (length, width), uv)  # right cap
    _assign_run(runs[2], (length, width), (0.0, width), uv)   # top rail
    _assign_run(runs[3], (0.0, width), (0.0, 0.0), uv)        # left cap

    boundary = set(uv.keys())
    all_verts = {v for f in sel_faces for v in f.verts}
    interior = [v for v in all_verts if v not in boundary]

    # Seed the interior with a quick axis-based guess so the relaxation
    # converges fast (fewer iterations needed).
    pca_raw, _ = _pca_uvs(sel_faces)
    s_vals = [s for s, _ in pca_raw.values()]
    o_vals = [o for _, o in pca_raw.values()]
    s_min, s_rng = min(s_vals), (max(s_vals) - min(s_vals)) or 1.0
    o_min, o_rng = min(o_vals), (max(o_vals) - min(o_vals)) or 1.0
    for v in interior:
        s, o = pca_raw[v]
        uv[v] = [(s - s_min) / s_rng * length, (o - o_min) / o_rng * width]

    # Relax: each interior vertex drifts to the average of its neighbours.
    # With the boundary fixed to a rectangle, this is a harmonic map: smooth,
    # and provably free of folds/overlaps.
    adj = _vertex_adjacency(sel_faces)
    aspect = length / (width or 1.0)
    iterations = int(min(800, max(120, 25 * aspect)))
    for _ in range(iterations):
        for v in interior:
            neighbours = adj.get(v)
            if not neighbours:
                continue
            su = sv = 0.0
            count = 0
            for nb in neighbours:
                p = uv.get(nb)
                if p is None:
                    continue
                su += p[0]
                sv += p[1]
                count += 1
            if count:
                uv[v][0] = su / count
                uv[v][1] = sv / count

    # --- Even out the texture density along the strip ----------------------
    # The relaxed U isn't proportional to real distance, so a texture looks
    # stretched on curves. Fix it by remapping U to the TRUE arc length along
    # the strip's centreline. The centreline is built from the two rails (which
    # are smooth and ordered) rather than from scattered vertices, so this is
    # stable even when the mesh is denser on the curves than on the straights.
    rail_lo = runs[0]                 # pinned u: 0 -> length
    rail_hi = list(reversed(runs[2]))  # was L -> 0; reversed so it's 0 -> length
    f_lo = _rail_fractions(rail_lo)
    f_hi = _rail_fractions(rail_hi)

    samples = 200
    centre = []
    for m in range(samples):
        t = m / (samples - 1)
        centre.append((_point_on_rail(rail_lo, f_lo, t) + _point_on_rail(rail_hi, f_hi, t)) * 0.5)
    centre_arc = _polyline_cumlen(centre)   # true distance along the centreline

    def _to_arc(u):
        t = u / (length or 1.0)
        if t <= 0.0:
            return centre_arc[0]
        if t >= 1.0:
            return centre_arc[-1]
        fm = t * (samples - 1)
        i = int(fm)
        if i >= samples - 1:
            return centre_arc[-1]
        return centre_arc[i] + (centre_arc[i + 1] - centre_arc[i]) * (fm - i)

    raw = {v: (_to_arc(uv[v][0]), uv[v][1]) for v in all_verts}
    return raw, width


def _pca_uvs(sel_faces):
    """Fallback: flatten using the strip's main axis (for mild curves)."""
    verts = list({v for f in sel_faces for v in f.verts})
    center = Vector((0.0, 0.0, 0.0))
    for v in verts:
        center += v.co
    center /= len(verts)

    # 2x2 covariance in the XY plane.
    sxx = sxy = syy = 0.0
    for v in verts:
        dx = v.co.x - center.x
        dy = v.co.y - center.y
        sxx += dx * dx
        sxy += dx * dy
        syy += dy * dy
    theta = 0.5 * math.atan2(2.0 * sxy, (sxx - syy) if (sxx - syy) != 0 else 1e-9)
    long_axis = Vector((math.cos(theta), math.sin(theta), 0.0))
    short_axis = Vector((-math.sin(theta), math.cos(theta), 0.0))

    raw = {}
    off_min = off_max = None
    for v in verts:
        d = v.co - center
        s = d.dot(long_axis)
        off = d.dot(short_axis)
        raw[v] = (s, off)
        off_min = off if off_min is None else min(off_min, off)
        off_max = off if off_max is None else max(off_max, off)

    width = (off_max - off_min) or 1.0
    # shift s so it starts at 0
    s_min = min(s for s, _ in raw.values())
    raw = {v: (s - s_min, off) for v, (s, off) in raw.items()}
    return raw, width


def _write_uvs(bm, faces, raw, width, fill_width, tiles, rotate, flip_length, flip_width):
    """Write the computed (u, v) values (world units) into the UV map, applying
    the tiling / fill / rotate / flip options."""
    length = max(s for s, _ in raw.values()) or 1.0
    off_min = min(off for _, off in raw.values())

    uv_layer = bm.loops.layers.uv.verify()
    for f in faces:
        for lp in f.loops:
            s, off = raw[lp.vert]

            if fill_width:
                v = (off - off_min) / width          # 0..1 across the width
            else:
                v = (off - off_min) / length         # keep true proportions
            u = (s / width) * tiles                  # square-ish tiling

            if flip_length:
                u = -u
            if flip_width:
                v = 1.0 - v
            if rotate:
                u, v = v, u

            lp[uv_layer].uv = (u, v)


# ===========================================================================
#  Operator
# ===========================================================================

class MESH_OT_quadify(bpy.types.Operator):
    """Rebuild the selected curved strip into clean quads and map a straight, even UV along it"""
    bl_idname = "mesh.quadify"
    bl_label = "Quadify"
    bl_options = {'REGISTER', 'UNDO'}

    rebuild_quads: BoolProperty(
        name="Rebuild as Quads",
        description="Replace the selected triangles with a clean quad grid before "
                    "mapping. This is what stops the stretched/broken texture on "
                    "curves. Turn off to only reassign UVs on the existing mesh",
        default=True,
    )
    length_segments: IntProperty(
        name="Length Segments",
        description="Number of quad rows along the strip (0 = keep the strip's own "
                    "resolution so the curve stays smooth)",
        default=0,
        min=0,
        soft_max=400,
    )
    width_segments: IntProperty(
        name="Width Segments",
        description="Number of quad columns across the strip's width",
        default=1,
        min=1,
        soft_max=20,
    )
    fill_width: BoolProperty(
        name="Fill Width",
        description="Stretch the strip's width to fill the 0..1 texture width",
        default=True,
    )
    tiles: FloatProperty(
        name="Tiling",
        description="How many times the texture repeats along the strip's length",
        default=1.0,
        min=0.001,
        soft_max=50.0,
    )
    rotate: BoolProperty(
        name="Rotate 90°",
        description="Swap length and width (rotate the texture a quarter turn)",
        default=False,
    )
    flip_length: BoolProperty(name="Flip Length", default=False)
    flip_width: BoolProperty(name="Flip Width", default=False)

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return obj is not None and obj.type == 'MESH'

    def execute(self, context):
        obj = context.active_object

        started_in_object_mode = (obj.mode == 'OBJECT')
        if started_in_object_mode:
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_all(action='SELECT')

        me = obj.data
        bm = bmesh.from_edit_mesh(me)
        sel_faces = [f for f in bm.faces if f.select]

        if not sel_faces:
            if started_in_object_mode:
                bpy.ops.object.mode_set(mode='OBJECT')
            self.report({'WARNING'}, "Quadify: select the strip's faces first")
            return {'CANCELLED'}

        if self.rebuild_quads:
            # Retopologise the strip into a clean quad grid, then map it. This is
            # the fix for stretched/broken texture: quads map without the shear
            # that triangles create on a curve.
            rebuilt = _rebuild_as_quads(bm, me, sel_faces,
                                        self.length_segments, self.width_segments)
            if rebuilt is None:
                if started_in_object_mode:
                    bpy.ops.object.mode_set(mode='OBJECT')
                self.report(
                    {'WARNING'},
                    "Quadify: couldn't trace a clean strip. Select just one strip "
                    "with no holes or branches.",
                )
                return {'CANCELLED'}
            faces, raw, width = rebuilt
            _write_uvs(bm, faces, raw, width, self.fill_width, self.tiles,
                       self.rotate, self.flip_length, self.flip_width)
            bmesh.update_edit_mesh(me, destructive=True)
            message = "Quadify: rebuilt into %d clean quads" % len(faces)
        else:
            # UV-only mode: keep the existing geometry, just reassign UVs.
            result = _ribbon_uvs(sel_faces)
            used_fallback = False
            if result is None:
                result = _pca_uvs(sel_faces)
                used_fallback = True
            raw, width = result
            _write_uvs(bm, sel_faces, raw, width, self.fill_width, self.tiles,
                       self.rotate, self.flip_length, self.flip_width)
            bmesh.update_edit_mesh(me)
            message = ("Quadify: straightened by main axis (couldn't trace a clean outline)."
                       if used_fallback
                       else "Quadify: unrolled %d faces" % len(sel_faces))

        if started_in_object_mode:
            bpy.ops.object.mode_set(mode='OBJECT')

        self.report({'INFO'}, message)
        return {'FINISHED'}


# ===========================================================================
#  UI
#  Two homes only: the top bar, and the right-click context menu.
# ===========================================================================

def _draw_menu_entry(self, context):
    self.layout.operator("mesh.quadify", text="Quadify", icon='MOD_SIMPLEDEFORM')


def _draw_context_entry(self, context):
    # A divider above keeps our entry visually separate from the rest.
    self.layout.separator()
    self.layout.operator("mesh.quadify", text="Quadify", icon='MOD_SIMPLEDEFORM')


classes = (MESH_OT_quadify,)

# Where the button appears. Missing menus in a given Blender build are skipped,
# never enough to break registration.
_TOP_MENUS = ("TOPBAR_MT_editor_menus",)
_CONTEXT_MENUS = ("VIEW3D_MT_edit_mesh_context_menu", "VIEW3D_MT_object_context_menu")


def register():
    # Register classes defensively: a leftover class from a previous install
    # must not abort the whole registration (that's what makes the button vanish).
    for cls in classes:
        try:
            bpy.utils.register_class(cls)
        except Exception:
            try:
                bpy.utils.unregister_class(cls)
                bpy.utils.register_class(cls)
            except Exception:
                pass

    for menu_name in _TOP_MENUS:
        menu = getattr(bpy.types, menu_name, None)
        if menu is not None:
            try:
                menu.append(_draw_menu_entry)
            except Exception:
                pass

    for menu_name in _CONTEXT_MENUS:
        menu = getattr(bpy.types, menu_name, None)
        if menu is not None:
            try:
                menu.append(_draw_context_entry)
            except Exception:
                pass


def unregister():
    for menu_name in _TOP_MENUS:
        menu = getattr(bpy.types, menu_name, None)
        if menu is not None:
            try:
                menu.remove(_draw_menu_entry)
            except Exception:
                pass

    for menu_name in _CONTEXT_MENUS:
        menu = getattr(bpy.types, menu_name, None)
        if menu is not None:
            try:
                menu.remove(_draw_context_entry)
            except Exception:
                pass

    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except Exception:
            pass


if __name__ == "__main__":
    register()
