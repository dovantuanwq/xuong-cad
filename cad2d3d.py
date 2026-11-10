"""cad2d3d — đọc bản vẽ 2D (DXF/DWG/PDF vector) và dựng khối 3D bằng build123d, xuất STEP/STL + ảnh xem trước.

Dùng:
    from cad2d3d import *
    d = load("ban_ve.dxf")            # .dxf .dwg .pdf (vector)
    report(d, "views.png")             # in danh sách hình chiếu + ảnh đánh số view
    part = extrude_view(d.views[0], 6) # tấm phẳng dày 6
    export(part, "chi_tiet")           # chi_tiet.step / .stl / _3d.png
"""
from __future__ import annotations
import math, os, re, subprocess, json, shutil, tempfile
from dataclasses import dataclass, field

import ezdxf
from ezdxf import path as ezpath
import shapely
from shapely.geometry import LineString, Point, Polygon, box as sbox
from shapely.ops import unary_union, polygonize

ANNOT_LAYER = re.compile(r"(dim|kich|kt|text|chu|note|hatch|mat.?cat|center|tam|axis|truc|hidden|khuat|an|defpoints|title|khung|border|frame|vien.?giay|tbl|bang)", re.I)
CENTER_LT = re.compile(r"(center|dashdot|phantom|chain|axis)", re.I)
HIDDEN_LT = re.compile(r"(hidden|dash|dot)", re.I)


# ---------------------------------------------------------------- data
@dataclass
class Ent:
    t: str                  # line | arc | circle
    a: tuple = None         # line start
    b: tuple = None         # line end
    c: tuple = None         # arc/circle centre
    r: float = 0.0
    a0: float = 0.0         # arc start angle (deg, ccw)
    a1: float = 360.0
    layer: str = "0"
    kind: str = "outline"   # outline | center | hidden

    def geom(self, n=48):
        if self.t == "line":
            return LineString([self.a, self.b])
        a0, a1 = self.a0, self.a1
        if self.t == "circle":
            a0, a1 = 0.0, 360.0
        while a1 <= a0:
            a1 += 360.0
        k = max(8, int(n * (a1 - a0) / 360))
        pts = [(self.c[0] + self.r * math.cos(math.radians(a0 + (a1 - a0) * i / k)),
                self.c[1] + self.r * math.sin(math.radians(a0 + (a1 - a0) * i / k))) for i in range(k + 1)]
        return LineString(pts)


@dataclass
class View:
    id: int
    ents: list
    bbox: tuple             # x0,y0,x1,y1
    region: object = None   # shapely polygon (outer minus holes) — xấp xỉ, chỉ để phân tích
    circles: list = field(default_factory=list)   # [(cx,cy,r)] lỗ tròn kín
    hidden: list = field(default_factory=list)
    centers: list = field(default_factory=list)
    main: list = field(default_factory=list)      # nét của đường bao chính
    inner: list = field(default_factory=list)     # các cụm nét kín bên trong (lỗ, rãnh, hốc…)

    @property
    def size(self):
        return (self.bbox[2] - self.bbox[0], self.bbox[3] - self.bbox[1])


@dataclass
class Drawing:
    path: str
    ents: list
    texts: list
    dims: list
    views: list
    frame: object = None
    units: str = ""


# ---------------------------------------------------------------- loading
def _dwg_to_dxf(path):
    """DWG -> DXF bằng LibreDWG (gói npm dwg2dxf-converter)."""
    out = tempfile.mktemp(suffix=".dxf")
    here = os.path.dirname(os.path.abspath(__file__))
    nodedir = os.path.join(here, "node")
    if not os.path.isdir(os.path.join(nodedir, "node_modules", "dwg2dxf-converter")):
        os.makedirs(nodedir, exist_ok=True)
        subprocess.run(["npm", "i", "--prefix", nodedir, "dwg2dxf-converter"], check=True, capture_output=True)
    js = ("const {convertDwgToDxf}=require(process.argv[3]);convertDwgToDxf(process.argv[1],process.argv[2],{timeout:120000})"
          ".then(r=>{console.log(JSON.stringify(r));process.exit(r.success?0:1)})")
    mod = os.path.join(nodedir, "node_modules", "dwg2dxf-converter")
    r = subprocess.run(["node", "-e", js, path, out, mod], capture_output=True, text=True, timeout=180)
    if r.returncode != 0 or not os.path.exists(out):
        raise RuntimeError("Không chuyển được DWG sang DXF: " + (r.stdout + r.stderr)[-400:])
    return out


def _classify(e, layer):
    lt = ""
    try:
        lt = e.dxf.get("linetype", "") or ""
        if lt.upper() in ("BYLAYER", "") and e.doc and layer in e.doc.layers:
            lt = e.doc.layers.get(layer).dxf.get("linetype", "") or ""
    except Exception:
        pass
    if CENTER_LT.search(lt) or re.search(r"(center|tam|truc|axis)", layer, re.I):
        return "center"
    if HIDDEN_LT.search(lt) or re.search(r"(hidden|khuat|net.?dut)", layer, re.I):
        return "hidden"
    return "outline"


def _walk(entities, out, texts, dims, depth=0):
    for e in entities:
        t = e.dxftype()
        layer = e.dxf.get("layer", "0")
        try:
            if t == "INSERT" and depth < 8:
                _walk(list(e.virtual_entities()), out, texts, dims, depth + 1)
            elif t == "DIMENSION":
                try:
                    v = e.get_measurement()
                    v = float(v) if not hasattr(v, "x") else None
                except Exception:
                    v = None
                txt = e.dxf.get("text", "")
                p = e.dxf.get("text_midpoint", None)
                dims.append({"value": v, "text": txt, "p": (p.x, p.y) if p else None, "layer": layer})
            elif t in ("TEXT", "MTEXT", "ATTRIB"):
                s = e.plain_text() if t == "MTEXT" else e.dxf.get("text", "")
                p = e.dxf.insert
                texts.append({"s": s, "p": (p.x, p.y), "h": e.dxf.get("char_height" if t == "MTEXT" else "height", 0), "layer": layer})
            elif t in ("HATCH", "SOLID", "POINT", "VIEWPORT", "IMAGE", "WIPEOUT", "LEADER", "MULTILEADER", "MLEADER", "TOLERANCE"):
                continue
            elif ANNOT_LAYER.search(layer) and not re.search(r"(vien|outline|net.?lien|visible|0$)", layer, re.I) and _classify(e, layer) == "outline" and re.search(r"(dim|kich|text|chu|hatch|title|khung|defpoints|border|frame)", layer, re.I):
                continue
            elif t == "LINE":
                a, b = e.dxf.start, e.dxf.end
                if (a - b).magnitude > 1e-9:
                    out.append(Ent("line", a=(a.x, a.y), b=(b.x, b.y), layer=layer, kind=_classify(e, layer)))
            elif t == "CIRCLE":
                c = e.dxf.center
                out.append(Ent("circle", c=(c.x, c.y), r=e.dxf.radius, layer=layer, kind=_classify(e, layer)))
            elif t == "ARC":
                c = e.dxf.center
                out.append(Ent("arc", c=(c.x, c.y), r=e.dxf.radius, a0=e.dxf.start_angle, a1=e.dxf.end_angle, layer=layer, kind=_classify(e, layer)))
            elif t in ("LWPOLYLINE", "POLYLINE"):
                k = _classify(e, layer)
                for v in e.virtual_entities():
                    v.dxf.layer = layer
                    _walk([v], out, texts, dims, depth + 1)
                for x in out[-200:]:
                    if x.layer == layer and x.kind == "outline":
                        x.kind = k
            elif t in ("SPLINE", "ELLIPSE"):
                k = _classify(e, layer)
                pts = [(p.x, p.y) for p in ezpath.make_path(e).flattening(0.01)]
                for p, q in zip(pts, pts[1:]):
                    out.append(Ent("line", a=p, b=q, layer=layer, kind=k))
        except Exception:
            continue


def _load_pdf(path, page=0, scale=1.0):
    import pymupdf
    doc = pymupdf.open(path)
    pg = doc[page]
    H = pg.rect.height
    ents, texts = [], []
    for d in pg.get_drawings():
        dashed = bool(d.get("dashes")) and d.get("dashes") not in ("[] 0", "[] 0.0")
        if d.get("fill") is not None and d.get("color") is None:
            continue   # mảng tô (nền, hatch đặc) không phải nét
        its = d["items"]
        if d.get("closePath") and its and its[0][0] in ("l", "c"):
            first, lastp = its[0][1], its[-1][-1]
            if abs(first.x - lastp.x) > 1e-6 or abs(first.y - lastp.y) > 1e-6:
                its = list(its) + [("l", lastp, first)]
        for it in its:
            if it[0] == "l":
                a, b = it[1], it[2]
                ents.append(Ent("line", a=(a.x * scale, (H - a.y) * scale), b=(b.x * scale, (H - b.y) * scale), kind="hidden" if dashed else "outline"))
            elif it[0] == "c":
                p0, p1, p2, p3 = it[1:5]
                pts = []
                for i in range(9):
                    t = i / 8
                    x = (1-t)**3*p0.x + 3*(1-t)**2*t*p1.x + 3*(1-t)*t*t*p2.x + t**3*p3.x
                    y = (1-t)**3*p0.y + 3*(1-t)**2*t*p1.y + 3*(1-t)*t*t*p2.y + t**3*p3.y
                    pts.append((x * scale, (H - y) * scale))
                for p, q in zip(pts, pts[1:]):
                    ents.append(Ent("line", a=p, b=q, kind="hidden" if dashed else "outline"))
            elif it[0] in ("re", "qu"):
                r = it[1]
                c = [(r.x0, r.y0), (r.x1, r.y0), (r.x1, r.y1), (r.x0, r.y1)] if it[0] == "re" else [(q.x, q.y) for q in (r.ul, r.ur, r.lr, r.ll)]
                for p, q in zip(c, c[1:] + c[:1]):
                    ents.append(Ent("line", a=(p[0]*scale, (H-p[1])*scale), b=(q[0]*scale, (H-q[1])*scale)))
    for b in pg.get_text("words"):
        texts.append({"s": b[4], "p": (b[0] * scale, (H - b[3]) * scale), "h": (b[3] - b[1]) * scale})
    if not ents:
        raise RuntimeError("PDF này là ảnh quét (không có nét vector). Hãy để Claude đọc bằng mắt và dựng theo kích thước ghi trên bản vẽ.")
    return _fit_circles(ents), texts


def _fit_circles(ents, tol=0.02):
    """Gộp chuỗi đoạn thẳng nhỏ khép kín thành đường tròn (PDF vẽ tròn bằng Bézier)."""
    from collections import defaultdict
    key = lambda p: (round(p[0], 3), round(p[1], 3))
    adj = defaultdict(list)
    small = [e for e in ents if e.t == "line"]
    for i, e in enumerate(small):
        adj[key(e.a)].append(i); adj[key(e.b)].append(i)
    used, out = set(), []
    for i, e in enumerate(small):
        if i in used:
            continue
        chain, cur, p = [i], i, key(small[i].b)
        while True:
            nx = [j for j in adj[p] if j not in chain]
            if len(nx) != 1:
                break
            j = nx[0]; chain.append(j)
            p = key(small[j].b) if key(small[j].a) == p else key(small[j].a)
            if len(chain) > 400:
                break
        if len(chain) >= 8 and p == key(small[i].a):
            pts = [small[j].a for j in chain]
            cx = sum(q[0] for q in pts) / len(pts); cy = sum(q[1] for q in pts) / len(pts)
            rs = [math.hypot(q[0] - cx, q[1] - cy) for q in pts]; r = sum(rs) / len(rs)
            if r > 0 and max(abs(x - r) for x in rs) < tol * r + 1e-6:
                used.update(chain); out.append(Ent("circle", c=(cx, cy), r=r, kind=small[i].kind)); continue
    return [e for k, e in enumerate(small) if k not in used] + out + [e for e in ents if e.t != "line"]


def load(path, pdf_page=0, pdf_scale=1.0):
    ext = os.path.splitext(path)[1].lower()
    texts, dims, units = [], [], ""
    if ext == ".pdf":
        ents, texts = _load_pdf(path, pdf_page, pdf_scale)
    else:
        src = _dwg_to_dxf(path) if ext == ".dwg" else path
        try:
            doc = ezdxf.readfile(src)
        except Exception:
            from ezdxf import recover
            doc, _ = recover.readfile(src)
        units = {1: "in", 2: "ft", 4: "mm", 5: "cm", 6: "m"}.get(doc.header.get("$INSUNITS", 0), "")
        ents = []
        _walk(list(doc.modelspace()), ents, texts, dims)
    d = Drawing(path, ents, texts, dims, [], units=units)
    _find_views(d)
    return d


# ---------------------------------------------------------------- views
def _find_views(d: Drawing):
    outl = [e for e in d.ents if e.kind == "outline"]
    if not outl:
        return
    geoms = [e.geom() for e in outl]
    ext = unary_union(geoms).bounds
    tol = max(ext[2] - ext[0], ext[3] - ext[1]) * 1e-4
    # nhóm theo tiếp xúc hình học
    n = len(geoms); par = list(range(n))
    def f(i):
        while par[i] != i:
            par[i] = par[par[i]]; i = par[i]
        return i
    from shapely.strtree import STRtree
    buf = [g.buffer(tol) for g in geoms]
    tree = STRtree(buf)
    for i, g in enumerate(buf):
        for j in tree.query(g, predicate="intersects"):
            j = int(j)
            if j > i:
                a, b = f(i), f(j)
                if a != b:
                    par[a] = b
    comps = {}
    for i in range(n):
        comps.setdefault(f(i), []).append(i)
    groups = []
    for idx in comps.values():
        es = [outl[i] for i in idx]
        raw = unary_union([geoms[i] for i in idx])
        u = shapely.set_precision(raw, tol / 4)
        faces = list(polygonize(u))
        poly = unary_union(faces) if faces else None
        hull = poly if poly is not None and not poly.is_empty else u.convex_hull
        groups.append({"ents": es, "own": list(es), "geom": u, "poly": poly, "bbox": raw.bounds, "area": hull.area if hull else 0})
    # khung tên/viền giấy: nhóm có bao chứa >= 2 nhóm khác và gần như là chữ nhật lớn nhất
    groups.sort(key=lambda g: -(g["bbox"][2] - g["bbox"][0]) * (g["bbox"][3] - g["bbox"][1]))
    def inside(a, b):  # bbox a inside bbox b
        return a[0] >= b[0] - tol and a[1] >= b[1] - tol and a[2] <= b[2] + tol and a[3] <= b[3] + tol
    if len(groups) > 2:
        g0 = groups[0]
        cnt = sum(1 for g in groups[1:] if inside(g["bbox"], g0["bbox"]))
        ext_all = unary_union([g["geom"] for g in groups[1:]]).bounds
        if cnt >= 2 and inside(ext_all, g0["bbox"]):
            d.frame = groups.pop(0)
            # bảng tên thường nằm trong khung: loại nhóm toàn đường thẳng ngang/đứng nằm sát mép dưới-phải khung có chữ
    # gộp nhóm nằm trong vùng kín của nhóm khác (lỗ, rãnh bên trong)
    top = []
    for g in groups:
        host = None
        for h in top:
            if h["poly"] is not None and not h["poly"].is_empty and h["poly"].buffer(tol).contains(g["geom"]):
                host = h; break
        if host:
            host["ents"] += g["ents"]; host["children"].append(g)
        else:
            g["children"] = []; top.append(g)
    others = [e for e in d.ents if e.kind != "outline"]
    views = []
    for k, g in enumerate(sorted(top, key=lambda g: (-round(g["bbox"][3]), g["bbox"][0]))):
        bb = g["bbox"]
        v = View(k, g["ents"], bb, main=g["own"], inner=[ch["own"] for ch in g["children"]])
        outer = g["poly"]
        region = outer
        circ = []
        for e in g["ents"]:
            if e.t == "circle":
                circ.append((e.c[0], e.c[1], e.r))
        if region is not None and not region.is_empty:
            for ch in g["children"]:
                if ch["poly"] is not None and not ch["poly"].is_empty:
                    region = region.difference(ch["poly"])
        v.region = region
        v.circles = [c for c in circ if region is None or not region.buffer(-tol).contains(Point(c[0], c[1]))]
        bx = sbox(*bb).buffer(tol * 5)
        v.hidden = [e for e in others if e.kind == "hidden" and bx.intersects(e.geom())]
        v.centers = [e for e in others if e.kind == "center" and bx.intersects(e.geom())]
        views.append(v)
    # bỏ nhóm quá nhỏ (mũi tên, ký hiệu) so với view lớn nhất
    if views:
        big = max(max(v.size) for v in views)
        views = [v for v in views if max(v.size) > big * 0.04]
        for i, v in enumerate(views):
            v.id = i
    d.views = views


def _dimtxt(x):
    v = f"{x['value']:.4g}" if x.get("value") is not None else ""
    t = (x.get("text") or "").replace("\\P", " ")
    return t.replace("<>", v) if t and t not in ("<>", " ") else v


def texts_near(d: Drawing, view: View, margin=0.35):
    x0, y0, x1, y1 = view.bbox
    m = max(x1 - x0, y1 - y0) * margin
    return [t for t in d.texts + [{"s": _dimtxt(x), "p": x["p"], "dim": x["value"]} for x in d.dims if x["p"]]
            if t["p"] and x0 - m <= t["p"][0] <= x1 + m and y0 - m <= t["p"][1] <= y1 + m]


def report(d: Drawing, png=None):
    """In tóm tắt các hình chiếu + chữ/kích thước gần mỗi view; vẽ ảnh đánh số view."""
    lines = [f"File: {os.path.basename(d.path)} · đơn vị: {d.units or 'không ghi'} · {len(d.ents)} nét · {len(d.dims)} kích thước · {len(d.texts)} chữ"]
    if d.frame:
        lines.append(f"Khung giấy: {fmt_bb(d.frame['bbox'])} (đã loại)")
    for v in d.views:
        w, h = v.size
        holes = ", ".join(f"Ø{2*r:.4g}@({x - v.bbox[0]:.4g},{y - v.bbox[1]:.4g})" for x, y, r in v.circles[:12])
        lines.append(f"View {v.id}: {w:.4g} × {h:.4g}  bbox {fmt_bb(v.bbox)}  nét={len(v.ents)} khuất={len(v.hidden)} tâm={len(v.centers)}"
                     + (f"\n   lỗ tròn: {holes}" if holes else ""))
        tn = [t["s"] for t in texts_near(d, v) if t["s"]]
        if tn:
            lines.append("   chữ/kích thước gần: " + " | ".join(tn[:25]))
    if d.dims:
        lines.append("Mọi kích thước: " + ", ".join(sorted({_dimtxt(x) for x in d.dims})))
    s = "\n".join(lines)
    print(s)
    if png:
        _plot(d, png)
    return s


def fmt_bb(b):
    return f"({b[0]:.4g},{b[1]:.4g})–({b[2]:.4g},{b[3]:.4g})"


def _plot(d, png):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(14, 9), dpi=110)
    col = {"outline": "#111", "hidden": "#c77", "center": "#7a7"}
    for e in d.ents:
        g = e.geom(); x, y = g.xy
        ax.plot(x, y, color=col[e.kind], lw=0.7 if e.kind == "outline" else 0.5, ls="-" if e.kind == "outline" else "--")
    for t in d.texts:
        ax.text(t["p"][0], t["p"][1], t["s"][:30], fontsize=6, color="#36c")
    for v in d.views:
        x0, y0, x1, y1 = v.bbox
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec="#e60", lw=1.2, ls=":"))
        ax.text(x0, y1, f" View {v.id}", color="#e60", fontsize=11, weight="bold", va="bottom")
    ax.set_aspect("equal"); ax.axis("off"); fig.tight_layout(); fig.savefig(png); plt.close(fig)


# ---------------------------------------------------------------- 3D (build123d)
def _b():
    import build123d as b
    return b


def _edges(ents, ox, oy):
    b = _b()
    out = []
    for e in ents:
        if e.t == "line":
            out.append(b.Edge.make_line((e.a[0] - ox, e.a[1] - oy), (e.b[0] - ox, e.b[1] - oy)))
        elif e.t == "circle":
            out.append(b.Edge.make_circle(e.r, b.Plane.XY.offset(0).shift_origin((e.c[0] - ox, e.c[1] - oy, 0))))
        else:
            sw = (e.a1 - e.a0) % 360 or 360
            out.append(b.Edge.make_circle(e.r, b.Plane.XY.shift_origin((e.c[0] - ox, e.c[1] - oy, 0)), start_angle=e.a0, end_angle=e.a0 + sw))
    return out


def _region(ents, ox, oy, tol):
    """Miền phẳng (Face) bao bởi một cụm nét — chịu được nét chia trong, chữ T, giao cắt."""
    b = _b()
    from OCP.BOPAlgo import BOPAlgo_Builder, BOPAlgo_Tools
    from OCP.TopoDS import TopoDS_Compound
    from OCP.BRep import BRep_Builder
    edges = _edges(ents, ox, oy)
    if not edges:
        return None
    bld = BOPAlgo_Builder()
    for e in edges:
        bld.AddArgument(e.wrapped)
    bld.SetFuzzyValue(tol)
    bld.Perform()
    wires, faces = TopoDS_Compound(), TopoDS_Compound()
    bb = BRep_Builder(); bb.MakeCompound(wires); bb.MakeCompound(faces)
    BOPAlgo_Tools.EdgesToWires_s(bld.Shape(), wires, False, 1e-6)
    BOPAlgo_Tools.WiresToFaces_s(wires, faces, 1e-6)
    fs = list(b.Compound(faces).faces())
    if not fs:   # vòng kín đơn giản (vd. một đường tròn): ghép trực tiếp
        for w in b.Wire.combine(edges, tol=max(tol, 1e-6) * 10):
            if w.is_closed:
                try:
                    fs.append(b.Face(w))
                except Exception:
                    pass
    if not fs:
        return None
    f = fs[0]
    for g in fs[1:]:
        f = f.fuse(g)
    try:
        f = f.clean()
    except Exception:
        pass
    return f


def view_faces(view: View, origin=None, holes="auto"):
    """Mặt 2D (trong mặt XY, gốc = góc dưới-trái bbox hoặc `origin`) dựng từ đúng các nét của view, cung/tròn giữ dạng thật.
    holes: 'auto' = trừ mọi cụm nét kín bên trong; 'circles' = chỉ trừ lỗ tròn; 'none'; hoặc list chỉ số cụm trong view.inner."""
    ox, oy = origin if origin else (view.bbox[0], view.bbox[1])
    tol = max(view.size) * 1e-5
    face = _region(view.main, ox, oy, tol)
    if face is None:
        raise RuntimeError(f"View {view.id}: không tạo được miền kín từ đường bao. Kiểm tra nét hở hoặc dựng tay.")
    for k, comp in enumerate(view.inner):
        is_circ = all(e.t == "circle" for e in comp)
        if holes == "none" or (holes == "circles" and not is_circ) or (isinstance(holes, (list, tuple)) and k not in holes):
            continue
        h = _region(comp, ox, oy, tol)
        if h is not None:
            face = face - h
    return face


def extrude_view(view: View, depth, plane="XY", holes="auto", origin=None):
    """Tấm/gá/mặt bích: đùn biên dạng của view theo chiều dày `depth`. plane: 'XY' (nhìn từ trên), 'XZ' (nhìn từ trước), 'YZ' (nhìn cạnh)."""
    b = _b()
    face = view_faces(view, origin, holes)
    pl = {"XY": b.Plane.XY, "XZ": b.Plane.XZ, "YZ": b.Plane.YZ}[plane] if isinstance(plane, str) else plane
    face = pl.from_local_coords(face) if hasattr(pl, "from_local_coords") else face
    return b.extrude(_on_plane(face, pl), amount=depth)


def _on_plane(face, pl):
    b = _b()
    loc = b.Location(pl)
    return face.moved(loc)


def revolve_view(view: View, axis_y=None, holes="none"):
    """Chi tiết tiện: lấy nửa biên dạng phía trên trục (đường tâm nằm ngang ở y=axis_y, mặc định = đường tâm dài nhất của view) rồi xoay 360° quanh trục X."""
    b = _b()
    if axis_y is None:
        hz = [e for e in view.centers if e.t == "line" and abs(e.a[1] - e.b[1]) < 1e-6 * max(view.size) + 1e-9]
        axis_y = max(hz, key=lambda e: abs(e.a[0] - e.b[0])).a[1] if hz else (view.bbox[1] + view.bbox[3]) / 2
    ox = view.bbox[0]
    face = view_faces(view, origin=(ox, axis_y), holes=holes)
    half = face & b.Rectangle(view.size[0] * 4, view.size[1] * 2, align=(b.Align.MIN, b.Align.MIN)).moved(b.Location((-view.size[0], 0)))
    prof = _on_plane(half, b.Plane.XZ)
    return b.revolve(prof, b.Axis.X, 360)


def _place(face, x_dir, y_dir, flip=False):
    """Đặt mặt 2D (toạ độ cục bộ XY) vào không gian: trục u của view -> x_dir, trục v -> y_dir."""
    b = _b()
    if flip:
        face = face.mirror(b.Plane.YZ)
    import numpy as np
    z = tuple(np.cross(x_dir, y_dir))
    return face.moved(b.Location(b.Plane(origin=(0, 0, 0), x_dir=x_dir, z_dir=z)))


def _norm(solid):
    b = _b()
    bb = solid.bounding_box()
    return solid.moved(b.Location((-bb.min.X, -bb.min.Y, -bb.min.Z)))


def intersect_views(front: View = None, top: View = None, side: View = None, holes="circles", flip_top=False, flip_side=False):
    """Chi tiết nhiều bậc / khối phay: đùn biên dạng từng hình chiếu xuyên suốt rồi lấy phần giao (cần >= 2 view).
    Quy ước: front -> mặt XZ (ngang = X, đứng = Z); top -> mặt XY (ngang = X, đứng = Y); side -> mặt YZ (ngang = Y, đứng = Z).
    flip_* lật ngang view nếu hình chiếu đặt theo kiểu nhìn ngược (ví dụ hình chiếu cạnh trái/phải theo góc chiếu thứ nhất)."""
    b = _b()
    sx = front.size[0] if front else top.size[0]
    sz = front.size[1] if front else side.size[1]
    sy = top.size[1] if top else side.size[0]
    solids = []
    if front:
        solids.append(_norm(b.extrude(_place(view_faces(front, holes=holes), (1, 0, 0), (0, 0, 1)), amount=sy)))
    if top:
        solids.append(_norm(b.extrude(_place(view_faces(top, holes=holes), (1, 0, 0), (0, 1, 0), flip_top), amount=sz)))
    if side:
        solids.append(_norm(b.extrude(_place(view_faces(side, holes=holes), (0, 1, 0), (0, 0, 1), flip_side), amount=sx)))
    part = solids[0]
    for s_ in solids[1:]:
        part = part & s_
    return part


def plate(profile_view: View, thickness, holes="auto"):
    """Viết tắt cho tấm phẳng (mặt XY, dày theo +Z)."""
    return extrude_view(profile_view, thickness, "XY", holes)


def sheet_metal(side_pts, thickness, width, bend_radius=None, outside="left"):
    """Chi tiết tấm gấp: `side_pts` = các đỉnh đường gấp nhìn từ cạnh (u ngang, v đứng — theo một mép tấm),
    dày `thickness` về phía `outside` ('left'/'right' theo chiều đi của các điểm), rộng `width`.
    Kết quả nằm trong mặt XZ, chiều rộng theo +Y. Bán kính gấp trong mặc định = thickness. Đục lỗ bằng cut_hole()."""
    b = _b()
    r = thickness if bend_radius is None else bend_radius
    with b.BuildLine() as ln:
        if r > 0 and len(side_pts) > 2:
            b.FilletPolyline(*side_pts, radius=r)
        else:
            b.Polyline(*side_pts)
    w = ln.wire().offset_2d(thickness, kind=b.Kind.ARC, side=b.Side.LEFT if outside == "left" else b.Side.RIGHT, closed=True)
    face = _place(b.Face(w), (1, 0, 0), (0, 0, 1))
    return _norm(b.extrude(face, amount=width))


def cut_hole(part, center, diameter, direction=(0, 0, 1), depth=None):
    """Khoan lỗ trụ Ø`diameter` tại `center` (x,y,z) theo hướng `direction`; depth=None là khoan xuyên."""
    b = _b()
    L = depth or 10 * max(part.bounding_box().size)
    cyl = b.Cylinder(diameter / 2, L * 2 if depth is None else L, align=(b.Align.CENTER, b.Align.CENTER, b.Align.CENTER if depth is None else b.Align.MAX))
    pl = b.Plane(origin=center, z_dir=direction)
    return part - cyl.moved(b.Location(pl))


# ---------------------------------------------------------------- export
def export(part, base, title=""):
    """Xuất base.step, base.stl và base_3d.png (4 góc nhìn). Trả về dict thông số khối."""
    b = _b()
    b.export_step(part, base + ".step")
    b.export_stl(part, base + ".stl", tolerance=0.02)
    bb = part.bounding_box()
    info = {"step": base + ".step", "stl": base + ".stl", "png": base + "_3d.png",
            "size": [round(bb.size.X, 4), round(bb.size.Y, 4), round(bb.size.Z, 4)],
            "volume": round(part.volume, 3), "faces": len(part.faces())}
    _render(part, base + "_3d.png", title or os.path.basename(base), info)
    return info


def _render(part, png, title, info):
    import numpy as np
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    vs, tris = part.tessellate(0.05 * max(info["size"]) / 50, 0.3)
    V = np.array([[v.X, v.Y, v.Z] for v in vs]); T = np.array(tris)
    fig = plt.figure(figsize=(12, 9), dpi=110)
    fig.suptitle(f"{title} · {info['size'][0]:g} × {info['size'][1]:g} × {info['size'][2]:g}", fontsize=13)
    light = np.array([0.4, -0.5, 0.8]); light /= np.linalg.norm(light)
    tri = V[T]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); n /= (np.linalg.norm(n, axis=1)[:, None] + 1e-12)
    shade = 0.35 + 0.65 * np.abs(n @ light)
    colors = np.c_[0.45 * shade, 0.6 * shade, 0.85 * shade, np.ones_like(shade)]
    for k, (el, az, name) in enumerate([(28, -55, "Isometric"), (0, -90, "Trước (XZ)"), (90, -90, "Trên (XY)"), (0, 0, "Cạnh (YZ)")]):
        ax = fig.add_subplot(2, 2, k + 1, projection="3d")
        ax.add_collection3d(Poly3DCollection(tri, facecolors=colors, edgecolor="none", linewidths=0))
        mn, mx = V.min(0), V.max(0); c = (mn + mx) / 2; r = (mx - mn).max() / 2
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(el, az); ax.set_title(name); ax.set_axis_off()
    fig.tight_layout(); fig.savefig(png); plt.close(fig)


# ---------------------------------------------------------------- Onshape
def onshape_upload(step_path, document_name=None, did=None, wid=None):
    """Đẩy file STEP lên Onshape và tự chuyển thành Part Studio.
    Cần biến môi trường ONSHAPE_ACCESS_KEY, ONSHAPE_SECRET_KEY (tạo ở https://dev-portal.onshape.com/keys)
    và quyền mạng tới cad.onshape.com. Không truyền did/wid thì tạo document mới."""
    import base64, urllib.request, uuid
    ak, sk = os.environ.get("ONSHAPE_ACCESS_KEY"), os.environ.get("ONSHAPE_SECRET_KEY")
    if not ak or not sk:
        raise RuntimeError("Thiếu ONSHAPE_ACCESS_KEY / ONSHAPE_SECRET_KEY.")
    base = os.environ.get("ONSHAPE_URL", "https://cad.onshape.com")
    auth = "Basic " + base64.b64encode(f"{ak}:{sk}".encode()).decode()

    def call(method, url, body=None, ctype="application/json"):
        req = urllib.request.Request(base + url, data=body, method=method, headers={"Authorization": auth, "Accept": "application/json", "Content-Type": ctype})
        with urllib.request.urlopen(req, timeout=120) as r:
            return json.loads(r.read() or b"{}")

    if not did:
        doc = call("POST", "/api/v6/documents", json.dumps({"name": document_name or os.path.splitext(os.path.basename(step_path))[0], "isPublic": False}).encode())
        did, wid = doc["id"], doc["defaultWorkspace"]["id"]
    bnd = uuid.uuid4().hex
    data = open(step_path, "rb").read()
    parts = [f'--{bnd}\r\nContent-Disposition: form-data; name="{k}"\r\n\r\n{v}\r\n'.encode() for k, v in
             {"translate": "true", "flattenAssemblies": "false", "formatName": "STEP", "encodedFilename": os.path.basename(step_path), "allowFaultyParts": "true"}.items()]
    body = b"".join(parts) + f'--{bnd}\r\nContent-Disposition: form-data; name="file"; filename="{os.path.basename(step_path)}"\r\nContent-Type: application/octet-stream\r\n\r\n'.encode() + data + f"\r\n--{bnd}--\r\n".encode()
    r = call("POST", f"/api/v6/blobelements/d/{did}/w/{wid}", body, f"multipart/form-data; boundary={bnd}")
    return {"document": f"{base}/documents/{did}/w/{wid}", "translation": r.get("translationId"), "element": r.get("id")}


# ---------------------------------------------------------------- dựng tay (cho ảnh chụp / PDF scan)
def profile_face(points, arcs=None, holes=()):
    """Mặt 2D từ danh sách đỉnh (đọc từ ảnh/bản vẽ scan). arcs: {chỉ_số_đỉnh: bán_kính_bo}. holes: [(x, y, đường_kính)]."""
    b = _b()
    with b.BuildSketch() as sk:
        with b.BuildLine():
            if arcs:
                rr = sorted(set(arcs.values()))
                b.FilletPolyline(*points, radius=rr[0], close=True) if len(rr) == 1 and len(arcs) == len(points) else b.Polyline(*points, close=True)
            else:
                b.Polyline(*points, close=True)
        b.make_face()
        if arcs and not (len(set(arcs.values())) == 1 and len(arcs) == len(points)):
            for i, r in arcs.items():
                vs = [v for v in sk.vertices() if abs(v.X - points[i][0]) < 1e-6 and abs(v.Y - points[i][1]) < 1e-6]
                if vs:
                    b.fillet(vs, radius=r)
        for x, y, dd in holes:
            with b.Locations((x, y)):
                b.Circle(dd / 2, mode=b.Mode.SUBTRACT)
    return sk.sketch.faces()[0]


def extrude_face(face, depth, plane="XY"):
    """Đùn một mặt 2D (vd. từ profile_face) theo mặt phẳng 'XY' | 'XZ' | 'YZ'."""
    b = _b()
    axes = {"XY": ((1, 0, 0), (0, 1, 0)), "XZ": ((1, 0, 0), (0, 0, 1)), "YZ": ((0, 1, 0), (0, 0, 1))}[plane]
    return _norm(b.extrude(_place(face, *axes), amount=depth))
