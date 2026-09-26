import json
import math
import os
import struct


ROOT = os.path.dirname(os.path.abspath(__file__))
MODELS = [
    ("BumbleBot Base Plate", "bumblebot/Base_Plate.stl", 1800),
    ("Skycam Camera Front", "skycam_camera_mount/Skycam-camera-front.stl", 2200),
    ("Skycam Camera Back", "skycam_camera_mount/Skycam-camera-back.stl", 1800),
    ("Skycam Camera Pan", "skycam_camera_mount/Skycam-camera-pan.stl", 1200),
    ("Skycam Camera Tilt", "skycam_camera_mount/Skycam-camera-tilt.stl", 1200),
    ("Skycam Pan Tilt Top", "skycam_camera_mount/Skycam-pan-tilt-top.stl", 800),
]


def read_stl(path):
    data = open(path, "rb").read()
    if len(data) >= 84:
        tri_count = struct.unpack_from("<I", data, 80)[0]
        expected = 84 + tri_count * 50
        if expected == len(data):
            tris = []
            offset = 84
            for _ in range(tri_count):
                offset += 12
                pts = []
                for _ in range(3):
                    pts.append(struct.unpack_from("<fff", data, offset))
                    offset += 12
                offset += 2
                tris.append(pts)
            return tris

    text = data.decode("utf-8", errors="ignore")
    verts = []
    tris = []
    for line in text.splitlines():
        parts = line.strip().split()
        if len(parts) == 4 and parts[0] == "vertex":
            verts.append(tuple(float(v) for v in parts[1:]))
            if len(verts) == 3:
                tris.append(verts)
                verts = []
    return tris


def simplify(tris, limit):
    if len(tris) <= limit:
        return tris
    step = max(1, math.ceil(len(tris) / limit))
    return tris[::step][:limit]


def normalize(tris):
    pts = [p for tri in tris for p in tri]
    xs, ys, zs = zip(*pts)
    cx = (min(xs) + max(xs)) / 2
    cy = (min(ys) + max(ys)) / 2
    cz = (min(zs) + max(zs)) / 2
    extent = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)) or 1.0
    out = []
    for tri in tris:
        flat = []
        for x, y, z in tri:
            flat.extend(
                [
                    round((x - cx) / extent, 5),
                    round((y - cy) / extent, 5),
                    round((z - cz) / extent, 5),
                ]
            )
        out.append(flat)
    return out, extent


def build_model_data():
    models = []
    for title, rel_path, limit in MODELS:
        path = os.path.join(ROOT, rel_path)
        original = read_stl(path)
        sampled = simplify(original, limit)
        triangles, extent = normalize(sampled)
        models.append(
            {
                "title": title,
                "path": rel_path,
                "originalTriangles": len(original),
                "sampledTriangles": len(sampled),
                "extent": round(extent, 4),
                "triangles": triangles,
            }
        )
    return models


MODEL_DATA = json.dumps(build_model_data(), separators=(",", ":"))

HTML_TEMPLATE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>AMR STL 360 Preview</title>
  <style>
    :root {{
      color-scheme: light;
      --border: #d8dde3;
      --text: #17202a;
      --muted: #5d6b78;
      --surface: #f6f7f8;
      --button: #ffffff;
      --button-hover: #edf2f6;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #ffffff;
      color: var(--text);
    }}
    header {{
      padding: 18px 22px 10px;
      border-bottom: 1px solid var(--border);
    }}
    h1 {{
      font-size: 20px;
      margin: 0 0 6px;
      font-weight: 650;
      letter-spacing: 0;
    }}
    p {{
      margin: 0;
      color: var(--muted);
      font-size: 13px;
      line-height: 1.4;
    }}
    main {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(340px, 1fr));
      gap: 14px;
      padding: 16px;
    }}
    .model {{
      border: 1px solid var(--border);
      border-radius: 8px;
      padding: 12px;
      background: #fff;
      min-width: 0;
    }}
    .model:focus-within {{
      outline: 2px solid #7aa7d9;
      outline-offset: 2px;
    }}
    .title {{
      font-size: 14px;
      font-weight: 650;
      margin-bottom: 3px;
    }}
    .path {{
      font-size: 11px;
      color: var(--muted);
      margin-bottom: 10px;
      overflow-wrap: anywhere;
      line-height: 1.35;
    }}
    .toolbar {{
      display: flex;
      gap: 6px;
      flex-wrap: wrap;
      margin-bottom: 10px;
    }}
    button {{
      appearance: none;
      border: 1px solid #cbd3dc;
      background: var(--button);
      color: #24313d;
      border-radius: 6px;
      font-size: 12px;
      line-height: 1;
      padding: 7px 9px;
      cursor: pointer;
    }}
    button:hover {{ background: var(--button-hover); }}
    button[aria-pressed="true"] {{
      background: #223142;
      color: #fff;
      border-color: #223142;
    }}
    .viewer {{
      width: 100%;
      aspect-ratio: 5 / 4;
      display: block;
      border: 1px solid #edf0f2;
      border-radius: 8px;
      background: var(--surface);
      cursor: grab;
      touch-action: none;
      user-select: none;
    }}
    .viewer.dragging {{ cursor: grabbing; }}
    .hint {{
      margin-top: 8px;
      color: var(--muted);
      font-size: 11px;
    }}
    dialog {{
      width: min(1100px, calc(100vw - 28px));
      border: 1px solid var(--border);
      border-radius: 8px;
      padding: 0;
      background: #fff;
      color: var(--text);
    }}
    dialog::backdrop {{ background: rgba(12, 18, 24, 0.45); }}
    .modal-head {{
      display: flex;
      align-items: start;
      justify-content: space-between;
      gap: 12px;
      padding: 14px 16px 10px;
      border-bottom: 1px solid var(--border);
    }}
    .modal-title {{
      font-weight: 650;
      font-size: 16px;
      margin-bottom: 3px;
    }}
    .modal-body {{ padding: 14px 16px 16px; }}
    .modal-viewer {{ aspect-ratio: 16 / 10; }}
  </style>
</head>
<body>
  <header>
    <h1>AMR STL 360 Preview</h1>
    <p>Filled SVG previews generated from local STL files. Drag to rotate, wheel to zoom, click a model for a larger viewer.</p>
  </header>
  <main id="gallery"></main>

  <dialog id="modal">
    <div class="modal-head">
      <div>
        <div class="modal-title" id="modal-title"></div>
        <p id="modal-path"></p>
      </div>
      <button id="modal-close" type="button">Close</button>
    </div>
    <div class="modal-body">
      <div class="toolbar" id="modal-toolbar"></div>
      <svg id="modal-viewer" class="viewer modal-viewer" viewBox="0 0 800 500" role="img"></svg>
      <div class="hint">Drag to rotate. Use mouse wheel to zoom. Filled surfaces are depth sorted rear-first; back faces are culled when winding allows it.</div>
    </div>
  </dialog>

  <script>
    const MODELS = __MODEL_DATA__;
    const gallery = document.getElementById('gallery');
    const modal = document.getElementById('modal');
    const modalTitle = document.getElementById('modal-title');
    const modalPath = document.getElementById('modal-path');
    const modalToolbar = document.getElementById('modal-toolbar');
    const modalViewer = document.getElementById('modal-viewer');
    const modalClose = document.getElementById('modal-close');
    const viewers = new Set();
    let activeModalViewer = null;

    const svgNS = 'http://www.w3.org/2000/svg';

    function createSvg(tag, attrs = {{}}) {{
      const el = document.createElementNS(svgNS, tag);
      for (const [key, value] of Object.entries(attrs)) el.setAttribute(key, value);
      return el;
    }}

    function rotatePoint(x, y, z, yaw, pitch) {{
      const cy = Math.cos(yaw), sy = Math.sin(yaw);
      const cp = Math.cos(pitch), sp = Math.sin(pitch);
      const x1 = x * cy - y * sy;
      const y1 = x * sy + y * cy;
      const z1 = z;
      return [x1, y1 * cp - z1 * sp, y1 * sp + z1 * cp];
    }}

    function cross(ax, ay, az, bx, by, bz) {{
      return [ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx];
    }}

    function triangleRecord(tri, state, width, height) {{
      const p0 = rotatePoint(tri[0], tri[1], tri[2], state.yaw, state.pitch);
      const p1 = rotatePoint(tri[3], tri[4], tri[5], state.yaw, state.pitch);
      const p2 = rotatePoint(tri[6], tri[7], tri[8], state.yaw, state.pitch);
      const ux = p1[0] - p0[0], uy = p1[1] - p0[1], uz = p1[2] - p0[2];
      const vx = p2[0] - p0[0], vy = p2[1] - p0[1], vz = p2[2] - p0[2];
      const n = cross(ux, uy, uz, vx, vy, vz);
      const area2 = Math.hypot(n[0], n[1], n[2]);
      if (area2 < 0.000001) return null;

      const scale = Math.min(width, height) * 0.82 * state.zoom;
      const pts = [p0, p1, p2].map((p) => [
        width / 2 + p[0] * scale + state.panX,
        height / 2 - p[1] * scale + state.panY,
        p[2],
      ]);
      const depth = (p0[2] + p1[2] + p2[2]) / 3;
      const light = [0.22, -0.35, 0.91];
      const norm = [n[0] / area2, n[1] / area2, n[2] / area2];
      const facing = norm[2];
      const lambert = Math.max(0, norm[0] * light[0] + norm[1] * light[1] + Math.abs(norm[2]) * light[2]);
      const shade = Math.round(190 + lambert * 48 + Math.max(-18, Math.min(18, depth * 20)));
      return {{ pts, depth, facing, shade: Math.max(176, Math.min(238, shade)) }};
    }}

    function render(viewer) {{
      const {{ svg, model, state }} = viewer;
      const width = Number(svg.viewBox.baseVal.width || 360);
      const height = Number(svg.viewBox.baseVal.height || 288);
      const bg = createSvg('rect', {{
        x: 0, y: 0, width, height, rx: 8,
        fill: '#f6f7f8'
      }});

      const records = [];
      let frontFacing = 0;
      for (const tri of model.triangles) {{
        const rec = triangleRecord(tri, state, width, height);
        if (!rec) continue;
        if (rec.facing > 0) frontFacing += 1;
        records.push(rec);
      }}

      const cullingLooksValid = frontFacing > records.length * 0.08 && frontFacing < records.length * 0.92;
      const visible = records
        .filter((rec) => !cullingLooksValid || rec.facing > 0)
        .sort((a, b) => a.depth - b.depth);

      const frag = document.createDocumentFragment();
      frag.appendChild(bg);
      for (const rec of visible) {{
        const points = rec.pts.map((p) => `${{p[0].toFixed(1)}},${{p[1].toFixed(1)}}`).join(' ');
        frag.appendChild(createSvg('polygon', {{
          points,
          fill: `rgb(${{rec.shade}},${{rec.shade}},${{rec.shade}})`,
          stroke: '#202832',
          'stroke-width': width > 500 ? 0.7 : 0.55,
          'stroke-opacity': 0.62,
          'fill-opacity': 0.96,
          'vector-effect': 'non-scaling-stroke',
        }}));
      }}
      svg.replaceChildren(frag);
    }}

    function fit(viewer) {{
      viewer.state.zoom = 1;
      viewer.state.panX = 0;
      viewer.state.panY = 0;
      render(viewer);
    }}

    function reset(viewer) {{
      viewer.state.yaw = -0.65;
      viewer.state.pitch = -0.78;
      viewer.state.zoom = 1;
      viewer.state.panX = 0;
      viewer.state.panY = 0;
      render(viewer);
    }}

    function makeToolbar(viewer) {{
      const toolbar = document.createElement('div');
      toolbar.className = 'toolbar';

      const fitBtn = document.createElement('button');
      fitBtn.type = 'button';
      fitBtn.textContent = 'Fit Model';
      fitBtn.addEventListener('click', (event) => {{ event.stopPropagation(); fit(viewer); }});

      const resetBtn = document.createElement('button');
      resetBtn.type = 'button';
      resetBtn.textContent = 'Reset View';
      resetBtn.addEventListener('click', (event) => {{ event.stopPropagation(); reset(viewer); }});

      const autoBtn = document.createElement('button');
      autoBtn.type = 'button';
      autoBtn.textContent = 'Auto Rotate';
      autoBtn.setAttribute('aria-pressed', String(viewer.state.auto));
      autoBtn.addEventListener('click', (event) => {{
        event.stopPropagation();
        viewer.state.auto = !viewer.state.auto;
        autoBtn.setAttribute('aria-pressed', String(viewer.state.auto));
      }});
      viewer.autoButton = autoBtn;

      toolbar.append(fitBtn, resetBtn, autoBtn);
      return toolbar;
    }}

    function attachPointerControls(viewer, openLargeOnClick) {{
      const {{ svg, state }} = viewer;
      let dragging = false;
      let moved = false;
      let lastX = 0;
      let lastY = 0;

      svg.addEventListener('pointerdown', (event) => {{
        dragging = true;
        moved = false;
        lastX = event.clientX;
        lastY = event.clientY;
        state.auto = false;
        if (viewer.autoButton) viewer.autoButton.setAttribute('aria-pressed', 'false');
        svg.classList.add('dragging');
        svg.setPointerCapture(event.pointerId);
      }});

      svg.addEventListener('pointermove', (event) => {{
        if (!dragging) return;
        const dx = event.clientX - lastX;
        const dy = event.clientY - lastY;
        if (Math.abs(dx) + Math.abs(dy) > 3) moved = true;
        state.yaw += dx * 0.012;
        state.pitch = Math.max(-1.45, Math.min(1.45, state.pitch + dy * 0.012));
        lastX = event.clientX;
        lastY = event.clientY;
        render(viewer);
      }});

      svg.addEventListener('pointerup', (event) => {{
        dragging = false;
        svg.classList.remove('dragging');
        try {{ svg.releasePointerCapture(event.pointerId); }} catch (_err) {{}}
        if (!moved && openLargeOnClick) openModal(viewer.modelIndex);
      }});

      svg.addEventListener('wheel', (event) => {{
        event.preventDefault();
        state.zoom *= event.deltaY < 0 ? 1.12 : 0.89;
        state.zoom = Math.max(0.28, Math.min(5, state.zoom));
        render(viewer);
      }}, {{ passive: false }});
    }}

    function createViewer(model, modelIndex, svg, openLargeOnClick) {{
      const viewer = {{
        model,
        modelIndex,
        svg,
        state: {{ yaw: -0.65, pitch: -0.78, zoom: 1, panX: 0, panY: 0, auto: true }},
        autoButton: null,
      }};
      attachPointerControls(viewer, openLargeOnClick);
      render(viewer);
      viewers.add(viewer);
      return viewer;
    }}

    function renderGallery() {{
      MODELS.forEach((model, index) => {{
        const card = document.createElement('section');
        card.className = 'model';

        const title = document.createElement('div');
        title.className = 'title';
        title.textContent = model.title;

        const path = document.createElement('div');
        path.className = 'path';
        path.textContent = `${{model.path}} · ${{model.sampledTriangles}} / ${{model.originalTriangles}} triangles sampled`;

        const svg = createSvg('svg', {{
          class: 'viewer',
          viewBox: '0 0 360 288',
          role: 'img',
          'aria-label': `${{model.title}} STL preview`,
          tabindex: '0',
        }});

        const viewer = createViewer(model, index, svg, true);
        const toolbar = makeToolbar(viewer);
        card.append(title, path, toolbar, svg);
        const hint = document.createElement('div');
        hint.className = 'hint';
        hint.textContent = 'Drag rotate · Wheel zoom · Click opens larger view';
        card.appendChild(hint);
        gallery.appendChild(card);
      }});
    }}

    function openModal(index) {{
      const model = MODELS[index];
      modalTitle.textContent = model.title;
      modalPath.textContent = `${{model.path}} · ${{model.sampledTriangles}} / ${{model.originalTriangles}} triangles sampled`;
      modalToolbar.replaceChildren();
      modalViewer.replaceChildren();
      activeModalViewer = createViewer(model, index, modalViewer, false);
      activeModalViewer.state.auto = false;
      activeModalViewer.autoButton = null;
      modalToolbar.appendChild(makeToolbar(activeModalViewer));
      modal.showModal();
      reset(activeModalViewer);
    }}

    modalClose.addEventListener('click', () => modal.close());
    modal.addEventListener('close', () => {{
      if (activeModalViewer) viewers.delete(activeModalViewer);
      activeModalViewer = null;
    }});

    function tick() {{
      for (const viewer of viewers) {{
        if (viewer.state.auto) {{
          viewer.state.yaw += 0.018;
          render(viewer);
        }}
      }}
      requestAnimationFrame(tick);
    }}

    renderGallery();
    requestAnimationFrame(tick);
  </script>
</body>
</html>
"""

out_path = os.path.join(ROOT, "stl_360_preview.html")
with open(out_path, "w", encoding="utf-8") as handle:
    html_doc = HTML_TEMPLATE.replace("__MODEL_DATA__", MODEL_DATA)
    html_doc = html_doc.replace("{{", "{").replace("}}", "}")
    handle.write(html_doc)
print(out_path)
