"""SVG loading (flatten to polylines) and bbox-fit coordinate mapping for Stage C."""
from svgpathtools import svg2paths

CURVE_SAMPLES = 20  # points sampled per curved segment when flattening to a polyline


def load_svg_polylines(file_path):
    """Parses basic path/line/polyline geometry out of an SVG (ignores fills, text,
    styling). Returns (polylines, bbox):
      polylines - list of [(x, y), ...] point lists, SVG user-space (y grows down)
      bbox      - (xmin, ymin, xmax, ymax) covering every point across every polyline
    """
    paths, _attributes = svg2paths(file_path)

    polylines = []
    xmin = ymin = float("inf")
    xmax = ymax = float("-inf")

    for path in paths:
        for sub in path.continuous_subpaths():
            points = []
            for segment in sub:
                is_line = type(segment).__name__ == "Line"
                samples = 2 if is_line else CURVE_SAMPLES
                for i in range(samples):
                    t = i / (samples - 1)
                    pt = segment.point(t)
                    points.append((pt.real, pt.imag))
            if len(points) < 2:
                continue
            polylines.append(points)
            for x, y in points:
                xmin = min(xmin, x)
                ymin = min(ymin, y)
                xmax = max(xmax, x)
                ymax = max(ymax, y)

    if not polylines:
        raise ValueError("no drawable path/line/polyline geometry found in SVG")

    return polylines, (xmin, ymin, xmax, ymax)


def fit_bbox_transform(bbox, target_x, target_y, target_w, target_h, flip_y=False):
    """Builds a pair of affine mapping functions that fit `bbox` (xmin, ymin, xmax,
    ymax) into the rectangle (target_x, target_y, target_w, target_h), preserving
    aspect ratio and centering it. If flip_y, source y-down is mapped to target
    y-up (used for SVG-space -> physical workspace mm).

    Returns (to_target, to_source), each a function (x, y) -> (x, y).
    """
    xmin, ymin, xmax, ymax = bbox
    src_w = xmax - xmin
    src_h = ymax - ymin
    scale = min(target_w / src_w, target_h / src_h) if src_w > 0 and src_h > 0 else 1.0

    disp_w = src_w * scale
    disp_h = src_h * scale
    off_x = target_x + (target_w - disp_w) / 2
    off_y = target_y + (target_h - disp_h) / 2

    def to_target(x, y):
        tx = off_x + (x - xmin) * scale
        ny = (y - ymin) * scale
        ty = (off_y + disp_h - ny) if flip_y else (off_y + ny)
        return tx, ty

    def to_source(tx, ty):
        x = xmin + (tx - off_x) / scale
        ny = (off_y + disp_h - ty) if flip_y else (ty - off_y)
        y = ymin + ny / scale
        return x, y

    return to_target, to_source


def anchor_bbox_transform(bbox, scale_w, scale_h, origin, anchor_frac):
    """Builds a forward mapping (x, y) SVG-space (y grows down) -> workspace mm
    (y grows up). `bbox` is sized (preserving aspect ratio) to fit within a
    scale_w x scale_h rectangle, then positioned so the point at `anchor_frac`
    (u, v fractions of the scaled bbox; u: left->right, v: bottom->top) lands
    exactly at the physical `origin` (x, y) mm - not centered like
    fit_bbox_transform, anchored at a specific point instead.
    """
    xmin, ymin, xmax, ymax = bbox
    src_w = xmax - xmin
    src_h = ymax - ymin
    scale = min(scale_w / src_w, scale_h / src_h) if src_w > 0 and src_h > 0 else 1.0
    disp_w = src_w * scale
    disp_h = src_h * scale

    ox, oy = origin
    u, v = anchor_frac

    def to_workspace(x, y):
        local_x = (x - xmin) * scale
        local_y = (ymax - y) * scale  # flip: SVG y-down -> workspace-up local coords
        return (ox + local_x - u * disp_w, oy + local_y - v * disp_h)

    return to_workspace
