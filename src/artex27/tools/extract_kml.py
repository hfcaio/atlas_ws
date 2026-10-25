#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
extract_kml.py - extrai features operacionais de um KML do ARTEX.

Sem dependencias alem da stdlib. Pensado para ser re-executado quando sair
uma versao nova do KML (v174, v175, ...).

Uso tipico
----------
  # listar tudo
  ./extract_kml.py "../../ARTEX 26 Geo v173.kml" --list

  # so o que interessa a missao (regex, case-insensitive)
  ./extract_kml.py "../../ARTEX 26 Geo v173.kml" \
      --name 'UAS TO/LD Area 11|^Runway$|LP-D25|LP-TRA56|Boundary|Air Ops|GCS Container|Certification Box'

  # saida JSON (para alimentar build_mission.py / validate_mission.py)
  ./extract_kml.py "../../ARTEX 26 Geo v173.kml" --name 'UAS TO/LD Area 11' --json features.json

  # KMLs reduzidos, um poligono por arquivo (formato que o QGC importa em
  # Plan > Fence > Load KML/SHP)
  ./extract_kml.py "../../ARTEX 26 Geo v173.kml" --name 'UAS TO/LD Area 11|LP-D25|LP-TRA56' \
      --kml-out kml/

Notas de geometria
------------------
- O KML guarda coordenadas como lon,lat,alt (nessa ordem). As altitudes do
  ARTEX estao em metros AMSL.
- Areas circulares (ex.: "UAS TO/LD Area 11 TF") vem no KML como LineString
  com ~73 vertices. O script faz um ajuste de circulo por minimos quadrados e
  reporta centro + raio quando o residuo e pequeno.
"""

import argparse
import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

KNS = "{http://www.opengis.net/kml/2.2}"
R_EARTH = 6378137.0  # WGS84 semi-eixo maior, suficiente para escala local


# ---------------------------------------------------------------- geometria

def local_proj(lat0, lon0):
    """Projecao equiretangular local (metros). Valida para alguns km."""
    coslat = math.cos(math.radians(lat0))

    def fwd(lat, lon):
        return (math.radians(lon - lon0) * R_EARTH * coslat,
                math.radians(lat - lat0) * R_EARTH)

    def inv(e, n):
        return (lat0 + math.degrees(n / R_EARTH),
                lon0 + math.degrees(e / (R_EARTH * coslat)))

    return fwd, inv


def distance_m(a, b):
    """Distancia entre (lat, lon) e (lat, lon) em metros."""
    fwd, _ = local_proj(a[0], a[1])
    e, n = fwd(b[0], b[1])
    return math.hypot(e, n)


def bearing_deg(a, b):
    """Azimute verdadeiro de a para b, em graus [0, 360)."""
    la1, lo1 = math.radians(a[0]), math.radians(a[1])
    la2, lo2 = math.radians(b[0]), math.radians(b[1])
    y = math.sin(lo2 - lo1) * math.cos(la2)
    x = math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(lo2 - lo1)
    return math.degrees(math.atan2(y, x)) % 360.0


def offset_latlon(origin, bearing, dist_m):
    """Ponto a `dist_m` metros de `origin` no azimute `bearing` (graus)."""
    fwd, inv = local_proj(origin[0], origin[1])
    br = math.radians(bearing)
    return inv(math.sin(br) * dist_m, math.cos(br) * dist_m)


def _solve3(A, b):
    M = [row[:] + [bb] for row, bb in zip(A, b)]
    for i in range(3):
        p = max(range(i, 3), key=lambda r: abs(M[r][i]))
        M[i], M[p] = M[p], M[i]
        if abs(M[i][i]) < 1e-12:
            return None
        for r in range(3):
            if r != i:
                f = M[r][i] / M[i][i]
                for c in range(i, 4):
                    M[r][c] -= f * M[i][c]
    return [M[i][3] / M[i][i] for i in range(3)]


def fit_circle(points):
    """Ajuste de circulo por minimos quadrados.

    points: lista de (lat, lon). Retorna (lat_c, lon_c, raio_m, residuo_max_m)
    ou None se o ajuste falhar.
    """
    if len(points) < 8:
        return None
    lat0 = sum(p[0] for p in points) / len(points)
    lon0 = sum(p[1] for p in points) / len(points)
    fwd, inv = local_proj(lat0, lon0)
    P = [fwd(la, lo) for la, lo in points]
    n = len(P)
    Sx = sum(x for x, _ in P)
    Sy = sum(y for _, y in P)
    A = [[sum(2 * x * x for x, _ in P), sum(2 * x * y for x, y in P), Sx],
         [sum(2 * x * y for x, y in P), sum(2 * y * y for _, y in P), Sy],
         [2 * Sx, 2 * Sy, float(n)]]
    b = [sum(x * (x * x + y * y) for x, y in P),
         sum(y * (x * x + y * y) for x, y in P),
         sum(x * x + y * y for x, y in P)]
    sol = _solve3(A, b)
    if sol is None:
        return None
    cx, cy, c = sol
    inner = c + cx * cx + cy * cy
    if inner <= 0:
        return None
    rad = math.sqrt(inner)
    resid = max(abs(math.hypot(x - cx, y - cy) - rad) for x, y in P)
    lat_c, lon_c = inv(cx, cy)
    return (lat_c, lon_c, rad, resid)


# ---------------------------------------------------------------- parsing

def parse_coords(text):
    """'lon,lat[,alt] lon,lat[,alt] ...' -> [(lat, lon, alt), ...]"""
    out = []
    for tok in text.split():
        parts = tok.split(",")
        if len(parts) < 2:
            continue
        lon = float(parts[0])
        lat = float(parts[1])
        alt = float(parts[2]) if len(parts) > 2 else 0.0
        out.append((lat, lon, alt))
    return out


def extract(kml_path):
    """Retorna lista de dicts, um por geometria encontrada."""
    root = ET.parse(kml_path).getroot()
    feats = []
    for pm in root.iter(KNS + "Placemark"):
        nel = pm.find(KNS + "name")
        name = (nel.text or "").strip() if nel is not None and nel.text else "(sem nome)"
        for gtag in ("Point", "LineString", "LinearRing", "Polygon"):
            for geom in pm.iter(KNS + gtag):
                if gtag == "Polygon":
                    continue  # tratado pelo LinearRing interno
                cel = geom.find(KNS + "coordinates")
                if cel is None or not cel.text:
                    continue
                pts = parse_coords(cel.text)
                if not pts:
                    continue
                alts = sorted({round(p[2], 1) for p in pts})
                lats = [p[0] for p in pts]
                lons = [p[1] for p in pts]
                f = {
                    "name": name,
                    "geometry": gtag,
                    "n_points": len(pts),
                    "alt_m_amsl": alts[0] if len(alts) == 1 else alts,
                    "centroid": [round(sum(lats) / len(lats), 7),
                                 round(sum(lons) / len(lons), 7)],
                    "bbox": [round(min(lats), 7), round(min(lons), 7),
                             round(max(lats), 7), round(max(lons), 7)],
                    "points": [[round(p[0], 7), round(p[1], 7), p[2]] for p in pts],
                }
                if gtag in ("LineString", "LinearRing") and len(pts) >= 8:
                    fc = fit_circle([(p[0], p[1]) for p in pts])
                    # residuo < 1% do raio => e um circulo de verdade
                    if fc and fc[3] < max(2.0, 0.01 * fc[2]):
                        f["circle"] = {
                            "center": [round(fc[0], 7), round(fc[1], 7)],
                            "radius_m": round(fc[2], 2),
                            "fit_residual_m": round(fc[3], 3),
                        }
                feats.append(f)
    return feats


def dedupe(feats):
    """O KML do ARTEX repete a mesma feature em varias pastas. Mantem a 1a."""
    seen = set()
    out = []
    for f in feats:
        key = (f["name"], f["geometry"], f["n_points"],
               tuple(f["centroid"]), str(f["alt_m_amsl"]))
        if key in seen:
            continue
        seen.add(key)
        out.append(f)
    return out


# ---------------------------------------------------------------- saidas

def slug(name):
    s = re.sub(r"[^A-Za-z0-9]+", "_", name).strip("_").lower()
    return s or "feature"


KML_TMPL = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
  <name>{docname}</name>
{body}</Document>
</kml>
"""

KML_POLY = """  <Placemark>
    <name>{name}</name>
    <description>{desc}</description>
    <Polygon>
      <tessellate>1</tessellate>
      <outerBoundaryIs><LinearRing><coordinates>
{coords}
      </coordinates></LinearRing></outerBoundaryIs>
    </Polygon>
  </Placemark>
"""

KML_POINT = """  <Placemark>
    <name>{name}</name>
    <description>{desc}</description>
    <Point><coordinates>{coords}</coordinates></Point>
  </Placemark>
"""


def feature_to_kml_body(f):
    desc = "geometry=%s alt_amsl=%s" % (f["geometry"], f["alt_m_amsl"])
    if f["geometry"] == "Point":
        p = f["points"][0]
        return KML_POINT.format(name=f["name"], desc=desc,
                                coords="%.7f,%.7f,%.1f" % (p[1], p[0], p[2]))
    pts = list(f["points"])
    if pts[0][:2] != pts[-1][:2]:
        pts.append(pts[0])
    coords = "\n".join("        %.7f,%.7f,%.1f" % (p[1], p[0], p[2]) for p in pts)
    return KML_POLY.format(name=f["name"], desc=desc, coords=coords)


def write_kmls(feats, outdir):
    os.makedirs(outdir, exist_ok=True)
    written = []
    used = {}
    for f in feats:
        base = slug(f["name"])
        # mesmo nome com geometrias diferentes (ponto + poligono): nao sobrescrever
        if base in used:
            base = "%s_%s" % (base, f["geometry"].lower())
            n = 2
            while base in used:
                base, n = "%s_%d" % (base, n), n + 1
        used[base] = True
        path = os.path.join(outdir, base + ".kml")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(KML_TMPL.format(docname=f["name"],
                                     body=feature_to_kml_body(f)))
        written.append(path)
    combined = os.path.join(outdir, "artex_areas.kml")
    with open(combined, "w", encoding="utf-8") as fh:
        fh.write(KML_TMPL.format(docname="ARTEX - areas operacionais (reduzido)",
                                 body="".join(feature_to_kml_body(f) for f in feats)))
    written.append(combined)
    return written


def human(f):
    lines = []
    lines.append("### %s  [%s, %d pts]" % (f["name"], f["geometry"], f["n_points"]))
    lines.append("    teto/alt AMSL : %s m" % f["alt_m_amsl"])
    lines.append("    centroide     : %.7f, %.7f" % tuple(f["centroid"]))
    if "circle" in f:
        c = f["circle"]
        lines.append("    CIRCULO       : centro %.7f, %.7f  raio %.2f m  (residuo %.3f m)"
                     % (c["center"][0], c["center"][1], c["radius_m"], c["fit_residual_m"]))
    if f["geometry"] == "Point":
        p = f["points"][0]
        lines.append("    ponto         : %.7f, %.7f" % (p[0], p[1]))
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kml", help="caminho do arquivo .kml")
    ap.add_argument("--name", default=None,
                    help="regex (case-insensitive) para filtrar pelo nome do placemark")
    ap.add_argument("--list", action="store_true",
                    help="so lista os nomes encontrados")
    ap.add_argument("--json", metavar="FILE", default=None,
                    help="grava as features filtradas em JSON")
    ap.add_argument("--kml-out", metavar="DIR", default=None,
                    help="grava um KML reduzido por feature (+ artex_areas.kml combinado)")
    ap.add_argument("--relative-to", metavar="LAT,LON", default=None,
                    help="imprime distancia e azimute de cada centroide ate este ponto")
    ap.add_argument("--no-dedupe", action="store_true",
                    help="nao remove placemarks repetidos")
    args = ap.parse_args(argv)

    if not os.path.exists(args.kml):
        print("ERRO: nao encontrei %s" % args.kml, file=sys.stderr)
        return 2

    feats = extract(args.kml)
    if not args.no_dedupe:
        feats = dedupe(feats)

    if args.name:
        rx = re.compile(args.name, re.I)
        feats = [f for f in feats if rx.search(f["name"])]

    if args.list:
        for n in sorted({f["name"] for f in feats}):
            print(n)
        return 0

    ref = None
    if args.relative_to:
        ref = tuple(float(x) for x in args.relative_to.split(","))

    for f in feats:
        print(human(f))
        if ref:
            c = f["centroid"]
            print("    vs ref        : %.1f m  az %.1f deg"
                  % (distance_m(ref, c), bearing_deg(ref, c)))
        print()

    print("total: %d geometrias" % len(feats))

    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(feats, fh, indent=1, ensure_ascii=False)
        print("JSON  -> %s" % args.json)

    if args.kml_out:
        for p in write_kmls(feats, args.kml_out):
            print("KML   -> %s" % p)

    return 0


if __name__ == "__main__":
    sys.exit(main())
