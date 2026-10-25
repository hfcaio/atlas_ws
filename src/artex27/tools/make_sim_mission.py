#!/usr/bin/env python3
"""Variante da missao ARTEX para SITL: insere um NAV_TAKEOFF no inicio.

A missao real comeca em LOITER_TO_ALT porque a decolagem e' MANUAL -- o piloto
sobe ate ~100 m sobre a pista e so' entao passa para AUTO. Em SITL nao ha piloto,
por isso acrescenta-se um MAV_CMD_NAV_TAKEOFF (22) que leva o aviao a' mesma
altitude de entrada em AUTO. O resto da missao fica intacto.

Uso:
    ./make_sim_mission.py missions/artex27.waypoints missions/artex27_sim.waypoints
"""
import sys

TAKEOFF_CMD = 22
FRAME_REL = 3
TAKEOFF_ALT = 100.0   # m relativos -- igual a' altitude de entrada em AUTO
TAKEOFF_PITCH = 15.0  # graus


def main(src, dst):
    with open(src) as f:
        lines = [ln.rstrip('\n') for ln in f if ln.strip()]

    header, rows = lines[0], [ln.split('\t') for ln in lines[1:]]
    home = rows[0]

    # O takeoff herda lat/lon do HOME; o ArduPlane usa o rumo da pista.
    takeoff = ['1', '0', str(FRAME_REL), str(TAKEOFF_CMD),
               f'{TAKEOFF_PITCH:.8f}', '0.00000000', '0.00000000', '0.00000000',
               home[8], home[9], f'{TAKEOFF_ALT:.6f}', '1']

    out = [home, takeoff]
    for i, row in enumerate(rows[1:], start=2):
        row = list(row)
        row[0] = str(i)       # renumerar
        row[1] = '0'          # so' o HOME fica com is_current=1
        out.append(row)

    with open(dst, 'w') as f:
        f.write(header + '\n')
        for row in out:
            f.write('\t'.join(row) + '\n')

    print(f'{dst}: {len(out)} itens (era {len(rows)}) -- NAV_TAKEOFF a {TAKEOFF_ALT:.0f} m inserido no indice 1')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
