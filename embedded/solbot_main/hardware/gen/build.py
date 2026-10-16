"""Regenerate everything: library, schematic, project files (PCB: gen_pcb.py)."""
import json
import os

import gen_lib
import gen_sch

OUT = gen_sch.OUT
P = gen_sch.PROJECT


def project_file():
    hv = ['BUS+', 'BUS_VM', 'VAUX', 'BUCK_SW', 'BUCK_BST'] + [
        '%s_%s' % (ch, sfx) for ch in ('BAT*', 'PV') for sfx in ('IN', 'SH', 'COM', 'G', 'VM', 'SNP', 'SNN')]
    pro = {
        'meta': {'filename': P + '.kicad_pro', 'version': 1},
        'board': {'design_settings': {'rules': {
            'min_clearance': 0.2, 'min_track_width': 0.2, 'min_via_diameter': 0.6,
            'min_through_hole_diameter': 0.2, 'min_copper_edge_clearance': 0.3}}},
        'net_settings': {
            'classes': [
                {'name': 'Default', 'clearance': 0.2, 'track_width': 0.25, 'via_diameter': 0.6,
                 'via_drill': 0.3, 'diff_pair_gap': 0.25, 'diff_pair_width': 0.2,
                 'microvia_diameter': 0.3, 'microvia_drill': 0.1, 'wire_width': 6, 'bus_width': 12,
                 'line_style': 0, 'schematic_color': 'rgba(0, 0, 0, 0.000)', 'pcb_color': 'rgba(0, 0, 0, 0.000)'},
                {'name': 'HV', 'clearance': 0.5, 'track_width': 0.3, 'via_diameter': 0.8,
                 'via_drill': 0.4, 'diff_pair_gap': 0.25, 'diff_pair_width': 0.2,
                 'microvia_diameter': 0.3, 'microvia_drill': 0.1, 'wire_width': 6, 'bus_width': 12,
                 'line_style': 0, 'schematic_color': 'rgba(0, 0, 0, 0.000)', 'pcb_color': 'rgba(0, 0, 0, 0.000)'},
                {'name': 'Power', 'clearance': 0.2, 'track_width': 0.6, 'via_diameter': 0.8,
                 'via_drill': 0.4, 'diff_pair_gap': 0.25, 'diff_pair_width': 0.2,
                 'microvia_diameter': 0.3, 'microvia_drill': 0.1, 'wire_width': 6, 'bus_width': 12,
                 'line_style': 0, 'schematic_color': 'rgba(0, 0, 0, 0.000)', 'pcb_color': 'rgba(0, 0, 0, 0.000)'},
            ],
            'meta': {'version': 3},
            'netclass_patterns': [{'netclass': 'HV', 'pattern': p} for p in hv] +
                                 [{'netclass': 'Power', 'pattern': p} for p in ('GND', '+5V', '5V_ESP', '+3V3A', '+3V3_POT')],
        },
        'sheets': [[gen_sch.ROOT, 'Root']] + [[gen_sch.uid('sheet', n), n] for n, _ in gen_sch.SHEETS],
    }
    json.dump(pro, open(os.path.join(OUT, P + '.kicad_pro'), 'w'), indent=2)
    open(os.path.join(OUT, 'sym-lib-table'), 'w').write(
        '(sym_lib_table\n  (lib (name "solbot")(type "KiCad")(uri "${KIPRJMOD}/lib/solbot.kicad_sym")(options "")(descr "Project parts"))\n)\n')
    open(os.path.join(OUT, 'fp-lib-table'), 'w').write(
        '(fp_lib_table\n  (lib (name "solbot")(type "KiCad")(uri "${KIPRJMOD}/lib/solbot.pretty")(options "")(descr "Project footprints"))\n)\n')


def bom():
    """Grouped BOM CSV from netlist.json."""
    import csv
    parts = json.load(open(os.path.join(gen_sch.HERE, 'netlist.json')))
    groups = {}
    for p in parts:
        if p['lib_id'].startswith('Mechanical:MountingHole') and 'lug' not in p['value']:
            continue
        key = (p['value'], p['footprint'], p['fields'].get('MPN', ''))
        groups.setdefault(key, []).append(p['ref'])
    out = os.path.join(OUT, 'bom.csv')
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['Qty', 'Value', 'Footprint', 'MPN / note', 'References'])
        for (val, fp, mpn), refs in sorted(groups.items(), key=lambda x: (x[1][0][0], x[0][0])):
            w.writerow([len(refs), val, fp.split(':')[1], mpn, ' '.join(refs)])
    print('BOM ->', out, len(groups), 'lines')


if __name__ == '__main__':
    gen_lib.write()
    project_file()
    gen_sch.main()
    bom()
