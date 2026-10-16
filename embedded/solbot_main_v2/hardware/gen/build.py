"""Regenerate everything: library, schematic, project files (PCB: gen_pcb.py)."""
import json
import os

import gen_lib
import gen_sch

OUT = gen_sch.OUT
P = gen_sch.PROJECT


def project_file():
    hv = ['BUS_IN', 'BUS_VM']
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
                                 [{'netclass': 'Power', 'pattern': p} for p in ('+5V', '5V_ESP', '+12V', '12V_RAW', '12V_F')],
        },
        'sheets': [[gen_sch.ROOT, 'Root']] + [[gen_sch.uid('sheet', n), n] for n, _ in gen_sch.SHEETS],
    }
    json.dump(pro, open(os.path.join(OUT, P + '.kicad_pro'), 'w'), indent=2)
    open(os.path.join(OUT, 'sym-lib-table'), 'w').write(
        '(sym_lib_table\n  (lib (name "solbot")(type "KiCad")(uri "${KIPRJMOD}/lib/solbot.kicad_sym")(options "")(descr "Project parts"))\n)\n')
    open(os.path.join(OUT, 'fp-lib-table'), 'w').write(
        '(fp_lib_table\n  (lib (name "solbot")(type "KiCad")(uri "${KIPRJMOD}/lib/solbot.pretty")(options "")(descr "Project footprints"))\n)\n')


# manufacturer part numbers as listed by JLCPCB (Comment = MPN -> no "comment mismatch" prompts)
JLC_MPN = {
    'C22775': '0603WAF1000T5E', 'C21190': '0603WAF1001T5E', 'C25804': '0603WAF1002T5E',
    'C25803': '0603WAF1003T5E', 'C22935': '0603WAF1004T5E', 'C4190': '0603WAF2201T5E',
    'C22984': '0603WAF3002T5E', 'C23179': '0603WAF4700T5E', 'C23162': '0603WAF4701T5E',
    'C17709': '0805W8F4703T5E', 'C17958': '1206W4F220JT5E', 'C8545': '2N7002', 'C5605': '74HC14D,653',
    'C6187': 'AMS1117-5.0', 'C2500': 'BAV99,215', 'C14663': 'CC0603KRX7R9BB104',
    'C15849': 'CL10A105KB8NNNC', 'C15850': 'CL21A106KAYNNNE', 'C45783': 'CL21A226MAQNNNE',
    'C13585': 'CL31A106KBHNNNE', 'C2286': 'KT-0603R', 'C2297': 'KT-0805G', 'C2296': 'KT-0805Y',
    'C8678': 'SS34', 'C5446': 'XC6206P332MR-G', 'C19077573': 'SMBJ18A',
}


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
    # JLCPCB assembly BOM (SMD parts with an LCSC number only)
    jl = {}
    hand = []
    for p in parts:
        lcsc = p['fields'].get('LCSC')
        if not lcsc:
            if p['fields'].get('Assembly') != 'none':
                hand.append(p['ref'])
            continue
        jl.setdefault((p['footprint'].split(':')[1], lcsc), []).append(p['ref'])
    out2 = os.path.join(OUT, 'bom_jlc.csv')
    with open(out2, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['Comment', 'Designator', 'Footprint', 'LCSC Part #'])
        for (fp, lcsc), refs in sorted(jl.items(), key=lambda x: x[0][1]):
            w.writerow([JLC_MPN.get(lcsc, lcsc), ','.join(refs), fp, lcsc])
    print('JLC BOM ->', out2, '%d lines; hand-solder: %s' % (len(jl), ' '.join(hand)))


if __name__ == '__main__':
    gen_lib.write()
    project_file()
    gen_sch.main()
    bom()
