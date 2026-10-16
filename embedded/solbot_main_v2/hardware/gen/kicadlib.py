"""Load KiCad symbol libraries, flatten `extends`, expose pin geometry."""
import copy
import os

from sexpr import parse, find, findall, Sym

SYS_SYM = '/usr/share/kicad/symbols'
HERE = os.path.dirname(os.path.abspath(__file__))
LOCAL_SYM = os.path.join(HERE, '..', 'lib')

_libs = {}


def _lib(name):
    if name not in _libs:
        for d in (LOCAL_SYM, SYS_SYM):
            p = os.path.join(d, name + '.kicad_sym')
            if os.path.exists(p):
                _libs[name] = parse(open(p).read())
                break
        else:
            raise FileNotFoundError(name)
    return _libs[name]


def _raw(lib, name):
    for s in findall(_lib(lib), 'symbol'):
        if s[1] == name:
            return s
    raise KeyError('%s:%s' % (lib, name))


def _rename_units(sym, old, new):
    for u in findall(sym, 'symbol'):
        if u[1].startswith(old + '_'):
            u[1] = new + u[1][len(old):]


def flat_symbol(lib_id):
    """Return a self-contained symbol s-expr named 'lib:name'."""
    lib, name = lib_id.split(':')
    s = copy.deepcopy(_raw(lib, name))
    ext = find(s, 'extends')
    if ext:
        parent = flat_symbol(lib + ':' + ext[1])
        pname = parent[1].split(':')[1]
        merged = [x for x in parent if not (isinstance(x, list) and x[0] == 'property')]
        # child's properties override parent's
        props = {p[1]: p for p in findall(parent, 'property')}
        for p in findall(s, 'property'):
            props[p[1]] = p
        merged = merged[:2] + [x for x in merged[2:] if not (isinstance(x, list) and x[0] == 'symbol')]
        merged += list(props.values())
        units = [copy.deepcopy(u) for u in findall(parent, 'symbol')]
        merged += units
        s = merged
        _rename_units(s, pname, name)
    s[1] = lib + ':' + name
    return s


class SymInfo:
    def __init__(self, lib_id):
        self.lib_id = lib_id
        self.sexpr = flat_symbol(lib_id)
        self.pins = []  # (number, name, x, y, angle, length, type, unit)
        for u in findall(self.sexpr, 'symbol'):
            unit = int(u[1].rsplit('_', 2)[-2])
            for p in findall(u, 'pin'):
                at = find(p, 'at')
                ln = find(p, 'length')
                nm = find(p, 'name')
                nb = find(p, 'number')
                self.pins.append(dict(num=str(nb[1]), name=str(nm[1]), x=float(at[1]), y=float(at[2]),
                                      ang=float(at[3]) if len(at) > 3 else 0.0, length=float(ln[1]),
                                      type=str(p[1]), unit=unit,
                                      hidden=any(x == 'hide' for x in p)))
        self.props = {p[1]: p[2] for p in findall(self.sexpr, 'property')}
        self.power = bool(find(self.sexpr, 'power'))

    def pin(self, key):
        """Look up by number, then by name (case-insensitive)."""
        hits = [p for p in self.pins if p['num'] == str(key)]
        if not hits:
            hits = [p for p in self.pins if p['name'].lower() == str(key).lower()]
        return hits


_cache = {}


def sym(lib_id):
    if lib_id not in _cache:
        _cache[lib_id] = SymInfo(lib_id)
    return _cache[lib_id]


if __name__ == '__main__':
    import sys
    for lid in sys.argv[1:]:
        s = sym(lid)
        print(lid, s.props.get('Footprint'))
        for p in s.pins:
            print('   %4s %-12s %-14s (%6.2f,%6.2f) a=%3d u=%d%s' % (
                p['num'], p['name'], p['type'], p['x'], p['y'], p['ang'], p['unit'], ' H' if p['hidden'] else ''))
