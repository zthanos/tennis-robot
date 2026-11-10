"""Split the full CAD scene into its installed bodies, using the CAD's own code.

The sources are copied to out/src/ and every statement inside a module body is
prefixed with a path selector:

    let($p=concat($p,["<id>"]), _d=$disc?echo(str("ST|",$p)) 0:0) if(_on($p)) <stmt>

and every `for` adds its loop values to $p.  A discovery run (echo export, no
geometry) lists every statement path and every module entered.  A body is the
first call site of a printed-part module (a module that an STL dispatcher
branch exports), or a part-free subtree hanging off an assembly.  Exporting a
body sets $sel to its path, so only that branch of the real scene renders.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

BUILTINS = {
    "translate", "rotate", "scale", "mirror", "multmatrix", "color", "offset",
    "hull", "minkowski", "union", "difference", "intersection", "render",
    "linear_extrude", "rotate_extrude", "projection", "resize", "cube", "sphere",
    "cylinder", "polyhedron", "square", "circle", "polygon", "text", "import",
    "surface", "children", "echo", "assert", "let", "for", "intersection_for",
    "if", "else", "each", "roof", "fill",
}

ON_FUNC = ('function _on(p) = is_undef($sel) ? true : let(n=min(len(p),len($sel))) '
           'n==0 ? true : [for(i=[0:1:n-1]) p[i]] == [for(i=[0:1:n-1]) $sel[i]];\n')

TOKEN = re.compile(r'''
    (?P<ws>\s+)|(?P<lc>//[^\n]*)|(?P<bc>/\*.*?\*/)|(?P<str>"(?:\\.|[^"\\])*")|
    (?P<inc>(?:use|include)\s*<[^>]*>)|
    (?P<num>\d+\.?\d*(?:[eE][-+]?\d+)?|\.\d+(?:[eE][-+]?\d+)?)|
    (?P<id>\$?[A-Za-z_][A-Za-z0-9_]*)|
    (?P<op>==|!=|<=|>=|&&|\|\||[-+*/%^<>=!?:;,.(){}\[\]#])
''', re.S | re.X)


@dataclass
class Tok:
    kind: str
    text: str
    pos: int
    end: int


def tokenize(src: str) -> list[Tok]:
    out, i = [], 0
    while i < len(src):
        m = TOKEN.match(src, i)
        if not m:
            raise SyntaxError(f"bad char {src[i]!r} at {i}")
        k = m.lastgroup
        if k not in ("ws", "lc", "bc"):
            out.append(Tok(k, m.group(), m.start(), m.end()))
        i = m.end()
    return out


@dataclass
class Parsed:
    inserts: list[tuple[int, str]] = field(default_factory=list)
    modules: dict[str, str] = field(default_factory=dict)       # name -> body text
    dispatch: list[tuple[str, list[str]]] = field(default_factory=list)  # (literal, calls)


class Parser:
    def __init__(self, src: str, fid: int):
        self.src, self.fid = src, fid
        self.t = tokenize(src)
        self.i = 0
        self.out = Parsed()

    # -- helpers -------------------------------------------------------
    def peek(self, k=0):
        j = self.i + k
        return self.t[j] if j < len(self.t) else Tok("eof", "", len(self.src), len(self.src))

    def skip_group(self):
        """At an opening ( or [ : skip to after its match. Returns end index."""
        open_, close = self.peek().text, {"(": ")", "[": "]", "{": "}"}[self.peek().text]
        depth = 0
        while True:
            tk = self.peek()
            if tk.kind == "eof":
                raise SyntaxError("unbalanced")
            if tk.text in "([{" and tk.kind == "op":
                depth += 1
            elif tk.text in ")]}" and tk.kind == "op":
                depth -= 1
            self.i += 1
            if depth == 0:
                return self.t[self.i - 1].end

    def skip_to_semicolon(self):
        while True:
            tk = self.peek()
            if tk.kind == "eof":
                return
            if tk.text in "([{" and tk.kind == "op":
                self.skip_group()
                continue
            self.i += 1
            if tk.text == ";":
                return

    # -- grammar -------------------------------------------------------
    def parse_file(self):
        while self.peek().kind != "eof":
            self.statement(top=True, in_module=False)
        return self.out

    def block(self, in_module):
        assert self.peek().text == "{"
        self.i += 1
        while self.peek().text != "}":
            if self.peek().kind == "eof":
                raise SyntaxError("eof in block")
            self.statement(top=False, in_module=in_module)
        self.i += 1

    def statement(self, top, in_module, instrument=True):
        tk = self.peek()
        if tk.kind == "inc":
            self.i += 1
            return
        if tk.text == ";":
            self.i += 1
            return
        if tk.text == "module":
            self.module_def()
            return
        if tk.text == "function":
            self.skip_to_semicolon()
            return
        if tk.kind == "id" and self.peek(1).text == "=":
            self.skip_to_semicolon()
            return
        if top and tk.text == "if":
            self.dispatch_chain()
            return
        if in_module and instrument:
            sid = f"{self.fid}:{tk.pos}"
            self.out.inserts.append((tk.pos,
                f'let($p=concat($p,["{sid}"]), _d=$disc?echo(str("ST|",$p)) 0:0) if(_on($p)) '))
        self.chain(in_module)

    def chain(self, in_module):
        """Transform/call chain ending in ';', a block, or an if/for."""
        while True:
            tk = self.peek()
            if tk.text in ("%", "#", "!", "*") and tk.kind == "op":
                self.i += 1
                continue
            if tk.text == "{":
                self.block(in_module)
                return
            if tk.text == ";":
                self.i += 1
                return
            if tk.text == "if":
                self.i += 1
                self.skip_group()
                self.statement(False, in_module, instrument=False)
                if self.peek().text == "else":
                    self.i += 1
                    self.statement(False, in_module, instrument=False)
                return
            if tk.text in ("for", "intersection_for"):
                self.i += 1
                hstart = self.i
                end = self.skip_group()
                names = self.for_vars(hstart + 1, self.i - 1)
                if names:
                    tag = ",".join(f'"{n}=",{n}' for n in names)
                    self.out.inserts.append((end, f' let($p=concat($p,[str({tag})]))'))
                self.statement(False, in_module, instrument=False)
                return
            if tk.kind == "id":
                self.i += 1
                if self.peek().text == "(":
                    self.skip_group()
                continue
            raise SyntaxError(f"unexpected {tk.text!r} at {tk.pos}: "
                              f"{self.src[tk.pos-60:tk.pos+40]!r}")

    def for_vars(self, a, b):
        names, depth, expect = [], 0, True
        for tk in self.t[a:b]:
            if tk.text in "([{" and tk.kind == "op":
                depth += 1
            elif tk.text in ")]}" and tk.kind == "op":
                depth -= 1
            elif depth == 0 and tk.text == ";":
                return []
            elif depth == 0 and tk.text == ",":
                expect = True
            elif depth == 0 and expect and tk.kind == "id":
                names.append(tk.text)
                expect = False
        return names

    def module_def(self):
        self.i += 1
        name = self.peek().text
        self.i += 1
        self.skip_group()
        tk = self.peek()
        echo = f'if($disc) echo(str("MOD|",$p,"|{name}"));'
        if tk.text == "{":
            start = tk.pos
            self.out.inserts.append((tk.end, echo))
            self.block(in_module=True)
        else:
            # Single-statement body: wrap it in a block whose first statement
            # is the entry echo.  The brace is queued after the statement's
            # own selector so that, at the shared position, it lands outside
            # it: `{echo; let(...) if(_on) stmt}` rather than the selector
            # wrapping the echo and shifting the module's recorded path.
            start = tk.pos
            self.statement(False, True)
            self.out.inserts.append((tk.pos, "{" + echo))
            self.out.inserts.append((self.t[self.i - 1].end, "}"))
        self.out.modules[name] = self.src[start:self.t[self.i - 1].end]

    def dispatch_chain(self):
        """Top-level if/else chain: record literal -> module calls per branch."""
        while True:
            self.i += 1                      # 'if'
            c0 = self.i
            self.skip_group()
            lits = [tk.text[1:-1] for tk in self.t[c0:self.i] if tk.kind == "str"]
            b0 = self.i
            self.statement(False, False, instrument=False)
            calls = [tk.text for k, tk in enumerate(self.t[b0:self.i])
                     if tk.kind == "id" and self.t[b0 + k + 1].text == "("
                     and tk.text not in BUILTINS]
            for lit in lits:
                self.out.dispatch.append((lit, calls))
            if self.peek().text != "else":
                return
            self.i += 1
            if self.peek().text != "if":
                self.statement(False, False, instrument=False)
                return


def mirror(cad: Path, dst: Path) -> tuple[set[str], dict[int, str], set[str]]:
    """Instrument every .scad under cad/ into dst/. Returns (part modules,
    file ids, helper modules)."""
    files = [p for p in cad.rglob("*.scad")
             if not {"archive", "independent-check"} & set(p.relative_to(cad).parts)]
    stls = {p.stem for p in cad.rglob("*.stl")
            if not {"archive", "independent-check"} & set(p.relative_to(cad).parts)}
    parts, helpers, fids = set(), set(), {}
    dispatch = []
    for fid, path in enumerate(sorted(files)):
        fids[fid] = str(path.relative_to(cad))
        src = path.read_text()
        try:
            res = Parser(src, fid).parse_file()
        except SyntaxError as e:
            print(f"  parse skip {path.relative_to(cad)}: {e}")
            res = None
        out = src
        if res:
            for pos, text in sorted(res.inserts, key=lambda x: -x[0]):
                out = out[:pos] + text + out[pos:]
            for name, body in res.modules.items():
                if "children(" in body:
                    helpers.add(name)
            dispatch += res.dispatch
        out = re.sub(r'import\s*\(\s*(file\s*=\s*)?"(?!/)([^"]+)"',
                     lambda m: f'import({m.group(1) or ""}"{(path.parent / m.group(2)).resolve()}"',
                     out)
        def lib(m):
            ref = (path.parent / m.group(2)).resolve()
            return m.group(0) if cad in ref.parents else f"{m.group(1)} <{ref}>"
        out = re.sub(r'(use|include)\s*<([^>/][^>]*)>', lib, out)
        target = dst / path.relative_to(cad)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(ON_FUNC + out)
    for lit, calls in dispatch:
        if lit in stls:
            parts.update(c for c in calls if c not in helpers)
    return parts, fids, helpers


def parse_echo(text: str):
    """Return (statement paths, {path: [module names entered there]})."""
    import ast
    stmts, mods = set(), {}
    for line in text.splitlines():
        m = re.match(r'ECHO: "(ST|MOD)\|(\[.*?\])(?:\|(\w+))?"$', line.replace('\\"', '"'))
        if not m:
            continue
        path = tuple(ast.literal_eval(m.group(2)))
        if m.group(1) == "ST":
            stmts.add(path)
        else:
            mods.setdefault(path, []).append(m.group(3))
    return stmts, mods


def bodies(stmts, mods, parts, helpers):
    """Top-down: stop at part call sites and at part-free subtrees."""
    nodes = set(stmts) | set(mods)
    children = {}
    for n in nodes:
        for k in range(1, len(n) + 1):
            children.setdefault(n[:k - 1], set()).add(n[:k])
    part_at = {n for n, names in mods.items() if any(x in parts for x in names)}
    has_part = set()
    for n in part_at:
        for k in range(len(n) + 1):
            has_part.add(n[:k])
    out = []

    def name_of(n):
        below = sorted((p for p in mods if p[:len(n)] == n), key=len)
        for p in below:
            for x in mods[p]:
                if x not in helpers:
                    return x
        return "geometry"

    def walk(n):
        if n in part_at and n != ():
            out.append(n)
            return
        if n not in has_part:
            if n != ():
                out.append(n)
            return
        for c in sorted(children.get(n, ())):
            if c != n:
                walk(c)

    walk(())
    named = []
    for n in out:
        first = next((x for x in mods.get(n, []) if x in parts), None)
        base = first or name_of(n)
        loops = [s for s in n if not re.match(r"^\d+:\d+$", s)]
        named.append((base + ("[" + ";".join(loops) + "]" if loops else ""), n))
    return named
