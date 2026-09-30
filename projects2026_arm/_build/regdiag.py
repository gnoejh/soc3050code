"""Draw register bit maps to scale, from a few lines of text in a Slide.md.

A deck writes

    ```regs
    # SysTick control and status, 0xE000E010
    SYST_CSR ; control and status | 32 | 16 COUNTFLAG, 2 CLKSOURCE, 1 TICKINT, !0 ENABLE
    ```

and build-slides.py hands the body to render(), which returns an <svg> that
uses the same helper classes (.reg .hifill .dash .lbl .mono) as the decks'
hand-written diagrams, so it follows the light and dark themes.

One register per line:

    NAME [; note] | WIDTH [= VALUE] | FIELDS

    NAME    shown in monospace; may contain spaces ("IDR  ODR  OTYPER")
    note    optional second line under the name, in the muted colour
    WIDTH   bits in the register: 8, 16, 32 or 64
    VALUE   optional, 0x... or 0b...; every bit's digit is written in its cell
    FIELDS  comma-separated, one of
              hi:lo NAME     a multi-bit field
              bit NAME       a one-bit field
              hi:lo / bit    a field with no name
            A leading ! shades the field (the accent colour).
            Bits no field covers are drawn dashed, as reserved.
    or      each N [used U] [pins] [mark a b ...]
            U/N identical N-bit fields from bit 0 up (U defaults to WIDTH),
            e.g. MODER is "each 2"; `pins` numbers them 0, 1, 2 ... inside;
            fields numbered in `mark` are shaded and labelled with their bit
            range below the bar.

Lines starting with # are the caption, printed above the rows.

Every row is drawn at the same scale and right-aligned on bit 0, so rows can
be compared by eye: a 2-bit field really is twice as wide as a 1-bit one.
"""
import html
import re

CH_MONO = 7.9     # px per character, 13 px monospace
CH_SANS = 6.4     # px per character, 11 px sans (.lbl)
BAR_H = 26


def _esc(s):
    return html.escape(s, quote=True)


def _parse_value(tok):
    tok = tok.strip().replace("_", "")
    if tok.lower().startswith("0b"):
        return int(tok[2:], 2)
    return int(tok, 0)


def _parse_row(line):
    parts = [p.strip() for p in line.split("|")]
    if len(parts) != 3:
        raise ValueError("regs: need NAME | WIDTH | FIELDS: %r" % line)
    name, note = (parts[0].split(";", 1) + [""])[:2]
    wv = parts[1].split("=")
    width = int(wv[0])
    value = _parse_value(wv[1]) if len(wv) > 1 else None

    fields = []                 # (hi, lo, name, shaded, below_label)
    spec = parts[2]
    each = False
    m = re.match(r"^each\s+(\d+)(?:\s+used\s+(\d+))?(\s+pins)?"
                 r"(?:\s+mark\s+([\d\s]+))?$", spec)
    if m:
        each = True
        n = int(m.group(1))
        used = int(m.group(2) or width)
        pins = bool(m.group(3))
        marks = {int(x) for x in (m.group(4) or "").split()}
        for k in range(used // n):
            lo = k * n
            hi = lo + n - 1
            lab = ("%d" % lo if n == 1 else "%d:%d" % (hi, lo)) if k in marks else ""
            fields.append((hi, lo, str(k) if pins else "", k in marks, lab))
    elif spec:
        for tok in spec.split(","):
            tok = tok.strip()
            if not tok:
                continue
            shaded = tok.startswith("!")
            tok = tok.lstrip("!").strip()
            m = re.match(r"^(\d+)(?::(\d+))?\s*(.*)$", tok)
            if not m:
                raise ValueError("regs: bad field %r" % tok)
            hi = int(m.group(1))
            lo = int(m.group(2)) if m.group(2) is not None else hi
            if hi < lo:
                hi, lo = lo, hi
            if hi >= width:
                raise ValueError("regs: field %r outside a %d-bit register" % (tok, width))
            fname = m.group(3).strip()
            if fname.startswith("!"):           # "5:4 !NAME" shades too
                shaded, fname = True, fname[1:].strip()
            fields.append((hi, lo, fname, shaded, ""))

    covered = set()
    for hi, lo, *_ in fields:
        bits = set(range(lo, hi + 1))
        if bits & covered:
            raise ValueError("regs: overlapping fields in %r" % line)
        covered |= bits
    return dict(name=name.strip(), note=note.strip(), width=width,
                value=value, fields=fields, covered=covered, each=each)


def _reserved_runs(width, covered):
    runs, b = [], 0
    while b < width:
        if b in covered:
            b += 1
            continue
        lo = b
        while b < width and b not in covered:
            b += 1
        runs.append((b - 1, lo))
    return runs


class _Placer:
    """Keeps text labels on one line from overlapping; returns False if it can't fit."""
    def __init__(self):
        self.spans = []

    def take(self, x0, x1, force=False):
        for a, b in self.spans:
            if x0 < b + 3 and x1 > a - 3:
                if not force:
                    return False
        self.spans.append((x0, x1))
        return True


def render(body):
    caption, rows = [], []
    for raw in body:
        line = raw.strip()
        if not line:
            continue
        if line.startswith("#"):
            caption.append(line.lstrip("#").strip())
        else:
            rows.append(_parse_row(line))
    if not rows:
        raise ValueError("regs: no registers")

    maxw = max(r["width"] for r in rows)
    # One scale for every diagram in every deck, so a bit is the same size
    # everywhere; only a 64-bit pair (AFR[1]:AFR[0]) is drawn at half.
    pb = 20 if maxw <= 32 else 10               # px per bit
    for r in rows:
        r["vtxt"] = ("= 0x%0*X" % ((r["width"] + 3) // 4, r["value"])
                     if r["value"] is not None else "")
    label_w = max(max(len(r["name"]) * CH_MONO for r in rows),
                  max(len(r["note"]) * CH_SANS for r in rows),
                  max(len(r["vtxt"]) * CH_SANS for r in rows), 90) + 24
    left = 16 + label_w
    xr = left + maxw * pb                       # right edge of bit 0
    total_w = xr + 16
    right = total_w                             # grows if a label overhangs

    def bx(bit):                                # left edge of `bit`
        return xr - pb * (bit + 1)

    out = []
    y = 10
    for c in caption:
        out.append('<text x="%.0f" y="%d" text-anchor="middle" class="lbl">%s</text>'
                   % (total_w / 2, y + 6, _esc(c)))
        y += 18
    if caption:
        y += 6

    for r in rows:
        w = r["width"]
        top = y + 18                            # room for bit numbers above
        above = _Placer()
        below = [_Placer() for _ in range(4)]
        below_used = 0
        leaders, labels = [], []
        items = []                              # deferred svg for this row

        # name, then note and value underneath it in the label column
        lines = [(r["name"], "mono")]
        if r["note"]:
            lines.append((r["note"], "lbl"))
        if r["vtxt"]:
            lines.append((r["vtxt"], "mono lbl"))
        ly = top + (BAR_H // 2 if len(lines) == 1 else 7)
        for txt, cls in lines:
            items.append('<text x="16" y="%d" class="%s">%s</text>' % (ly, cls, _esc(txt)))
            ly += 15
        label_bottom = ly - 8

        def tick(txt, x, anchor, force=False):
            wdt = len(txt) * CH_SANS
            x0 = {"start": x, "end": x - wdt, "middle": x - wdt / 2}[anchor]
            if above.take(x0, x0 + wdt, force):
                items.append('<text x="%.1f" y="%d" text-anchor="%s" class="mono lbl">%s</text>'
                             % (x, top - 8, anchor, txt))

        # always the two ends
        tick(str(w - 1), bx(w - 1) + 1, "start", True)
        tick("0", xr - 1, "end", True)

        # reserved runs
        for hi, lo in _reserved_runs(w, r["covered"]):
            x0, wd = bx(hi), pb * (hi - lo + 1)
            items.append('<rect class="dash" x="%.1f" y="%d" width="%.1f" height="%d"/>'
                         % (x0, top, wd, BAR_H))
            if r["value"] is None and wd >= 8 * CH_SANS + 8:
                items.append('<text x="%.1f" y="%d" text-anchor="middle" class="lbl">reserved</text>'
                             % (x0 + wd / 2, top + BAR_H // 2))

        # Bit numbers over every field, unless there are so many fields that
        # they would crowd each other out; then only the ends and the shaded.
        many = r["each"] or len(r["fields"]) > 8

        for hi, lo, name, shaded, blab in r["fields"]:
            x0, wd = bx(hi), pb * (hi - lo + 1)
            items.append('<rect class="%s" x="%.1f" y="%d" width="%.1f" height="%d"/>'
                         % ("hifill" if shaded else "reg", x0, top, wd, BAR_H))
            # small bit ticks inside multi-bit fields
            if hi > lo:
                for b in range(lo + 1, hi + 1):
                    xx = xr - pb * b
                    items.append('<path class="reg" stroke-width="0.6" d="M%.1f %d V%d M%.1f %d V%d"/>'
                                 % (xx, top, top + 4, xx, top + BAR_H - 4, top + BAR_H))
            # bit numbers above the field
            if not many or (shaded and not r["each"]):
                if hi == lo:
                    tick(str(lo), x0 + wd / 2, "middle")
                else:
                    tick(str(hi), x0 + 1, "start")
                    tick(str(lo), x0 + wd - 1, "end")
            # the field's name: inside if it fits and no digits are drawn there
            if name:
                fits = len(name) * CH_MONO + 6 <= wd and r["value"] is None
                if fits:
                    items.append('<text x="%.1f" y="%d" text-anchor="middle" class="mono">%s</text>'
                                 % (x0 + wd / 2, top + BAR_H // 2, _esc(name)))
                else:
                    blab = blab or name
            if blab:
                tw = len(blab) * CH_MONO
                fx = x0 + wd / 2
                # centred under the field, but never hanging into the name
                # column; it may overhang the right, and the canvas grows
                cx = max(fx, left + tw / 2)
                lvl = 0
                while lvl < len(below) - 1 and not below[lvl].take(cx - tw / 2, cx + tw / 2):
                    lvl += 1
                if lvl == len(below) - 1:
                    below[lvl].take(cx - tw / 2, cx + tw / 2, True)
                below_used = max(below_used, lvl + 1)
                right = max(right, cx + tw / 2 + 4)
                ly = top + BAR_H + 13 + 16 * lvl
                # Leaders go under every label (drawn first), and each label
                # carries a halo in the page colour, so a leader passing a
                # label on a nearer line is hidden behind it, not through it.
                if lvl or cx != fx:
                    leaders.append('<path class="dash" d="M%.1f %d L%.1f %d"/>'
                                   % (fx, top + BAR_H, cx, ly - 7))
                labels.append('<text x="%.1f" y="%d" text-anchor="middle" class="mono halo%s">%s</text>'
                              % (cx, ly, "" if (shaded or not name) else " lbl", _esc(blab)))

        # the value, one digit per bit
        if r["value"] is not None:
            for b in range(w):
                d = (r["value"] >> b) & 1
                items.append('<text x="%.1f" y="%d" text-anchor="middle" class="mono%s">%d</text>'
                             % (bx(b) + pb / 2, top + BAR_H // 2, "" if d else " lbl", d))

        out.extend(items + leaders + labels)
        y = max(top + BAR_H + 16 * below_used + (26 if below_used else 22),
                label_bottom + 22)

    aria = "; ".join(caption + ["%s, %d bits" % (r["name"], r["width"]) for r in rows])
    # An explicit width keeps the drawing at its natural size: without it the
    # svg stretches to the column, and an 8-bit register would be drawn four
    # times the size of a 32-bit one.  max-width: 100% still shrinks it.
    return ('<svg width="%.0f" viewBox="0 0 %.0f %d" role="img" aria-label="%s">\n%s\n</svg>'
            % (right, right, y, _esc(aria), "\n".join(out)))
