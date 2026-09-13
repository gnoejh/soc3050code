#!/usr/bin/env python3
"""Render every lesson's Slide.md into a presentable HTML deck.

The decks are written in a consistent subset of Markdown - headings, fenced
code, tables, lists and horizontal rules - and each `---` starts a new slide.
This turns them into self-contained HTML files you can project: arrow keys or
space to move, `o` for an overview grid, `p` to print or save as PDF.

No third-party packages. The repository promises that a fresh clone needs no
downloads, so the small amount of Markdown these decks use is rendered here
rather than by pulling in a library a student may not have.

Usage:
    python _build/build-slides.py            # all lessons -> _slides/
    python _build/build-slides.py 11_ADC_Basic
"""
import html
import os
import re
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# The one reference every lesson leans on. It is pinned into the deck
# chrome rather than left on the title slide alone, so it is reachable
# from any slide without going back to the start.
DATASHEET = "https://www.st.com/resource/en/reference_manual/rm0490-stm32c0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf"

# The course notebook, pinned beside the reference manual. One shared notebook
# serves every deck: it installs arm-none-eabi-gcc on Colab's Ubuntu, clones
# this tree, and runs the builds and disassembly the Part 0 slides quote, so a
# student with no toolchain can still follow along. Colab renders any notebook
# that GitHub can serve, so this URL is just the repository path with
# colab.research.google.com/github/ in front - change the constant, re-render.
COLAB = ("https://colab.research.google.com/github/gnoejh/soc3050code/blob/"
         "main/projects2026_arm/_notebooks/SOC3050_ARM.ipynb")
OUT = os.path.join(BASE, "_slides")


# --------------------------------------------------------------------------
# Inline markdown
# --------------------------------------------------------------------------
def inline(text):
    """Render inline markdown in already-HTML-escaped text.

    Code spans are pulled out first and restored at the end so that emphasis
    and link syntax inside them is left alone.
    """
    spans = []

    def stash(m):
        spans.append(m.group(1))
        return "\x00%d\x00" % (len(spans) - 1)

    text = re.sub(r"`([^`]+)`", stash, text)
    text = re.sub(r"\[([^\]]+)\]\(([^)]+)\)",
                  r'<a href="\2" target="_blank" rel="noopener">\1</a>', text)
    # Bold first, non-greedily, and allowing asterisks inside: a bold run may
    # contain nested *emphasis*, which a [^*]+ body would refuse to match and
    # would then leave the surrounding ** pairing up across the wrong spans.
    text = re.sub(r"\*\*(.+?)\*\*", r"<strong>\1</strong>", text)
    text = re.sub(r"(?<![\w*])\*([^*]+)\*(?![\w*])", r"<em>\1</em>", text)
    text = re.sub(r"\x00(\d+)\x00",
                  lambda m: "<code>%s</code>" % spans[int(m.group(1))], text)
    return text


def cell_align(spec):
    spec = spec.strip()
    if spec.startswith(":") and spec.endswith(":"):
        return " style=\"text-align:center\""
    if spec.endswith(":"):
        return " style=\"text-align:right\""
    return ""


# --------------------------------------------------------------------------
# Block markdown
# --------------------------------------------------------------------------
def render_blocks(lines):
    out = []
    i = 0
    n = len(lines)

    while i < n:
        raw = lines[i]
        line = raw.rstrip()

        # fenced code
        if line.startswith("```"):
            lang = line[3:].strip()
            i += 1
            body = []
            while i < n and not lines[i].rstrip().startswith("```"):
                body.append(lines[i].rstrip("\n"))
                i += 1
            i += 1
            # ```svg passes through verbatim instead of being escaped, so a
            # deck can carry a real diagram rather than ASCII art.  The
            # content is trusted: Slide.md files live in this repository
            # beside the renderer itself.  Author with currentColor, the
            # --accent / --muted variables, or the helper classes in PAGE,
            # so one diagram is legible in both the light and dark themes.
            if lang == "svg":
                out.append('<figure class="diagram">%s</figure>'
                           % chr(10).join(body))
                continue
            cls = ' class="lang-%s"' % html.escape(lang) if lang else ""
            out.append("<pre%s><code>%s</code></pre>"
                       % (cls, html.escape("\n".join(body))))
            continue

        # table: a header row followed by a |---|---| separator
        if (line.startswith("|") and i + 1 < n
                and re.match(r"^\s*\|[\s:|\\-]+\|\s*$", lines[i + 1])):
            def cells(s):
                # Split on unescaped pipes only: a cell may contain \| , which
                # these decks use for bit masks such as `(1 << CS11) \| (1 << CS10)`.
                s = s.strip()
                s = re.sub(r"^\||\|$", "", s)
                parts = re.split(r"(?<!\\)\|", s)
                return [p.replace("\\|", "|").strip() for p in parts]

            head = cells(line)
            aligns = [cell_align(c) for c in cells(lines[i + 1])]
            i += 2
            rows = []
            while i < n and lines[i].strip().startswith("|"):
                rows.append(cells(lines[i].rstrip()))
                i += 1
            t = ["<table><thead><tr>"]
            for j, c in enumerate(head):
                a = aligns[j] if j < len(aligns) else ""
                t.append("<th%s>%s</th>" % (a, inline(html.escape(c))))
            t.append("</tr></thead><tbody>")
            for r in rows:
                t.append("<tr>")
                for j, c in enumerate(r):
                    a = aligns[j] if j < len(aligns) else ""
                    t.append("<td%s>%s</td>" % (a, inline(html.escape(c))))
                t.append("</tr>")
            t.append("</tbody></table>")
            out.append("".join(t))
            continue

        # headings
        m = re.match(r"^(#{1,6})\s+(.*)$", line)
        if m:
            lvl = len(m.group(1))
            out.append("<h%d>%s</h%d>"
                       % (lvl, inline(html.escape(m.group(2).strip())), lvl))
            i += 1
            continue

        # lists
        if re.match(r"^\s*[-*+]\s+", line) or re.match(r"^\s*\d+[.)]\s+", line):
            ordered = bool(re.match(r"^\s*\d+[.)]\s+", line))
            items = []
            while i < n:
                cur = lines[i].rstrip()
                m2 = re.match(r"^\s*(?:[-*+]|\d+[.)])\s+(.*)$", cur)
                if not m2:
                    # a wrapped continuation line belongs to the previous item
                    if items and cur.strip() and cur.startswith(("  ", "\t")):
                        items[-1] += " " + cur.strip()
                        i += 1
                        continue
                    break
                items.append(m2.group(1))
                i += 1
            tag = "ol" if ordered else "ul"
            out.append("<%s>%s</%s>"
                       % (tag,
                          "".join("<li>%s</li>" % inline(html.escape(x))
                                  for x in items),
                          tag))
            continue

        # blank
        if not line.strip():
            i += 1
            continue

        # paragraph: gather until a blank line or a block starter
        para = []
        while i < n:
            cur = lines[i].rstrip()
            if (not cur.strip() or cur.startswith("```") or cur.startswith("#")
                    or cur.startswith("|")
                    or re.match(r"^\s*(?:[-*+]|\d+[.)])\s+", cur)):
                break
            para.append(cur)
            i += 1
        if para:
            # Escape and join the whole paragraph before rendering inline
            # markup, so that **bold** or `code` wrapped across a source line
            # break is still recognised. A sentinel stands in for the line
            # break until after the inline pass, then becomes a <br> - these
            # decks lay their check-mark summaries out one item per line.
            joined = "\x01".join(html.escape(p.rstrip("\\").rstrip())
                                 for p in para)
            out.append("<p>%s</p>" % inline(joined).replace("\x01", "<br>"))

    return "\n".join(out)


def split_slides(text):
    """Split on horizontal rules, keeping any leading title block as slide 1."""
    text = text.replace("\r\n", "\n")
    parts = re.split(r"(?m)^\s*---\s*$", text)
    slides = []
    for p in parts:
        lines = p.split("\n")
        if any(l.strip() for l in lines):
            slides.append(lines)
    return slides


PAGE = """<title>{title}</title>
<style>
  :root {{
    --bg: #ffffff; --fg: #1a1c1f; --muted: #5b6270; --rule: #dfe3ea;
    --accent: #1f5fa8; --code-bg: #f5f7fa; --code-fg: #22262b;
    --th-bg: #eef2f7; --mark: #0a7a4a;
  }}
  @media (prefers-color-scheme: dark) {{
    :root:not([data-theme="light"]) {{
      --bg: #14171c; --fg: #e6e9ee; --muted: #9aa3b2; --rule: #2b313b;
      --accent: #78b4f0; --code-bg: #1b1f26; --code-fg: #dfe4ec;
      --th-bg: #1f242c; --mark: #56d39a;
    }}
  }}
  :root[data-theme="dark"] {{
    --bg: #14171c; --fg: #e6e9ee; --muted: #9aa3b2; --rule: #2b313b;
    --accent: #78b4f0; --code-bg: #1b1f26; --code-fg: #dfe4ec;
    --th-bg: #1f242c; --mark: #56d39a;
  }}

  body {{
    background: var(--bg); color: var(--fg); margin: 0;
    font: 16px/1.55 -apple-system, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
  }}
  .slide {{
    display: none; box-sizing: border-box; min-height: 100vh;
    padding: 4.6rem 4rem 5rem; max-width: 1180px; margin: 0 auto;
  }}
  .slide.active {{ display: block; }}
  h1 {{ font-size: 2.3rem; line-height: 1.15; margin: 0 0 .4rem; letter-spacing: -.02em; }}
  h2 {{ font-size: 1.65rem; margin: 0 0 1rem; color: var(--accent); letter-spacing: -.01em; }}
  h3 {{ font-size: 1.12rem; margin: 1.5rem 0 .5rem; }}
  p  {{ margin: .6rem 0; }}
  ul, ol {{ margin: .5rem 0 .9rem; padding-left: 1.4rem; }}
  li {{ margin: .28rem 0; }}
  a {{ color: var(--accent); }}
  code {{
    background: var(--code-bg); color: var(--code-fg); padding: .1em .35em;
    border-radius: 4px; font-size: .89em;
    font-family: "Cascadia Mono", Consolas, "DejaVu Sans Mono", monospace;
  }}
  pre {{
    background: var(--code-bg); border: 1px solid var(--rule); border-radius: 8px;
    padding: .85rem 1rem; overflow-x: auto; line-height: 1.45;
  }}
  pre code {{ background: none; padding: 0; font-size: .85rem; }}
  table {{ border-collapse: collapse; margin: .8rem 0; display: block; overflow-x: auto; }}
  th, td {{ border: 1px solid var(--rule); padding: .38rem .7rem; text-align: left; }}
  th {{ background: var(--th-bg); }}
  strong {{ font-weight: 650; }}

  .topbar {{
    position: fixed; left: 0; right: 0; top: 0; height: 2.2rem; z-index: 5;
    display: flex; align-items: center; gap: 1rem; padding: 0 1.2rem;
    background: var(--bg); border-bottom: 1px solid var(--rule);
    font-size: .8rem; color: var(--muted);
  }}
  .topbar .spacer {{ flex: 1; }}
  .topbar .deck-name {{ overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }}
  /* ---- diagrams -------------------------------------------------------
     A ```svg block lands here.  Shapes inherit the page colour through
     currentColor, so one diagram works in both themes.  The helper classes
     save every deck from restating the same fills and strokes. */
  .diagram {{ margin: 1.1rem 0; text-align: center; color: var(--fg); }}
  .diagram svg {{ max-width: 100%; height: auto; overflow: visible; }}
  .diagram text {{
    fill: currentColor; font-family: -apple-system, "Segoe UI", Roboto, sans-serif;
    font-size: 13px; dominant-baseline: middle;
  }}
  .diagram .mono   {{ font-family: ui-monospace, "Cascadia Mono", Consolas, monospace; }}
  .diagram .lbl    {{ fill: var(--muted); font-size: 11px; }}
  .diagram .box    {{ fill: var(--code-bg); stroke: currentColor; stroke-width: 1.5; }}
  .diagram .reg    {{ fill: none; stroke: currentColor; stroke-width: 1.2; }}
  .diagram .hi     {{ fill: none; stroke: var(--accent); stroke-width: 2; }}
  .diagram .hifill {{ fill: var(--accent); fill-opacity: .14; stroke: var(--accent); stroke-width: 1.5; }}
  .diagram .ok     {{ fill: none; stroke: var(--mark); stroke-width: 2; }}
  .diagram .wire   {{ fill: none; stroke: currentColor; stroke-width: 1.5; }}
  .diagram .dash   {{ fill: none; stroke: var(--muted); stroke-width: 1.2; stroke-dasharray: 4 3; }}
  .diagram figcaption {{ color: var(--muted); font-size: .8rem; margin-top: .4rem; }}

  .topbar a {{ color: var(--accent); text-decoration: none; font-weight: 600; }}
  .topbar a:hover {{ text-decoration: underline; }}

  .bar {{
    position: fixed; left: 0; right: 0; bottom: 0; height: 2.6rem;
    display: flex; align-items: center; gap: 1rem; padding: 0 1.2rem;
    background: var(--bg); border-top: 1px solid var(--rule);
    font-size: .8rem; color: var(--muted);
  }}
  .bar .spacer {{ flex: 1; }}
  .bar button {{
    font: inherit; color: var(--muted); background: none;
    border: 1px solid var(--rule); border-radius: 5px; padding: .1rem .55rem;
    cursor: pointer;
  }}
  .bar button:hover {{ color: var(--fg); }}

  /* overview grid */
  body.grid .slide {{
    display: block; min-height: 0; padding: 1rem; margin: 0;
    border: 1px solid var(--rule); border-radius: 8px;
    zoom: .42; cursor: pointer; overflow: hidden; height: 44rem;
  }}
  body.grid #deck {{
    display: grid; grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
    gap: .8rem; padding: 3rem 1rem 4rem;
  }}

  @media print {{
    .bar, .topbar {{ display: none; }}
    .slide {{ display: block !important; page-break-after: always; min-height: 0; padding: 1.2cm; }}
    pre, table {{ page-break-inside: avoid; }}
  }}
</style>

<div class="topbar">
  <span><a href="{datasheet}" target="_blank" rel="noopener">STM32 Reference Manual (PDF)</a></span>
  <span><a href="{colab}" target="_blank" rel="noopener">Open in Colab</a></span>
  <span class="spacer"></span>
  <span class="deck-name">{title}</span>
</div>

<div id="deck">
{slides}
</div>

<div class="bar">
  <span><a href="index.html">&larr; all decks</a></span>
  <span class="spacer"></span>
  <button onclick="go(-1)">&larr;</button>
  <span id="pos"></span>
  <button onclick="go(1)">&rarr;</button>
  <span class="spacer"></span>
  <button onclick="toggleGrid()">overview (o)</button>
  <button onclick="window.print()">print (p)</button>
</div>

<script>
  var slides = Array.prototype.slice.call(document.querySelectorAll('.slide'));
  var at = 0;

  function show(n) {{
    at = Math.max(0, Math.min(slides.length - 1, n));
    slides.forEach(function (s, i) {{ s.classList.toggle('active', i === at); }});
    document.getElementById('pos').textContent = (at + 1) + ' / ' + slides.length;
    try {{ localStorage.setItem('slide:{key}', at); }} catch (e) {{}}
    if (!document.body.classList.contains('grid')) window.scrollTo(0, 0);
  }}
  function go(d) {{ show(at + d); }}

  function toggleGrid() {{
    document.body.classList.toggle('grid');
    if (!document.body.classList.contains('grid')) show(at);
  }}

  slides.forEach(function (s, i) {{
    s.addEventListener('click', function () {{
      if (document.body.classList.contains('grid')) {{ toggleGrid(); show(i); }}
    }});
  }});

  document.addEventListener('keydown', function (e) {{
    if (e.key === 'ArrowRight' || e.key === 'PageDown' || e.key === ' ') {{ go(1); e.preventDefault(); }}
    else if (e.key === 'ArrowLeft' || e.key === 'PageUp') {{ go(-1); e.preventDefault(); }}
    else if (e.key === 'Home') show(0);
    else if (e.key === 'End') show(slides.length - 1);
    else if (e.key === 'o') toggleGrid();
    else if (e.key === 'p') window.print();
  }});

  var saved = 0;
  try {{ saved = parseInt(localStorage.getItem('slide:{key}') || '0', 10) || 0; }} catch (e) {{}}
  show(saved);
</script>
"""

INDEX = """<title>SOC3050 Lecture Decks - ARM</title>
<style>
  :root {{ --bg:#fff; --fg:#1a1c1f; --muted:#5b6270; --rule:#dfe3ea; --accent:#1f5fa8; --card:#f7f9fc; }}
  @media (prefers-color-scheme: dark) {{
    :root:not([data-theme="light"]) {{ --bg:#14171c; --fg:#e6e9ee; --muted:#9aa3b2; --rule:#2b313b; --accent:#78b4f0; --card:#1b1f26; }}
  }}
  :root[data-theme="dark"] {{ --bg:#14171c; --fg:#e6e9ee; --muted:#9aa3b2; --rule:#2b313b; --accent:#78b4f0; --card:#1b1f26; }}
  body {{ background:var(--bg); color:var(--fg); margin:0; padding:3rem 1.5rem 5rem;
         font:16px/1.55 -apple-system,"Segoe UI",Roboto,Helvetica,Arial,sans-serif; }}
  .wrap {{ max-width: 940px; margin: 0 auto; }}
  h1 {{ font-size:2rem; margin:0 0 .3rem; letter-spacing:-.02em; }}
  .sub {{ color:var(--muted); margin:0 0 2rem; }}
  ol {{ list-style:none; padding:0; margin:0; display:grid;
        grid-template-columns:repeat(auto-fill,minmax(280px,1fr)); gap:.7rem; }}
  a.card {{ display:block; background:var(--card); border:1px solid var(--rule);
            border-radius:9px; padding:.85rem 1rem; text-decoration:none; color:inherit; }}
  a.card:hover {{ border-color:var(--accent); }}
  .n {{ color:var(--muted); font-size:.8rem; font-variant-numeric:tabular-nums; }}
  .t {{ display:block; font-weight:600; margin:.15rem 0 .2rem; }}
  .f {{ color:var(--muted); font-size:.87rem; }}
</style>
<div class="wrap">
  <h1>SOC3050 &mdash; STM32 Lecture Decks</h1>
  <p class="sub">{count} decks, 2026 ARM edition. Arrow keys to move, <code>o</code> for an overview, <code>p</code> to print or save as PDF.</p>
  <p class="sub"><a href="{datasheet}" target="_blank" rel="noopener">STM32 Reference Manual (PDF)</a> &mdash; the reference behind every deck, also pinned to the top of each slide.</p>
  <p class="sub"><a href="{colab}" target="_blank" rel="noopener">Open the course notebook in Colab</a> &mdash; a real <code>arm-none-eabi-gcc</code> in the browser, with no toolchain to install. Builds, sections and disassembly; it cannot flash or run a board.</p>
  <ol>
{cards}
  </ol>
</div>
"""


def deck_title(lines):
    for l in lines:
        m = re.match(r"^#\s+(.*)$", l.strip())
        if m:
            return m.group(1).strip()
    return None


def main(argv):
    wanted = argv or None
    os.makedirs(OUT, exist_ok=True)

    lessons = sorted(d for d in os.listdir(BASE)
                     if re.match(r"^\d\d_", d)
                     and os.path.isfile(os.path.join(BASE, d, "Slide.md")))
    # A named lesson limits which decks are *re-rendered*, never which ones the
    # index lists: rebuilding index.html from one lesson would drop every other
    # link while leaving those .html files sitting there unreachable.
    if wanted:
        rendering = [l for l in lessons if l in wanted]
        if not rendering:
            print("no matching lesson with a Slide.md")
            return 1
    else:
        rendering = lessons

    cards = []
    for name in lessons:
        src = os.path.join(BASE, name, "Slide.md")
        text = open(src, encoding="utf-8-sig", errors="replace").read()
        chunks = split_slides(text)
        title = deck_title(chunks[0] if chunks else []) or name

        if name in rendering:
            body = "\n".join(
                '<section class="slide">\n%s\n</section>' % render_blocks(c)
                for c in chunks)
            page = PAGE.format(title=html.escape(title), slides=body,
                               key=name, datasheet=DATASHEET,
                               colab=COLAB)

            dest = os.path.join(OUT, name + ".html")
            with open(dest, "w", encoding="utf-8", newline="\n") as fh:
                fh.write(page)

        # the focus line the lesson README opens with makes a good subtitle
        focus = ""
        rp = os.path.join(BASE, name, "README.md")
        if os.path.isfile(rp):
            for l in open(rp, encoding="utf-8").read().split("\n")[1:6]:
                if l.strip():
                    focus = l.strip()
                    break
        cards.append(
            '    <li><a class="card" href="{f}.html">'
            '<span class="n">{n}</span>'
            '<span class="t">{t}</span>'
            '<span class="f">{d}</span></a></li>'.format(
                f=name, n=name.split("_")[0],
                t=html.escape(title), d=html.escape(focus)))
        if name in rendering:
            print("  %-28s %3d slides" % (name, len(chunks)))

    with open(os.path.join(OUT, "index.html"), "w", encoding="utf-8",
              newline="\n") as fh:
        fh.write(INDEX.format(count=len(lessons), cards="\n".join(cards),
                              datasheet=DATASHEET, colab=COLAB))

    print("\n%d deck(s) rendered, %d listed in the index -> %s"
          % (len(rendering), len(lessons), os.path.relpath(OUT, BASE)))
    print("open %s" % os.path.join(os.path.relpath(OUT, BASE), "index.html"))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
