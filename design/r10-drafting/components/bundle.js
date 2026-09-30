/* @ds-bundle: {"format":4,"namespace":"R10","components":[{"name":"Workspace"},{"name":"TextModeMenu"},{"name":"StatusLine"},{"name":"MenuBar"},{"name":"PulldownMenu"},{"name":"ScreenMenu"},{"name":"CommandLine"},{"name":"Dialog"},{"name":"Button"},{"name":"EditBox"},{"name":"Toggle"},{"name":"ListBox"},{"name":"ColorSwatch"},{"name":"LayerTable"},{"name":"IconMenu"},{"name":"HatchSwatch"},{"name":"Viewport"},{"name":"Line"},{"name":"Circle"},{"name":"LinearDim"},{"name":"DrawingText"},{"name":"TitleBlock"}]} */
(function () {
  var React = window.React, h = React.createElement, F = React.Fragment;
  function cx() { return Array.prototype.filter.call(arguments, Boolean).join(' '); }
  var ACI_NAMES = { 1: 'RED', 2: 'YELLOW', 3: 'GREEN', 4: 'CYAN', 5: 'BLUE', 6: 'MAGENTA', 7: 'WHITE', 8: '8' };
  function aciVar(n) { return 'var(--aci-' + (n || 7) + ')'; }
  function ltVar(lt) { return lt && lt !== 'continuous' ? 'var(--lt-' + lt + ')' : 'none'; }
  function pad(s, n) { s = String(s); while (s.length < n) s += ' '; return s; }

  /* ---------- chrome ---------- */
  function StatusLine(p) {
    var modes = p.modes || [];
    return h('div', { className: cx('r10 r10-status', p.className) },
      h('span', { className: 'r10-layer' }, ['Layer', p.layer || '0'].concat(modes).join(' '),
        p.layerColor ? h('span', { className: 'r10-chip', style: { background: aciVar(p.layerColor) } }) : null),
      h('span', { className: 'r10-coords' }, p.coords || '0.0000,0.0000'));
  }

  function MenuBar(p) {
    return h('div', { className: cx('r10 r10-mbar', p.className) }, (p.items || []).map(function (it, i) {
      return h('span', { key: i, className: i === p.active ? 'is-active' : null }, it);
    }));
  }

  function PulldownMenu(p) {
    return h('div', { className: cx('r10 r10-pull', p.className), role: 'menu', style: p.style }, (p.items || []).map(function (it, i) {
      if (it === '-' || it.separator) return h('hr', { key: i });
      if (typeof it === 'string') it = { label: it };
      return h('div', { key: i, role: 'menuitem', 'aria-disabled': it.disabled || undefined,
        className: cx(i === p.active && 'is-active', it.disabled && 'is-disabled', it.checked && 'is-checked') },
        it.label, it.key ? h('span', { className: 'r10-key' }, it.key) : null);
    }));
  }

  function ScreenMenu(p) {
    var title = p.title == null ? 'R10' : p.title; // 8 columns max
    return h('nav', { className: cx('r10 r10-smenu', p.className), style: p.style },
      h('div', { className: 'r10-smenu-title' }, title),
      h('div', { className: 'r10-smenu-sep' }, '* * * *'),
      (p.items || []).map(function (it, i) {
        if (it === '' || it == null) return h('div', { key: i, style: { height: 16 } });
        if (typeof it === 'string') it = { label: it };
        return h('button', { key: i, type: 'button', className: cx(i === p.active && 'is-active', it.dim && 'is-dim'),
          onClick: p.onSelect ? function () { p.onSelect(it, i); } : undefined }, it.label);
      }),
      p.footer ? h(F, null, h('div', { style: { flex: 1 } }), (p.footer).map(function (f, i) {
        return h('button', { key: 'f' + i, type: 'button' }, f);
      })) : null);
  }

  function CommandLine(p) {
    var rows = p.rows || 3, hist = (p.history || []).slice(-(rows - 1));
    while (hist.length < rows - 1) hist.unshift('');
    return h('div', { className: cx('r10 r10-cmd', p.className) },
      hist.map(function (l, i) { return h('div', { key: i, className: 'r10-hist' }, l); }),
      h('div', null, (p.prompt == null ? 'Command: ' : p.prompt) + (p.value || ''), p.cursor === false ? null : h('span', { className: 'r10-cursor', 'aria-hidden': true })));
  }

  /* ---------- dialog parts ---------- */
  function Dialog(p) {
    return h('div', { className: cx('r10 r10-dlg', p.className), role: 'dialog', 'aria-label': p.title, style: Object.assign({ width: p.width }, p.style) },
      h('div', { className: 'r10-dlg-title' }, p.title),
      h('div', { className: 'r10-dlg-body' }, p.children,
        p.actions ? h('div', { className: 'r10-row is-end' }, p.actions) : null));
  }
  function Button(p) {
    var rest = Object.assign({}, p); delete rest.isDefault; delete rest.focus; delete rest.pressed;
    return h('button', Object.assign({ type: 'button' }, rest, { className: cx('r10-btn', p.isDefault && 'is-default', p.focus && 'is-focus', p.pressed && 'is-pressed', p.className) }), p.children);
  }
  function EditBox(p) {
    var cols = p.cols || 12;
    return h('label', { className: cx('r10-edit', p.className) }, p.label,
      h('input', { defaultValue: p.value, value: p.onChange ? p.value : undefined, onChange: p.onChange, style: { width: cols * 8 + 10 }, spellCheck: false, 'aria-label': p.label }));
  }
  function Toggle(p) {
    var st = React.useState(!!p.checked), on = p.onChange ? !!p.checked : st[0];
    return h('button', { type: 'button', role: 'checkbox', 'aria-checked': on, className: cx('r10-toggle', p.className),
      onClick: function () { if (p.onChange) p.onChange(!on); else st[1](!on); } },
      h('span', { className: 'r10-box', 'aria-hidden': true }, on ? 'X' : ' '), p.label);
  }
  function ListBox(p) {
    var rows = p.rows || 6, items = p.items || [];
    return h('div', { className: cx('r10-list', p.className), role: 'listbox', style: { height: rows * 16 + 2, width: p.width } },
      items.slice(0, rows).map(function (it, i) {
        return h('div', { key: i, role: 'option', 'aria-selected': i === p.selected, className: i === p.selected ? 'is-active' : null }, it);
      }));
  }
  function ColorSwatch(p) {
    var n = p.aci || 7, s = p.size || 16;
    return h('span', { className: cx('r10-swatch', p.className), style: { display: 'inline-flex', gap: 8, alignItems: 'center', whiteSpace: 'pre' } },
      h('span', { style: { width: s, height: s, background: aciVar(n), border: '1px solid var(--rule)', display: 'inline-block' } }),
      p.label === false ? null : (p.label || pad(ACI_NAMES[n] || String(n), 8)));
  }

  function LayerTable(p) {
    var layers = p.layers || [];
    var head = pad('Layer name', 14) + pad('On', 4) + pad('Frz', 5) + pad('Color', 10) + 'Linetype';
    return h('div', { className: cx('r10-list', p.className), style: { width: p.width || 'auto', padding: 0 } },
      h('div', { style: { background: 'var(--panel-shade)', color: 'var(--on-panel-shade)' } }, head),
      layers.map(function (l, i) {
        return h('div', { key: i, className: i === p.selected ? 'is-active' : null, style: { display: 'flex' } },
          pad(l.name, 14) + pad(l.on === false ? '.' : 'On', 4) + pad(l.frozen ? 'F' : '.', 5),
          h('span', { style: { flex: '0 0 88px', display: 'inline-flex', gap: 6, alignItems: 'center' } },
            h('span', { style: { width: 10, height: 10, background: aciVar(l.color), display: 'inline-block', outline: '1px solid var(--rule)' } }),
            ACI_NAMES[l.color] ? ACI_NAMES[l.color].toLowerCase() : String(l.color)),
          (l.linetype || 'CONTINUOUS').toUpperCase());
      }));
  }

  /* ---------- hatch + icon menu ---------- */
  var HATCH = {
    ANSI31: function (c) { return h('path', { d: 'M0 16L16 0M-4 4L4 -4M12 20L20 12', stroke: c }); },
    ANSI32: function (c) { return h('path', { d: 'M0 16L16 0M-4 4L4 -4M12 20L20 12M-4 12L12 -4M4 20L20 4', stroke: c }); },
    ANSI37: function (c) { return h('path', { d: 'M0 16L16 0M-4 4L4 -4M12 20L20 12M0 0L16 16M-4 12L4 20M12 -4L20 4', stroke: c }); },
    BRICK: function (c) { return h('path', { d: 'M0 0.5H16M0 8.5H16M0.5 0V8M8.5 8V16', stroke: c }); },
    NET: function (c) { return h('path', { d: 'M0 0.5H16M0.5 0V16', stroke: c }); },
    DOTS: function (c) { return h('rect', { x: 4, y: 4, width: 1, height: 1, fill: c }); },
    EARTH: function (c) { return h('path', { d: 'M1 2.5H7M1 5.5H7M2.5 9V15M5.5 9V15M9 10.5H15M9 13.5H15M10.5 1V7M13.5 1V7', stroke: c }); },
    SOLID: function (c) { return h('rect', { width: 16, height: 16, fill: c }); }
  };
  var hid = 0;
  function HatchSwatch(p) {
    var id = React.useMemo(function () { hid += 1; return 'r10h' + hid; }, []);
    var w = p.width || 64, ht = p.height || 48, c = aciVar(p.aci), draw = HATCH[p.pattern] || HATCH.ANSI31;
    var sz = p.pattern === 'DOTS' ? 8 : 16;
    return h('svg', { width: w, height: ht, viewBox: '0 0 ' + w + ' ' + ht, className: p.className, role: 'img', 'aria-label': (p.pattern || 'ANSI31') + ' hatch' },
      h('defs', null, h('pattern', { id: id, width: sz, height: sz, patternUnits: 'userSpaceOnUse' }, h('g', { strokeWidth: 1, fill: 'none' }, draw(c)))),
      h('rect', { width: w, height: ht, fill: 'url(#' + id + ')' }));
  }
  function IconMenu(p) {
    var cols = p.columns || 4;
    return h('div', { className: cx('r10 r10-imenu', p.className) },
      h('div', { className: 'r10-imenu-title' }, p.title),
      h('div', { className: 'r10-imenu-grid', style: { gridTemplateColumns: 'repeat(' + cols + ', auto)' } },
        (p.tiles || []).map(function (t, i) {
          return h('div', { key: i, className: i === p.active ? 'is-active' : null },
            t.render ? t.render() : h(HatchSwatch, { pattern: t.pattern, aci: t.aci }), t.label || t.pattern);
        })));
  }

  /* ---------- drawing area ---------- */
  function Line(p) {
    return h('line', { className: 'ent', x1: p.x1, y1: p.y1, x2: p.x2, y2: p.y2, stroke: aciVar(p.aci), strokeWidth: p.weight,
      strokeLinecap: p.linetype === 'dot' || p.linetype === 'dashdot' ? 'round' : 'butt', style: { strokeDasharray: ltVar(p.linetype) } });
  }
  function Circle(p) {
    return h('circle', { className: 'ent', cx: p.cx, cy: p.cy, r: p.r, stroke: aciVar(p.aci), strokeWidth: p.weight, style: { strokeDasharray: ltVar(p.linetype) } });
  }
  function DrawingText(p) {
    return h('text', { x: p.x, y: p.y, fill: aciVar(p.aci), fontSize: p.size || 16, textAnchor: p.anchor || 'start',
      style: { fontFamily: p.duplex ? 'var(--font-duplex)' : 'var(--font-simplex)' }, transform: p.rotate ? 'rotate(' + p.rotate + ' ' + p.x + ' ' + p.y + ')' : undefined }, p.children);
  }
  function arrow(x, y, ang, c, s) {
    var a1 = ang + Math.PI - 0.22, a2 = ang + Math.PI + 0.22;
    return h('path', { d: 'M' + x + ' ' + y + 'L' + (x + s * Math.cos(a1)) + ' ' + (y + s * Math.sin(a1)) + 'L' + (x + s * Math.cos(a2)) + ' ' + (y + s * Math.sin(a2)) + 'Z', fill: c, stroke: 'none' });
  }
  function LinearDim(p) {
    var c = aciVar(p.aci || 2), off = p.offset == null ? -24 : p.offset, vert = p.direction === 'vertical';
    var x1 = p.x1, y1 = p.y1, x2 = p.x2, y2 = p.y2, a, b, txt, g = [];
    if (!vert) {
      var s = off < 0 ? -1 : 1, y = (off < 0 ? Math.min(y1, y2) : Math.max(y1, y2)) + off; a = [x1, y]; b = [x2, y];
      g.push(h('path', { key: 'e', className: 'ent', stroke: c, d: 'M' + x1 + ' ' + (y1 + 3 * s) + 'V' + (y + 4 * s) + 'M' + x2 + ' ' + (y2 + 3 * s) + 'V' + (y + 4 * s) }));
      txt = h('text', { key: 't', x: (x1 + x2) / 2, y: y - 4, textAnchor: 'middle', fill: c, fontSize: p.size || 14 }, p.text || (Math.abs(x2 - x1) / (p.unit || 16)).toFixed(4));
    } else {
      var x = Math.max(x1, x2) + Math.abs(off); a = [x, y1]; b = [x, y2];
      g.push(h('path', { key: 'e', className: 'ent', stroke: c, d: 'M' + (x1 + 3) + ' ' + y1 + 'H' + (x + 4) + 'M' + (x2 + 3) + ' ' + y2 + 'H' + (x + 4) }));
      var my = (y1 + y2) / 2;
      txt = h('text', { key: 't', x: x - 4, y: my, textAnchor: 'middle', fill: c, fontSize: p.size || 14, transform: 'rotate(-90 ' + (x - 4) + ' ' + my + ')' }, p.text || (Math.abs(y2 - y1) / (p.unit || 16)).toFixed(4));
    }
    var ang = Math.atan2(b[1] - a[1], b[0] - a[0]);
    g.push(h('line', { key: 'd', className: 'ent', stroke: c, x1: a[0], y1: a[1], x2: b[0], y2: b[1] }));
    g.push(h(F, { key: 'a' }, arrow(a[0], a[1], ang + Math.PI, c, 9), arrow(b[0], b[1], ang, c, 9)));
    g.push(txt);
    return h('g', { className: 'r10-dim-g' }, g);
  }
  function Viewport(p) {
    var w = p.width || 480, ht = p.height || 320, pitch = p.gridPitch || 16, kids = [];
    if (p.grid) {
      var dots = [];
      for (var gx = pitch; gx < w; gx += pitch) for (var gy = pitch; gy < ht; gy += pitch) dots.push('M' + gx + ' ' + gy + 'h1');
      kids.push(h('path', { key: 'grid', d: dots.join(''), stroke: 'var(--ink-faint)', strokeWidth: 1, shapeRendering: 'crispEdges' }));
    }
    kids.push(h('g', { key: 'ents' }, p.children));
    if (p.ucs !== false) {
      var ox = 16.5, oy = ht - 16.5, L = 40;
      kids.push(h('g', { key: 'ucs', stroke: 'var(--ink)', fill: 'none', strokeWidth: 1, shapeRendering: 'crispEdges' },
        h('path', { d: 'M' + ox + ' ' + oy + 'H' + (ox + L) + 'M' + (ox + L - 6) + ' ' + (oy - 4) + 'L' + (ox + L) + ' ' + oy + 'L' + (ox + L - 6) + ' ' + (oy + 4) }),
        h('path', { d: 'M' + ox + ' ' + oy + 'V' + (oy - L) + 'M' + (ox - 4) + ' ' + (oy - L + 6) + 'L' + ox + ' ' + (oy - L) + 'L' + (ox + 4) + ' ' + (oy - L + 6) }),
        h('rect', { x: ox, y: oy - 8, width: 8, height: 8 }),
        h('text', { x: ox + L + 4, y: oy + 5, fill: 'var(--ink)', stroke: 'none', fontSize: 13 }, 'X'),
        h('text', { x: ox - 4, y: oy - L - 6, fill: 'var(--ink)', stroke: 'none', fontSize: 13 }, 'Y'),
        h('text', { x: ox + 12, y: oy - 14, fill: 'var(--ink)', stroke: 'none', fontSize: 11 }, 'W')));
    }
    if (p.crosshair) {
      var chx = Math.round(p.crosshair.x) + 0.5, chy = Math.round(p.crosshair.y) + 0.5, pb = p.pickbox ? 4 : 0;
      kids.push(h('g', { key: 'xh', stroke: 'var(--ink)', strokeWidth: 1, shapeRendering: 'crispEdges', fill: 'none' },
        h('path', { d: 'M0 ' + chy + 'H' + (chx - pb) + 'M' + (chx + pb) + ' ' + chy + 'H' + w + 'M' + chx + ' 0V' + (chy - pb) + 'M' + chx + ' ' + (chy + pb) + 'V' + ht }),
        pb ? h('rect', { x: chx - pb, y: chy - pb, width: pb * 2, height: pb * 2 }) : null));
    }
    return h('svg', { className: cx('r10-vp', p.className), width: w, height: ht, viewBox: '0 0 ' + w + ' ' + ht, role: 'img', 'aria-label': p.label || 'Drawing area', style: p.style }, kids);
  }

  function TitleBlock(p) {
    function cell(label, val, extra) { return h('div', extra || null, h('small', null, label), val); }
    return h('div', { className: cx('r10-tb', p.className), style: { width: p.width } },
      h('div', { className: 'r10-tb-title' }, h('small', null, p.company || 'TITLE'), p.title),
      cell('DWG NO', p.number || '-'), cell('REV', p.rev || 'A'),
      cell('SCALE', p.scale || '1:1'), cell('SHEET', p.sheet || '1 OF 1'),
      cell('DRAWN', p.drawn || '-'), cell('DATE', p.date || '-'), cell('UNITS', p.units || 'INCH'));
  }

  function Workspace(p) {
    var w = p.width || 640, ht = p.height || 400, mw = 68, drawW = w - mw, drawH = ht - (p.menuBar ? 17 : 16) - 49;
    return h('div', { className: cx('r10 r10-ws', p.className), style: { width: w, height: ht } },
      p.menuBar ? h(MenuBar, p.menuBar) : h(StatusLine, p.status || {}),
      h('div', { className: 'r10-ws-draw' },
        h(Viewport, Object.assign({ width: drawW, height: drawH }, p.viewport || {}), p.children),
        p.pulldown ? h(PulldownMenu, p.pulldown) : null),
      h(ScreenMenu, Object.assign({ style: { height: drawH } }, p.screenMenu || {})),
      h(CommandLine, p.command || {}));
  }

  function TextModeMenu(p) {
    var rows = [];
    (p.header || []).forEach(function (l) { rows.push(l); });
    if (p.header) rows.push('');
    if (p.heading) { rows.push(p.heading); rows.push(''); }
    (p.options || []).forEach(function (o) { rows.push(o === '' ? '' : '  ' + o.n + '.  ' + o.label); });
    rows.push('');
    return h('div', { className: cx('r10 r10-tmenu', p.className), style: p.style },
      rows.map(function (r, i) { return h('div', { key: i }, r); }),
      h('div', null, (p.prompt || 'Enter selection: ') + (p.value || ''), p.cursor === false ? null : h('span', { className: 'r10-cursor', 'aria-hidden': true })));
  }

  window.R10 = Object.assign(window.R10 || {}, {
    TextModeMenu: TextModeMenu,
    Workspace: Workspace, StatusLine: StatusLine, MenuBar: MenuBar, PulldownMenu: PulldownMenu, ScreenMenu: ScreenMenu, CommandLine: CommandLine,
    Dialog: Dialog, Button: Button, EditBox: EditBox, Toggle: Toggle, ListBox: ListBox, ColorSwatch: ColorSwatch, LayerTable: LayerTable,
    IconMenu: IconMenu, HatchSwatch: HatchSwatch, Viewport: Viewport, Line: Line, Circle: Circle, LinearDim: LinearDim, DrawingText: DrawingText, TitleBlock: TitleBlock,
    HATCH_PATTERNS: Object.keys(HATCH)
  });
})();
