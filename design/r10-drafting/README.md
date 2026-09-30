A drafting workstation, circa 1988: a black VGA screen, one bitmap font on an 8x16 character grid, seven bright ACI colors for geometry, and vector lettering that looks like it came off a pen plotter. Everything here is either chrome (text-mode UI in `system-text` green) or drawing (thin colored vectors on `ground`). Keep those two worlds apart and the look holds.

## Content fundamentals

The software talks like a terse instrument, not a person.

- Prompts name the input and list the options: `Command:`, `From point:`, `Diameter/<Radius>:`. Options are separated by `/`, the default goes in angle brackets, and every prompt ends with a colon and one space.
- Screen-menu labels are ALL CAPS, 8 characters max. A trailing colon (`LAYER:`) runs a command with options; no colon (`DRAW`) opens a submenu.
- Pull-down and dialog labels are title case (`Drawing Aids`, `X Spacing`). A trailing `...` opens a dialog; `Pick <` closes the dialog so the user can pick on screen.
- Errors and state changes are plain text in the command history: `*Cancel*`, `*Invalid*`, `0 found.`, `<Grid on>`, `<Ortho on>`, `Grid too dense to display`. No color, no icons, no apology.
- Numbers follow the drawing's unit precision, four decimals by default (`-5.1214,-6.1452`); coordinates are comma-separated with no space, angles are bare degrees.
- Text-mode menus number their options from 0 (always the exit) and end with `Enter selection:`; long lists pause on `-- Press RETURN for more --`.
- Drawing text (titles, notes, dimensions) is ALL CAPS: `FLANGE PLATE`, `4X Ø.5625 THRU`.
- No "I", no "you", no exclamation marks, no emoji.

## Visual foundations

### Two themes

- **Screen (VGA)** is the default: `ground` is pure black, geometry uses the bright EGA/VGA values of the ACI colors, system text is `system-text` DOS green, frames are `rule` red, and white `ink` is kept for geometry.
- **Plot (paper)** is the same drawing as it comes off the plotter: `ground` becomes warm vellum, ACI 7 turns black, and each ACI color darkens to a pen that still reads on paper. Use it for sheets, title blocks and printed output, never for the editor chrome.

### Color

- All system text, in the editor and the text-mode screens alike, is `system-text` (#55ff55, VGA bright green; black on Plot). The two frame lines (screen-menu divider, command-area rule) are `rule`, which is ACI 1 red on Screen. `ink-dim` gray is only for dimmed items. White `ink` is geometry (ACI 7), the crosshair and the UCS icon, never system text. Dialogs use `panel`, `on-panel`, `panel-shade`, `field`, `on-field`. Apart from `rule`, ACI colors never style UI.
- Geometry uses `aci-1` to `aci-8` by the drafting convention: visible edges `aci-7`, center lines `aci-3`, hidden lines `aci-4`, dimensions `aci-2`, hatch `aci-1`, construction and reference `aci-6`, underlays `aci-8`.
- `aci-5` blue is only 4.1:1 on Screen: use it for lines and fills, and for text only at 24px or larger.
- `ink-faint` and `aci-8` are for marks, not words: grid dots, disabled glyphs, background geometry.
- The menu cursor is a full-width light-gray `highlight` bar with dark-gray `on-highlight` text. That pair is only 3.2:1, kept because it is what the screen draws; don't reuse it for body text, and never tint a selected item with an ACI color.
- `vga-blue` with `on-vga-blue` is reserved for setup and configuration screens, the DOS-installer blue. It never appears inside the drawing editor.
- The focus mark is solid `focus` (yellow on Screen, blue on Plot) on `ground` and `field`, and a dotted `focus-on-panel` rectangle inside dialogs. Both are at least 9:1 on every surface they land on.
- No gradients, no transparency, no shadows. The palette is sixteen solid colors.

### Type

- All UI text is `screen-text`: the IBM VGA 8x16 bitmap face at exactly 16px on a 16px line. One character is one 8x16 cell. For larger text use `screen-text-2x` or `screen-text-3x`, never an in-between size, so the pixels stay square.
- Drawing lettering uses the vector faces: `drawing-title` (Plotter Duplex, a two-stroke face) for sheet titles, `drawing-label`, `drawing-note` and `dimension` (Plotter Simplex, single-stroke) for everything else. The Simplex face includes the drafting marks Ø ° ±.
- Never mix them: bitmap type is the machine talking, vector type is the drawing.
- Credits: IBM VGA 8x16 is from VileR's Ultimate Oldschool PC Font Pack, CC BY-SA 4.0 (license file in `fonts/`). Plotter Simplex and Duplex are outlined from the public-domain Hershey vector font data.

### Grid, spacing and layout

- Lay out chrome on the character grid: widths in multiples of `cell-x` (8px), heights in multiples of `cell-y` (16px). `half-cell` is the only smaller step, used for focus insets and dialog padding.
- The editor layout is fixed: the status line across the top with no rule under it (`status-line-height`), coordinates starting at the middle column; the 8-column screen menu on the right behind a 1px red divider (`screen-menu-width`); the three-row command area at the bottom behind a 1px red rule (`command-area-height`); the drawing area filling the rest, borderless. Text sits flush at column 0 with no padding. The `Workspace` component assembles it.
- Inside the drawing, the display grid is dots at `grid-pitch`, drawn in `ink-faint`.
- Every corner is square (`radius-0`).

### Linework

- On Screen every entity is a 1px stroke (`stroke-hair`), whatever its role. On Plot, give outlines `stroke-pen-bold` and dimension, hatch, center and hidden lines `stroke-pen-thin`.
- Linetypes are dash patterns: `lt-center` for axes, `lt-hidden` for hidden edges, `lt-phantom` for adjacent parts, `lt-dashed` for alternate positions. `lt-dot` and `lt-dashdot` need round line caps.
- Dimensions use closed filled arrowheads, extension lines that start with a small gap off the object and run 4px past the dimension line, and Simplex text centered above the line.
- Rules and frames are 1px `rule`. Dialogs get a 1px `on-panel` frame; the default button gets a 2px frame.
- Pull-down menus, dialogs, icon menus and grips belong to the later releases of the era (1987–92). The status line, screen menu, command area and text-mode menus are the original 1987 core; lean on those first.

### States and motion

- Hover and keyboard focus in menus show the same inverse `highlight` bar. Pressed buttons fill with `panel-shade`. Disabled items drop to `ink-faint` (menus) or `panel-shade` (dialogs) and never highlight.
- The only motion is the blinking underscore cursor in `focus`, on a 1.06s cycle. It stops under reduced motion. Menus and dialogs appear instantly: no fades, slides or easing.

## Iconography

The system has no icon set. Commands are words, and pictures only appear as vector slides in an `IconMenu`, drawn with the same thin ACI strokes as the drawing.

- Hatch patterns (`assets/Hatch`, or `HatchSwatch` in code) are the main pictorial vocabulary.
- The drawing-area markers (UCS icon, crosshair with pickbox, grips) are in `assets/Glyphs`; `Viewport` draws the first two live.
- Checkmarks are the VGA font's √ glyph, check boxes hold an `X`, and state columns use `.` for off. There is no emoji and no icon font.
- The crosshair is two full-length 1px `ink` lines across the drawing area.

## Don't

- Don't use a proportional UI font, a font size between the 16px steps, or anti-aliased-looking weights and italics in chrome.
- Don't put ACI colors on chrome, or `system-text` green on geometry: a green line reads as ACI 3, a center line.
- Don't round corners, add shadows, or animate panels.
- Don't use a mouse-arrow pointer inside the drawing area: the crosshair is the cursor.
- Don't show a product name or logo. The screen-menu title defaults to "R10".
