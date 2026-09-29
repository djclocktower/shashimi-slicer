# Interface Themes

## Why it exists

The interface can be drawn in an optional theme instead of the regular light and dark
modes. The one theme today is **R10 Drafting**, a drafting workstation from the late
1980s: a black screen, green system text in the IBM VGA 8x16 bitmap face, gray
inverse-video highlight bars, red 1px frame rules and square corners. Its design system
(palette, type, rules) is in `design/r10-drafting/`.

The choice is `Preferences > General > Interface theme`, stored as `ui_theme` in
`AppConfig` (`default` or `r10`). `default` leaves every existing code path as it was.

## Latched at startup

`UITheme::init()` (`src/slic3r/GUI/UITheme.cpp`) reads the key once, at the start of
`GUI_App::on_init_inner()`, before fonts, label colors or any window exist. The theme
then stays fixed for the session. Fonts, widget colors and cached bitmaps are all built
from it, and switching them live would leave windows half in one theme and half in the
other, so a change in Preferences applies on the next launch.

## A layer on top of dark mode

R10 is a black screen, so it forces the dark color mode: `GUI_App::dark_mode()` returns
true while it is active, on every platform. On GTK it asks for the dark theme variant
and on macOS it sets the dark Aqua appearance, so native controls match. Everything
that already reacts to dark mode then only needs its dark colors swapped for the R10
palette. The layers:

| Layer | Where | How |
|-------|-------|-----|
| Custom widgets and `UpdateDarkUI` | `Widgets/StateColor.cpp` | `gR10Colors` replaces `gDarkColors` as the light-to-dark map |
| Buttons | `Widgets/Button.cpp` | R10 style tables: framed words, gray bar on hover, gray panel for the default action |
| Corners | `Widgets/StaticBox.cpp` | square whatever radius the widget asks for |
| Labels | `GUI_App::init_label_colours()` | system text, yellow for modified values |
| Fonts | `Widgets/Label.cpp`, `ImGuiWrapper::init_font()` | the VGA face, one weight |
| Icons | `BitmapCache::load_svg()`, `GLTexture` | rasterized icons recolored by `UITheme::recolor_icon()` |
| Windows native chrome | `dark_mode.cpp` | the NppDarkMode colors |
| 3D view | `GLCanvas3D::on_change_color_mode()` | black background and plate, ink-faint grid lines |
| Overlays | `ImGuiWrapper` | global style, plus a remap of the finished draw lists |

### The color map

`gR10Colors` has a key for every light color `gDarkColors` maps, and sends each to one of
six final values: ground, system text, highlight gray, panel-shade gray, rule red and
focus yellow. None of the final values is itself a key, so mapping a color that was
already mapped leaves it alone. That matters because `UpdateDarkUI` reads a window's
current color and maps it again. It is also why the ground is `#010101` and not
`#000000`: pure black is the key for light-mode text. Text that sits on the accent in
light mode (`#FFFFFE`, `#FEFEFE`, `#FFFFFD`) turns black, because the accent turns gray.
`tests/slic3rutils/test_ui_theme.cpp` checks the coverage, the palette and the
idempotence.

### Icons

`UITheme::recolor_icon()` works on rasterized pixels rather than SVG source, so it covers
every icon without a list of their colors. Neutral tones go to three steps: light line
work becomes system text, mid-gray button fills panel shade and dark fills ground, so
lines stay readable over the fills they sit on. The accent becomes white. Saturated colors are left alone, since
they carry meaning (warnings, errors, axes). Filament-colored icons and printer
thumbnails are pictures of real things and are not recolored. Bed textures are not
either; in `GLTexture` only `_dark.svg` UI icons are.

### Overlays

Most ImGui windows push their own dark-mode colors at the call site. Rather than touch
each one, `render_draw_data()` rewrites vertex colors in the finished draw lists, the way
the wx map works: only the exact chrome colors of the dark styles in `ImGuiWrapper` are
replaced (window and toolbar backgrounds, buttons, separators, the accent, 88% white
text). Filament and legend colors never match those exactly, and vertices drawn with a
texture other than the font atlas (icons, thumbnails) keep their tint. The shared style
helpers also drop their rounding and draw a 1px frame.

## Fonts and languages

The VGA face covers Latin, Greek and Cyrillic. For Chinese, Japanese, Korean, Thai and
Vietnamese the theme keeps its colors but uses the regular fonts, since the bitmap face
has no glyphs for them (`UITheme::use_r10_font()`). In ImGui the regular font is merged
behind the VGA face for the symbols it lacks. The face is bold-less by design, so bold
requests get the regular weight.
