# Viewport

The drawing area as an SVG: `ground` field, optional dot grid at `grid-pitch`, the UCS icon bottom-left, and a full-width crosshair with optional pickbox.

Provide `width`, `height`, `grid`, `gridPitch`, `crosshair` (`{x, y}`), `pickbox`, `ucs` (default on) and geometry children (Line, Circle, LinearDim, DrawingText, or raw SVG with ACI color vars).

- Every entity is a 1px stroke; weight belongs to the Plot theme.
- The crosshair spans the whole area in `ink`; there is no arrow pointer inside the drawing.
