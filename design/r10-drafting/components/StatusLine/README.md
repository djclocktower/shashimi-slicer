# StatusLine

The top line of the screen: "Layer", the current layer name and the active modes, one space apart, then the live cursor coordinates starting at the middle column.

Provide `layer`, `modes` (strings like "Snap", "Ortho", "Tablet"; only ones that are on), `coords` (comma-separated, no spaces, at the drawing's unit precision) and optionally `layerColor` (an ACI chip; later releases only).

- No rule under the status line, no padding: text starts at column 0.
- Coordinates begin at column 40 of 80 and update continuously; never animate them.
- Mode words appear only while on; there is no "off" state to show.
