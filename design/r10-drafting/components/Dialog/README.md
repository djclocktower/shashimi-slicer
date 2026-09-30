# Dialog

A modal dialog: `panel` body, `panel-shade` title bar with centered title, square 1px `on-panel` frame, controls on the character grid, buttons centered on the last row.

Provide `title`, `width` (a multiple of 8), `children` (rows of controls; wrap a horizontal group in `<div className="r10-row">`) and `actions` (Buttons).

- Button order is always OK, Cancel, then Help...; OK is the default.
- Label text is sentence case; pad labels with spaces so fields align in columns.
- No icons, no shadows, no rounded corners.
