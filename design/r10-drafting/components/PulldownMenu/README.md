# PulldownMenu

A dropped menu: a 1px `rule` frame on `ground`, one item per row, the item under the cursor in `highlight`/`on-highlight`.

Provide `items`: strings, `'-'` for a separator, or `{label, key, checked, disabled}`; `active` is the highlighted index.

- End a label with "..." when it opens a dialog.
- A `key` is a typed alias shown flush right, not a modifier chord.
- Disabled items use `ink-faint` and never highlight.
