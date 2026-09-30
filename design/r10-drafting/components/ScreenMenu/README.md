# ScreenMenu

The right-hand column menu: a title, the `* * * *` row, then one command per row in `system-text`, 8 columns wide behind a 1px `rule` divider and 3px gap.

Provide `items` (strings of 8 characters max, or `{label, dim}`; an empty string leaves a blank row), `active`, `onSelect(item, index)` and `footer` (rows pinned to the bottom, such as `__LAST__` and the parent menu name centered in 8 columns). `title` defaults to "R10".

- Labels are ALL CAPS, 8 characters max; they may fill the full width. A trailing colon means it runs a command that takes options; no colon means it opens a submenu. Paging items are lowercase: `next`, `previous`.
- The item under the menu cursor is a full-width `highlight` bar with `on-highlight` text (gray on gray, 3.2:1, as the original draws it).
- Blank rows group items; there are no divider lines inside the menu.
