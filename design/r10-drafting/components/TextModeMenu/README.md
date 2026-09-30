# TextModeMenu

The text-mode screen shown before a drawing opens: a header block, a heading, a numbered option list with blank-row groups, and a selection prompt, all in `system-text` green on black.

Provide `header` (lines), `heading`, `options` (`{n, label}` or `''` for a blank row), `prompt` (default "Enter selection: "), `value` and `cursor`.

- Options are numbered from 0, where 0 is always the exit. Two spaces before the number, a period and two spaces after it.
- Defaults are offered in angle brackets in the prompt: `Enter selection <0>:`.
- Long lists stop with `-- Press RETURN for more --`.
