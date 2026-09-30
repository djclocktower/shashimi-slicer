# CommandLine

The three-row command area at the bottom, behind a 1px `rule`: two rows of history and the live prompt, all in `system-text` green, flush to column 0, with a blinking underscore cursor in `focus`.

Provide `history` (strings, oldest first), `prompt` (default "Command: "), `value` (typed text) and `rows` (default 3). `cursor: false` hides the cursor.

- Prompts follow the era's grammar: options separated by "/", the default in angle brackets — "Diameter/<Radius>:".
- Errors and state changes are plain text in the history: `*Cancel*`, `*Invalid*`, `<Grid on>`, `<Ortho on>`, `Grid too dense to display` — no color, no icons.
- The cursor blinks at 1.06s and stops under reduced motion.
