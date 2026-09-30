# Button

A square dialog button on `panel`: 1px frame, 2px frame when `isDefault`, `panel-shade` fill while pressed, dotted `focus-on-panel` rectangle when focused.

Provide `children` (label), `isDefault`, `disabled`, `onClick`. `focus` and `pressed` force those states for documentation.

- Minimum 80px wide (10 cells). Labels are one or two words.
- "Pick <" means the dialog closes so the user can pick in the drawing; "..." means another dialog opens.
- Only inside a Dialog — buttons never sit on the black screen.
