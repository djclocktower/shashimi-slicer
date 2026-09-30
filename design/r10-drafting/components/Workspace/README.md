# Workspace

The complete drawing editor: status line, drawing area, right-hand screen menu and three-line command area, sized in character cells.

Use it as the frame of any full-screen experience. Provide `status` (StatusLine props), `screenMenu` (ScreenMenu props), `command` (CommandLine props), `viewport` (Viewport props) and SVG geometry as children. Pass `menuBar` (MenuBar props) instead of `status` to show the pull-down bar state (the bar replaces the status line while the cursor is at the top), and `pulldown` to drop a PulldownMenu over the drawing.

- The drawing area is always `ground` with geometry in ACI colors; chrome is `ink`/`ink-dim` only.
- Keep the layout fixed: status line top with no rule under it, screen menu right behind a 1px `rule` divider (`screen-menu-width`), command area bottom behind a 1px `rule` (`command-area-height`). Both rules are ACI 1 red on Screen. Do not move or restyle them.
- Width and height should be multiples of 8 and 16 so the text grid lands on whole pixels.
