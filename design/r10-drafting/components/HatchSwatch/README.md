# HatchSwatch

A rectangle filled with one of the standard hatch patterns (ANSI31, ANSI32, ANSI37, BRICK, NET, DOTS, EARTH, SOLID) in an ACI color.

Provide `pattern`, `aci`, `width` and `height`. `window.R10.HATCH_PATTERNS` lists the names.

- ANSI31 (45° lines at 1/8") is the default section hatch; draw it in a single thin color, never `ink` over busy geometry.
