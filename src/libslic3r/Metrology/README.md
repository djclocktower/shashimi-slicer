# Shashimi Metrology kernel (`Slic3r::Metrology`)

Headless: no wx, no GUI, no libslic3r types, only the standard library. The Measure tab
(`src/slic3r/GUI/Metrology/`) is its only client. The namespace is not `Measure` because
`Slic3r::Measure` already belongs to the measure gizmo.

## What it does

| File | Job |
|---|---|
| `MetrologyTypes.hpp` | `P2`, `P3`, `Region`, `Mat3` (homographies), `GrayImage`, `Tolerance` / `Judge` |
| `Geometry` | Linear solve, normalized-DLT homography, total-least-squares lines, circle fit (Kasa then Gauss-Newton), robust refits, polygon helpers |
| `Image` | Blur, light-on minus light-off subtraction, frame averaging, Otsu, frame motion, rectification onto a mm grid |
| `Edges` | Caliper scans: grey profiles along scan lines, gradient extrema with sub-pixel parabola refinement; rectangle, rectangle-pair (width) and annulus tools |
| `Calibration` | The measuring mat (dot ring with a marker), its detection and homography, two-point scale; `CameraModel` with the camera centre for parallax and the bed-height readout |
| `Features` | The inspection program: line, circle, width, distance and angle features with tolerances and OK/NG |
| `Contour` | Marching squares into regions (outer CCW, holes CW), rasterization, SVG |
| `Reverse` | The reverse-engineering session (survey, grid, step bisection rounds) and the stepped model |
| `Probe` | Touch-batch and repeatability G-code, still-frame detection, repeatability statistics |
| `Budget` | The error budget and inverse-variance fusion |
| `Simulate` | A pinhole camera and nozzle probe over prism parts on the mat, for the GUI's simulator and the tests |

## Conventions

- **Units.** mm and degrees. Image coordinates are pixels, (0, 0) at the centre of the top-left
  pixel, y down.
- **Plate frame.** The mat seen from above: X right, Y up (towards the rear), origin at the centre
  of the marker dot, Z up from the mat's top face. Every mm coordinate uses it. `ProbeSettings`
  places the frame on the printer (`mat_to_printer()`).
- **Regions.** Outer loops CCW, holes CW, so the material is on the left of every loop.
- **Bed offset.** Positive when the bed is closer to the camera than in the reference frame. On a
  P1P a smaller bed Z is a higher bed: offset = reference Z - Z.
- **Errors.** Plain-language sentences in `error` fields; the kernel never throws.

## Assumptions not yet checked on hardware

- The P1P's camera frames can be captured at 720p (the GUI currently loads saved frames).
- `G380 S3` stops the bed at contact and the bed stays there through the `M400 S<n>` dwell, so
  the camera can read where it stopped.
- Trigger repeatability and the nozzle face's effective radius (0.5 mm by default).
- The camera has no significant lens distortion: calibration is a plane homography.
