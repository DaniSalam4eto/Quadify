# Quadify

A small Blender add-on that helps with UV mapping aprons and taxiways so you can curve the UVs.

If you have ever tried to texture a curved apron or taxiway, you know the texture likes to stretch and break around the bends. Quadify rebuilds the selected strip into clean quads and lays down a straight, even UV that follows the curve, so your asphalt and markings flow along the shape without stretching.

## Installation

1. Download `quadify.py` from this repository.
2. In Blender, go to Edit > Preferences > Add-ons.
3. Click Install from Disk (or Install) and pick `quadify.py`.
4. Enable the checkbox next to Quadify.

## Usage

1. Enter Edit Mode (Tab).
2. Select the faces of the strip you want to map.
3. Right-click and choose Quadify, or use the button in the top bar.

After running it, you can tweak the result in the redo panel at the bottom left of the viewport:

- Rebuild as Quads: on by default. Turn it off to only reassign UVs without changing the mesh.
- Length Segments and Width Segments: control the density of the quad grid.
- Tiling: how many times the texture repeats along the length.
- Fill Width, Rotate 90, Flip Length, Flip Width: adjust the texture scale and orientation.

## Notes

- Select one strip at a time for the best result. A strip with holes or branches cannot be traced cleanly.
- Rebuild as Quads replaces the strip's geometry with a clean quad grid. The new edges sit on the original outline, so the shape is preserved. Press Ctrl+Z to undo if needed.
- Tested on Blender 3.6.

## License

Apache License 2.0. See the LICENSE file for details.
