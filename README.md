# fbx2md5

A command-line converter from FBX to idTech4 MD5 skeletal mesh and
animation formats. A single invocation reads one FBX and writes:

- `<output>.md5mesh` — the rest-pose skinned mesh plus its skeleton
- `<output>_<stack-name>.md5anim` — one file per animation stack in the FBX

The input FBX must contain a skinned mesh with a skeleton. Static meshes and
pure anim-only files are not supported (the MD5 format itself requires both).

Both **MD5 Version 10** (the classic idTech4 format) and **MD5 Version 12**
(StormEngine2 extension with per-vertex normals, MikkTSpace tangents, and
optional vertex colors) are supported.

## Building

### Windows (Visual Studio 2022)

Requires CMake 3.15+ and Visual Studio 2022 with the **Desktop development
with C++** workload.

```
generate_vs2022.bat
```

Open `build\fbx2md5.sln` in Visual Studio and build the `fbx2md5` target.
The executable lands in `build\Release\fbx2md5.exe`.

You can also build from the command line once the solution exists:

```
cmake --build build --config Release
```

### Linux / macOS

```
make
```

Binary appears as `build/fbx2md5`. Requires `g++` (or `clang++`) with C++17
support, `gcc`/`clang` for the C side, and the standard pthread + libm.

For a debug build: `make debug`. To clean: `make clean`.

The CMakeLists.txt also works on Linux if you prefer:

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## Usage

```
fbx2md5 input.fbx output [-scale X.X] [-fps N] [-noaxes] [-nounits] [-v12]
                         [-noAnimPrefix]
```

### Positional arguments

| Argument     | Description |
|--------------|-------------|
| `input.fbx`  | Source FBX file. Must contain a skinned mesh with a skeleton. |
| `output`     | Output filename stem (no extension). Written as `<output>.md5mesh` plus one `<output>_<stack>.md5anim` per FBX animation stack. |

### Options

| Flag             | Description |
|------------------|-------------|
| `-scale X.X`     | Extra uniform multiplier applied to all positions. Default 1.0. |
| `-fps N`         | Override the animation sample rate. Defaults to the FBX scene's declared frame rate. |
| `-noaxes`        | Skip idTech4 axis conversion. By default the scene is re-oriented to right-handed, Z-up. |
| `-nounits`       | Skip FBX unit conversion. By default the tool assumes the FBX is authored in centimeters (the Maya / Max / Blender default) and divides positions by 100 so that one output unit equals one meter. Pass this flag when the source FBX is already authored in idTech4 units. |
| `-v12`           | Emit MD5 Version 12 mesh output. Adds per-vertex normals, MikkTSpace tangents with bitangent sign, and (when the FBX has them) vertex colors. Requires a v12-aware engine build such as [StormEngine2](https://github.com/motorsep/StormEngine2). Animation output remains Version 10 — the engine accepts v10 anims against v12 meshes. |
| `-noAnimPrefix`  | Drop the mesh-stem prefix from animation filenames. Writes `<stack>.md5anim` (e.g. `slash1.md5anim`) instead of the default `<o>_<stack>.md5anim` (e.g. `hero_slash1.md5anim`). If the output stem includes a directory (`path/to/hero`), that directory is preserved — the anims land next to the mesh. |

### Animation filenames and stack-name cleanup

Blender exports animation stacks with the owning armature's name glued to
the front, separated by a pipe (`|`). A rig named `imp` with an action
`slash1` exports as the stack `imp|slash1` — Blender's action picker shows
this composite name directly, and this is how the name appears inside the
FBX. Blender does this because one Blender action can apply to several
objects, so the exporter disambiguates by prefixing the owner.

Other DCC tools do **not** do this:

- **Maya** names stacks `Take 001`, `Take 002`, … by default, or the scene
  filename when "Use scene name" is enabled.
- **3ds Max** — identical behavior to Maya.
- **Cascadeur** exports one animation per FBX file; there's no multi-take
  naming scheme to worry about.
- **Akeytsu** uses animation names straight from its Animbank (e.g. `Run`,
  `Idle`) with no object prefix.

The converter strips everything up to and including the **last** `|` in
the stack name when building the output filename. On Blender output this
is the fix you want: `imp|slash1` → `slash1`. On everything else it's a
no-op because the name has no `|` in it. This happens unconditionally;
`-noAnimPrefix` only controls the mesh-stem prefix on top.

Combined with the default `output` stem, typical results look like:

| Source tool & stack name           | Without `-noAnimPrefix`       | With `-noAnimPrefix`  |
|------------------------------------|-------------------------------|-----------------------|
| Blender: `imp\|slash1`             | `output_slash1.md5anim`       | `slash1.md5anim`      |
| Blender: `character\|idle`         | `output_idle.md5anim`         | `idle.md5anim`        |
| Maya / Max: `Take 001`             | `output_Take_001.md5anim`     | `Take_001.md5anim`    |
| Akeytsu / Cascadeur: `walk`        | `output_walk.md5anim`         | `walk.md5anim`        |

### Tell if `-nounits` is needed

Look at the `scene unit: X m/unit` line the tool prints at load time. If it
says `0.010000`, the file is cm-authored and the default is correct. If it
says `1.000000` or similar and your output looks 100× too small, add
`-nounits`.

### About `-v12`

MD5 Version 12 ships pre-baked tangent-space data with the mesh, which skips
the engine's runtime tangent derivation and lets you ship meshes whose
lighting is authored exactly the way the DCC tool (and MikkTSpace) intended.
Practical consequences:

- The FBX **must** have vertex normals. If it doesn't, the tool asks ufbx to
  generate them automatically — but hand-authored normals are always better.
- Tangents and bitangents are read from the FBX if present; the bitangent
  sign (the 4th tangent component) is computed from the handedness of the
  tangent basis and is critical for normal maps to shade correctly.
- Vertex colors are exported only when the FBX actually contains a color
  channel. Otherwise the `numvertexcolors` block is omitted.
- Seam vertices (same position, different normal/tangent) are kept separate,
  so v12 meshes typically have a slightly higher vertex count than the same
  model exported as v10.
- Loading a v12 mesh in a stock idTech4 engine will fail with a
  version check error. Use v10 (the default) for stock engines.

### Examples

Convert a standard Blender-exported character:

```
fbx2md5 hero.fbx hero
```

Convert the same character as MD5 Version 12 for StormEngine2:

```
fbx2md5 hero.fbx hero -v12
```

Convert an asset already authored in idTech4 units:

```
fbx2md5 hero.fbx hero -nounits
```

Force 60 fps sampling and double the size:

```
fbx2md5 hero.fbx hero -fps 60 -scale 2.0
```

Drop the mesh-stem prefix from anim filenames (useful when animating a
single character and you want `walk.md5anim`, `run.md5anim`, etc.):

```
fbx2md5 hero.fbx hero -noAnimPrefix
```

## Acknowledgements

FBX parsing is provided by [**ufbx**](https://github.com/ufbx/ufbx) by
Samuli Raivio — a single-file C library that reads the full FBX format
including skinning, blend shapes, and animation curves. It's bundled as
`src/ufbx.c` / `src/ufbx.h` and distributed under its own MIT license.
Many thanks to the ufbx authors; this tool would be orders of magnitude
more work without it.

The MD5 mesh/anim format specification was cross-referenced against the
idTech4 Maya exporter source code during development. The MD5 Version 12
output path was cross-referenced against the
[StormEngine2 MD5v12 engine guide](https://github.com/motorsep/StormEngine2/blob/master/MD5v12_ENGINE_GUIDE.md)
to match the expected bone-local
normal/tangent convention.

## License

MIT. See `LICENSE`.
