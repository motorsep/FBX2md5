# fbx2md5

A command-line converter from FBX to Doom 3 / idTech 4 MD5 skeletal mesh and
animation formats. A single invocation reads one FBX and writes:

- `<output>.md5mesh` — the rest-pose skinned mesh plus its skeleton
- `<output>_<stack-name>.md5anim` — one file per animation stack in the FBX

The input FBX must contain a skinned mesh with a skeleton. Static meshes and
pure anim-only files are not supported (the MD5 format itself requires both).

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
fbx2md5 input.fbx output [-scale X.X] [-fps N] [-noaxes] [-nounits]
```

### Positional arguments

| Argument     | Description |
|--------------|-------------|
| `input.fbx`  | Source FBX file. Must contain a skinned mesh with a skeleton. |
| `output`     | Output filename stem (no extension). Written as `<output>.md5mesh` plus one `<output>_<stack>.md5anim` per FBX animation stack. |

### Options

| Flag          | Description |
|---------------|-------------|
| `-scale X.X`  | Extra uniform multiplier applied to all positions. Default 1.0. |
| `-fps N`      | Override the animation sample rate. Defaults to the FBX scene's declared frame rate. |
| `-noaxes`     | Skip idTech 4 axis conversion. By default the scene is re-oriented to right-handed, Z-up. |
| `-nounits`    | Skip FBX unit conversion. By default the tool assumes the FBX is authored in centimeters (the Maya / Max / Blender default) and divides positions by 100 so that one output unit equals one meter. Pass this flag when the source FBX is already authored in Doom 3 units. |

### Tell if `-nounits` is needed

Look at the `scene unit: X m/unit` line the tool prints at load time. If it
says `0.010000`, the file is cm-authored and the default is correct. If it
says `1.000000` or similar and your output looks 100× too small, add
`-nounits`.

### Examples

Convert a standard Blender-exported character:

```
fbx2md5 hero.fbx hero
```

Convert an asset already authored in Doom 3 units:

```
fbx2md5 hero.fbx hero -nounits
```

Force 60 fps sampling and double the size:

```
fbx2md5 hero.fbx hero -fps 60 -scale 2.0
```

## Acknowledgements

FBX parsing is provided by [**ufbx**](https://github.com/ufbx/ufbx) by
Samuli Raivio — a single-file C library that reads the full FBX format
including skinning, blend shapes, and animation curves. It's bundled as
`src/ufbx.c` / `src/ufbx.h` and distributed under its own MIT license.
Many thanks to the ufbx authors; this tool would be orders of magnitude
more work without it.

The MD5 mesh/anim format specification was cross-referenced against the
idTech 4 Maya exporter source code during development.

## License

MIT. See `LICENSE`.
