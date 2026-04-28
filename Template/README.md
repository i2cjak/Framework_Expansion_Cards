# American Embedded KiCad Template

<p align="center">
  <img src="meta/logo.png" alt="American Embedded Logo">
</p>

This repository contains a KiCad project template provided by American Embedded. It is designed to be a starting point for new electronic designs, pre-configured with common settings, layers, and manufacturing outputs.

## Features

*   **Manufacturing Outputs:** Includes a `Build.kicad_jobset` file for generating common manufacturing data for JLCPCB, NextPCB
*   **Build Script:** A `build.sh` script is provided to automate the process of running the jobset and formatting the component placement file.

## Usage

Use this repository as a template directly in KiCad.

1.  In the KiCad project manager, select **File > New Project from Template...**
2.  Navigate to the directory containing this repository.
3.  KiCad will prompt for a project name and handle all file renaming automatically.
4.  You can then begin your design work as usual.

## Building

To generate all the output files, run the provided shell script:

```bash
./build.sh
```

This script will:
1.  Execute the KiCad CLI to run the jobs defined in `Build.kicad_jobset`.
2.  Place all generated files into the `build/` directory.
3.  Convert the raw component placement file into supported format

The `Build.kicad_jobset` is configured to output:
- ERC and DRC reports
- PDF of the schematic
- BOMs for JLCPCB and NextPCB
- Gerbers and drill files (in a zip archive)
- 3D model (STEP file)
- Component position files
