# Third-party notices

The Dex URDF and mesh files under
`src/dex_ik/assets/dex_description/` are derived from the
`tiangong2dex_urdf` directory of Open-X-Humanoid/TienKung_URDF, revision
`5c221783fb92fcc4af891ef1dc0502963caf2266`:

https://github.com/Open-X-Humanoid/TienKung_URDF/tree/main/tiangong2dex_urdf

Changes made for this distribution:

- Renamed the model package to `dex_description`.
- Renamed the URDF file to `dex.urdf`.
- Changed the URDF robot name to `dex`.
- Changed package mesh URIs from `tiangong2dex_urdf` to `dex_description`.
- Included only the URDF and mesh files required by the IK package.

The derived URDF and mesh files remain under the OpenAtom Open Hardware
License, Version 1.0. The license text is provided in
`OpenAtom-Open-Hardware-License-1.0.txt` in this directory.
