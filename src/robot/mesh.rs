//! URDF mesh resolution and loading.
//!
//! URDF visuals reference meshes through URIs, most commonly
//! `package://<package>/<path>` (see <https://wiki.ros.org/urdf/XML/link>).
//! [`MeshResolver`] turns those URIs into file paths using an explicit
//! package map plus fallback directories; [`load_mesh`] parses the file
//! (STL/OBJ/COLLADA via the `mesh-loader` crate) into renderer-agnostic
//! triangle data, so this module stays free of Bevy types and works in
//! headless tests and wasm (via [`load_mesh_from_slice`]).

use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// Triangulated mesh data ready for conversion into a renderer mesh.
#[derive(Debug, Clone, Default)]
pub struct LoadedMesh {
    pub vertices: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
    /// Per-vertex colors when the file provides them (e.g. COLLADA).
    pub colors: Vec<[f32; 4]>,
}

/// Resolves URDF mesh URIs to local file paths.
#[derive(Debug, Clone, Default)]
pub struct MeshResolver {
    /// Explicit `package name -> directory` mappings (CLI `--package`).
    pub packages: HashMap<String, PathBuf>,
    /// Directories tried in order when the package is not mapped; the
    /// URDF file's own directory and its parent are natural candidates.
    pub fallback_dirs: Vec<PathBuf>,
}

impl MeshResolver {
    /// A resolver anchored at a URDF file location: its directory and the
    /// directory above it become fallbacks, which covers descriptions kept
    /// inside their ROS package (`package://pkg/...` next to the URDF).
    pub fn for_urdf_file(urdf_path: impl AsRef<Path>) -> Self {
        let mut fallback_dirs = Vec::new();
        if let Some(dir) = urdf_path.as_ref().parent() {
            fallback_dirs.push(dir.to_path_buf());
            if let Some(parent) = dir.parent() {
                fallback_dirs.push(parent.to_path_buf());
            }
        }
        Self {
            packages: HashMap::new(),
            fallback_dirs,
        }
    }

    /// Map a package name to a directory.
    pub fn with_package(mut self, name: impl Into<String>, dir: impl Into<PathBuf>) -> Self {
        self.packages.insert(name.into(), dir.into());
        self
    }

    /// Resolve a URDF mesh URI to an existing file path.
    ///
    /// Supported forms: `package://pkg/rest`, `file:///abs/path`, and plain
    /// (possibly relative) paths. For `package://`, the mapped directory is
    /// tried first, then each fallback directory joined with `rest` (with
    /// and without the package-name prefix).
    pub fn resolve(&self, uri: &str) -> Option<PathBuf> {
        if let Some(rest) = uri.strip_prefix("package://") {
            let (package, rel) = rest.split_once('/')?;
            if let Some(dir) = self.packages.get(package) {
                let path = dir.join(rel);
                if path.exists() {
                    return Some(path);
                }
            }
            for dir in &self.fallback_dirs {
                for candidate in [dir.join(rel), dir.join(package).join(rel)] {
                    if candidate.exists() {
                        return Some(candidate);
                    }
                }
            }
            return None;
        }
        if let Some(path) = uri.strip_prefix("file://") {
            let path = PathBuf::from(path);
            return path.exists().then_some(path);
        }
        let path = PathBuf::from(uri);
        if path.exists() {
            return Some(path);
        }
        self.fallback_dirs
            .iter()
            .map(|d| d.join(uri))
            .find(|p| p.exists())
    }
}

/// Load and triangulate a mesh file (STL, OBJ or COLLADA `.dae`).
pub fn load_mesh(path: impl AsRef<Path>) -> anyhow::Result<LoadedMesh> {
    let scene = mesh_loader::Loader::default()
        .merge_meshes(false)
        .load(path.as_ref())?;
    scene_to_mesh(scene, path.as_ref())
}

/// Load a mesh from in-memory bytes; `name_hint` selects the parser by
/// extension (used on wasm where files arrive over HTTP).
pub fn load_mesh_from_slice(bytes: &[u8], name_hint: &str) -> anyhow::Result<LoadedMesh> {
    let scene = mesh_loader::Loader::default()
        .merge_meshes(false)
        .load_from_slice(bytes, name_hint)?;
    scene_to_mesh(scene, Path::new(name_hint))
}

/// Merge submeshes with explicit vertex offsets.
///
/// Not `mesh_loader::Mesh::merge`: as of mesh-loader 0.1.13 it advances the
/// index offset by `last_face[2] + 1`, which mis-stitches COLLADA files whose
/// submesh faces do not end on their highest vertex index (e.g. the UR e-series
/// visual meshes) — most of the geometry then ends up unreferenced.
fn scene_to_mesh(scene: mesh_loader::Scene, path: &Path) -> anyhow::Result<LoadedMesh> {
    let mut out = LoadedMesh::default();
    let all_normals = scene
        .meshes
        .iter()
        .all(|m| m.normals.len() == m.vertices.len());
    let all_colors = scene.meshes.iter().all(|m| {
        m.colors
            .first()
            .is_some_and(|c| c.len() == m.vertices.len())
    });

    for mesh in scene.meshes {
        let offset = out.vertices.len() as u32;
        out.indices
            .extend(mesh.faces.iter().flatten().map(|i| i + offset));
        out.vertices.extend(mesh.vertices);
        if all_normals {
            out.normals.extend(mesh.normals);
        }
        if all_colors {
            out.colors.extend(mesh.colors[0].iter().copied());
        }
    }
    anyhow::ensure!(
        !out.vertices.is_empty(),
        "no geometry in mesh file {}",
        path.display()
    );
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::io::Write;

    /// A minimal ASCII STL with one triangle.
    const TRIANGLE_STL: &str = "solid tri
facet normal 0 0 1
  outer loop
    vertex 0 0 0
    vertex 1 0 0
    vertex 0 1 0
  endloop
endfacet
endsolid tri
";

    #[test]
    fn loads_ascii_stl() {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join("tri.stl");
        fs::File::create(&path)
            .and_then(|mut f| f.write_all(TRIANGLE_STL.as_bytes()))
            .expect("write stl");

        let mesh = load_mesh(&path).expect("loads");
        assert_eq!(mesh.vertices.len(), 3);
        assert_eq!(mesh.indices.len(), 3);
    }

    #[test]
    fn loads_from_slice() {
        let mesh =
            load_mesh_from_slice(TRIANGLE_STL.as_bytes(), "tri.stl").expect("loads from bytes");
        assert_eq!(mesh.vertices.len(), 3);
    }

    /// Two-submesh COLLADA whose first submesh's *last* face references low
    /// vertex indices: `mesh_loader::Mesh::merge` (0.1.13) computes the next
    /// submesh's offset from that face and mis-stitches the geometry, which
    /// is why [`scene_to_mesh`] merges with explicit per-submesh offsets.
    const TWO_SUBMESH_DAE: &str = r##"<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <library_geometries>
    <geometry id="g1"><mesh>
      <source id="g1-pos">
        <float_array id="g1-pos-array" count="18">0 0 0 1 0 0 0 1 0 2 0 0 3 0 0 2 1 0</float_array>
        <technique_common><accessor source="#g1-pos-array" count="6" stride="3">
          <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
        </accessor></technique_common>
      </source>
      <vertices id="g1-v"><input semantic="POSITION" source="#g1-pos"/></vertices>
      <triangles count="2"><input semantic="VERTEX" source="#g1-v" offset="0"/>
        <p>3 4 5 0 1 2</p>
      </triangles>
    </mesh></geometry>
    <geometry id="g2"><mesh>
      <source id="g2-pos">
        <float_array id="g2-pos-array" count="9">5 0 0 6 0 0 5 1 0</float_array>
        <technique_common><accessor source="#g2-pos-array" count="3" stride="3">
          <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
        </accessor></technique_common>
      </source>
      <vertices id="g2-v"><input semantic="POSITION" source="#g2-pos"/></vertices>
      <triangles count="1"><input semantic="VERTEX" source="#g2-v" offset="0"/>
        <p>0 1 2</p>
      </triangles>
    </mesh></geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="scene">
      <node id="n1"><instance_geometry url="#g1"/></node>
      <node id="n2"><instance_geometry url="#g2"/></node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>"##;

    #[test]
    fn merges_collada_submeshes_with_correct_offsets() {
        let mesh = load_mesh_from_slice(TWO_SUBMESH_DAE.as_bytes(), "two.dae").expect("loads");
        // All vertices from both submeshes must be referenced by indices.
        let max_idx = *mesh.indices.iter().max().expect("has indices");
        assert_eq!(
            max_idx as usize,
            mesh.vertices.len() - 1,
            "second submesh's faces must reference its own (offset) vertices"
        );
        // The second submesh's triangle lives at x>=5; make sure it survived.
        let has_far_vertex = mesh
            .indices
            .iter()
            .any(|&i| mesh.vertices[i as usize][0] >= 5.0);
        assert!(
            has_far_vertex,
            "second submesh's geometry was lost in merge"
        );
    }

    #[test]
    fn resolves_package_uri_via_map_and_fallbacks() {
        let dir = tempfile::tempdir().expect("tempdir");
        let meshes = dir.path().join("meshes");
        fs::create_dir_all(&meshes).expect("mkdir");
        let file = meshes.join("part.stl");
        fs::write(&file, TRIANGLE_STL).expect("write");

        // Explicit mapping.
        let resolver = MeshResolver::default().with_package("my_robot_description", dir.path());
        assert_eq!(
            resolver.resolve("package://my_robot_description/meshes/part.stl"),
            Some(file.clone())
        );

        // Fallback: URDF sitting inside the package directory.
        let resolver = MeshResolver::for_urdf_file(dir.path().join("robot.urdf"));
        assert_eq!(
            resolver.resolve("package://my_robot_description/meshes/part.stl"),
            Some(file.clone())
        );

        // Unresolvable.
        assert_eq!(
            MeshResolver::default().resolve("package://nowhere/meshes/part.stl"),
            None
        );

        // Plain relative path against fallback dir.
        let resolver = MeshResolver::for_urdf_file(dir.path().join("robot.urdf"));
        assert_eq!(resolver.resolve("meshes/part.stl"), Some(file));
    }
}
