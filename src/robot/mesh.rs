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
        .merge_meshes(true)
        .load(path.as_ref())?;
    scene_to_mesh(scene, path.as_ref())
}

/// Load a mesh from in-memory bytes; `name_hint` selects the parser by
/// extension (used on wasm where files arrive over HTTP).
pub fn load_mesh_from_slice(bytes: &[u8], name_hint: &str) -> anyhow::Result<LoadedMesh> {
    let scene = mesh_loader::Loader::default()
        .merge_meshes(true)
        .load_from_slice(bytes, name_hint)?;
    scene_to_mesh(scene, Path::new(name_hint))
}

fn scene_to_mesh(scene: mesh_loader::Scene, path: &Path) -> anyhow::Result<LoadedMesh> {
    let mesh = mesh_loader::Mesh::merge(scene.meshes);
    anyhow::ensure!(
        !mesh.vertices.is_empty(),
        "no geometry in mesh file {}",
        path.display()
    );
    let indices = mesh.faces.iter().flatten().copied().collect();
    Ok(LoadedMesh {
        vertices: mesh.vertices,
        normals: mesh.normals,
        indices,
        colors: mesh.colors[0].clone(),
    })
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
