use std::fs;
use std::path::PathBuf;

fn scan_dir_recursively(entry_path: PathBuf, cuda_files: &mut Vec<PathBuf>) {
    for entry in fs::read_dir(entry_path).unwrap().filter_map(Result::ok) {
        let entry_path = entry.path();
        let entry_type = entry.file_type().unwrap();

        if entry_type.is_file()
            && entry_path
                .extension()
                .map(|ext| ext == "cu")
                .unwrap_or_default()
        {
            cuda_files.push(entry_path);
            continue;
        }

        if entry_type.is_dir() {
            scan_dir_recursively(entry_path, cuda_files);
        }
    }
}

fn main() {
    #[cfg(feature = "cuda")]
    {
        let mut cuda_files = vec![];
        scan_dir_recursively(PathBuf::from("src/cuda_kernels"), &mut cuda_files);

        if !cuda_files.is_empty() {
            cc::Build::new()
                .compiler("nvcc")
                .no_default_flags(true)
                .warnings(false)
                .files(cuda_files)
                .compile("algorithms_cuda_kernels")
        }
    }
}
