all: build\triangulate_mesh.exe build\mvc_mesh.exe

build\triangulate_mesh.exe: src\triangulate_mesh.cu src\mesh_io.hpp
	mkdir -p build
	nvcc -o build\triangulate_mesh.exe src\triangulate_mesh.cu -Isrc -O3

build\mvc_mesh.exe: src\mvc_mesh.cu src\mesh_io.hpp
	mkdir -p build
	nvcc -o build\mvc_mesh.exe src\mvc_mesh.cu -Isrc -O3